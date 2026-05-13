/*
 * BMS CAN Protocol v0.1 — main entry point
 *
 * Brings up the MC33772B dual-chain BMS, IVT-S shunt, and the external CAN
 * broadcast layer defined in EXTERNAL_PLAN.md (0x351–0x35C, 0x400–0x405).
 * UDS firmware update (§10.5) is deferred to a future build.
 *
 * Hardware assumptions:
 *   - Taycan 6S2P modules with MC33772B ICs — always 6 cells/module, MC33772 device type
 *   - Dual BCC chains (chain 0 = bcc0, chain 1 = bcc1)
 *   - IVT-S shunt on HV CAN (0x521–0x528)
 *   - Contactors driven by DRV8874 (HV side)
 */

#include "main.h"
#include "bms/bms_can.h"
#include <STM32FreeRTOS.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"

// ─── CAN buses ───────────────────────────────────────────────────────────────
static CANBus *hv_can = nullptr;

// ─── Subsystems ──────────────────────────────────────────────────────────────
static IVTShunt                    *ivt_shunt = nullptr;
static BatteryManagementSystem     *bms       = nullptr;
static BMSCANBroadcaster           *bms_can   = nullptr;

// ─── FreeRTOS queues ─────────────────────────────────────────────────────────
static QueueHandle_t hv_can_queue = nullptr;
#define CAN_QUEUE_LENGTH 32

// ─── BCC hardware config ──────────────────────────────────────────────────────
static BatteryCellControllerConfig bcc0_config;
static BatteryCellControllerConfig bcc1_config;

// ─── CAN RX polling task ──────────────────────────────────────────────────────
static void can_rx_task(void *) {
  CAN_FRAME frame;
  while (true) {
    if (hv_can && hv_can->available()) {
      while (hv_can->read(frame))
        xQueueSend(hv_can_queue, &frame, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ─── IVT-S frame dispatch task ────────────────────────────────────────────────
static void ivt_process_task(void *) {
  CAN_FRAME frame;
  while (true) {
    if (xQueueReceive(hv_can_queue, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
      if ((frame.id >= 0x521 && frame.id <= 0x528) || frame.id == 0x511) {
        if (ivt_shunt) ivt_shunt->process_can_frame(&frame);
      }
    }
  }
}

// ─── Arduino entry points ─────────────────────────────────────────────────────
void setup() {
  DebugSerial.begin(115200);
#ifdef DEBUG_WAIT_FOR_SERIAL
  while (!DebugSerial) {}
#endif

  debug_println("\n=== BMS CAN Broadcaster ===");
  debug_println("Protocol: BMS CAN v0.1 (EXTERNAL_PLAN.md)");

  Param::LoadDefaults();

  // ── CAN buses ────────────────────────────────────────────────────────────
  hv_can = new CANBus(HV_CAN_RX, HV_CAN_TX);
  hv_can->begin(500000);  // 500 kbps per protocol spec

  hv_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));

  // ── IVT-S ────────────────────────────────────────────────────────────────
  ivt_shunt = new IVTShunt();
  ivt_shunt->begin(hv_can);
  ivt_shunt->start();

  // ── BCC hardware config — Taycan 6S2P, MC33772B always ───────────────────
  const uint8_t chain0_count = (uint8_t)Param::GetInt(Param::bcc0DeviceCount);
  const uint8_t chain1_count = (uint8_t)Param::GetInt(Param::bcc1DeviceCount);

  bcc0_config.device_count = chain0_count;
  bcc0_config.cell_count   = 6;
  bcc0_config.device_type  = BCC_DEVICE_MC33772;
  for (uint8_t i = 0; i < chain0_count; i++)
    bcc0_config.devices[i] = BCC_DEVICE_MC33772;
  bcc0_config.enable_pin = BCC0_ENABLE;
  bcc0_config.intb_pin   = BCC0_INTB;
  bcc0_config.cs_pin     = BCC0_TX_CS;
  bcc0_config.loopback   = false;

  bcc1_config.device_count = chain1_count;
  bcc1_config.cell_count   = 6;
  bcc1_config.device_type  = BCC_DEVICE_MC33772;
  for (uint8_t i = 0; i < chain1_count; i++)
    bcc1_config.devices[i] = BCC_DEVICE_MC33772;
  bcc1_config.enable_pin = BCC1_ENABLE;
  bcc1_config.intb_pin   = BCC1_INTB;
  bcc1_config.cs_pin     = BCC1_TX_CS;
  bcc1_config.loopback   = false;

  // ── BMS ──────────────────────────────────────────────────────────────────
  bms = new BatteryManagementSystem(&bcc0_config, &bcc1_config);

  BMSChargingConfig charging_cfg;
  charging_cfg.target_cell_voltage      = Param::GetFloat(Param::targetCellVolt) / 1000.0f;
  charging_cfg.balance_threshold_mv     = Param::GetFloat(Param::balanceThreshold);
  charging_cfg.balance_target_mv        = Param::GetFloat(Param::balanceTarget);
  charging_cfg.balancing_timer_min      = (uint16_t)Param::GetInt(Param::balanceTimerMin);
  charging_cfg.measurement_interval_ms  = (uint16_t)Param::GetInt(Param::measureInterval);
  charging_cfg.battery_capacity_ah      = Param::GetFloat(Param::batteryCapacity);
  charging_cfg.max_charge_current_a     = Param::GetFloat(Param::maxChargeCurrent);
  charging_cfg.min_soc_percent          = Param::GetFloat(Param::minSocPercent);
  charging_cfg.max_soc_percent          = Param::GetFloat(Param::maxSocPercent);
  bms->set_charging_config(charging_cfg);

  bms->set_contactor_pins(
    HV_CONTACTOR_1_PIN,
    HV_CONTACTOR_2_PIN,
    HV_CONTACTOR_NSLEEP_PIN,
    HV_CONTACTOR_FAULT_PIN
  );
  bms->set_ivt_shunt(ivt_shunt);
  bms->set_can_buses(nullptr, hv_can);
  bms->initialize(nullptr);

  // ── BMSCANBroadcaster ────────────────────────────────────────────────────
  bms_can = new BMSCANBroadcaster(bms, ivt_shunt, hv_can);

  const uint16_t total_cells = bms->get_bcc0_total_cell_count() + bms->get_bcc1_total_cell_count();
  const float target_cell_v  = Param::GetFloat(Param::targetCellVolt) / 1000.0f;  // mV → V
  const float max_chg_a      = Param::GetFloat(Param::maxChargeCurrent);

  BMSCANConfig can_cfg;
  // Pack charge voltage limit: 10 mV/LSB (ecosystem standard for 0x351)
  can_cfg.charge_voltage_10mv = (uint16_t)((target_cell_v * total_cells * 1000.0f) / 10.0f);
  // Current limits: 100 mA/LSB
  can_cfg.charge_current_limit_100ma    = (uint16_t)(max_chg_a * 10.0f);
  can_cfg.discharge_current_limit_100ma = (uint16_t)(max_chg_a * 10.0f);
  // Min discharge voltage: 2.5 V/cell, 10 mV/LSB (ecosystem standard for 0x351)
  can_cfg.discharge_voltage_10mv = (uint16_t)((2500.0f * total_cells) / 10.0f);
  can_cfg.soh_percent_x100     = 10000;  // 100.00% default; writable via UDS DID 0xD105 later
  can_cfg.balance_mode         = BALANCE_DELTA_V;
  can_cfg.balance_param_mv     = (uint16_t)Param::GetFloat(Param::balanceThreshold);
  can_cfg.chain0_modules       = chain0_count;
  can_cfg.chain1_modules       = chain1_count;
  can_cfg.contactor_closed_mask = 0x00;  // All contactors open at boot

  bms_can->set_config(can_cfg);

  debug_printf("BMS CAN: chain0=%d modules, chain1=%d modules, total cells=%d\r\n",
               chain0_count, chain1_count, total_cells);

  // ── Start tasks ──────────────────────────────────────────────────────────
  bms->start_tasks();
  bms_can->start_tasks();

  xTaskCreate(can_rx_task,      "CAN_RX",      512, nullptr, 3, nullptr);
  xTaskCreate(ivt_process_task, "IVT_Process", 512, nullptr, 2, nullptr);

  debug_println("BMS CAN: starting scheduler");
  vTaskStartScheduler();
  while (true) {}
}

void loop() {
  // All work is in FreeRTOS tasks.
}

#pragma GCC diagnostic pop
