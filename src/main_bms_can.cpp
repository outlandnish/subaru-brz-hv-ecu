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
#include "bms/bms_uds.h"
#include "bms/bms_uds_tp.h"
#ifdef BMS_M3_CAN
#include "bms/m3_can.h"
#endif
#include "param_save.h"
#include <STM32FreeRTOS.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"

extern "C" void *_sbrk(int);

// ─── CAN buses ───────────────────────────────────────────────────────────────
static CANBus *hv_can = nullptr;
#ifdef BMS_M3_CAN
static CANBus *m3_can = nullptr;
static M3CANManager *m3_mgr = nullptr;
#endif

// ─── Subsystems ──────────────────────────────────────────────────────────────
static IVTShunt                    *ivt_shunt = nullptr;
static BatteryManagementSystem     *bms       = nullptr;
static BMSCANBroadcaster           *bms_can   = nullptr;
static BMSUDSServer                *bms_uds   = nullptr;

// ─── FreeRTOS queues ─────────────────────────────────────────────────────────
static QueueHandle_t hv_can_queue  = nullptr;
static QueueHandle_t uds_can_queue = nullptr;
#define CAN_QUEUE_LENGTH 32

// ─── BCC hardware config ──────────────────────────────────────────────────────
static BatteryCellControllerConfig bcc0_config;
static BatteryCellControllerConfig bcc1_config;

// ─── CAN RX polling task ──────────────────────────────────────────────────────
static void can_rx_task(void *) {
  CAN_FRAME frame;
  while (true) {
    if (hv_can && hv_can->available()) {
      while (hv_can->read(frame)) {
        if (frame.id == UDS_REQ_ID || frame.id == UDS_FUNC_ID) {
          xQueueSend(uds_can_queue, &frame, 0);
        } else {
          xQueueSend(hv_can_queue, &frame, 0);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ─── IVT-S frame dispatch task ────────────────────────────────────────────────
static void ivt_process_task(void *) {
  if (ivt_shunt) ivt_shunt->configure_if_needed();

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
  for (volatile uint32_t i = 0; i < 4000000U; i++);
#endif

  debug_serial_init();
  debug_println("\n=== BMS CAN Broadcaster ===");
  debug_println("Protocol: BMS CAN v0.1");

  Param::LoadDefaults();
  parm_load();

  // ── CAN buses ────────────────────────────────────────────────────────────
  hv_can = new CANBus(HV_CAN_RX, HV_CAN_TX);
  hv_can->begin(500000);  // 500 kbps

#ifdef BMS_M3_CAN
  m3_can = new CANBus(M3_CAN_RX, M3_CAN_TX);
  m3_can->begin(500000);  // 500 kbps
  m3_mgr = new M3CANManager(m3_can);
#endif

  hv_can_queue  = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));
  uds_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));

  // ── IVT-S ────────────────────────────────────────────────────────────────
  ivt_shunt = new IVTShunt();
  ivt_shunt->begin(hv_can);

  // ── BCC hardware config — device counts from persisted params ────────────
  // bcc0DeviceCount == 0 means "not yet configured"; BCC init is deferred until
  // the user writes a nonzero count via UDS DID 0xD401 and reboots.
  const uint8_t chain0_count = (uint8_t)Param::GetInt(Param::bcc0DeviceCount);
  const uint8_t chain1_count = (uint8_t)Param::GetInt(Param::bcc1DeviceCount);
  const bcc_device_t chain0_type = (Param::GetInt(Param::bcc0DeviceType) == 0)
                                   ? BCC_DEVICE_MC33771 : BCC_DEVICE_MC33772;
  const bcc_device_t chain1_type = (Param::GetInt(Param::bcc1DeviceType) == 0)
                                   ? BCC_DEVICE_MC33771 : BCC_DEVICE_MC33772;

  bcc0_config.device_count = chain0_count;
  bcc0_config.cell_count   = 6;
  bcc0_config.device_type  = chain0_type;
  for (uint8_t i = 0; i < chain0_count; i++)
    bcc0_config.devices[i] = chain0_type;
  bcc0_config.enable_pin = BCC0_ENABLE;
  bcc0_config.intb_pin   = BCC0_INTB;
  bcc0_config.cs_pin     = BCC0_TX_CS;
  bcc0_config.loopback   = false;

  bcc1_config.device_count = chain1_count;
  bcc1_config.cell_count   = 6;
  bcc1_config.device_type  = chain1_type;
  for (uint8_t i = 0; i < chain1_count; i++)
    bcc1_config.devices[i] = chain1_type;
  bcc1_config.enable_pin = BCC1_ENABLE;
  bcc1_config.intb_pin   = BCC1_INTB;
  bcc1_config.cs_pin     = BCC1_TX_CS;
  bcc1_config.loopback   = false;

  if (chain0_count == 0)
    debug_println("BMS: BCC0 not configured — set bcc0DeviceCount via UDS and reboot");

  // ── BMS ──────────────────────────────────────────────────────────────────
  BMSChargingConfig charging_cfg;
  charging_cfg.target_cell_voltage      = Param::GetInt(Param::ovpThresholdMv) / 1000.0f;
  charging_cfg.balance_threshold_mv     = (float)Param::GetInt(Param::balanceDeltaMv);
  charging_cfg.balance_target_mv        = (float)Param::GetInt(Param::balanceAbsMv);
  charging_cfg.balancing_timer_min      = (uint16_t)Param::GetInt(Param::balanceTimerMin);
  charging_cfg.measurement_interval_ms  = (uint16_t)Param::GetInt(Param::measureInterval);
  charging_cfg.battery_capacity_ah      = Param::GetFloat(Param::batteryCapacity);
  charging_cfg.max_charge_current_a     = Param::GetInt(Param::ocpChargeMa) / 1000.0f;
  charging_cfg.min_soc_percent          = Param::GetFloat(Param::minSocPercent);
  charging_cfg.max_soc_percent          = Param::GetFloat(Param::maxSocPercent);

  // Construct BMS and initialize BCC hardware before any other setup disturbs SPI/DMA
  bms = new BatteryManagementSystem(&bcc0_config, &bcc1_config);
  bms->initialize(nullptr);  // BCC begin() immediately after construction, like simple_charger
  bms->set_charging_config(charging_cfg);
  bms->set_contactor_pins(
    HV_CONTACTOR_1_PIN,
    HV_CONTACTOR_2_PIN,
    HV_CONTACTOR_NSLEEP_PIN,
    HV_CONTACTOR_FAULT_PIN
  );
  bms->set_ivt_shunt(ivt_shunt);
#ifdef BMS_M3_CAN
  bms->set_can_buses(m3_can, hv_can);
  bms->set_m3_can_manager(m3_mgr);
#else
  bms->set_hv_can(hv_can);
#endif

  // ── BMSCANBroadcaster ────────────────────────────────────────────────────
  bms_can = new BMSCANBroadcaster(bms, ivt_shunt, hv_can);

  const uint16_t total_cells = bms->get_bcc0_total_cell_count() + bms->get_bcc1_total_cell_count();
  const float target_cell_v  = Param::GetInt(Param::ovpThresholdMv) / 1000.0f;
  const float max_chg_a      = Param::GetInt(Param::ocpChargeMa) / 1000.0f;

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
  can_cfg.balance_param_mv     = (uint16_t)Param::GetInt(Param::balanceDeltaMv);
  can_cfg.chain0_modules       = chain0_count;
  can_cfg.chain1_modules       = chain1_count;
  can_cfg.contactor_closed_mask = 0x00;  // All contactors open at boot

  bms_can->set_config(can_cfg);

  debug_printf("BMS CAN: chain0=%d modules (%s), chain1=%d modules (%s), total cells=%d\r\n",
               chain0_count, (chain0_type == BCC_DEVICE_MC33771) ? "MC33771" : "MC33772",
               chain1_count, (chain1_type == BCC_DEVICE_MC33771) ? "MC33771" : "MC33772",
               total_cells);

  // ── UDS server ───────────────────────────────────────────────────────────
  bms_uds = new BMSUDSServer(bms, ivt_shunt, &bms_can->get_config_ref());
  bms_uds_tp_init(hv_can, uds_can_queue);
  bms_uds->init();

  // ── Start tasks ──────────────────────────────────────────────────────────
  {
    extern char _end, _estack;
    char *heap_now = (char*)_sbrk(0);
    debug_printf("Pre-task heap: used=%d free=%d\r\n",
      (int)(heap_now - &_end),
      (int)(&_estack - heap_now));
  }
  bms->start_tasks();
  bms_can->start_tasks();
  bms_uds->start_tasks();

  xTaskCreate(can_rx_task,      "CAN_RX",      4096, nullptr, 3, nullptr);
  xTaskCreate(ivt_process_task, "IVT_Process", 4096, nullptr, 2, nullptr);

  debug_println("BMS CAN: starting scheduler");
  vTaskStartScheduler();
  while (true) {}
}

void loop() {
  // All work is in FreeRTOS tasks.
}

#pragma GCC diagnostic pop
