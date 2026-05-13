#include "bms_can.h"
#include "debug_serial.h"
#include "Arduino.h"
#include <string.h>

// CRC-8/AUTOSAR lookup table (poly=0x2F, init=0xFF, no reflect, final XOR=0xFF)
static const uint8_t CRC8_AUTOSAR_TABLE[256] = {
    0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD,
    0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A,
    0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63,
    0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
    0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE,
    0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
    0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10,
    0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
    0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B,
    0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
    0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85,
    0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
    0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58,
    0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
    0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6,
    0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
    0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E,
    0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
    0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80,
    0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
    0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D,
    0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
    0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3,
    0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
    0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8,
    0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F,
    0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66,
    0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
    0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB,
    0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
    0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15,
    0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42,
};

BMSCANBroadcaster::BMSCANBroadcaster(BatteryManagementSystem *bms_, IVTShunt *ivt_, CANBus *can_bus)
    : bms(bms_), ivt(ivt_), can(can_bus) {
  memset(&config, 0, sizeof(config));
  config.soh_percent_x100 = 10000; // 100.00% default
  config.balance_mode = BALANCE_DISABLED;
  task_ecosystem_handle = nullptr;
  task_cell_voltage_handle = nullptr;
  task_balancing_temp_handle = nullptr;
  task_config_contactor_handle = nullptr;
}

void BMSCANBroadcaster::set_config(const BMSCANConfig &cfg) {
  config = cfg;
}

bool BMSCANBroadcaster::start_tasks() {
  BaseType_t r;

  r = xTaskCreate(ecosystem_task_wrapper, "BMS_CAN_Eco", 768, this, 1, &task_ecosystem_handle);
  if (r != pdPASS) { debug_println("BMS_CAN: failed to create ecosystem task"); return false; }

  r = xTaskCreate(cell_voltage_task_wrapper, "BMS_CAN_CV", 768, this, 1, &task_cell_voltage_handle);
  if (r != pdPASS) { debug_println("BMS_CAN: failed to create cell voltage task"); return false; }

  r = xTaskCreate(balancing_temp_task_wrapper, "BMS_CAN_BT", 768, this, 1, &task_balancing_temp_handle);
  if (r != pdPASS) { debug_println("BMS_CAN: failed to create balancing/temp task"); return false; }

  r = xTaskCreate(config_contactor_task_wrapper, "BMS_CAN_CC", 768, this, 1, &task_config_contactor_handle);
  if (r != pdPASS) { debug_println("BMS_CAN: failed to create config/contactor task"); return false; }

  debug_println("BMS_CAN: all tasks started");
  return true;
}

// --- Static task wrappers ---
void BMSCANBroadcaster::ecosystem_task_wrapper(void *pv) {
  static_cast<BMSCANBroadcaster *>(pv)->ecosystem_task_loop();
}
void BMSCANBroadcaster::cell_voltage_task_wrapper(void *pv) {
  static_cast<BMSCANBroadcaster *>(pv)->cell_voltage_task_loop();
}
void BMSCANBroadcaster::balancing_temp_task_wrapper(void *pv) {
  static_cast<BMSCANBroadcaster *>(pv)->balancing_temp_task_loop();
}
void BMSCANBroadcaster::config_contactor_task_wrapper(void *pv) {
  static_cast<BMSCANBroadcaster *>(pv)->config_contactor_task_loop();
}

// --- CRC helper ---
uint8_t BMSCANBroadcaster::crc8_autosar(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc = CRC8_AUTOSAR_TABLE[crc ^ data[i]];
  }
  return crc ^ 0xFF;
}

// --- State mapping ---
ExtBMSState BMSCANBroadcaster::map_bms_state() const {
  HV_State hv = bms->get_hv_state();
  BMS_State st = bms->get_state();

  if (hv == HV_Fault || st == BMS_Error)     return EXT_BMS_FAULT;
  if (hv == HV_Precharge)                    return EXT_BMS_PRECHARGE;
  if (hv == HV_Shutdown)                     return EXT_BMS_READY;
  if (st == BMS_Initialization)              return EXT_BMS_INIT;
  if (st == BMS_CellBalancing)               return EXT_BMS_BALANCING;
  if (st == BMS_Charging)                    return EXT_BMS_CHARGING;
  if (hv == HV_Active)                       return EXT_BMS_ACTIVE;
  return EXT_BMS_READY;
}

// ===== Task loops =====

// Ecosystem task: 0x351,0x355,0x35C @ 1000 ms; 0x356,0x359 @ 500 ms
void BMSCANBroadcaster::ecosystem_task_loop() {
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t tick500 = 0;  // counts 500 ms intervals; send 1000 ms frames every 2nd

  while (true) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));

    send_0x356();
    send_0x406();
    send_0x359();

    if (tick500 & 1) {
      send_0x351();
      send_0x355();
      send_0x35C();
    }
    tick500++;
  }
}

// Cell voltage task: for each active module send 0x400 then 0x401, 100 ms between modules
void BMSCANBroadcaster::cell_voltage_task_loop() {
  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    uint8_t total = config.chain0_modules + config.chain1_modules;
    if (total == 0) total = 1;

    uint32_t voltages[BCC_MAX_CELLS];
    uint8_t count = 0;
    bms->get_cell_voltages(voltages, &count);

    for (uint8_t m = 0; m < total; m++) {
      send_0x400_0x401(m, voltages, count);
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
  }
}

// Balancing + temperature task: for each active module send 0x402 then 0x403, 500 ms between modules
void BMSCANBroadcaster::balancing_temp_task_loop() {
  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    uint8_t total = config.chain0_modules + config.chain1_modules;
    if (total == 0) total = 1;

    uint32_t voltages[BCC_MAX_CELLS];
    uint8_t count = 0;
    bms->get_cell_voltages(voltages, &count);

    for (uint8_t m = 0; m < total; m++) {
      send_0x402(m, voltages, count);
      send_0x403(m, voltages, count);
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
    }
  }
}

// Config + contactor task: 0x404 every 5000 ms (and on-change handled by periodic check),
// 0x405 every 100 ms.
void BMSCANBroadcaster::config_contactor_task_loop() {
  TickType_t last_wake = xTaskGetTickCount();
  uint32_t tick100 = 1;  // start at 1 so %50==0 fires after 5 s, not immediately at boot

  // Send 0x404 immediately on start so receivers don't wait up to 5 s.
  send_0x404();

  while (true) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));

    send_0x405();

    if (tick100 % 50 == 0) {
      send_0x404();
    }
    tick100++;
  }
}

// ===== Frame encoders =====

// 0x351 — Charge Voltage & Current Limits (1000 ms)
void BMSCANBroadcaster::send_0x351() {
  uint8_t d[8] = {0};
  d[0] = config.charge_voltage_10mv & 0xFF;
  d[1] = (config.charge_voltage_10mv >> 8) & 0xFF;
  d[2] = config.charge_current_limit_100ma & 0xFF;
  d[3] = (config.charge_current_limit_100ma >> 8) & 0xFF;
  d[4] = config.discharge_current_limit_100ma & 0xFF;
  d[5] = (config.discharge_current_limit_100ma >> 8) & 0xFF;
  d[6] = config.discharge_voltage_10mv & 0xFF;
  d[7] = (config.discharge_voltage_10mv >> 8) & 0xFF;
  can->sendMessage(0x351, d, 8);
}

// 0x355 — State of Charge / Health (1000 ms)
void BMSCANBroadcaster::send_0x355() {
  // SOC in 0.01%/LSB — bms->get_soc_precise() returns 0..100%
  uint16_t soc_x100 = (uint16_t)(bms->get_soc_precise() * 100.0f);
  uint16_t soh = config.soh_percent_x100;

  uint8_t d[8] = {0};
  d[0] = soc_x100 & 0xFF;
  d[1] = (soc_x100 >> 8) & 0xFF;
  d[2] = soh & 0xFF;
  d[3] = (soh >> 8) & 0xFF;
  d[4] = soc_x100 & 0xFF;  // SOC_High = same value
  d[5] = (soc_x100 >> 8) & 0xFF;
  // d[6-7] = 0x0000
  can->sendMessage(0x355, d, 8);
}

// 0x356 — Pack Summary (500 ms)
void BMSCANBroadcaster::send_0x356() {
  // Pack voltage: 10 mV/LSB, int16 — ecosystem standard (SimpBMS/Victron/SMA/Fronius)
  float pack_v = ivt ? ivt->get_voltage() : 0.0f;
  int16_t pack_v_enc = (int16_t)(pack_v * 100.0f);  // V → 10 mV/LSB

  // Pack current: 100 mA/LSB, int16 — positive = charging
  float pack_a = ivt ? ivt->get_current() : 0.0f;
  int16_t pack_a_enc = (int16_t)(pack_a * 10.0f);  // A → 100 mA units

  // Pack temp: 0.1°C/LSB, int16. Broadcast invalid (0x7FFF) until BCC NTC readback is wired in.
  int16_t temp_enc = 0x7FFF;

  uint8_t d[8] = {0};
  d[0] = (uint8_t)(pack_v_enc & 0xFF);
  d[1] = (uint8_t)((pack_v_enc >> 8) & 0xFF);
  d[2] = (uint8_t)(pack_a_enc & 0xFF);
  d[3] = (uint8_t)((pack_a_enc >> 8) & 0xFF);
  d[4] = (uint8_t)(temp_enc & 0xFF);
  d[5] = (uint8_t)((temp_enc >> 8) & 0xFF);
  // d[6-7] = 0
  can->sendMessage(0x356, d, 8);
}

// 0x406 — Extended Pack Summary (500 ms)
// High-resolution pack data for non-ecosystem receivers. Not compatible with Victron/SMA/Fronius.
// Bytes 0–3: PackVoltage  uint32, 1 mV/LSB,  little-endian. Range 0–4294967 V.
// Bytes 4–7: PackCurrent  int32,  1 mA/LSB,  little-endian. Positive = charging.
void BMSCANBroadcaster::send_0x406() {
  float pack_v = ivt ? ivt->get_voltage() : 0.0f;
  float pack_a = ivt ? ivt->get_current() : 0.0f;

  uint32_t v_enc = (uint32_t)(pack_v * 1000.0f);   // V → 1 mV/LSB
  int32_t  a_enc = (int32_t)(pack_a  * 1000.0f);   // A → 1 mA/LSB

  uint8_t d[8];
  d[0] = (uint8_t)(v_enc & 0xFF);
  d[1] = (uint8_t)((v_enc >> 8)  & 0xFF);
  d[2] = (uint8_t)((v_enc >> 16) & 0xFF);
  d[3] = (uint8_t)((v_enc >> 24) & 0xFF);
  d[4] = (uint8_t)(a_enc & 0xFF);
  d[5] = (uint8_t)((a_enc >> 8)  & 0xFF);
  d[6] = (uint8_t)((a_enc >> 16) & 0xFF);
  d[7] = (uint8_t)((a_enc >> 24) & 0xFF);
  can->sendMessage(0x406, d, 8);
}

// 0x359 — Fault Flags (500 ms)
void BMSCANBroadcaster::send_0x359() {
  uint16_t faults[11] = {0};
  bms->get_fault_status(faults);

  bool ov = (faults[BCC_FS_CELL_OV] != 0);
  bool uv = (faults[BCC_FS_CELL_UV] != 0);
  bool ot = (faults[BCC_FS_AN_OT_UT] & 0x00FF) != 0;  // over-temp bits
  bool ut = (faults[BCC_FS_AN_OT_UT] & 0xFF00) != 0;  // under-temp bits (upper byte)

  float pack_a = ivt ? ivt->get_current() : 0.0f;
  bool ocp_chg = (pack_a > 0 && bms->has_faults());   // simplified; real OCP from BCC
  bool ocp_dis = (pack_a < 0 && bms->has_faults());

  uint8_t total = config.chain0_modules + config.chain1_modules;
  // Count active modules — all are active when no comm faults
  bool chain0_fault = (faults[BCC_FS_COMM] != 0);

  uint8_t d[8] = {0};
  // Byte 0: protection flags
  if (ov) d[0] |= (1 << 0);
  if (uv) d[0] |= (1 << 1);
  if (ot) d[0] |= (1 << 2);
  if (ut) d[0] |= (1 << 3);
  // Byte 1: OCP
  if (ocp_chg) d[1] |= (1 << 0);
  if (ocp_dis) d[1] |= (1 << 1);
  // Byte 2: warnings — high/low voltage thresholds (reuse OV/UV as warnings too)
  if (ov) d[2] |= (1 << 0);
  if (uv) d[2] |= (1 << 1);
  if (ot) d[2] |= (1 << 2);
  // Byte 3: current warnings
  if (ocp_chg) d[3] |= (1 << 0);
  if (ocp_dis) d[3] |= (1 << 1);
  // Byte 4: active modules
  d[4] = chain0_fault ? 0 : total;
  // Byte 5: balancing modules — non-zero when in balancing state
  d[5] = (bms->get_state() == BMS_CellBalancing) ? total : 0;
  // d[6-7] = 0
  can->sendMessage(0x359, d, 8);
}

// 0x35C — Charger Control (1000 ms)
void BMSCANBroadcaster::send_0x35C() {
  uint8_t d[8] = {0};
  BMS_State st = bms->get_state();
  HV_State hv = bms->get_hv_state();

  bool charge_enable   = (st == BMS_Charging || st == BMS_Idle) && !bms->has_faults() && hv != HV_Fault;
  bool discharge_enable = (hv == HV_Active) && !bms->has_faults();

  if (charge_enable)   d[0] |= (1 << 0);
  if (discharge_enable) d[0] |= (1 << 2);
  can->sendMessage(0x35C, d, 8);
}

// 0x400 + 0x401 — Cell Voltages (100 ms/module)
void BMSCANBroadcaster::send_0x400_0x401(uint8_t module_idx, const uint32_t *voltages, uint8_t count) {

  const uint8_t cells_per_module = 6;
  uint8_t base = module_idx * cells_per_module;

  // 0x400: cells 1-3
  {
    uint8_t d[8] = {0};
    d[0] = module_idx;
    for (uint8_t c = 0; c < 3; c++) {
      uint16_t mv = (base + c < count) ? (uint16_t)(voltages[base + c] / 1000) : 0;
      d[1 + c * 2] = mv & 0xFF;
      d[2 + c * 2] = (mv >> 8) & 0xFF;
    }
    d[7] = crc8_autosar(d, 7);
    can->sendMessage(0x400, d, 8);
  }

  // 0x401: cells 4-6
  {
    uint8_t d[8] = {0};
    d[0] = module_idx;
    for (uint8_t c = 0; c < 3; c++) {
      uint16_t mv = (base + 3 + c < count) ? (uint16_t)(voltages[base + 3 + c] / 1000) : 0;
      d[1 + c * 2] = mv & 0xFF;
      d[2 + c * 2] = (mv >> 8) & 0xFF;
    }
    d[7] = crc8_autosar(d, 7);
    can->sendMessage(0x401, d, 8);
  }
}

// 0x402 — Balancing Status (500 ms/module)
void BMSCANBroadcaster::send_0x402(uint8_t module_idx, const uint32_t *voltages, uint8_t count) {
  (void)voltages; (void)count;  // voltage snapshot unused now; balance mask comes from BMS
  uint8_t balance_mask = bms->get_balance_mask(module_idx);

  uint8_t d[8] = {0};
  d[0] = module_idx;
  d[1] = balance_mask;
  d[2] = (uint8_t)config.balance_mode;
  d[3] = config.balance_param_mv & 0xFF;
  d[4] = (config.balance_param_mv >> 8) & 0xFF;
  // d[5-6] = 0
  d[7] = crc8_autosar(d, 7);
  can->sendMessage(0x402, d, 8);
}

// 0x403 — Temperature (500 ms/module). NTC channels broadcast 0x7FFF (invalid) until BCC
// temperature readback is surfaced through the BMS API.
void BMSCANBroadcaster::send_0x403(uint8_t module_idx, const uint32_t *voltages, uint8_t count) {
  (void)voltages; (void)count;
  // 0.01°C/LSB, int16. 0x7FFF = invalid/disconnected sensor.
  int16_t temp_a    = 0x7FFF;
  int16_t temp_b    = 0x7FFF;
  int16_t temp_die  = 0x7FFF;

  // Use IVT temperature for the first module as a plausible pack temperature
  // until per-module BCC readback is wired in.
  if (module_idx == 0 && ivt && ivt->is_alive()) {
    float t = ivt->get_temperature();
    // int16 range check before cast
    if (t > -327.0f && t < 327.0f) {
      temp_a   = (int16_t)(t * 100.0f);
      temp_b   = (int16_t)(t * 100.0f);
      temp_die = 0x7FFF;
    }
  }

  uint8_t d[8] = {0};
  d[0] = module_idx;
  d[1] = (uint8_t)(temp_a & 0xFF);
  d[2] = (uint8_t)((temp_a >> 8) & 0xFF);
  d[3] = (uint8_t)(temp_b & 0xFF);
  d[4] = (uint8_t)((temp_b >> 8) & 0xFF);
  d[5] = (uint8_t)(temp_die & 0xFF);
  d[6] = (uint8_t)((temp_die >> 8) & 0xFF);
  d[7] = crc8_autosar(d, 7);
  can->sendMessage(0x403, d, 8);
}

// 0x404 — BMS Config (5000 ms + on change)
void BMSCANBroadcaster::send_0x404() {
  uint8_t total = config.chain0_modules + config.chain1_modules;
  ExtBMSState ext_state = map_bms_state();

  uint8_t d[8] = {0};
  d[0] = config.chain0_modules;
  d[1] = config.chain1_modules;
  d[2] = total;
  d[3] = 0x01;  // Protocol_Version
  d[4] = (uint8_t)ext_state;
  // d[5-6] = 0
  d[7] = crc8_autosar(d, 7);
  can->sendMessage(0x404, d, 8);
}

// 0x405 — Contactor Status (100 ms)
void BMSCANBroadcaster::send_0x405() {
  // IVT V1 = pack side, V2 = inverter side per existing wiring in bms.cpp
  // Encoding: 100 mV/LSB, uint16 — supports up to 6553.5 V (hw max ~756 V)
  uint16_t precharge_mv = 0;
  uint16_t pack_mv      = 0;
  if (ivt && ivt->is_alive()) {
    float v2 = ivt->get_voltage2();
    float v1 = ivt->get_voltage();
    if (v2 > 0) precharge_mv = (uint16_t)(v2 * 10.0f);  // V → 100 mV/LSB
    if (v1 > 0) pack_mv      = (uint16_t)(v1 * 10.0f);  // V → 100 mV/LSB
  }

  ExtPrechargeState pc = EXT_PRECHARGE_IDLE;
  switch (bms->get_hv_state()) {
    case HV_Precharge: pc = EXT_PRECHARGE_IN_PROGRESS; break;
    case HV_Active:    pc = EXT_PRECHARGE_COMPLETE;    break;
    case HV_Fault:     pc = EXT_PRECHARGE_FAULT;       break;
    case HV_Shutdown:  pc = EXT_PRECHARGE_IDLE;        break;
    default:           pc = EXT_PRECHARGE_IDLE;        break;
  }

  uint8_t d[8] = {0};
  d[0] = config.contactor_closed_mask;
  d[1] = (uint8_t)pc;
  d[2] = precharge_mv & 0xFF;
  d[3] = (precharge_mv >> 8) & 0xFF;
  d[4] = pack_mv & 0xFF;
  d[5] = (pack_mv >> 8) & 0xFF;
  d[6] = 0;
  d[7] = crc8_autosar(d, 7);
  can->sendMessage(0x405, d, 8);
}
