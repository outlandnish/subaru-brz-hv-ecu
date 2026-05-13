#pragma once
#include <stdint.h>
#include <STM32FreeRTOS.h>
#include "can.h"
#include "bms.h"
#include "ivt-s/ivt_shunt.h"

// Balancing mode matching EXTERNAL_PLAN §6
enum BalanceMode : uint8_t {
  BALANCE_DISABLED   = 0x00,
  BALANCE_DELTA_V    = 0x01,
  BALANCE_ABSOLUTE   = 0x02,
};

// BMS CAN protocol state, corresponding to EXTERNAL_PLAN §8 BMS_State encoding
enum ExtBMSState : uint8_t {
  EXT_BMS_INIT       = 0x00,
  EXT_BMS_READY      = 0x01,
  EXT_BMS_ACTIVE     = 0x02,
  EXT_BMS_BALANCING  = 0x03,
  EXT_BMS_CHARGING   = 0x04,
  EXT_BMS_FAULT      = 0x05,
  EXT_BMS_PRECHARGE  = 0x06,
};

// Precharge state encoding per EXTERNAL_PLAN §9
enum ExtPrechargeState : uint8_t {
  EXT_PRECHARGE_IDLE        = 0x00,
  EXT_PRECHARGE_IN_PROGRESS = 0x01,
  EXT_PRECHARGE_COMPLETE    = 0x02,
  EXT_PRECHARGE_TIMEOUT     = 0x03,
  EXT_PRECHARGE_FAULT       = 0x04,
};

// Config exposed to BMSCANBroadcaster so the broadcast layer doesn't reach into
// BMS internals for pack-level limits. All voltages in mV, currents in 10mA steps.
struct BMSCANConfig {
  uint16_t charge_voltage_10mv;          // Max charge voltage, raw wire value (10 mV/LSB for 0x351)
  uint16_t charge_current_limit_100ma;   // Max charge current (100 mA/LSB)
  uint16_t discharge_current_limit_100ma;
  uint16_t discharge_voltage_10mv;       // Min discharge voltage, raw wire value (10 mV/LSB)

  uint16_t soh_percent_x100;           // SOH 0.01%/LSB, e.g. 9500 = 95.00%

  BalanceMode balance_mode;
  uint16_t balance_param_mv;           // Delta-V threshold or absolute target in mV

  uint8_t chain0_modules;
  uint8_t chain1_modules;

  // Contactor role bitmask: bit N = K(N+1) closed
  // Matches Contactor_State in 0x405
  uint8_t contactor_closed_mask;       // Set by caller; broadcast reflects this
};

class BMSCANBroadcaster {
public:
  BMSCANBroadcaster(BatteryManagementSystem *bms, IVTShunt *ivt, CANBus *can_bus);

  // Must be called before start_tasks(). Sets pack-level broadcast config.
  void set_config(const BMSCANConfig &cfg);

  // Live setters for fields that change at runtime
  void set_contactor_mask(uint8_t mask) { config.contactor_closed_mask = mask; }

  // Start all FreeRTOS broadcast tasks
  bool start_tasks();

private:
  BatteryManagementSystem *bms;
  IVTShunt *ivt;
  CANBus *can;

  BMSCANConfig config;

  // Task handles
  TaskHandle_t task_ecosystem_handle;
  TaskHandle_t task_cell_voltage_handle;
  TaskHandle_t task_balancing_temp_handle;
  TaskHandle_t task_config_contactor_handle;

  // --- CRC-8/AUTOSAR (poly 0x2F, init 0xFF, no reflect, final XOR 0xFF) ---
  static uint8_t crc8_autosar(const uint8_t *data, uint8_t len);

  // --- Frame encoders ---
  void send_0x351();
  void send_0x355();
  void send_0x356();
  void send_0x359();
  void send_0x35C();
  void send_0x400_0x401(uint8_t module_idx, const uint32_t *voltages, uint8_t count);
  void send_0x402(uint8_t module_idx, const uint32_t *voltages, uint8_t count);
  void send_0x403(uint8_t module_idx, const uint32_t *voltages, uint8_t count);
  void send_0x404();
  void send_0x405();

  // --- Helper: map internal BMS/HV states to external protocol state ---
  ExtBMSState map_bms_state() const;

  // --- Task loops ---
  void ecosystem_task_loop();      // 0x351, 0x355, 0x35C @ 1000ms; 0x356, 0x359 @ 500ms
  void cell_voltage_task_loop();   // 0x400, 0x401 @ 100ms/module
  void balancing_temp_task_loop(); // 0x402, 0x403 @ 500ms/module
  void config_contactor_task_loop(); // 0x404 @ 5000ms + on change; 0x405 @ 100ms

  // Static wrappers for FreeRTOS
  static void ecosystem_task_wrapper(void *pv);
  static void cell_voltage_task_wrapper(void *pv);
  static void balancing_temp_task_wrapper(void *pv);
  static void config_contactor_task_wrapper(void *pv);
};
