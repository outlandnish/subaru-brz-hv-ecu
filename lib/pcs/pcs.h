#pragma once

/*
 * Tesla Model 3 PCS (Power Conversion System) Controller
 *
 * High-level controller that manages the PCS hardware pins and coordinates
 * with the low-level PCSCan communication layer.
 *
 * Based on reference implementation from:
 * https://github.com/damienmaguire/Tesla-Model-3-Charger
 *
 * STATE MACHINE & BMS COORDINATION:
 * ---------------------------------
 * The PCS owns the precharge circuit and performs the actual HV precharging
 * for ALL HV system operations (charging, drive mode, etc).
 * The BMS coordinates the overall HV system and must monitor PCS state.
 *
 * THREAD SAFETY: All BMS→PCS commands MUST use async methods (queue-based)
 *
 * Typical HV activation sequence (charging):
 *  1. BMS closes negative contactor (HV precharge begins via external circuit)
 *  2. BMS calls PCSController::start_charging_async()
 *  3. PCS enters PCS_STATE_PRECHARGE (internal initialization)
 *  4. PCS waits for initialization timer (~3 seconds for CAN startup, etc)
 *  5. PCS enters PCS_STATE_ACTIVATE, then PCS_STATE_CHARGING
 *  6. BMS polls is_precharge_complete() to verify PCS is ready
 *  7. BMS verifies precharge voltage with IVT shunt
 *  8. BMS closes positive contactor (completes HV connection)
 *  9. BMS transitions to HV_Active state
 * 10. Charging begins
 *
 * Typical HV activation sequence (drive mode):
 *  1. BMS closes negative contactor (HV precharge begins)
 *  2. BMS calls PCSController::start_drive_mode_async()
 *  3. PCS performs same initialization sequence
 *  4. PCS enters PCS_STATE_DRIVE (DCDC only, no charging)
 *  5. BMS verifies precharge voltage
 *  6. BMS closes positive contactor
 *  7. 12V DCDC operates for vehicle accessories
 *
 * BMS coordination methods:
 *  - start_charging_async() - Request precharge + charging mode
 *  - start_drive_mode_async() - Request precharge + DCDC mode
 *  - stop_async() - Request graceful shutdown
 *  - emergency_stop_async() - Request immediate shutdown
 *
 * BMS monitoring methods:
 *  - is_precharging() - PCS is actively precharging
 *  - is_precharge_complete() - PCS ready for HV connection
 *  - is_active() - PCS in active operation (charging or drive)
 *  - is_faulted() - PCS has encountered a fault
 */

#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "can.h"
#include "pcs-can.h"

// PCS state machine states
enum PCSState : uint8_t {
  PCS_STATE_INIT,
  PCS_STATE_OFF,
  PCS_STATE_WAITSTART,
  PCS_STATE_PRECHARGE,      // HV enabled, waiting for precharge timer
  PCS_STATE_ACTIVATE,       // Set charge/DCDC enables
  PCS_STATE_CHARGING,       // Active charging (EVSE activated)
  PCS_STATE_DRIVE,          // Drive mode (DCDC only)
  PCS_STATE_STOP,           // Shutdown sequence
  PCS_STATE_FAULT
};

// PCS command queue message types
enum PCSCommandType : uint8_t {
  PCS_CMD_SET_CHARGE_POWER,
  PCS_CMD_SET_HV_VOLTAGE,
  PCS_CMD_SET_DCDC_VOLTAGE,
  PCS_CMD_SET_AC_LIMIT,
  PCS_CMD_ENABLE_PCS,
  // State machine commands (for BMS coordination)
  PCS_CMD_START_CHARGING,
  PCS_CMD_START_DRIVE_MODE,
  PCS_CMD_STOP,
  PCS_CMD_EMERGENCY_STOP
};

struct PCSCommand {
  PCSCommandType type;
  union {
    uint16_t value_u16;
    float value_float;
    bool value_bool;
  };
};

class PCSController {
    // Direct control methods (only for PCS task internal use)
  static void set_charge_power(uint16_t power_w);
  static void set_hv_voltage(uint16_t voltage_v);
  static void set_dcdc_voltage(float voltage_v);
  static void set_ac_current_limit(uint8_t limit_a);
  static void set_evse_limit(uint8_t limit_a);
  static void set_cable_limit(uint8_t limit);
  static void enable_pcs(bool enable);
  
public:
  // Initialize the PCS controller with control pins
  static void begin(CANBus *ipc_can_bus, uint8_t pcs_enable_pin, uint8_t charge_enable_pin, uint8_t dcdc_enable_pin);

  // Start the PCS FreeRTOS task
  static bool start_task();

  // Periodic update - called by task loop
  static void update();

  // State machine control - INTERNAL USE ONLY (called from queue processing)
  // BMS should use the async versions below for thread-safe operation
  static void start_charging();     // Initiate precharge and charging sequence
  static void start_drive_mode();   // Initiate precharge and drive mode (DCDC only)
  static void stop();               // Stop operation (graceful shutdown)
  static void emergency_stop();     // Immediate shutdown

  // Thread-safe state machine control (queue-based, safe to call from BMS task)
  static bool start_charging_async();    // Request charging mode with precharge
  static bool start_drive_mode_async();  // Request drive mode (DCDC) with precharge
  static bool stop_async();              // Request graceful shutdown
  static bool emergency_stop_async();    // Request immediate shutdown

  // Async control methods (queue-based, safe to call from any task)
  static bool set_charge_power_async(uint16_t power_w);
  static bool set_hv_voltage_async(uint16_t voltage_v);
  static bool set_dcdc_voltage_async(float voltage_v);
  static bool set_ac_current_limit_async(uint8_t limit_a);
  static bool enable_pcs_async(bool enable);

  // State accessors
  static PCSState get_state() { return current_state; }
  static PCSMode get_mode() { return PCSCan::get_mode(); }

  // BMS coordination - check PCS readiness
  static bool is_precharging() { return current_state == PCS_STATE_PRECHARGE; }
  static bool is_precharge_complete() {
    return current_state == PCS_STATE_ACTIVATE ||
           current_state == PCS_STATE_CHARGING ||
           current_state == PCS_STATE_DRIVE;
  }
  static bool is_active() {
    return current_state == PCS_STATE_CHARGING ||
           current_state == PCS_STATE_DRIVE;
  }
  static bool is_off() { return current_state == PCS_STATE_OFF; }
  static bool is_faulted() { return current_state == PCS_STATE_FAULT; }

  // Pin state accessors
  static bool is_pcs_enabled() { return pcs_pin_enabled; }
  static bool is_charge_enabled() { return charge_pin_enabled; }
  static bool is_dcdc_enabled() { return dcdc_pin_enabled; }

  // Pass-through struct-level accessors to PCSCan status
  static const ChargerStatus& get_charger_status() { return PCSCan::get_charger_status(); }
  static const DCDCStatus& get_dcdc_status() { return PCSCan::get_dcdc_status(); }
  static const ACStatus& get_ac_status() { return PCSCan::get_ac_status(); }
  static const TemperatureData& get_temperature_data() { return PCSCan::get_temperature_data(); }
  static const VoltageData& get_voltage_data() { return PCSCan::get_voltage_data(); }
  static const DCCurrentData& get_dc_current_data() { return PCSCan::get_dc_current_data(); }
  static const AlertData& get_alert_data() { return PCSCan::get_alert_data(); }

private:
  // Control pins
  static uint8_t pcs_enable_pin;
  static uint8_t charge_enable_pin;
  static uint8_t dcdc_enable_pin;

  // Pin states
  static bool pcs_pin_enabled;
  static bool charge_pin_enabled;
  static bool dcdc_pin_enabled;

  // State tracking
  static PCSState current_state;
  static PCSState target_state;  // Target state after precharge (CHARGING or DRIVE)

  // Precharge timing (in 100ms ticks, default 30 = 3 seconds)
  static uint8_t precharge_timer;
  static const uint8_t PRECHARGE_TIME_TICKS = 30;

  // Power down timing (in 100ms ticks, default 10 = 1 second)
  static uint8_t powerdown_timer;
  static const uint8_t POWERDOWN_TIME_TICKS = 10;

  // Message timing
  static uint32_t last_update_ms;
  static uint32_t last_100ms_update;

  // Command queue for FreeRTOS tasks
  static QueueHandle_t command_queue;

  // FreeRTOS task functions
  static void task_loop();
  static void task_wrapper(void *pvParameters);

  // Internal helper functions
  static void process_command_queue();
  static void send_periodic_messages();
  static void update_control_pins();
  static void run_state_machine();
  static void disable_all();
};
