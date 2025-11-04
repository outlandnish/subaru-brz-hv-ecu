#pragma once
#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include "can.h"

// PCS operational modes
enum PCSMode : uint8_t {
  PCS_MODE_OFF = 0x00,           // Both charger and DCDC off
  PCS_MODE_DCDC_ONLY = 0x09,     // DCDC only, charger disabled
  PCS_MODE_CHARGE_ONLY = 0x04,   // Charge only, DCDC disabled
  PCS_MODE_CHARGE_DCDC = 0x0D    // Both charger and DCDC enabled
};

// PCS command queue message types
enum PCSCommandType : uint8_t {
  PCS_CMD_SET_CHARGE_POWER,
  PCS_CMD_SET_TARGET_VOLTAGE,
  PCS_CMD_SET_MAX_POWER,
  PCS_CMD_ENABLE_CHARGING,
  PCS_CMD_ENABLE_DCDC
};

struct PCSCommand {
  PCSCommandType type;
  uint16_t value;
};

// PCS state machine states
enum PCSState : uint8_t {
  PCS_STATE_INIT,
  PCS_STATE_STANDBY,
  PCS_STATE_CHARGE_PREP,
  PCS_STATE_CHARGING,
  PCS_STATE_CHARGE_STOP,
  PCS_STATE_DCDC_ACTIVE,
  PCS_STATE_FAULT
};

/**
 * Tesla Model 3 Power Conversion System (PCS) Controller
 *
 * Manages communication with the Tesla Model 3 onboard charger/DCDC converter via CAN.
 * Handles both IPC CAN (PCS control messages) and M3 CAN (VCU communication).
 *
 * Based on Damien Maguire's implementation from:
 * https://github.com/damienmaguire/Tesla-Model-3-Charger
 */
class TeslaM3PCSController {
private:
  // Pin assignments
  uint8_t pcs_enable_pin;
  uint8_t pcs_charge_pin;
  uint8_t pcs_dcdc_pin;

  // CAN interfaces
  CANBus *ipc_can;  // IPC CAN for PCS control (500kbps)
  CANBus *m3_can;   // M3 CAN for VCU communication

  // State
  PCSState current_state;
  PCSMode current_mode;

  // Control parameters
  uint16_t target_voltage_mv;      // Target HV bus voltage in millivolts
  uint16_t charge_power_w;         // Requested charge power in watts
  uint16_t max_charge_power_w;     // Maximum charge power limit

  bool pcs_enabled;
  bool charge_enabled;
  bool dcdc_enabled;

  // Command queue for FreeRTOS tasks
  QueueHandle_t command_queue;

  // Message timing
  uint32_t last_0x22a_ms;
  uint32_t last_0x2b2_ms;
  uint32_t last_0x3b2_ms;
  uint32_t last_0x545_ms;
  uint32_t last_0x333_ms;

  // Message counters
  uint8_t msg_0x545_counter;
  bool msg_0x3b2_mux;

  // CAN message builders
  void send_0x22a_control();
  void send_0x2b2_power_request();
  void send_0x3b2_bms_log();
  void send_0x545_vcfront();
  void send_0x333_ui_request();

  // Command queue processing
  void process_command_queue();

  // Helper functions
  uint8_t calculate_crc(uint8_t *data, uint8_t len, uint16_t can_id);
  void update_control_outputs();

public:
  TeslaM3PCSController(uint8_t enable_pin, uint8_t charge_pin, uint8_t dcdc_pin);

  void begin(CANBus *ipc_can_bus, CANBus *m3_can_bus);
  void update();

  // Control methods (can be called from any task)
  void set_mode(PCSMode mode);
  bool set_target_voltage_mv_async(uint16_t voltage_mv);  // Queue-based for other tasks
  bool set_charge_power_w_async(uint16_t power_w);        // Queue-based for other tasks
  bool set_max_charge_power_w_async(uint16_t max_power_w); // Queue-based for other tasks
  bool enable_charging_async(bool enable);                 // Queue-based for other tasks
  bool enable_dcdc_async(bool enable);                     // Queue-based for other tasks

  // Direct methods (only for PCS task internal use)
  void set_target_voltage_mv(uint16_t voltage_mv);
  void set_charge_power_w(uint16_t power_w);
  void set_max_charge_power_w(uint16_t max_power_w);
  void enable_charging(bool enable);
  void enable_dcdc(bool enable);

  // State accessors
  PCSState get_state() const { return current_state; }
  PCSMode get_mode() const { return current_mode; }
  uint16_t get_target_voltage_mv() const { return target_voltage_mv; }
  uint16_t get_charge_power_w() const { return charge_power_w; }
  uint16_t get_max_charge_power_w() const { return max_charge_power_w; }
  bool is_enabled() const { return pcs_enabled; }
  bool is_charging() const { return charge_enabled; }
  bool is_dcdc_active() const { return dcdc_enabled; }

  // Task functions for FreeRTOS
  void task_loop();
  static void task_wrapper(void *pvParameters);
};
