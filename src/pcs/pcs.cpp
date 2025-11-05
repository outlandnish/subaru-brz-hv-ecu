/*
 * Tesla Model 3 PCS (Power Conversion System) Controller
 *
 * High-level controller that manages the PCS hardware pins and coordinates
 * with the low-level PCSCan communication layer.
 */

#include "pcs.h"

// Initialize static members
uint8_t PCSController::pcs_enable_pin = 0;
uint8_t PCSController::charge_enable_pin = 0;
uint8_t PCSController::dcdc_enable_pin = 0;

bool PCSController::pcs_pin_enabled = false;
bool PCSController::charge_pin_enabled = false;
bool PCSController::dcdc_pin_enabled = false;

PCSState PCSController::current_state = PCS_STATE_INIT;
PCSState PCSController::target_state = PCS_STATE_OFF;
uint8_t PCSController::precharge_timer = 0;
uint8_t PCSController::powerdown_timer = 0;
uint32_t PCSController::last_update_ms = 0;
uint32_t PCSController::last_100ms_update = 0;
QueueHandle_t PCSController::command_queue = nullptr;

void PCSController::begin(CANBus *ipc_can_bus, uint8_t pcs_en_pin, uint8_t charge_en_pin, uint8_t dcdc_en_pin) {
  // Store pin assignments
  pcs_enable_pin = pcs_en_pin;
  charge_enable_pin = charge_en_pin;
  dcdc_enable_pin = dcdc_en_pin;

  // Configure pins as outputs
  pinMode(pcs_enable_pin, OUTPUT);
  pinMode(charge_enable_pin, OUTPUT);
  pinMode(dcdc_enable_pin, OUTPUT);

  // Initialize to safe state (all disabled)
  disable_all();

  // Initialize low-level CAN communication
  PCSCan::begin(ipc_can_bus);

  // Create command queue (10 commands deep)
  command_queue = xQueueCreate(10, sizeof(PCSCommand));
  if (command_queue == nullptr) {
    Serial.println("PCS: Failed to create command queue");
    return;
  }

  current_state = PCS_STATE_OFF;
  Serial.println("PCS: Controller initialized with precharge state machine");
}

bool PCSController::start_task() {
  BaseType_t result = xTaskCreate(
    task_wrapper,
    "PCS",
    2048,  // Stack size
    NULL,
    2,     // Priority (medium)
    NULL
  );

  if (result != pdPASS) {
    Serial.println("PCS: Failed to create task!");
    return false;
  }

  Serial.println("PCS: Task started");
  return true;
}

void PCSController::update() {
  uint32_t now = millis();

  // Process any pending commands from the queue
  process_command_queue();

  // Process incoming CAN messages
  PCSCan::process_messages();

  // Run state machine at 100ms intervals (10Hz)
  if (now - last_100ms_update >= 100) {
    run_state_machine();
    last_100ms_update = now;
  }

  // Update physical control pins
  update_control_pins();

  // Send periodic CAN messages (stagger them to reduce bus load)
  if (now - last_update_ms >= 10) {
    send_periodic_messages();
    last_update_ms = now;
  }
}

void PCSController::process_command_queue() {
  PCSCommand cmd;

  // Process all pending commands (non-blocking)
  while (xQueueReceive(command_queue, &cmd, 0) == pdTRUE) {
    switch (cmd.type) {
      case PCS_CMD_SET_CHARGE_POWER:
        set_charge_power(cmd.value_u16);
        break;

      case PCS_CMD_SET_HV_VOLTAGE:
        set_hv_voltage(cmd.value_u16);
        break;

      case PCS_CMD_SET_DCDC_VOLTAGE:
        set_dcdc_voltage(cmd.value_float);
        break;

      case PCS_CMD_SET_AC_LIMIT:
        set_ac_current_limit((uint8_t)cmd.value_u16);
        break;

      case PCS_CMD_ENABLE_PCS:
        enable_pcs(cmd.value_bool);
        break;

      // State machine commands
      case PCS_CMD_START_CHARGING:
        start_charging();
        break;

      case PCS_CMD_START_DRIVE_MODE:
        start_drive_mode();
        break;

      case PCS_CMD_STOP:
        stop();
        break;

      case PCS_CMD_EMERGENCY_STOP:
        emergency_stop();
        break;
    }
  }
}

void PCSController::send_periodic_messages() {
  uint32_t now = millis();

  // Core messages needed for operation (10ms cycle)
  PCSCan::Msg22A();   // Main control (mode and voltage)
  PCSCan::Msg2B2(0);  // Power request (get from PCSCan later)
  PCSCan::Msg333();   // UI watchdog

  // Send other messages at reduced rate (100ms)
  if ((now / 100) % 10 == 0) {
    PCSCan::Msg3B2();   // BMS log
    PCSCan::Msg545();   // VCFront
    PCSCan::Msg3A1();   // DCDC setpoint
  }

  // Static configuration messages (send infrequently, every 5 seconds)
  if ((now / 1000) % 5 == 0) {
    PCSCan::Msg20A();
    PCSCan::Msg212();
    PCSCan::Msg21D();
    PCSCan::Msg232();
    PCSCan::Msg25D();
    PCSCan::Msg321();
  }
}

void PCSController::update_control_pins() {
  // Update all control pins (HIGH = enabled)
  digitalWrite(pcs_enable_pin, pcs_pin_enabled ? HIGH : LOW);

  // Note: Charge and DCDC pins may need inverted logic (active LOW)
  // Reference implementation uses Set() to enable, which could be active LOW
  // Check your hardware and invert if needed
  digitalWrite(charge_enable_pin, charge_pin_enabled ? HIGH : LOW);
  digitalWrite(dcdc_enable_pin, dcdc_pin_enabled ? HIGH : LOW);
}

// Direct Control Methods

void PCSController::set_charge_power(uint16_t power_w) {
  PCSCan::set_charge_power(power_w);
}

void PCSController::set_hv_voltage(uint16_t voltage_v) {
  PCSCan::set_hv_voltage(voltage_v);
}

void PCSController::set_dcdc_voltage(float voltage_v) {
  PCSCan::set_dcdc_voltage(voltage_v);
}

void PCSController::set_ac_current_limit(uint8_t limit_a) {
  PCSCan::set_ac_current_limit(limit_a);
}

void PCSController::set_evse_limit(uint8_t limit_a) {
  PCSCan::set_evse_limit(limit_a);
}

void PCSController::set_cable_limit(uint8_t limit) {
  PCSCan::set_cable_limit(limit);
}

void PCSController::enable_pcs(bool enable) {
  pcs_pin_enabled = enable;
  Serial.printf("PCS: Master enable %s\r\n", enable ? "ON" : "OFF");
}

// Async Control Methods (Queue-based)

bool PCSController::set_charge_power_async(uint16_t power_w) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_CHARGE_POWER;
  cmd.value_u16 = power_w;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_hv_voltage_async(uint16_t voltage_v) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_HV_VOLTAGE;
  cmd.value_u16 = voltage_v;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_dcdc_voltage_async(float voltage_v) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_DCDC_VOLTAGE;
  cmd.value_float = voltage_v;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::set_ac_current_limit_async(uint8_t limit_a) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_AC_LIMIT;
  cmd.value_u16 = limit_a;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::enable_pcs_async(bool enable) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_ENABLE_PCS;
  cmd.value_bool = enable;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

// FreeRTOS Task Functions

void PCSController::task_loop() {
  while (true) {
    // Call the main update function
    update();

    // Run at 100Hz (10ms cycle)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void PCSController::task_wrapper(void *pvParameters) {
  task_loop();
}

// State Machine Methods

void PCSController::disable_all() {
  pcs_pin_enabled = false;
  charge_pin_enabled = false;
  dcdc_pin_enabled = false;

  digitalWrite(pcs_enable_pin, LOW);
  digitalWrite(charge_enable_pin, LOW);
  digitalWrite(dcdc_enable_pin, LOW);

  PCSCan::set_mode(PCS_MODE_OFF);
  PCSCan::set_charge_enable(false);
}

void PCSController::start_charging() {
  if (current_state == PCS_STATE_OFF) {
    Serial.println("PCS: Starting charging sequence");
    target_state = PCS_STATE_CHARGING;
    current_state = PCS_STATE_WAITSTART;
  } else {
    Serial.printf("PCS: Cannot start charging from state %d\r\n", current_state);
  }
}

void PCSController::start_drive_mode() {
  if (current_state == PCS_STATE_OFF) {
    Serial.println("PCS: Starting drive mode sequence (DCDC only)");
    target_state = PCS_STATE_DRIVE;
    current_state = PCS_STATE_WAITSTART;
  } else {
    Serial.printf("PCS: Cannot start drive mode from state %d\r\n", current_state);
  }
}

void PCSController::stop() {
  Serial.println("PCS: Stopping (graceful shutdown)");
  current_state = PCS_STATE_STOP;
}

void PCSController::emergency_stop() {
  Serial.println("PCS: EMERGENCY STOP");
  disable_all();
  current_state = PCS_STATE_OFF;
}

// Async state machine control methods (thread-safe)

bool PCSController::start_charging_async() {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_START_CHARGING;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::start_drive_mode_async() {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_START_DRIVE_MODE;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::stop_async() {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_STOP;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool PCSController::emergency_stop_async() {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_EMERGENCY_STOP;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

void PCSController::run_state_machine() {
  // Get charger status from CAN
  uint8_t pcs_charge_status = PCSCan::get_charger_status().status;

  switch (current_state) {
    case PCS_STATE_INIT:
      // Initialization complete, go to OFF state
      current_state = PCS_STATE_OFF;
      break;

    case PCS_STATE_OFF:
      // Safe state - everything disabled
      disable_all();
      precharge_timer = PRECHARGE_TIME_TICKS;  // Reset precharge timer
      powerdown_timer = POWERDOWN_TIME_TICKS;  // Reset powerdown timer
      // Transition happens via start_charging() call
      break;

    case PCS_STATE_WAITSTART:
      // Optional delay state before precharge
      // For now, immediately transition to precharge
      // TODO: Add configurable delay if needed
      current_state = PCS_STATE_PRECHARGE;
      Serial.println("PCS: Entering precharge state");
      break;

    case PCS_STATE_PRECHARGE:
      // Internal PCS initialization (CAN messaging, internal startup)
      // Note: Actual HV precharge is controlled by BMS via contactors

      // Decrement initialization timer (runs at 100ms rate)
      if (precharge_timer > 0) {
        precharge_timer--;

        if (precharge_timer == 0) {
          // Initialization complete - enable PCS
          Serial.println("PCS: Internal initialization complete, enabling PCS");
          pcs_pin_enabled = true;
          current_state = PCS_STATE_ACTIVATE;
        }
      }
      break;

    case PCS_STATE_ACTIVATE:
      // Enable charge and/or DCDC based on target state
      if (target_state == PCS_STATE_CHARGING) {
        charge_pin_enabled = true;
        dcdc_pin_enabled = false;  // TODO: Could enable both for charge+DCDC mode
        PCSCan::set_mode(PCS_MODE_CHARGE_ONLY);
        PCSCan::set_charge_enable(true);
        Serial.println("PCS: Transitioning to charging state");
      } else if (target_state == PCS_STATE_DRIVE) {
        charge_pin_enabled = false;
        dcdc_pin_enabled = true;
        PCSCan::set_mode(PCS_MODE_DCDC_ONLY);
        PCSCan::set_charge_enable(false);
        Serial.println("PCS: Transitioning to drive mode");
      }

      current_state = target_state;
      break;

    case PCS_STATE_CHARGING:
      // Active charging state
      // Wait for PCS to enter "wait for AC" state (status == 3) before enabling EVSE
      // This is handled by external EVSE controller

      // TODO: Add fault detection and voltage monitoring
      // if (voltage_too_high || timeout || unplugged) {
      //   current_state = PCS_STATE_STOP;
      // }
      break;

    case PCS_STATE_DRIVE:
      // Drive mode - DCDC only, no charging
      charge_pin_enabled = false;
      dcdc_pin_enabled = true;
      pcs_pin_enabled = true;

      PCSCan::set_mode(PCS_MODE_DCDC_ONLY);
      PCSCan::set_charge_enable(false);
      break;

    case PCS_STATE_STOP:
      // Graceful shutdown - ramp down power before disabling
      // Set charge power to zero
      PCSCan::set_charge_power(0);

      if (powerdown_timer > 0) {
        powerdown_timer--;

        if (powerdown_timer == 0) {
          Serial.println("PCS: Powerdown complete, returning to OFF state");
          disable_all();
          current_state = PCS_STATE_OFF;
        }
      }
      break;

    case PCS_STATE_FAULT:
      // Fault state - disable everything
      disable_all();
      // Recovery logic could be added here
      break;

    default:
      // Unknown state - go to safe OFF state
      Serial.printf("PCS: Unknown state %d, going to OFF\r\n", current_state);
      current_state = PCS_STATE_OFF;
      break;
  }
}
