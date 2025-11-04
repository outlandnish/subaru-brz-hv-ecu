#include "pcs.h"
#define Serial SerialUSB

TeslaM3PCSController::TeslaM3PCSController(uint8_t enable_pin, uint8_t charge_pin, uint8_t dcdc_pin)
  : pcs_enable_pin(enable_pin),
    pcs_charge_pin(charge_pin),
    pcs_dcdc_pin(dcdc_pin),
    ipc_can(nullptr),
    m3_can(nullptr),
    current_state(PCS_STATE_INIT),
    current_mode(PCS_MODE_OFF),
    target_voltage_mv(0),
    charge_power_w(0),
    max_charge_power_w(10000),  // Default 10kW max
    pcs_enabled(false),
    charge_enabled(false),
    dcdc_enabled(false),
    command_queue(nullptr),
    last_0x22a_ms(0),
    last_0x2b2_ms(0),
    last_0x3b2_ms(0),
    last_0x545_ms(0),
    last_0x333_ms(0),
    msg_0x545_counter(0),
    msg_0x3b2_mux(false) {
}

void TeslaM3PCSController::begin(CANBus *ipc_can_bus, CANBus *m3_can_bus) {
  ipc_can = ipc_can_bus;
  m3_can = m3_can_bus;

  // Create command queue (10 commands deep)
  command_queue = xQueueCreate(10, sizeof(PCSCommand));
  if (command_queue == nullptr) {
    Serial.println("PCS: Failed to create command queue");
    return;
  }

  // Configure control pins
  pinMode(pcs_enable_pin, OUTPUT);
  pinMode(pcs_charge_pin, OUTPUT);
  pinMode(pcs_dcdc_pin, OUTPUT);

  // Initialize to safe state (all disabled)
  // Note: According to reference code, these pins use inverted logic for gate drives
  digitalWrite(pcs_enable_pin, LOW);   // PCS disabled
  digitalWrite(pcs_charge_pin, HIGH);  // Charger disabled (inverted logic)
  digitalWrite(pcs_dcdc_pin, HIGH);    // DCDC disabled (inverted logic)

  current_state = PCS_STATE_STANDBY;

  Serial.println("PCS: Initialized");
}

void TeslaM3PCSController::update() {
  uint32_t now = millis();

  // Process any pending commands from the queue
  process_command_queue();

  // Send periodic CAN messages on IPC CAN
  if (ipc_can != nullptr) {
    // 0x22A - Main control message (10ms period)
    if (now - last_0x22a_ms >= 10) {
      send_0x22a_control();
      last_0x22a_ms = now;
    }

    // 0x2B2 - Charge power request (20ms period)
    if (now - last_0x2b2_ms >= 20) {
      send_0x2b2_power_request();
      last_0x2b2_ms = now;
    }

    // 0x3B2 - BMS log (100ms period)
    if (now - last_0x3b2_ms >= 100) {
      send_0x3b2_bms_log();
      last_0x3b2_ms = now;
    }

    // 0x545 - VCFront (100ms period)
    if (now - last_0x545_ms >= 100) {
      send_0x545_vcfront();
      last_0x545_ms = now;
    }

    // 0x333 - UI charge request (100ms period)
    if (now - last_0x333_ms >= 100) {
      send_0x333_ui_request();
      last_0x333_ms = now;
    }
  }

  // Update control outputs
  update_control_outputs();
}

void TeslaM3PCSController::send_0x22a_control() {
  // This is the "heart of the beast" - main PCS control message
  // Transmitted on IPC CAN at 500kbps every 10ms

  uint8_t tx_data[8];

  // Bytes 0-1: Precharge request voltage (16-bit signed, scale 0.1V)
  int16_t voltage_dv = target_voltage_mv / 100;  // Convert mV to decivolts
  tx_data[0] = voltage_dv & 0xFF;
  tx_data[1] = (voltage_dv >> 8) & 0xFF;

  // Byte 2: Mode control
  tx_data[2] = current_mode;

  // Byte 3: HV link voltage encoding (391V nominal = 0x87)
  // This appears to be a fixed value in the reference code
  tx_data[3] = 0x87;

  // Bytes 4-7: Additional control parameters (reference shows mostly zeros)
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;

  ipc_can->sendMessage(0x22A, tx_data, 8);
}

void TeslaM3PCSController::send_0x2b2_power_request() {
  // Charge power request message
  // US variant uses 3-byte DLC

  uint8_t tx_data[3];

  // Bytes 0-1: Target charging power in watts (16-bit)
  tx_data[0] = charge_power_w & 0xFF;
  tx_data[1] = (charge_power_w >> 8) & 0xFF;

  // Byte 2: Charge active flag
  tx_data[2] = charge_enabled ? 0x02 : 0x00;

  ipc_can->sendMessage(0x2B2, tx_data, 3);
}

void TeslaM3PCSController::send_0x3b2_bms_log() {
  // BMS log message - alternates mux bit to keep BMS alive

  uint8_t tx_data[8];

  // Alternate between two patterns
  if (msg_0x3b2_mux) {
    // Pattern 1
    tx_data[0] = 0x5E;
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
  } else {
    // Pattern 2
    tx_data[0] = 0x5D;
    tx_data[1] = 0x00;
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;
  }

  msg_0x3b2_mux = !msg_0x3b2_mux;

  ipc_can->sendMessage(0x3B2, tx_data, 8);
}

void TeslaM3PCSController::send_0x545_vcfront() {
  // VCFront message with counter and CRC

  uint8_t tx_data[8];

  // Pattern from reference code
  tx_data[0] = 0x00;
  tx_data[1] = 0x00;
  tx_data[2] = 0x00;
  tx_data[3] = 0x00;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;

  // Byte 6: 4-bit counter (upper nibble)
  tx_data[6] = (msg_0x545_counter & 0x0F) << 4;

  // Byte 7: CRC checksum
  tx_data[7] = calculate_crc(tx_data, 7, 0x545);

  // Increment counter (0-15)
  msg_0x545_counter = (msg_0x545_counter + 1) & 0x0F;

  ipc_can->sendMessage(0x545, tx_data, 8);
}

void TeslaM3PCSController::send_0x333_ui_request() {
  // UI charge request - static 4-byte frame to kill UI watchdog

  uint8_t tx_data[4] = {0x00, 0x00, 0x00, 0x00};

  ipc_can->sendMessage(0x333, tx_data, 4);
}

void TeslaM3PCSController::process_command_queue() {
  PCSCommand cmd;

  // Process all pending commands (non-blocking)
  while (xQueueReceive(command_queue, &cmd, 0) == pdTRUE) {
    switch (cmd.type) {
      case PCS_CMD_SET_CHARGE_POWER:
        set_charge_power_w(cmd.value);
        break;

      case PCS_CMD_SET_TARGET_VOLTAGE:
        set_target_voltage_mv(cmd.value);
        break;

      case PCS_CMD_SET_MAX_POWER:
        set_max_charge_power_w(cmd.value);
        break;

      case PCS_CMD_ENABLE_CHARGING:
        enable_charging(cmd.value != 0);
        break;

      case PCS_CMD_ENABLE_DCDC:
        enable_dcdc(cmd.value != 0);
        break;
    }
  }
}

uint8_t TeslaM3PCSController::calculate_crc(uint8_t *data, uint8_t len, uint16_t can_id) {
  // XOR checksum calculation
  uint16_t checksum = 0;

  // Sum all data bytes
  for (uint8_t i = 0; i < len; i++) {
    checksum += data[i];
  }

  // Add CAN ID
  checksum += can_id & 0xFF;
  checksum += (can_id >> 8) & 0xFF;

  // Return lower 8 bits
  return checksum & 0xFF;
}

void TeslaM3PCSController::update_control_outputs() {
  // Update physical control pins based on state

  // PCS Enable (HIGH = enabled)
  digitalWrite(pcs_enable_pin, pcs_enabled ? HIGH : LOW);

  // Charger and DCDC use inverted logic (LOW = enabled)
  digitalWrite(pcs_charge_pin, charge_enabled ? LOW : HIGH);
  digitalWrite(pcs_dcdc_pin, dcdc_enabled ? LOW : HIGH);
}

void TeslaM3PCSController::set_mode(PCSMode mode) {
  current_mode = mode;

  // Update individual enable flags based on mode
  switch (mode) {
    case PCS_MODE_OFF:
      pcs_enabled = false;
      charge_enabled = false;
      dcdc_enabled = false;
      break;

    case PCS_MODE_DCDC_ONLY:
      pcs_enabled = true;
      charge_enabled = false;
      dcdc_enabled = true;
      break;

    case PCS_MODE_CHARGE_ONLY:
      pcs_enabled = true;
      charge_enabled = true;
      dcdc_enabled = false;
      break;

    case PCS_MODE_CHARGE_DCDC:
      pcs_enabled = true;
      charge_enabled = true;
      dcdc_enabled = true;
      break;
  }

  Serial.printf("PCS: Mode set to 0x%02X\r\n", mode);
}

void TeslaM3PCSController::set_target_voltage_mv(uint16_t voltage_mv) {
  target_voltage_mv = voltage_mv;
}

void TeslaM3PCSController::set_charge_power_w(uint16_t power_w) {
  // Limit to maximum allowed power
  if (power_w > max_charge_power_w) {
    power_w = max_charge_power_w;
  }

  charge_power_w = power_w;
}

void TeslaM3PCSController::set_max_charge_power_w(uint16_t max_power_w) {
  max_charge_power_w = max_power_w;
}

void TeslaM3PCSController::enable_charging(bool enable) {
  charge_enabled = enable;

  // Update mode based on DCDC state
  if (enable && dcdc_enabled) {
    current_mode = PCS_MODE_CHARGE_DCDC;
  } else if (enable) {
    current_mode = PCS_MODE_CHARGE_ONLY;
  } else if (dcdc_enabled) {
    current_mode = PCS_MODE_DCDC_ONLY;
  } else {
    current_mode = PCS_MODE_OFF;
  }

  pcs_enabled = (charge_enabled || dcdc_enabled);
}

void TeslaM3PCSController::enable_dcdc(bool enable) {
  dcdc_enabled = enable;

  // Update mode based on charge state
  if (charge_enabled && enable) {
    current_mode = PCS_MODE_CHARGE_DCDC;
  } else if (charge_enabled) {
    current_mode = PCS_MODE_CHARGE_ONLY;
  } else if (enable) {
    current_mode = PCS_MODE_DCDC_ONLY;
  } else {
    current_mode = PCS_MODE_OFF;
  }

  pcs_enabled = (charge_enabled || dcdc_enabled);
}

void TeslaM3PCSController::task_loop() {
  while (true) {
    update();
    vTaskDelay(pdMS_TO_TICKS(10));  // Run at 100Hz
  }
}

void TeslaM3PCSController::task_wrapper(void *pvParameters) {
  TeslaM3PCSController *pcs = static_cast<TeslaM3PCSController*>(pvParameters);
  pcs->task_loop();
}

// Async methods for calling from other tasks via command queue

bool TeslaM3PCSController::set_target_voltage_mv_async(uint16_t voltage_mv) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_TARGET_VOLTAGE;
  cmd.value = voltage_mv;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool TeslaM3PCSController::set_charge_power_w_async(uint16_t power_w) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_CHARGE_POWER;
  cmd.value = power_w;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool TeslaM3PCSController::set_max_charge_power_w_async(uint16_t max_power_w) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_SET_MAX_POWER;
  cmd.value = max_power_w;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool TeslaM3PCSController::enable_charging_async(bool enable) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_ENABLE_CHARGING;
  cmd.value = enable ? 1 : 0;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}

bool TeslaM3PCSController::enable_dcdc_async(bool enable) {
  if (command_queue == nullptr) return false;

  PCSCommand cmd;
  cmd.type = PCS_CMD_ENABLE_DCDC;
  cmd.value = enable ? 1 : 0;

  return xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE;
}