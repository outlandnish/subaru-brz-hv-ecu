#include "chademo.h"

CHAdeMOController::CHAdeMOController() {
  can_bus = nullptr;
  current_state = CHADEMO_IDLE;
  last_tx_time = 0;
  state_entry_time = 0;
  session_active = false;
  charger_connected = false;

  // Initialize EVSE status
  evse_status.voltage_v = 0;
  evse_status.current_a = 0;
  evse_status.opmode = 0;
  evse_status.last_rx_time = 0;

  // Initialize battery request
  battery_request.battery_voltage_v = 0;
  battery_request.target_voltage_v = 0;
  battery_request.charge_current_a = 0;
  battery_request.soc = 0;
  battery_request.enable = false;

  // Default EVSE capabilities (can be updated)
  evse_max_voltage_v = 500;  // 500V max
  evse_max_current_a = 125;  // 125A max
}

void CHAdeMOController::begin(CANBus *bus) {
  can_bus = bus;
  Serial.println("CHAdeMO: Controller initialized");
}

void CHAdeMOController::process_can_message(CAN_FRAME *frame) {
  if (!can_bus) return;

  switch (frame->id) {
    case CHADEMO_EVSE_STATUS_ID: {
      // 0x102: EVSE voltage and current feedback
      // Byte 0-1: Voltage (big-endian, 0.1V resolution)
      // Byte 2-3: Current (big-endian, 0.1A resolution)
      // Byte 4: Operation mode

      evse_status.voltage_v = ((uint16_t)frame->data.uint8[0] << 8) | frame->data.uint8[1];
      evse_status.current_a = ((uint16_t)frame->data.uint8[2] << 8) | frame->data.uint8[3];
      evse_status.opmode = frame->data.uint8[4];
      evse_status.last_rx_time = millis();

      // Voltage and current are in 0.1 unit resolution, convert to whole units
      evse_status.voltage_v = evse_status.voltage_v / 10;
      evse_status.current_a = evse_status.current_a / 10;

      // Check if charger is in Current Demand mode (opmode = 13)
      if (evse_status.opmode == 13) {
        charger_connected = true;
      }

      Serial.printf("CHAdeMO RX 0x102: V=%dV, I=%dA, mode=%d\r\n",
                    evse_status.voltage_v, evse_status.current_a, evse_status.opmode);
      break;
    }

    case CHADEMO_EVSE_LIMITS_ID: {
      // 0x108: EVSE maximum voltage and current capability
      // Byte 0-1: Max Voltage (big-endian, 0.1V resolution)
      // Byte 2-3: Max Current (big-endian, 0.1A resolution)

      uint16_t max_voltage_dv = ((uint16_t)frame->data.uint8[0] << 8) | frame->data.uint8[1];
      uint16_t max_current_da = ((uint16_t)frame->data.uint8[2] << 8) | frame->data.uint8[3];

      // Convert from 0.1 unit resolution to whole units
      evse_max_voltage_v = max_voltage_dv / 10;
      evse_max_current_a = max_current_da / 10;

      Serial.printf("CHAdeMO RX 0x108: MaxV=%dV, MaxI=%dA\r\n",
                    evse_max_voltage_v, evse_max_current_a);
      break;
    }

    default:
      // Unknown message, ignore
      break;
  }
}

void CHAdeMOController::send_version_message() {
  if (!can_bus) return;

  uint8_t data[8] = {0xF0, 0xCC, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
  can_bus->sendMessage(CHADEMO_VERSION_ID, data, 8);
}

void CHAdeMOController::send_battery_request() {
  if (!can_bus) return;

  // 0x109: Battery status and charge request
  uint8_t data[8];
  data[0] = (battery_request.battery_voltage_v >> 8) & 0xFF;
  data[1] = battery_request.battery_voltage_v & 0xFF;
  data[2] = (battery_request.target_voltage_v >> 8) & 0xFF;
  data[3] = battery_request.target_voltage_v & 0xFF;
  data[4] = (battery_request.charge_current_a >> 8) & 0xFF;
  data[5] = battery_request.charge_current_a & 0xFF;
  data[6] = battery_request.soc;
  data[7] = battery_request.enable ? 0x01 : 0x00;

  can_bus->sendMessage(CHADEMO_BATTERY_STATUS_ID, data, 8);

  Serial.printf("CHAdeMO TX 0x109: BatV=%d.%dV, TargV=%d.%dV, I=%d.%dA, SOC=%d%%, En=%d\r\n",
                battery_request.battery_voltage_v / 10, battery_request.battery_voltage_v % 10,
                battery_request.target_voltage_v / 10, battery_request.target_voltage_v % 10,
                battery_request.charge_current_a / 10, battery_request.charge_current_a % 10,
                battery_request.soc,
                battery_request.enable);
}

void CHAdeMOController::check_timeout() {
  uint32_t time_since_rx = get_time_since_last_rx();

  if (time_since_rx > CHADEMO_RX_TIMEOUT_MS && charger_connected) {
    Serial.printf("CHAdeMO: Timeout detected (%lu ms since last RX)\r\n", time_since_rx);
    current_state = CHADEMO_TIMEOUT;
    session_active = false;
    charger_connected = false;
  }
}

void CHAdeMOController::update() {
  if (!can_bus) return;

  uint32_t current_time = millis();

  // Check for communication timeout
  if (session_active) {
    check_timeout();
  }

  // Send periodic messages (must be < 2s interval)
  if (current_time - last_tx_time >= CHADEMO_TX_INTERVAL_MS) {
    // Send version message (0x100)
    send_version_message();

    // Send battery request (0x109) - CRITICAL MESSAGE
    // Note: We receive EVSE limits on 0x108, we don't send them
    if (session_active) {
      send_battery_request();
    }

    last_tx_time = current_time;
  }

  // State machine
  switch (current_state) {
    case CHADEMO_IDLE:
      // Waiting for charge request
      break;

    case CHADEMO_CONNECTED:
      // Charger connected, waiting for enable
      if (battery_request.enable && charger_connected) {
        current_state = CHADEMO_PRECHARGE;
        state_entry_time = current_time;
        Serial.println("CHAdeMO: Starting precharge");
      }
      break;

    case CHADEMO_PRECHARGE:
      // Wait for voltage to match (simplified - could add voltage check)
      if (current_time - state_entry_time > 5000) {  // 5 second precharge
        current_state = CHADEMO_CHARGING;
        state_entry_time = current_time;
        Serial.println("CHAdeMO: Precharge complete, starting charge");
      }
      break;

    case CHADEMO_CHARGING:
      // Active charging - monitor for completion or stop request
      if (!battery_request.enable) {
        current_state = CHADEMO_ENDING;
        state_entry_time = current_time;
        Serial.println("CHAdeMO: Charge ending");
      }
      break;

    case CHADEMO_ENDING:
      // Graceful shutdown
      if (current_time - state_entry_time > 2000) {  // 2 second grace period
        current_state = CHADEMO_IDLE;
        session_active = false;
        Serial.println("CHAdeMO: Charge session ended");
      }
      break;

    case CHADEMO_ERROR:
    case CHADEMO_TIMEOUT:
      // Error state - require manual reset
      session_active = false;
      break;
  }
}

void CHAdeMOController::start_charging(uint16_t target_voltage_v, uint16_t max_current_a) {
  Serial.printf("CHAdeMO: Starting charge session (target %dV, max %dA)\r\n",
                target_voltage_v, max_current_a);

  battery_request.target_voltage_v = target_voltage_v * 10;  // Convert to 0.1V resolution
  battery_request.charge_current_a = max_current_a * 10;     // Convert to 0.1A resolution
  battery_request.enable = true;

  session_active = true;
  current_state = CHADEMO_CONNECTED;
  state_entry_time = millis();
}

void CHAdeMOController::stop_charging() {
  Serial.println("CHAdeMO: Stopping charge session");

  battery_request.enable = false;
  battery_request.charge_current_a = 0;

  if (current_state == CHADEMO_CHARGING) {
    current_state = CHADEMO_ENDING;
    state_entry_time = millis();
  } else {
    current_state = CHADEMO_IDLE;
    session_active = false;
  }
}

void CHAdeMOController::update_battery_status(uint16_t voltage_v, uint8_t soc, uint16_t current_a) {
  battery_request.battery_voltage_v = voltage_v * 10;    // Convert to 0.1V resolution
  battery_request.soc = soc;
  battery_request.charge_current_a = current_a * 10;     // Convert to 0.1A resolution
}

uint32_t CHAdeMOController::get_time_since_last_rx() const {
  if (evse_status.last_rx_time == 0) {
    return 0;
  }
  return millis() - evse_status.last_rx_time;
}
