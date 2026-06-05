#include "ivt_shunt.h"
#include "debug_serial.h"
#include "params.h"
#include "param_save.h"

IVTShunt::IVTShunt()
  : can(nullptr),
    current_amps(0.0f),
    voltage(0.0f),
    voltage2(0.0f),
    voltage3(0.0f),
    temperature_c(0.0f),
    power_kw(0.0f),
    amp_hours(0.0),
    kwh(0.0),
    frame_count(0),
    last_message_time(0),
    debug_enabled(true),
    configuring(false),
    first_frame(true),
    previous_as(0),
    previous_wh(0),
    overcurrent_flag(false),
    channel_error(false),
    any_measurement_error(false),
    system_error(false),
    counter_error(false) {
  memset(last_msg_counter, 0xFF, sizeof(last_msg_counter));
}

IVTShunt::~IVTShunt() {
}

void IVTShunt::begin(CANBus *can_bus) {
  can = can_bus;

  if (!can) return;
}

bool IVTShunt::configure_if_needed() {
  if (Param::GetInt(Param::ivtConfigured) != 0) return false;
  configure();
  Param::SetInt(Param::ivtConfigured, 1);
  parm_save();
  return true;
}

void IVTShunt::configure() {
  const uint8_t stop[]   = {0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t cfg_u1[] = {0x21, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t cfg_u2[] = {0x22, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t store[]  = {0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  const uint8_t start[]  = {0x34, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};  // byte 2 = 0x01: auto-start on next power-up

  configuring = true;
  send_command(stop);
  delay(10);
  send_command(cfg_u1);
  delay(10);
  send_command(cfg_u2);
  delay(10);
  send_command(store);
  delay(1000);
  send_command(start);
  // Reset counter tracking so pre-configure frames don't trigger jump warnings
  memset(last_msg_counter, 0xFF, sizeof(last_msg_counter));
  configuring = false;
}

void IVTShunt::gotFrame(CAN_FRAME *frame, int mailbox) {
  // Update frame count and timestamp
  frame_count++;
  last_message_time = millis();

  // Dispatch to appropriate handler based on CAN ID
  switch (frame->id) {
    case 0x511:
      handle_0x511_response(frame);
      break;

    case 0x521:
      handle_0x521_current(frame);
      break;

    case 0x522:
      handle_0x522_voltage(frame);
      break;

    case 0x523:
      handle_0x523_voltage2(frame);
      break;

    case 0x524:
      handle_0x524_voltage3(frame);
      break;

    case 0x525:
      handle_0x525_temperature(frame);
      break;

    case 0x526:
      handle_0x526_power(frame);
      break;

    case 0x527:
      handle_0x527_amphours(frame);
      break;

    case 0x528:
      handle_0x528_kwh(frame);
      break;

    default:
      // Unknown message ID
      break;
  }

}

void IVTShunt::handle_0x511_response(CAN_FRAME *frame) {
  (void)frame;
}

void IVTShunt::handle_0x521_current(CAN_FRAME *frame) {
  // Current in milliamps (32-bit signed, big-endian)
  // Byte 0: MuxID, Byte 1: counter/status, Bytes 2-5: value (big-endian)
  if (!validate_muxid(frame->data.uint8[0], 0x00, "Current")) return;
  parse_error_status(frame->data.uint8[1], frame->id);

  int32_t milliamps = (int32_t)((frame->data.uint8[2] << 24) |
                                 (frame->data.uint8[3] << 16) |
                                 (frame->data.uint8[4] << 8) |
                                 (frame->data.uint8[5]));

  current_amps = milliamps / 1000.0f;
}

void IVTShunt::handle_0x522_voltage(CAN_FRAME *frame) {
  // Voltage in millivolts (32-bit signed, big-endian)
  // Byte 0: MuxID, Byte 1: counter/status, Bytes 2-5: value (big-endian)
  if (!validate_muxid(frame->data.uint8[0], 0x01, "Voltage1")) return;
  parse_error_status(frame->data.uint8[1], frame->id);

  int32_t millivolts = (int32_t)((frame->data.uint8[2] << 24) |
                                  (frame->data.uint8[3] << 16) |
                                  (frame->data.uint8[4] << 8) |
                                  (frame->data.uint8[5]));

  voltage = millivolts / 1000.0f;
}

void IVTShunt::handle_0x523_voltage2(CAN_FRAME *frame) {
  // Voltage 2 in millivolts (32-bit signed, big-endian)
  // Byte 0: MuxID, Byte 1: counter/status, Bytes 2-5: value (big-endian)
  if (!validate_muxid(frame->data.uint8[0], 0x02, "Voltage2")) return;
  parse_error_status(frame->data.uint8[1], frame->id);

  int32_t millivolts = (int32_t)((frame->data.uint8[2] << 24) |
                                  (frame->data.uint8[3] << 16) |
                                  (frame->data.uint8[4] << 8) |
                                  (frame->data.uint8[5]));

  voltage2 = millivolts / 1000.0f;
}

void IVTShunt::handle_0x524_voltage3(CAN_FRAME *frame) {
  // Voltage 3 in millivolts (32-bit signed, big-endian)
  // Byte 0: MuxID, Byte 1: counter/status, Bytes 2-5: value (big-endian)
  if (!validate_muxid(frame->data.uint8[0], 0x03, "Voltage3")) return;
  parse_error_status(frame->data.uint8[1], frame->id);

  int32_t millivolts = (int32_t)((frame->data.uint8[2] << 24) |
                                  (frame->data.uint8[3] << 16) |
                                  (frame->data.uint8[4] << 8) |
                                  (frame->data.uint8[5]));

  voltage3 = millivolts / 1000.0f;
}

void IVTShunt::handle_0x525_temperature(CAN_FRAME *frame) {
  // Temperature in deci-degrees C (32-bit signed, big-endian)
  // Byte 0: MuxID, Byte 1: counter/status, Bytes 2-5: value (big-endian)
  if (!validate_muxid(frame->data.uint8[0], 0x04, "Temperature")) return;
  parse_error_status(frame->data.uint8[1], frame->id);

  int32_t deci_degrees = (int32_t)((frame->data.uint8[2] << 24) |
                                    (frame->data.uint8[3] << 16) |
                                    (frame->data.uint8[4] << 8) |
                                    (frame->data.uint8[5]));

  temperature_c = deci_degrees / 10.0f;
}

void IVTShunt::handle_0x526_power(CAN_FRAME *frame) {
  // Power in watts (32-bit signed, big-endian)
  // Byte 0: MuxID, Byte 1: counter/status, Bytes 2-5: value (big-endian)
  if (!validate_muxid(frame->data.uint8[0], 0x05, "Power")) return;
  parse_error_status(frame->data.uint8[1], frame->id);

  int32_t watts = (int32_t)((frame->data.uint8[2] << 24) |
                             (frame->data.uint8[3] << 16) |
                             (frame->data.uint8[4] << 8) |
                             (frame->data.uint8[5]));

  power_kw = watts / 1000.0f;
}

void IVTShunt::handle_0x527_amphours(CAN_FRAME *frame) {
  // Ampere-seconds (32-bit signed, big-endian)
  // Byte 0: MuxID, Byte 1: counter/status, Bytes 2-5: value (big-endian)
  if (!validate_muxid(frame->data.uint8[0], 0x06, "AmpHours")) return;
  parse_error_status(frame->data.uint8[1], frame->id);

  int32_t as = (int32_t)((frame->data.uint8[2] << 24) |
                          (frame->data.uint8[3] << 16) |
                          (frame->data.uint8[4] << 8) |
                          (frame->data.uint8[5]));

  // Calculate delta and accumulate (convert As to Ah)
  if (!first_frame) {
    amp_hours += (as - previous_as) / 3600.0;
  }
  previous_as = as;
  first_frame = false;
}

void IVTShunt::handle_0x528_kwh(CAN_FRAME *frame) {
  // Watt-hours (32-bit signed, big-endian)
  // Byte 0: MuxID, Byte 1: counter/status, Bytes 2-5: value (big-endian)
  if (!validate_muxid(frame->data.uint8[0], 0x07, "kWh")) return;
  parse_error_status(frame->data.uint8[1], frame->id);

  int32_t wh = (int32_t)((frame->data.uint8[2] << 24) |
                          (frame->data.uint8[3] << 16) |
                          (frame->data.uint8[4] << 8) |
                          (frame->data.uint8[5]));

  // Calculate delta and accumulate (convert Wh to kWh)
  if (!first_frame) {
    kwh += (wh - previous_wh) / 1000.0;
  }
  previous_wh = wh;
}

void IVTShunt::start() {
  const uint8_t cmd[] = {0x34, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};  // byte 2 = 0x01: auto-start on next power-up
  send_command(cmd);
}

void IVTShunt::stop() {
  const uint8_t cmd[] = {0x34, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_command(cmd);
}

void IVTShunt::restart() {
  const uint8_t cmd[] = {0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_command(cmd);

  // Reset local accumulators
  amp_hours = 0.0;
  kwh = 0.0;
  previous_as = 0;
  previous_wh = 0;
  first_frame = true;
}

void IVTShunt::set_defaults() {
  const uint8_t cmd[] = {0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_command(cmd);
}

void IVTShunt::send_store() {
  const uint8_t cmd[] = {0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  send_command(cmd);
}


void IVTShunt::send_command(const uint8_t data[8]) {
  if (!can) return;
  CAN_FRAME frame;
  frame.id = 0x411;
  frame.extended = 0;
  frame.rtr = 0;  // Data frame, not remote frame
  frame.length = 8;

  // Copy all 8 bytes from the input array
  for (int i = 0; i < 8; i++) {
    frame.data.uint8[i] = data[i];
  }

  can->sendFrame(frame);
}


bool IVTShunt::is_alive() const {
  // Consider alive if we've received a message in the last 5 seconds
  return last_message_time > 0 && (millis() - last_message_time) < 5000;
}

void IVTShunt::parse_error_status(uint8_t status_byte, uint32_t msg_id) {
  // Byte 1 format:
  // Lower nibble (bits 0-3): Message counter (0-15), increments per message ID
  // Upper nibble (bits 4-7): Error flags
  //   bit 4: Overcurrent (OCS)
  //   bit 5: Channel error
  //   bit 6: Any measurement error
  //   bit 7: System error

  uint8_t idx = (uint8_t)(msg_id - 0x521);
  uint8_t counter = status_byte & 0x0F;
  if (last_msg_counter[idx] != 0xFF) {
    uint8_t expected = (last_msg_counter[idx] + 1) % 16;
    if (counter != expected) {
      counter_error = true;
      if (debug_enabled && !configuring) {
        debug_printf("IVT: WARNING - Counter jump on 0x%03lX (expected %d, got %d)\r\n",
                      msg_id, expected, counter);
      }
    } else {
      counter_error = false;
    }
  }
  last_msg_counter[idx] = counter;

  // Extract error flags from upper nibble
  uint8_t error_flags = (status_byte >> 4) & 0x0F;
  
  system_error = (error_flags & 0x08) != 0;
  any_measurement_error = (error_flags & 0x04) != 0;
  channel_error = (error_flags & 0x02) != 0;
  overcurrent_flag = (error_flags & 0x01) != 0;

  // Log critical errors (suppress during configure sequence)
  if (debug_enabled && !configuring) {
    if (system_error) {
      debug_println("IVT: ERROR - System error! Sensor functionality not ensured!");
    }
    if (any_measurement_error) {
      debug_println("IVT: ERROR - Measurement error detected!");
    }
    if (channel_error) {
      debug_println("IVT: WARNING - Channel error!");
    }
    if (overcurrent_flag) {
      debug_println("IVT: WARNING - Overcurrent condition!");
    }
  }
}

bool IVTShunt::validate_muxid(uint8_t muxid, uint8_t expected, const char* msg_name) {
  if (muxid != expected) {
    if (debug_enabled) {
      debug_printf("IVT: ERROR - MuxID mismatch for %s (expected 0x%02X, got 0x%02X)\r\n",
                    msg_name, expected, muxid);
    }
    return false;
  }
  return true;
}
