#include "ivt_shunt.h"

#define Serial SerialUSB

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
    debug_enabled(false),
    first_frame(true),
    previous_as(0),
    previous_wh(0) {
}

IVTShunt::~IVTShunt() {
  if (can) {
    can->detachObj(this);
  }
}

void IVTShunt::begin(CANBus *can_bus) {
  can = can_bus;

  if (!can) {
    Serial.println("IVT: ERROR - CAN bus pointer is null!");
    return;
  }

  // Attach this listener to the CAN bus
  can->attachObj(this);

  // Set filter to receive IVT messages (0x520-0x528)
  can->watchFor(0x520, 0x7F0);  // Match 0x520-0x52F

  Serial.println("IVT: Initialized");

  // Initialize the shunt to current measurement mode
  init_current_mode();
}

void IVTShunt::gotFrame(CAN_FRAME *frame, int mailbox) {
  // Update frame count and timestamp
  frame_count++;
  last_message_time = millis();

  // Dispatch to appropriate handler based on CAN ID
  switch (frame->id) {
    case 0x511:
      // Response message - not currently handled
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

  if (debug_enabled) {
    print_frame(frame);
  }
}

void IVTShunt::handle_0x521_current(CAN_FRAME *frame) {
  // Current in milliamps (32-bit signed)
  int32_t milliamps = (int32_t)((frame->data.uint8[5] << 24) |
                                 (frame->data.uint8[4] << 16) |
                                 (frame->data.uint8[3] << 8) |
                                 (frame->data.uint8[2]));

  current_amps = milliamps / 1000.0f;
}

void IVTShunt::handle_0x522_voltage(CAN_FRAME *frame) {
  // Voltage in millivolts (32-bit signed)
  int32_t millivolts = (int32_t)((frame->data.uint8[5] << 24) |
                                  (frame->data.uint8[4] << 16) |
                                  (frame->data.uint8[3] << 8) |
                                  (frame->data.uint8[2]));

  voltage = millivolts / 1000.0f;
}

void IVTShunt::handle_0x523_voltage2(CAN_FRAME *frame) {
  // Voltage 2 in millivolts (32-bit signed)
  int32_t millivolts = (int32_t)((frame->data.uint8[5] << 24) |
                                  (frame->data.uint8[4] << 16) |
                                  (frame->data.uint8[3] << 8) |
                                  (frame->data.uint8[2]));

  voltage2 = millivolts / 1000.0f;
}

void IVTShunt::handle_0x524_voltage3(CAN_FRAME *frame) {
  // Voltage 3 in millivolts (32-bit signed)
  int32_t millivolts = (int32_t)((frame->data.uint8[5] << 24) |
                                  (frame->data.uint8[4] << 16) |
                                  (frame->data.uint8[3] << 8) |
                                  (frame->data.uint8[2]));

  voltage3 = millivolts / 1000.0f;
}

void IVTShunt::handle_0x525_temperature(CAN_FRAME *frame) {
  // Temperature in deci-degrees C (32-bit signed)
  int32_t deci_degrees = (int32_t)((frame->data.uint8[5] << 24) |
                                    (frame->data.uint8[4] << 16) |
                                    (frame->data.uint8[3] << 8) |
                                    (frame->data.uint8[2]));

  temperature_c = deci_degrees / 10.0f;
}

void IVTShunt::handle_0x526_power(CAN_FRAME *frame) {
  // Power in watts (32-bit signed)
  int32_t watts = (int32_t)((frame->data.uint8[5] << 24) |
                             (frame->data.uint8[4] << 16) |
                             (frame->data.uint8[3] << 8) |
                             (frame->data.uint8[2]));

  power_kw = watts / 1000.0f;
}

void IVTShunt::handle_0x527_amphours(CAN_FRAME *frame) {
  // Ampere-seconds (32-bit signed)
  int32_t as = (int32_t)((frame->data.uint8[5] << 24) |
                          (frame->data.uint8[4] << 16) |
                          (frame->data.uint8[3] << 8) |
                          (frame->data.uint8[2]));

  // Calculate delta and accumulate (convert As to Ah)
  if (!first_frame) {
    amp_hours += (as - previous_as) / 3600.0;
  }
  previous_as = as;
  first_frame = false;
}

void IVTShunt::handle_0x528_kwh(CAN_FRAME *frame) {
  // Watt-hours (32-bit signed)
  int32_t wh = (int32_t)((frame->data.uint8[5] << 24) |
                          (frame->data.uint8[4] << 16) |
                          (frame->data.uint8[3] << 8) |
                          (frame->data.uint8[2]));

  // Calculate delta and accumulate (convert Wh to kWh)
  if (previous_wh != 0) {
    kwh += (wh - previous_wh) / 1000.0;
  }
  previous_wh = wh;
}

void IVTShunt::start() {
  Serial.println("IVT: Sending START command");
  send_command(0x34, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
}

void IVTShunt::stop() {
  Serial.println("IVT: Sending STOP command");
  send_command(0x34, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00);
}

void IVTShunt::restart() {
  Serial.println("IVT: Sending RESTART command (resets Ah/kWh)");
  send_command(0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);

  // Reset local accumulators
  amp_hours = 0.0;
  kwh = 0.0;
  previous_as = 0;
  previous_wh = 0;
  first_frame = true;
}

void IVTShunt::setDefaults() {
  Serial.println("IVT: Sending DEFAULT command");
  send_command(0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
}

void IVTShunt::send_store() {
  send_command(0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
}

void IVTShunt::init_current_mode() {
  Serial.println("IVT: Initializing current measurement mode");

  stop();
  delay(500);

  // Configure current mode
  send_command(0x21, 0x42, 0x01, 0x61, 0x00, 0x00, 0x00, 0x00);
  delay(500);

  send_store();
  delay(500);

  start();
  delay(500);

  Serial.println("IVT: Initialization complete");
}

void IVTShunt::send_command(uint8_t cmd, uint8_t b1, uint8_t b2, uint8_t b3,
                             uint8_t b4, uint8_t b5, uint8_t b6, uint8_t b7) {
  if (!can) return;

  CAN_FRAME frame;
  frame.id = 0x411;
  frame.extended = 0;
  frame.rtr = 1;
  frame.length = 8;
  frame.data.uint8[0] = cmd;
  frame.data.uint8[1] = b1;
  frame.data.uint8[2] = b2;
  frame.data.uint8[3] = b3;
  frame.data.uint8[4] = b4;
  frame.data.uint8[5] = b5;
  frame.data.uint8[6] = b6;
  frame.data.uint8[7] = b7;

  if (!can->sendFrame(frame)) {
    Serial.println("IVT: ERROR - Failed to send command");
  }

  if (debug_enabled) {
    Serial.printf("IVT: TX 0x%03X [%d] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                  frame.id, frame.length,
                  frame.data.uint8[0], frame.data.uint8[1], frame.data.uint8[2], frame.data.uint8[3],
                  frame.data.uint8[4], frame.data.uint8[5], frame.data.uint8[6], frame.data.uint8[7]);
  }
}

void IVTShunt::print_frame(CAN_FRAME *frame) {
  Serial.printf("IVT: RX 0x%03X [%d] %02X %02X %02X %02X %02X %02X %02X %02X | ",
                frame->id, frame->length,
                frame->data.uint8[0], frame->data.uint8[1], frame->data.uint8[2], frame->data.uint8[3],
                frame->data.uint8[4], frame->data.uint8[5], frame->data.uint8[6], frame->data.uint8[7]);

  switch (frame->id) {
    case 0x521:
      Serial.printf("Current: %.2f A\r\n", current_amps);
      break;
    case 0x522:
      Serial.printf("Voltage: %.2f V\r\n", voltage);
      break;
    case 0x523:
      Serial.printf("Voltage2: %.2f V\r\n", voltage2);
      break;
    case 0x524:
      Serial.printf("Voltage3: %.2f V\r\n", voltage3);
      break;
    case 0x525:
      Serial.printf("Temperature: %.1f C\r\n", temperature_c);
      break;
    case 0x526:
      Serial.printf("Power: %.2f kW\r\n", power_kw);
      break;
    case 0x527:
      Serial.printf("Amp-Hours: %.3f Ah\r\n", amp_hours);
      break;
    case 0x528:
      Serial.printf("Energy: %.3f kWh\r\n", kwh);
      break;
    default:
      Serial.println();
      break;
  }
}

bool IVTShunt::is_alive() const {
  // Consider alive if we've received a message in the last 2 seconds
  return (millis() - last_message_time) < 2000;
}
