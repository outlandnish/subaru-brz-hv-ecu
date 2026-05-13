#pragma once
#include <Arduino.h>
#include "can.h"
#include "can_common.h"

/**
 * ISA/Isabellenhütte IVT-S Modular Current/Voltage Shunt Driver
 *
 * Communicates with IVT-S 1000A current shunt via CAN bus.
 * Measures:
 * - Current (±1000A typical)
 * - Up to 3 voltages
 * - Temperature
 * - Calculates power, energy (Ah, kWh)
 *
 * Based on reference implementation by Jack Rickard (EVtv)
 * Adapted for STM32 with can_common library
 */

class IVTShunt : public CANListener {
public:
  IVTShunt();
  ~IVTShunt();

  void begin(CANBus *can_bus);

  // Control methods
  void start();
  void stop();
  void restart();       // Resets accumulated Ah and kWh
  void set_defaults();   // Reset to factory defaults

  // State accessors
  float get_current() const { return current_amps; }
  float get_voltage() const { return voltage; }          // Main voltage (V1)
  float get_voltage2() const { return voltage2; }        // Voltage 2
  float get_voltage3() const { return voltage3; }        // Voltage 3
  float get_temperature() const { return temperature_c; }
  float get_power() const { return power_kw; }

  double get_amp_hours() const { return amp_hours; }
  double get_kilowatt_hours() const { return kwh; }

  uint32_t get_frame_count() const { return frame_count; }
  uint32_t get_last_message_time() const { return last_message_time; }
  bool is_alive() const;  // Returns true if messages received recently

  // Debug control
  void set_debug(bool enable) { debug_enabled = enable; }
  bool get_debug() const { return debug_enabled; }

  // Public method to process CAN frames (for queue-based processing)
  void process_can_frame(CAN_FRAME *frame) { gotFrame(frame, 0); }

protected:
  // CANListener interface implementation
  void gotFrame(CAN_FRAME *frame, int mailbox) override;

private:
  CANBus *can;

  // Measurement data
  float current_amps;
  float voltage;
  float voltage2;
  float voltage3;
  float temperature_c;
  float power_kw;

  // Accumulated energy
  double amp_hours;
  double kwh;

  // Internal state
  uint32_t frame_count;
  uint32_t last_message_time;
  bool debug_enabled;
  bool first_frame;

  // Previous values for accumulation
  long previous_as;  // Previous ampere-seconds
  long previous_wh;  // Previous watt-hours

  // Error tracking
  uint8_t message_counter;
  uint8_t last_message_counter;
  bool counter_error;
  bool system_error;
  bool any_measurement_error;
  bool precision_error;
  bool overcurrent_flag;

  // Message handlers
  void handle_0x521_current(CAN_FRAME *frame);
  void handle_0x522_voltage(CAN_FRAME *frame);
  void handle_0x523_voltage2(CAN_FRAME *frame);
  void handle_0x524_voltage3(CAN_FRAME *frame);
  void handle_0x525_temperature(CAN_FRAME *frame);
  void handle_0x526_power(CAN_FRAME *frame);
  void handle_0x527_amphours(CAN_FRAME *frame);
  void handle_0x528_kwh(CAN_FRAME *frame);

  // Helper methods
  void send_command(const uint8_t data[8]);
  void send_store();
  void init_current_mode();
  void print_frame(CAN_FRAME *frame);
  void parse_error_status(uint8_t status_byte);
  bool validate_muxid(uint8_t muxid, uint8_t expected, const char* msg_name);
};
