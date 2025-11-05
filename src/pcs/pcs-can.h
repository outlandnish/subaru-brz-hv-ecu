#pragma once

/*
 * Tesla Model 3 PCS CAN Communication Layer
 *
 * Low-level CAN message handling for the Tesla Model 3 onboard charger.
 * Based on reference implementation from:
 * https://github.com/damienmaguire/Tesla-Model-3-Charger
 */

#include <Arduino.h>
#include "can.h"

// PCS operational modes (for message 0x22A byte 2)
enum PCSMode : uint8_t {
  PCS_MODE_OFF = 0x00,           // Both charger and DCDC off
  PCS_MODE_CHARGE_ONLY = 0x05,   // Charge only, DCDC disabled (based on reference)
  PCS_MODE_DCDC_ONLY = 0x09,     // DCDC only, charger disabled
  PCS_MODE_CHARGE_DCDC = 0x0D    // Both charger and DCDC enabled
};

// Control parameters set by higher-level controller
struct ControlParams {
  uint16_t hv_voltage_v;          // HV bus voltage setpoint (V)
  uint16_t charge_power_w;        // Requested charge power (W)
  float dcdc_voltage_v;           // DCDC output voltage setpoint (V)
  uint8_t ac_current_limit_a;     // AC current limit (A)
  uint8_t evse_limit_a;           // EVSE current limit (A)
  uint8_t cable_limit;            // Cable limit code
};

// Charger status data (from message 0x204)
struct ChargerStatus {
  uint8_t hw_type;                // Hardware type (0=11kW, 1=7.7kW, 2=3.8kW)
  uint8_t status;                 // Charge status
  uint8_t grid_config;            // Grid configuration
  float power_available_kw;       // Available charge power (kW)
};

// DCDC status data (from message 0x224)
struct DCDCStatus {
  float current_a;                // DCDC output current (A)
  float power_w;                  // DCDC output power (W)
};

// AC line status (from message 0x264)
struct ACStatus {
  uint16_t current_limit_a;       // AC current limit from PCS (A)
  float power_kw;                 // AC input power (kW)
  uint16_t voltage_v;             // AC input voltage (V)
  float current_a;                // AC input current (A)
};

// Temperature readings (from message 0x2A4)
struct TemperatureData {
  int16_t local_c;                // Local temperature (°C)
  uint16_t ambient_raw;           // Ambient temperature (raw)
  uint16_t phase_a_raw;           // Phase A temperature (raw)
  uint16_t phase_b_raw;           // Phase B temperature (raw)
  uint16_t phase_c_raw;           // Phase C temperature (raw)
  uint16_t dcdc_raw;              // DCDC temperature (raw)
  float dcdc_b_c;                 // DCDC B temperature (°C)
};

// Voltage measurements (from message 0x2C4)
struct VoltageData {
  uint16_t hv_v;                  // HV bus voltage (V)
  float lv_v;                     // LV (12V) voltage (V)
};

// DC output current measurements (from messages 0x2C4 and 0x76C)
struct DCCurrentData {
  float phase_a_a;                // Phase A DC output current (A)
  float phase_b_a;                // Phase B DC output current (A)
  float phase_c_a;                // Phase C DC output current (A)
  float total_a;                  // Total DC output current (A)
};

// Alert tracking (from messages 0x424 and 0x504)
struct AlertData {
  uint8_t boot_id;                // Boot/firmware ID
  uint8_t count;                  // Alert counter
  uint8_t matrix[10];             // Alert history
};

// Message multiplexing state
struct MuxState {
  bool mux_3b2;                   // Message 0x3B2 multiplex toggle
  bool mux_545;                   // Message 0x545 multiplex toggle
  uint8_t count_545;              // Message 0x545 counter (0-15)
  uint8_t mux_2c4;                // Message 0x2C4 multiplex ID
  bool backup_2c4;                // Message 0x2C4 backup mode flag
  bool got_dci;                   // DC current from 0x2C4 (vs 0x76C)
};

class PCSCan {
public:
  // Initialize the PCS CAN interface
  static void begin(CANBus *ipc_can_bus);

  // Process incoming CAN messages
  static void process_messages();

  // CAN message transmission methods (called periodically)
  static void Msg13D();   // AC current limit message
  static void Msg20A();   // Static configuration message
  static void Msg212();   // Static configuration message
  static void Msg21D();   // EVSE and cable limits
  static void Msg22A();   // Main control message (mode and voltage)
  static void Msg232();   // Static configuration message
  static void Msg23D();   // AC current limit message (alternate)
  static void Msg25D();   // Static configuration message
  static void Msg2B2(uint16_t charge_power_w);  // Charge power request
  static void Msg321();   // Static configuration message
  static void Msg333();   // UI charge request watchdog
  static void Msg3A1();   // DCDC voltage setpoint
  static void Msg3B2();   // BMS log message (multiplexed)
  static void Msg545();   // VCFront message with counter/CRC

  // CAN message reception handlers
  static void handle204(uint32_t data[2]);  // Charger status
  static void handle224(uint32_t data[2]);  // DCDC data
  static void handle264(uint32_t data[2]);  // AC line status
  static void handle2A4(uint32_t data[2]);  // Temperature data
  static void handle2C4(uint32_t data[2]);  // Voltage log (multiplexed)
  static void handle3A4(uint32_t data[2]);  // Alert page
  static void handle424(uint32_t data[2]);  // Alert data
  static void handle504(uint32_t data[2]);  // Boot ID
  static void handle76C(uint32_t data[2]);  // Current measurements (alternate)

  // Control parameter setters (called by higher-level PCS controller)
  static void set_mode(PCSMode mode) { current_mode = mode; }
  static void set_hv_voltage(uint16_t voltage_v) { control_params.hv_voltage_v = voltage_v; }
  static void set_charge_power(uint16_t power_w) { control_params.charge_power_w = power_w; }
  static void set_dcdc_voltage(float voltage_v) { control_params.dcdc_voltage_v = voltage_v; }
  static void set_ac_current_limit(uint8_t limit_a) { control_params.ac_current_limit_a = limit_a; }
  static void set_evse_limit(uint8_t limit_a) { control_params.evse_limit_a = limit_a; }
  static void set_cable_limit(uint8_t limit) { control_params.cable_limit = limit; }
  static void set_charge_enable(bool enable) { charge_enable = enable; }

  // Struct-level status accessors (return references for efficient access)
  static const ChargerStatus& get_charger_status() { return charger_status; }
  static const DCDCStatus& get_dcdc_status() { return dcdc_status; }
  static const ACStatus& get_ac_status() { return ac_status; }
  static const TemperatureData& get_temperature_data() { return temperature_data; }
  static const VoltageData& get_voltage_data() { return voltage_data; }
  static const DCCurrentData& get_dc_current_data() { return dc_current_data; }
  static const AlertData& get_alert_data() { return alert_data; }
  static PCSMode get_mode() { return current_mode; }

private:
  // CAN interface
  static CANBus *can_bus;

  // State tracking
  static PCSMode current_mode;
  static bool charge_enable;

  // Organized data structures
  static ControlParams control_params;
  static ChargerStatus charger_status;
  static DCDCStatus dcdc_status;
  static ACStatus ac_status;
  static TemperatureData temperature_data;
  static VoltageData voltage_data;
  static DCCurrentData dc_current_data;
  static AlertData alert_data;
  static MuxState mux_state;

  // Helper functions
  static uint8_t calc_checksum(uint8_t *data, uint16_t id);
  static int16_t process_temp(uint16_t raw_val);
};
