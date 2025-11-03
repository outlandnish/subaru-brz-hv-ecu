#pragma once
#include <Arduino.h>

// EVSE cable current limits based on proximity pilot voltage thresholds
enum EVSECableLimit : uint8_t {
  EVSE_CABLE_DISCONNECTED = 0,  // No cable connected
  EVSE_CABLE_13A = 13,
  EVSE_CABLE_20A = 20,
  EVSE_CABLE_32A = 32,
  EVSE_CABLE_63A = 63
};

// EVSE connection state (SAE J1772 states)
enum EVSEState : uint8_t {
  EVSE_STATE_A,  // No vehicle connected (12V)
  EVSE_STATE_B,  // Vehicle connected, not ready (9V)
  EVSE_STATE_C,  // Vehicle ready to charge / charging (6V)
  EVSE_STATE_D,  // Vehicle with ventilation required (3V)
  EVSE_STATE_E,  // No power / fault (0V)
  EVSE_FAULT
};

// Vehicle signaling state (what we tell the EVSE)
enum EVSEVehicleState : uint8_t {
  EVSE_VEHICLE_NOT_READY = 0,  // State B - 2.7kΩ resistor to ground
  EVSE_VEHICLE_READY = 1,      // State C - 882Ω resistor to ground
  EVSE_VEHICLE_VENTILATION = 2 // State D - 246Ω resistor to ground (not commonly used)
};

/**
 * EVSE (Electric Vehicle Supply Equipment) Controller
 *
 * Handles Level 2 AC charging communication via:
 * 1. Proximity Pilot - Detects cable connection and current rating
 * 2. Control Pilot - PWM signal indicating available charging current
 */
class EVSEController {
private:
  uint8_t proximity_pilot_pin;
  uint8_t control_pilot_input_pin;
  uint8_t control_pilot_output_pin;

  EVSEState current_state;
  EVSECableLimit cable_limit;
  EVSEVehicleState vehicle_state;

  uint16_t available_current_ma;  // Available current in milliamps from control pilot PWM
  uint16_t max_charge_current_ma; // Maximum charging current (limited by cable rating)

  // PWM measurement variables (interrupt-based)
  volatile uint32_t pilot_high_time_us;
  volatile uint32_t pilot_period_us;
  volatile uint32_t last_edge_time_us;
  volatile bool last_edge_was_rising;
  volatile bool pilot_measurement_ready;

  // Proximity pilot ADC filtering
  static const uint8_t PROX_FILTER_SIZE = 10;
  uint16_t proximity_samples[PROX_FILTER_SIZE];
  uint8_t proximity_sample_index;

  // Static instance pointer for interrupt handling
  static EVSEController* instance;

  // Helper methods
  EVSECableLimit read_proximity_pilot();
  uint16_t calculate_control_pilot_current();
  uint16_t get_filtered_proximity_voltage();
  void setup_control_pilot_interrupt();
  void set_vehicle_signaling(EVSEVehicleState state);

  // Static interrupt handler
  static void control_pilot_interrupt_handler();

public:
  EVSEController(uint8_t prox_pin, uint8_t ctrl_in_pin, uint8_t ctrl_out_pin);

  void begin();
  void update();

  // State control
  void set_ready_to_charge(bool ready);
  void set_charging(bool charging);

  // State accessors
  EVSEState get_state() const { return current_state; }
  EVSECableLimit get_cable_limit() const { return cable_limit; }
  EVSEVehicleState get_vehicle_state() const { return vehicle_state; }
  uint16_t get_available_current_ma() const { return available_current_ma; }
  uint16_t get_max_charge_current_ma() const { return max_charge_current_ma; }

  bool is_connected() const { return current_state != EVSE_STATE_A && current_state != EVSE_STATE_E; }
  bool is_ready_to_charge() const { return current_state == EVSE_STATE_C; }

  // Internal interrupt handler (called by static handler)
  void handle_control_pilot_interrupt();
};
