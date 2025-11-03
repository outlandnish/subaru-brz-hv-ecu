#include "evse.h"

// Static member initialization
EVSEController* EVSEController::instance = nullptr;

EVSEController::EVSEController(uint8_t prox_pin, uint8_t ctrl_in_pin, uint8_t ctrl_out_pin)
  : proximity_pilot_pin(prox_pin),
    control_pilot_input_pin(ctrl_in_pin),
    control_pilot_output_pin(ctrl_out_pin),
    current_state(EVSE_STATE_A),
    cable_limit(EVSE_CABLE_DISCONNECTED),
    vehicle_state(EVSE_VEHICLE_NOT_READY),
    available_current_ma(0),
    max_charge_current_ma(0),
    pilot_high_time_us(0),
    pilot_period_us(0),
    last_edge_time_us(0),
    last_edge_was_rising(false),
    pilot_measurement_ready(false),
    proximity_sample_index(0) {

  memset(proximity_samples, 0, sizeof(proximity_samples));
  instance = this;
}

void EVSEController::begin() {
  pinMode(proximity_pilot_pin, INPUT);
  pinMode(control_pilot_input_pin, INPUT);
  pinMode(control_pilot_output_pin, OUTPUT);

  // Initialize to "not ready" state (State B signaling)
  set_vehicle_signaling(EVSE_VEHICLE_NOT_READY);

  // Initialize ADC for proximity pilot
  analogReadResolution(12); // 12-bit ADC (0-4095)

  // Setup interrupt for control pilot PWM measurement
  setup_control_pilot_interrupt();

  current_state = EVSE_STATE_A;
  cable_limit = EVSE_CABLE_DISCONNECTED;
}

void EVSEController::setup_control_pilot_interrupt() {
  // Attach interrupt on both rising and falling edges for PWM measurement
  attachInterrupt(digitalPinToInterrupt(control_pilot_input_pin),
                  control_pilot_interrupt_handler,
                  CHANGE);
}

void EVSEController::control_pilot_interrupt_handler() {
  if (instance) {
    instance->handle_control_pilot_interrupt();
  }
}

void EVSEController::handle_control_pilot_interrupt() {
  uint32_t now = micros();
  bool current_level = digitalRead(control_pilot_input_pin);

  if (current_level && !last_edge_was_rising) {
    // Rising edge - start of high pulse
    if (last_edge_time_us > 0) {
      pilot_period_us = now - last_edge_time_us;
    }
    last_edge_time_us = now;
    last_edge_was_rising = true;
  } else if (!current_level && last_edge_was_rising) {
    // Falling edge - end of high pulse
    pilot_high_time_us = now - last_edge_time_us;
    pilot_measurement_ready = true;
    last_edge_was_rising = false;
  }
}

void EVSEController::set_vehicle_signaling(EVSEVehicleState state) {
  vehicle_state = state;

  // Control the output pin to simulate different load resistances
  // In a real implementation, this would control a resistor network via MOSFETs
  // For now, we'll use PWM to approximate the voltage divider effect

  switch (state) {
    case EVSE_VEHICLE_NOT_READY:
      // State B - 2.7kΩ (creates ~9V)
      analogWrite(control_pilot_output_pin, 85);  // ~33% PWM
      break;

    case EVSE_VEHICLE_READY:
      // State C - 882Ω (creates ~6V)
      analogWrite(control_pilot_output_pin, 128);  // ~50% PWM
      break;

    case EVSE_VEHICLE_VENTILATION:
      // State D - 246Ω (creates ~3V)
      analogWrite(control_pilot_output_pin, 191);  // ~75% PWM
      break;
  }
}

void EVSEController::update() {
  // Read proximity pilot to detect cable connection and rating
  cable_limit = read_proximity_pilot();

  if (cable_limit == EVSE_CABLE_DISCONNECTED) {
    current_state = EVSE_STATE_A;
    available_current_ma = 0;
    max_charge_current_ma = 0;
    return;
  }

  // Cable is connected - state will be determined below based on pilot signal

  // Read control pilot PWM to get available current
  available_current_ma = calculate_control_pilot_current();

  // Maximum charge current is limited by the lesser of cable rating or control pilot
  uint16_t cable_limit_ma = cable_limit * 1000;
  max_charge_current_ma = min(available_current_ma, cable_limit_ma);

  // Update state based on available current and vehicle signaling
  // Note: Actual state detection would require measuring pilot voltage
  // For now, we'll infer state from our signaling and available current

  if (max_charge_current_ma >= 6000) { // Minimum 6A required for charging
    if (vehicle_state == EVSE_VEHICLE_READY) {
      current_state = EVSE_STATE_C;  // Ready to charge / charging
    } else {
      current_state = EVSE_STATE_B;  // Connected but not ready
    }
  } else if (max_charge_current_ma > 0) {
    current_state = EVSE_STATE_B;  // Connected but insufficient current
  }
}

void EVSEController::set_ready_to_charge(bool ready) {
  if (ready && is_connected()) {
    set_vehicle_signaling(EVSE_VEHICLE_READY);
  } else {
    set_vehicle_signaling(EVSE_VEHICLE_NOT_READY);
  }
  update();  // Refresh state
}

void EVSEController::set_charging(bool charging) {
  // Charging state is same as ready state in J1772
  set_ready_to_charge(charging);
}

EVSECableLimit EVSEController::read_proximity_pilot() {
  // Read and filter proximity pilot voltage
  uint16_t adc_value = analogRead(proximity_pilot_pin);

  // Add to circular buffer for filtering
  proximity_samples[proximity_sample_index] = adc_value;
  proximity_sample_index = (proximity_sample_index + 1) % PROX_FILTER_SIZE;

  // Get filtered ADC value
  uint16_t filtered_adc = get_filtered_proximity_voltage();

  // Type 2 connector proximity pilot voltage thresholds
  // Reference: SAE J1772 and IEC 61851
  // ADC is 12-bit (0-4095) with 3.3V reference
  // Thresholds adjusted for 3.3V ADC reference:

  // Type 2 connector proximity pilot voltage thresholds (for 3.3V ADC reference)
  // Based on SAE J1772 specification

  if (filtered_adc > 3500) {
    // > 2.8V - Unconnected (floating, pulled high)
    current_state = EVSE_STATE_A;
    return EVSE_CABLE_DISCONNECTED;
  } else if (filtered_adc > 2950) {
    // 2.4V-2.8V - 13A cable (150Ω + 480Ω resistor divider)
    return EVSE_CABLE_13A;
  } else if (filtered_adc > 2580) {
    // 2.1V-2.4V - 20A cable (150Ω + 330Ω resistor divider)
    return EVSE_CABLE_20A;
  } else if (filtered_adc > 1650) {
    // 1.35V-2.1V - 32A cable (150Ω + 220Ω resistor divider)
    return EVSE_CABLE_32A;
  } else if (filtered_adc > 925) {
    // 0.75V-1.35V - 63A cable (150Ω + 100Ω resistor divider)
    return EVSE_CABLE_63A;
  } else if (filtered_adc > 200) {
    // Low voltage but not zero - possible fault
    current_state = EVSE_FAULT;
    return EVSE_CABLE_DISCONNECTED;
  }

  // Very low/zero voltage - no power or fault
  current_state = EVSE_STATE_E;
  return EVSE_CABLE_DISCONNECTED;
}

uint16_t EVSEController::get_filtered_proximity_voltage() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < PROX_FILTER_SIZE; i++) {
    sum += proximity_samples[i];
  }
  return sum / PROX_FILTER_SIZE;
}

uint16_t EVSEController::calculate_control_pilot_current() {
  // Use interrupt-measured PWM values instead of polling
  // The control pilot is a ±12V square wave (1kHz) with duty cycle encoding current:
  // - Duty cycle (%) maps to available current (A)
  // - Formula: Current (A) = Duty Cycle (%) * 0.6
  // - Example: 50% duty = 30A, 80% duty = 48A

  if (!pilot_measurement_ready || pilot_period_us == 0) {
    // No valid PWM measurement available
    current_state = EVSE_STATE_E;  // No power
    return 0;
  }

  // Reset flag to catch next measurement
  pilot_measurement_ready = false;

  // Validate frequency (should be ~1kHz = 1000us period)
  if (pilot_period_us < 800 || pilot_period_us > 1200) {
    // Invalid frequency
    return 0;
  }

  // Calculate duty cycle (0-100%)
  float duty_cycle = ((float)pilot_high_time_us / (float)pilot_period_us) * 100.0f;

  // Validate duty cycle range (typically 10% - 96%)
  if (duty_cycle < 5.0f || duty_cycle > 97.0f) {
    // Invalid duty cycle
    return 0;
  }

  // Convert duty cycle to current in milliamps
  // Current (mA) = Duty Cycle (%) * 0.6 * 1000
  uint16_t current_ma = (uint16_t)(duty_cycle * 0.6f * 1000.0f);

  return current_ma;
}
