#pragma once

#include "can.h"
#include <Arduino.h>

// CHAdeMO CAN IDs (decimal from documentation)
#define CHADEMO_VERSION_ID        0x100  // 256 - Version/identification
#define CHADEMO_EVSE_STATUS_ID    0x102  // 258 - EVSE voltage/current status
#define CHADEMO_EVSE_LIMITS_ID    0x108  // 264 - EVSE max voltage/current capability
#define CHADEMO_BATTERY_STATUS_ID 0x109  // 265 - Battery voltage/current/SOC demand

// CHAdeMO message send interval (must be < 2000ms to avoid timeout)
#define CHADEMO_TX_INTERVAL_MS    100    // Send every 100ms (10Hz) for safety margin

// CHAdeMO timeout detection
#define CHADEMO_RX_TIMEOUT_MS     2000   // 2 second timeout as per spec

/**
 * CHAdeMO charge session states
 */
enum CHAdeMOState : uint8_t {
  CHADEMO_IDLE = 0,           // Not connected / inactive
  CHADEMO_CONNECTED,          // Charger connected, waiting for enable
  CHADEMO_PRECHARGE,          // Precharging in progress
  CHADEMO_CHARGING,           // Active charging
  CHADEMO_ENDING,             // Charge ending gracefully
  CHADEMO_ERROR,              // Error state
  CHADEMO_TIMEOUT             // Communication timeout
};

/**
 * CHAdeMO EVSE status from 0x102
 */
struct CHAdeMOEVSEStatus {
  uint16_t voltage_v;         // Actual charger output voltage (V)
  uint16_t current_a;         // Actual charger output current (A)
  uint8_t opmode;             // Operation mode (13 = Current Demand)
  uint32_t last_rx_time;      // Timestamp of last received message
};

/**
 * CHAdeMO Battery request to send on 0x109
 */
struct CHAdeMOBatteryRequest {
  uint16_t battery_voltage_v;   // Present battery voltage (V) * 10 (0.1V resolution)
  uint16_t target_voltage_v;    // Target charge end voltage (V) * 10
  uint16_t charge_current_a;    // Requested charge current (A) * 10 (0.1A resolution)
  uint8_t soc;                  // State of charge (0-100%)
  bool enable;                  // Enable charging flag
};

/**
 * CHAdeMO Controller for Foccci charge controller
 *
 * Implements CHAdeMO protocol over CAN bus to communicate with Foccci
 * charge controller. Manages charge session state, sends battery status,
 * and receives charger feedback.
 */
class CHAdeMOController {
private:
  CANBus *can_bus;
  CHAdeMOState current_state;
  CHAdeMOEVSEStatus evse_status;
  CHAdeMOBatteryRequest battery_request;

  // EVSE capabilities (sent on 0x108)
  uint16_t evse_max_voltage_v;
  uint16_t evse_max_current_a;

  // Timing
  uint32_t last_tx_time;
  uint32_t state_entry_time;

  // Session control
  bool session_active;
  bool charger_connected;

  // Internal methods
  void send_version_message();
  void send_battery_request();
  void check_timeout();

public:
  CHAdeMOController();

  /**
   * Initialize the CHAdeMO controller with a CAN bus
   */
  void begin(CANBus *bus);

  /**
   * Process incoming CHAdeMO CAN message
   */
  void process_can_message(CAN_FRAME *frame);

  /**
   * Update CHAdeMO state machine (call periodically, e.g., every 100ms)
   */
  void update();

  /**
   * Start a charging session
   * @param target_voltage_v Target charge voltage in volts
   * @param max_current_a Maximum charge current in amps
   */
  void start_charging(uint16_t target_voltage_v, uint16_t max_current_a);

  /**
   * Stop the charging session
   */
  void stop_charging();

  /**
   * Update battery status for next transmission
   * @param voltage_v Current battery voltage
   * @param soc Battery state of charge (0-100%)
   * @param current_a Requested charge current
   */
  void update_battery_status(uint16_t voltage_v, uint8_t soc, uint16_t current_a);

  // Getters
  CHAdeMOState get_state() const { return current_state; }
  bool is_charging() const { return current_state == CHADEMO_CHARGING; }
  bool is_connected() const { return charger_connected; }
  uint16_t get_evse_voltage() const { return evse_status.voltage_v; }
  uint16_t get_evse_current() const { return evse_status.current_a; }
  uint16_t get_evse_max_voltage() const { return evse_max_voltage_v; }
  uint16_t get_evse_max_current() const { return evse_max_current_a; }
  bool has_timeout() const { return current_state == CHADEMO_TIMEOUT; }

  /**
   * Get time since last EVSE message (for timeout detection)
   */
  uint32_t get_time_since_last_rx() const;
};
