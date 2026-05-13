/*
 * Simple Single-Device Battery Charger
 *
 * Controls charging for a single 6S battery using BCC0
 * - Charges to 21V (3.5V per cell)
 * - Balances cells when delta > 10mV
 * - Controls single contactor (pin 1 high, pin 2 low when active)
 *
 * Usage:
 *   Build with: pio run -e simple-charger
 *   Upload with: pio run -e simple-charger -t upload
 *   Monitor with: pio device monitor
 */

#include "debug_serial.h"
#include "Arduino.h"
#include "SPI.h"
#include "TPLSPI.h"
#include "BatteryCellController.h"
#include "bcc/bcc_config.h"
#include "hal/dma_config.h"
#include "hal/hv-ecu-v1-pins.h"
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>
#include "../lib/can/can.h"
#include "ivt-s/ivt_shunt.h"

// Configuration
#define DEVICE_COUNT 1          // Single device on the chain
#define CELL_COUNT 6            // 6S battery
#define BALANCE_THRESHOLD_UV 10000  // 10mV in microvolts
#define MEASUREMENT_INTERVAL_MS 100   // Measure every 100ms (must be < 256ms,
                                       // the max BCC SYS_CFG2 TIMEOUT_COMM; otherwise
                                       // the MC33772 sets COM_LOSS and stops responding)
#define JSON_INTERVAL_MS 1000          // Emit JSON status at 1Hz
#define BALANCING_TIMER_MIN 5   // Balance for 5 minutes at a time

// Contactor PWM settings
#define CONTACTOR_PWM_FREQ 25000     // 25kHz PWM frequency (above audible range)
#define CONTACTOR_ENGAGE_DUTY 100    // 100% duty cycle to engage (pull-in)
#define CONTACTOR_HOLD_DUTY 30       // 30% duty cycle to hold (economizer)
#define CONTACTOR_ENGAGE_TIME_MS 100 // Hold at 100% for 100ms before dropping to hold duty
#define CONTACTOR_CYCLE_INTERVAL_MS (60UL * 60UL * 1000UL)  // 1 hour in milliseconds
#define CONTACTOR_CYCLE_PAUSE_MS 1000  // 1 second pause during cycle

// Charging states
enum ChargeState {
  STATE_INIT,
  STATE_IDLE,
  STATE_CHARGING,
  STATE_BALANCING,
  STATE_COMPLETE,
  STATE_SLEEP,
  STATE_ERROR
};

// Global objects
TPLSPI *tpl0;
BatteryCellController *bcc0;
SPIClass *bcc0_tx_spi, *bcc0_rx_spi;
bcc_device_t devices_0[DEVICE_COUNT];

// CAN bus and IVT-S shunt
CANBus *hv_can = nullptr;
IVTShunt *ivt_shunt = nullptr;

// State variables
ChargeState current_state = STATE_INIT;
uint32_t cell_voltages_uv[CELL_COUNT];
float temperature_an2_c = 0.0f;  // Thermistor 1 temperature
float temperature_an3_c = 0.0f;  // Thermistor 2 temperature
uint32_t last_measurement_time = 0;
uint32_t last_json_time = 0;
bool contactor_enabled = false;
bool hardware_initialized = false;
uint32_t last_contactor_cycle_time = 0;  // Track when we last cycled contactors

// Target voltage (default 21V = 3.5V per cell)
uint32_t target_voltage_uv = 21000000;

// Calibration data from fuse mirror
struct CalibrationData {
  uint16_t vref_cal;           // 0x02
  uint16_t cell_offset_0;      // 0x03
  uint16_t cell_offset_1;      // 0x04
  uint16_t cell_gain;          // 0x05
  bool loaded;
} cal_data = {0, 0, 0, 0, false};

// HardwareTimer for contactor PWM control (only need PWM on pin 1)
HardwareTimer *contactor_timer = nullptr;
uint32_t contactor_channel = 0;

// Task handles
TaskHandle_t monitor_task_handle = NULL;
TaskHandle_t console_task_handle = NULL;

// Function prototypes
void enable_contactor();
void disable_contactor();
bool initialize_bcc();
bool measure_voltages();
void measure_temperatures();
void print_voltages();
void print_json_status();
uint32_t get_total_voltage();
uint32_t get_max_cell_delta();
void apply_cell_balancing();
void stop_cell_balancing();
void update_charging_state();
void dump_calibration_data();
void load_calibration_data();
uint32_t apply_calibration(uint32_t raw_voltage_uv, uint8_t cell_index);

// FreeRTOS task functions
void monitor_task(void *pvParameters);
void console_task(void *pvParameters);

void enable_contactor() {
  if (!contactor_enabled) {
    debug_println("Enabling contactor (charging ON)");

    // Enable the H-bridge driver
    digitalWrite(HV_CONTACTOR_NSLEEP_PIN, HIGH);
    delay(10);

    // Set contactor 2 LOW (constant)
    digitalWrite(HV_CONTACTOR_2_PIN, LOW);
    // Start PWM on contactor 1 at 100% to engage (pull-in)
    contactor_timer->setCaptureCompare(contactor_channel, CONTACTOR_ENGAGE_DUTY, PERCENT_COMPARE_FORMAT);
    contactor_timer->resume();

    // Wait for contactor to engage
    delay(CONTACTOR_ENGAGE_TIME_MS);

    // Reduce to hold duty cycle (economizer mode)
    contactor_timer->setCaptureCompare(contactor_channel, CONTACTOR_HOLD_DUTY, PERCENT_COMPARE_FORMAT);
    debug_printf("Contactor holding at %d%% duty cycle\r\n", CONTACTOR_HOLD_DUTY);

    contactor_enabled = true;
    last_contactor_cycle_time = millis();  // Reset cycle timer when enabling
  }
}

void cycle_contactor() {
  debug_println("\n=== Cycling Contactor (Resetting Charger) ===");

  // Disable contactor
  contactor_timer->pause();
  contactor_timer->setCaptureCompare(contactor_channel, 0, PERCENT_COMPARE_FORMAT);
  digitalWrite(HV_CONTACTOR_2_PIN, LOW);
  delay(10);
  digitalWrite(HV_CONTACTOR_NSLEEP_PIN, LOW);

  // Pause for 1 second
  debug_println("Pausing for 1 second...");
  delay(CONTACTOR_CYCLE_PAUSE_MS);

  // Re-enable contactor
  debug_println("Re-enabling contactor");
  digitalWrite(HV_CONTACTOR_NSLEEP_PIN, HIGH);
  delay(10);
  digitalWrite(HV_CONTACTOR_2_PIN, LOW);

  // Start PWM at 100% to engage
  contactor_timer->setCaptureCompare(contactor_channel, CONTACTOR_ENGAGE_DUTY, PERCENT_COMPARE_FORMAT);
  contactor_timer->resume();
  delay(CONTACTOR_ENGAGE_TIME_MS);

  // Reduce to hold duty cycle
  contactor_timer->setCaptureCompare(contactor_channel, CONTACTOR_HOLD_DUTY, PERCENT_COMPARE_FORMAT);
  debug_printf("Contactor re-engaged at %d%% duty cycle\r\n", CONTACTOR_HOLD_DUTY);

  last_contactor_cycle_time = millis();
}

void disable_contactor() {
  if (contactor_enabled) {
    debug_println("Disabling contactor (charging OFF)");

    // Stop PWM
    contactor_timer->pause();
    contactor_timer->setCaptureCompare(contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    digitalWrite(HV_CONTACTOR_2_PIN, LOW);
    delay(10);

    // Disable the H-bridge driver
    digitalWrite(HV_CONTACTOR_NSLEEP_PIN, LOW);
    contactor_enabled = false;
  }
}

void monitor_task(void *pvParameters) {
  // Hardware initialization in task context (like BMS)
  debug_println("\n=== Initializing Hardware ===");
  vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for system to stabilize

  // Initialize HV CAN bus (500kbps for IVT shunt)
  debug_println("Initializing HV CAN bus...");
  hv_can = new CANBus(HV_CAN_RX, HV_CAN_TX);
  hv_can->begin(500000);
  debug_println("HV CAN: Ready");

  // Initialize IVT-S current/voltage shunt
  debug_println("Initializing IVT-S shunt...");
  ivt_shunt = new IVTShunt();
  ivt_shunt->begin(hv_can);
  ivt_shunt->set_debug(false);  // Disable verbose CAN debug
  debug_println("IVT-S: Ready");
  debug_println();

  // Initialize BCC0
  debug_println("Initializing BCC0...");

  // Setup device type (MC33772C for 6-cell battery)
  devices_0[0] = BCC_DEVICE_MC33772;

  // Initialize SPI
  bcc0_tx_spi = new SPIClass(BCC0_TX_DATA, NC, BCC0_TX_SCK, NC);
  bcc0_rx_spi = new SPIClass(BCC0_RX_DATA, NC, BCC0_RX_SCK, BCC0_RX_CS);

  // Initialize TPL and BCC
  tpl0 = new TPLSPI(bcc0_tx_spi, bcc0_rx_spi, BCC0_TX_CS, configureDMA_HV_ECU);
  bcc0 = new BatteryCellController(tpl0, devices_0, DEVICE_COUNT, CELL_COUNT,
                                   BCC0_ENABLE, BCC0_INTB, false);

  pinMode(BCC0_TX_CS, OUTPUT);
  digitalWrite(BCC0_TX_CS, HIGH);

  debug_println("Starting BCC0...");

  // Initialize BCC hardware
  bcc_status_t error = bcc0->begin(nullptr);

  if (error == BCC_STATUS_SUCCESS) {
    debug_println("BCC0: Ready");
    hardware_initialized = true;
    current_state = STATE_IDLE;

    // Load calibration data from fuse mirror
    load_calibration_data();

    // Note: Full calibration dump available via 'dump' command
  } else {
    debug_printf("BCC0: Init failed (error %d)\r\n", error);
    current_state = STATE_ERROR;
  }

  // Main monitoring loop
  while (true) {
    // Process CAN messages for IVT-S
    if (hv_can != nullptr && hv_can->available()) {
      CAN_FRAME frame;
      while (hv_can->read(frame)) {
        if (ivt_shunt != nullptr) {
          ivt_shunt->process_can_frame(&frame);
        }
      }
    }

    if (hardware_initialized && current_state != STATE_ERROR && current_state != STATE_SLEEP) {
      // Always measure voltages and temperatures
      if (measure_voltages()) {
        measure_temperatures();
        
        // Update charging state if we're actively charging
        if (current_state != STATE_IDLE && current_state != STATE_COMPLETE) {
          uint32_t now = millis();
          if (now - last_measurement_time >= MEASUREMENT_INTERVAL_MS) {
            last_measurement_time = now;
            update_charging_state();
          }
        }
        
        // Output JSON status for web app at 1Hz
        uint32_t now = millis();
        if (now - last_json_time >= JSON_INTERVAL_MS) {
          last_json_time = now;
          print_json_status();
        }
      }
    }

    // Contactor cycling disabled for web app control
    /*
    // Check if we need to cycle the contactor (only while charging)
    if (current_state == STATE_CHARGING && contactor_enabled) {
          uint32_t now = millis();
          // Handle millis() rollover (happens every ~49 days)
          uint32_t elapsed = (now >= last_contactor_cycle_time)
                           ? (now - last_contactor_cycle_time)
                           : (0xFFFFFFFF - last_contactor_cycle_time + now + 1);

      if (elapsed >= CONTACTOR_CYCLE_INTERVAL_MS) {
        cycle_contactor();
      }
    }
    */

    vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
  }
}

void console_task(void *pvParameters) {
  debug_println("\n=== Simple Charger Console ===");
  debug_println("Commands: 'start', 'stop', 'end', 'status', 'dump', 'reset', 'get_voltage', 'set_voltage <uV>'");
  debug_println();

  String inputBuffer = "";

  while (true) {
    // Check for available serial data
    while (DebugSerial.available() > 0) {
      char c = DebugSerial.read();

      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          inputBuffer.trim();
          inputBuffer.toLowerCase();

          if (inputBuffer == "start") {
            if (hardware_initialized && (current_state == STATE_IDLE || current_state == STATE_COMPLETE)) {
              debug_println("\nStarting charging sequence...");
              current_state = STATE_CHARGING;
              enable_contactor();
              last_measurement_time = 0; // Force immediate measurement
            } else if (!hardware_initialized) {
              debug_println("\nERROR: Hardware not initialized yet");
            } else {
              debug_println("\nERROR: Cannot start from current state");
            }
          } else if (inputBuffer == "stop") {
            debug_println("\nStopping charging...");
            disable_contactor();
            stop_cell_balancing();
            current_state = STATE_IDLE;
            last_contactor_cycle_time = 0;  // Reset cycle timer
          } else if (inputBuffer == "end") {
            if (hardware_initialized) {
              debug_println("\nEntering low power mode...");
              disable_contactor();
              stop_cell_balancing();

              // Put BCC into low power mode
              bcc_status_t error = bcc0->enter_low_power_mode();
              if (error == BCC_STATUS_SUCCESS) {
                debug_println("BCC0 entered low power mode successfully");
                current_state = STATE_SLEEP;
              } else {
                debug_printf("ERROR: Failed to enter low power mode (error %d)\r\n", error);
                current_state = STATE_ERROR;
              }
              last_contactor_cycle_time = 0;  // Reset cycle timer
            } else {
              debug_println("\nERROR: Hardware not initialized yet");
            }
          } else if (inputBuffer == "status") {
            if (hardware_initialized) {
              // Just print the latest voltages - don't trigger new measurement
              print_voltages();
            } else {
              debug_println("\nERROR: Hardware not initialized yet");
            }
          } else if (inputBuffer == "dump") {
            if (hardware_initialized) {
              debug_println("\nDumping calibration data...");
              dump_calibration_data();
            } else {
              debug_println("\nERROR: Hardware not initialized yet");
            }
          } else if (inputBuffer == "reset") {
            debug_println("\nResetting STM32...");
            DebugSerial.flush();  // Ensure message is sent before reset
            delay(100);
            NVIC_SystemReset();
          } else if (inputBuffer == "get_voltage") {
            debug_printf("\nTarget voltage: %lu uV (%.3f V, %.4f V per cell)\r\n",
                         target_voltage_uv,
                         target_voltage_uv / 1000000.0f,
                         target_voltage_uv / 1000000.0f / CELL_COUNT);
          } else if (inputBuffer.startsWith("set_voltage ")) {
            String voltageStr = inputBuffer.substring(12);
            voltageStr.trim();
            uint32_t newVoltage = voltageStr.toInt();
            
            // Validate range (15V to 25.2V in microvolts)
            if (newVoltage >= 15000000 && newVoltage <= 25200000) {
              target_voltage_uv = newVoltage;
              debug_printf("\nTarget voltage set to: %lu uV (%.3f V, %.4f V per cell)\r\n",
                           target_voltage_uv,
                           target_voltage_uv / 1000000.0f,
                           target_voltage_uv / 1000000.0f / CELL_COUNT);
            } else {
              debug_printf("\nERROR: Voltage out of range. Must be 15000000-25200000 uV (15-25.2V)\r\n");
            }
          } else if (inputBuffer.length() > 0) {
            debug_printf("\nUnknown command: %s\r\n", inputBuffer.c_str());
          }

          inputBuffer = "";
        }
      } else {
        inputBuffer += c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

bool measure_voltages() {
  // Start ADC conversion for all cells using global conversion
  // ADC_CFG value 0x0717 configures:
  // - Cell voltage measurement
  // - Standard conversion mode
  // - All enabled cells
  bcc_status_t error = bcc0->start_conversion_global_async(0x0717);
  if (error != BCC_STATUS_SUCCESS) {
    debug_printf("ERROR: Failed to start conversion (%d)\r\n", error);
    return false;
  }

  // Wait for conversion to complete
  // Cell voltage ADC conversion takes approximately 20ms
  // Just wait instead of polling to avoid DMA conflicts
  delay(50);

  // Read cell voltages directly
  error = bcc0->get_cell_voltages(BCC_CID_DEV1, cell_voltages_uv);
  if (error != BCC_STATUS_SUCCESS) {
    debug_printf("ERROR: Failed to read voltages (%d)\r\n", error);
    return false;
  }

  return true;
}

void measure_temperatures() {
  // Read AN2 and AN3 analog voltages from BCC (in microvolts)
  uint32_t voltage_an2_uv = 0, voltage_an3_uv = 0;
  
  bcc_status_t error = bcc0->get_an_voltage(BCC_CID_DEV1, 2, &voltage_an2_uv);  // AN2
  if (error == BCC_STATUS_SUCCESS) {
    float voltage_an2 = voltage_an2_uv / 1000000.0f;
    
    // Porsche Taycan NTC temperature conversion
    // Calibration: 0.456V = 23°C (room temperature)
    // Circuit: 3.3V → 10k pull-up → V_measured → NTC → GND
    // Calculate NTC resistance from voltage divider
    if (voltage_an2 > 0.01f && voltage_an2 < 3.29f) {
      float r_ntc = (10000.0f * voltage_an2) / (3.3f - voltage_an2);
      
      // Steinhart-Hart B-parameter equation
      // Using B=3950 (typical automotive NTC) and R0=1600 ohms at T0=23°C (296.15K)
      // This is calibrated from hardware: 0.456V → 1600 ohms → 23°C
      float inv_temp = 1.0f / 296.15f + (1.0f / 3950.0f) * logf(r_ntc / 1600.0f);
      temperature_an2_c = (1.0f / inv_temp) - 273.15f;
      
      // Clamp to reasonable automotive battery range
      if (temperature_an2_c < -40.0f) temperature_an2_c = -40.0f;
      if (temperature_an2_c > 85.0f) temperature_an2_c = 85.0f;
    } else {
      temperature_an2_c = 0.0f;  // Invalid reading
    }
  }
  
  error = bcc0->get_an_voltage(BCC_CID_DEV1, 3, &voltage_an3_uv);  // AN3
  if (error == BCC_STATUS_SUCCESS) {
    float voltage_an3 = voltage_an3_uv / 1000000.0f;
    
    if (voltage_an3 > 0.01f && voltage_an3 < 3.29f) {
      float r_ntc = (10000.0f * voltage_an3) / (3.3f - voltage_an3);
      float inv_temp = 1.0f / 296.15f + (1.0f / 3950.0f) * logf(r_ntc / 1600.0f);
      temperature_an3_c = (1.0f / inv_temp) - 273.15f;
      
      if (temperature_an3_c < -40.0f) temperature_an3_c = -40.0f;
      if (temperature_an3_c > 85.0f) temperature_an3_c = 85.0f;
    } else {
      temperature_an3_c = 0.0f;
    }
  }
}

void print_voltages() {
  // Print IVT-S measurements first
  if (ivt_shunt != nullptr && ivt_shunt->is_alive()) {
    debug_println("\n=== IVT-S Measurements ===");
    debug_printf("  Current:     %.2f A\r\n", ivt_shunt->get_current());
    debug_printf("  Voltage:     %.2f V\r\n", ivt_shunt->get_voltage());
    debug_printf("  Power:       %.2f kW\r\n", ivt_shunt->get_power());
    debug_printf("  Temperature: %.1f °C\r\n", ivt_shunt->get_temperature());
    debug_printf("  Amp-Hours:   %.3f Ah\r\n", ivt_shunt->get_amp_hours());
    debug_printf("  Energy:      %.3f kWh\r\n", ivt_shunt->get_kilowatt_hours());
  } else {
    debug_println("\n=== IVT-S Measurements ===");
    debug_println("  Status: OFFLINE");
  }

  debug_println("\n=== Cell Voltages ===");
  uint32_t total_voltage = 0;
  uint32_t total_voltage_cal = 0;
  uint32_t min_v = cell_voltages_uv[0];
  uint32_t max_v = cell_voltages_uv[0];

  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    uint32_t cal_voltage = apply_calibration(cell_voltages_uv[i], i);

    if (cal_data.loaded) {
      debug_printf("  Cell %d: %.4f V  (cal: %.4f V)\r\n",
                    i + 1,
                    cell_voltages_uv[i] / 1000000.0f,
                    cal_voltage / 1000000.0f);
    } else {
      debug_printf("  Cell %d: %.4f V\r\n", i + 1, cell_voltages_uv[i] / 1000000.0f);
    }

    total_voltage += cell_voltages_uv[i];
    total_voltage_cal += cal_voltage;
    if (cell_voltages_uv[i] < min_v) min_v = cell_voltages_uv[i];
    if (cell_voltages_uv[i] > max_v) max_v = cell_voltages_uv[i];
  }

  float delta_mv = (max_v - min_v) / 1000.0f;
  debug_printf("\nTotal: %.3f V  |  Delta: %.2f mV\r\n",
                total_voltage / 1000000.0f, delta_mv);

  if (cal_data.loaded) {
    debug_printf("Total (cal): %.3f V\r\n", total_voltage_cal / 1000000.0f);
  }

  debug_printf("Min: %.4f V  |  Max: %.4f V\r\n",
                min_v / 1000000.0f, max_v / 1000000.0f);
  
  // Print temperatures
  debug_println("\n=== Battery Temperatures ===");
  
  // Read and print raw AN voltages
  uint32_t raw_an2_uv = 0, raw_an3_uv = 0;
  if (bcc0->get_an_voltage(BCC_CID_DEV1, 2, &raw_an2_uv) == BCC_STATUS_SUCCESS) {
    debug_printf("  AN2 Raw: %.3f V  (%lu uV)\r\n", raw_an2_uv / 1000000.0f, raw_an2_uv);
  }
  if (bcc0->get_an_voltage(BCC_CID_DEV1, 3, &raw_an3_uv) == BCC_STATUS_SUCCESS) {
    debug_printf("  AN3 Raw: %.3f V  (%lu uV)\r\n", raw_an3_uv / 1000000.0f, raw_an3_uv);
  }
  
  debug_printf("  Thermistor 1 (AN2): %.1f °C\r\n", temperature_an2_c);
  debug_printf("  Thermistor 2 (AN3): %.1f °C\r\n", temperature_an3_c);
  float avg_temp = (temperature_an2_c + temperature_an3_c) / 2.0f;
  float max_temp = (temperature_an2_c > temperature_an3_c) ? temperature_an2_c : temperature_an3_c;
  debug_printf("  Average: %.1f °C  |  Max: %.1f °C\r\n", avg_temp, max_temp);
}

void print_json_status() {
  // Output JSON for web app consumption
  DebugSerial.print("{");
  
  // Charging state
  DebugSerial.print("\"charging_state\":\"");
  switch (current_state) {
    case STATE_IDLE: DebugSerial.print("idle"); break;
    case STATE_CHARGING: DebugSerial.print("charging"); break;
    case STATE_BALANCING: DebugSerial.print("balancing"); break;
    case STATE_COMPLETE: DebugSerial.print("complete"); break;
    case STATE_ERROR: DebugSerial.print("error"); break;
    case STATE_SLEEP: DebugSerial.print("sleep"); break;
    default: DebugSerial.print("unknown"); break;
  }
  DebugSerial.print("\",");
  
  // Target voltage
  debug_printf("\"target_voltage\":%.3f,", target_voltage_uv / 1000000.0f);
  
  // Cell voltages
  DebugSerial.print("\"cell_voltages\":[");
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    if (i > 0) DebugSerial.print(",");
    debug_printf("%.4f", cell_voltages_uv[i] / 1000000.0f);
  }
  DebugSerial.print("],");
  
  // Temperatures
  float avg_temp = (temperature_an2_c + temperature_an3_c) / 2.0f;
  float max_temp = (temperature_an2_c > temperature_an3_c) ? temperature_an2_c : temperature_an3_c;
  DebugSerial.print("\"temperatures\":{");
  debug_printf("\"thermistor1\":%.1f,", temperature_an2_c);
  debug_printf("\"thermistor2\":%.1f,", temperature_an3_c);
  debug_printf("\"average\":%.1f,", avg_temp);
  debug_printf("\"max\":%.1f", max_temp);
  DebugSerial.print("},");
  
  // IVT-S shunt data
  DebugSerial.print("\"ivt_shunt\":{");
  if (ivt_shunt != nullptr && ivt_shunt->is_alive()) {
    debug_printf("\"online\":true,");
    debug_printf("\"current\":%.2f,", ivt_shunt->get_current());
    debug_printf("\"voltage\":%.2f,", ivt_shunt->get_voltage());
    debug_printf("\"power\":%.2f,", ivt_shunt->get_power());
    debug_printf("\"temperature\":%.1f,", ivt_shunt->get_temperature());
    debug_printf("\"amp_hours\":%.3f,", ivt_shunt->get_amp_hours());
    debug_printf("\"energy\":%.3f", ivt_shunt->get_kilowatt_hours());
  } else {
    DebugSerial.print("\"online\":false,");
    DebugSerial.print("\"current\":0.0,");
    DebugSerial.print("\"voltage\":0.0,");
    DebugSerial.print("\"power\":0.0,");
    DebugSerial.print("\"temperature\":0.0,");
    DebugSerial.print("\"amp_hours\":0.0,");
    DebugSerial.print("\"energy\":0.0");
  }
  DebugSerial.print("}");
  
  debug_println("}");
}

uint32_t get_total_voltage() {
  uint32_t total = 0;
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    total += cell_voltages_uv[i];
  }
  return total;
}

uint32_t get_max_cell_delta() {
  uint32_t min_v = cell_voltages_uv[0];
  uint32_t max_v = cell_voltages_uv[0];

  for (uint8_t i = 1; i < CELL_COUNT; i++) {
    if (cell_voltages_uv[i] < min_v) min_v = cell_voltages_uv[i];
    if (cell_voltages_uv[i] > max_v) max_v = cell_voltages_uv[i];
  }

  return max_v - min_v;
}

void apply_cell_balancing() {
  debug_println("\n=== Enabling Cell Balancing ===");

  // First, enable the cell balancing feature globally
  bcc_status_t error = bcc0->enable_cell_balancing(BCC_CID_DEV1, true);
  if (error != BCC_STATUS_SUCCESS) {
    debug_printf("ERROR: Failed to enable cell balancing feature (%d)\r\n", error);
    return;
  }

  // Find the minimum cell voltage
  uint32_t min_v = cell_voltages_uv[0];
  for (uint8_t i = 1; i < CELL_COUNT; i++) {
    if (cell_voltages_uv[i] < min_v) {
      min_v = cell_voltages_uv[i];
    }
  }

  // Enable balancing for cells that are significantly higher than the minimum
  bool any_balancing = false;

  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    uint32_t delta = cell_voltages_uv[i] - min_v;

    if (delta > BALANCE_THRESHOLD_UV) {
      // Enable balancing for this cell
      // Timer is in units of 5 minutes, so BALANCING_TIMER_MIN / 5
      uint16_t timer = BALANCING_TIMER_MIN / 5;
      if (timer == 0) timer = 1; // Minimum timer value

      error = bcc0->set_cell_balancing(BCC_CID_DEV1, i, true, timer);
      if (error == BCC_STATUS_SUCCESS) {
        debug_printf("  Cell %d: Balancing enabled (%.2f mV above min)\r\n",
                     i + 1, delta / 1000.0f);
        any_balancing = true;
      } else {
        debug_printf("  Cell %d: Failed to enable balancing\r\n", i + 1);
      }
    }
  }

  if (!any_balancing) {
    debug_println("  No cells need balancing");
  }
}

void stop_cell_balancing() {
  debug_println("\n=== Stopping Cell Balancing ===");

  // Disable balancing for all individual cells
  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    bcc0->set_cell_balancing(BCC_CID_DEV1, i, false, 0);
  }

  // Disable the cell balancing feature globally
  bcc0->enable_cell_balancing(BCC_CID_DEV1, false);
}

void load_calibration_data() {
  bcc_cid_t cid = BCC_CID_DEV1;

  if (bcc0->read_fuse_mirror(cid, 0x02, &cal_data.vref_cal) == BCC_STATUS_SUCCESS &&
      bcc0->read_fuse_mirror(cid, 0x03, &cal_data.cell_offset_0) == BCC_STATUS_SUCCESS &&
      bcc0->read_fuse_mirror(cid, 0x04, &cal_data.cell_offset_1) == BCC_STATUS_SUCCESS &&
      bcc0->read_fuse_mirror(cid, 0x05, &cal_data.cell_gain) == BCC_STATUS_SUCCESS) {
    cal_data.loaded = true;
    debug_println("\n=== Calibration Data Loaded ===");
    debug_printf("  VREF Cal:       0x%04X (%d decimal, %d signed)\n",
                  cal_data.vref_cal, cal_data.vref_cal, (int16_t)cal_data.vref_cal);
    debug_printf("  Cell Offset 0:  0x%04X (%d decimal, %d signed)\n",
                  cal_data.cell_offset_0, cal_data.cell_offset_0, (int16_t)cal_data.cell_offset_0);
    debug_printf("  Cell Offset 1:  0x%04X (%d decimal, %d signed)\n",
                  cal_data.cell_offset_1, cal_data.cell_offset_1, (int16_t)cal_data.cell_offset_1);
    debug_printf("  Cell Gain:      0x%04X (%d decimal, %d signed)\n",
                  cal_data.cell_gain, cal_data.cell_gain, (int16_t)cal_data.cell_gain);
    debug_println("=================================\n");
  } else {
    debug_println("WARNING: Failed to load calibration data from fuse mirror");
  }
}

// Apply calibration based on fuse mirror data
// NOTE: Calibration formula is disabled until we decode the NXP format
// This function currently just returns the raw value for comparison
uint32_t apply_calibration(uint32_t raw_voltage_uv, uint8_t cell_index) {
  if (!cal_data.loaded) {
    return raw_voltage_uv;
  }

  // For now, just return raw value - we need to decode the fuse format first
  // The fuse values 0x7C04 (offset 0) seem too large to be simple LSB offsets
  // They might be:
  // - Encoded ADC calibration coefficients (not direct offsets)
  // - Bitfield-packed values
  // - Reference to a lookup table
  // - NXP proprietary format requiring the datasheet

  // TODO: Decode actual NXP calibration formula from datasheet
  return raw_voltage_uv;
}

void dump_calibration_data() {
  debug_println("\n========================================");
  debug_println("BCC Fuse Mirror & EEPROM Dump");
  debug_println("========================================\n");

  for (uint8_t dev = 0; dev < DEVICE_COUNT; dev++) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);

    // Fuse Mirror Dump
    debug_printf("=== CID %d Fuse Mirror ===\n", cid);
    for (uint8_t addr = 0x00; addr <= 0x1F; addr++) {
      uint16_t fuseVal;
      bcc_status_t error = bcc0->read_fuse_mirror(cid, addr, &fuseVal);
      if (error == BCC_STATUS_SUCCESS) {
        debug_printf("0x%02X: 0x%04X\n", addr, fuseVal);
      } else {
        debug_printf("0x%02X: ERROR %d\n", addr, error);
      }
    }
    debug_println();

    // EEPROM Dump (128 bytes, 16 bytes per line)
    debug_printf("=== CID %d EEPROM ===\n", cid);
    for (uint16_t addr = 0x00; addr <= 0x7F; addr++) {
      uint8_t eepromVal;
      bcc_status_t error = bcc0->read_eeprom(cid, (uint8_t)addr, &eepromVal);

      if (addr % 16 == 0) {
        debug_printf("0x%02X: ", addr);
      }

      if (error == BCC_STATUS_SUCCESS) {
        debug_printf("%02X ", eepromVal);
      } else {
        DebugSerial.print("XX ");
      }

      if ((addr + 1) % 16 == 0 || addr == 0x7F) {
        debug_println();
      }
    }
    debug_println();
  }

  debug_println("========================================");
  debug_println("Dump complete");
  debug_println("========================================\n");
}

void update_charging_state() {
  // Measure voltages
  if (!measure_voltages()) {
    debug_println("ERROR: Failed to measure voltages");
    current_state = STATE_ERROR;
    disable_contactor();
    return;
  }

  uint32_t total_voltage = get_total_voltage();
  uint32_t cell_delta = get_max_cell_delta();

  // Print status every measurement
  print_voltages();

  switch (current_state) {
    case STATE_IDLE:
      debug_println("\n=== State: IDLE ===");
      debug_println("Type 'start' to begin charging");
      break;

    case STATE_CHARGING:
      debug_println("\n=== State: CHARGING ===");

      // Check if we've reached target voltage
      if (total_voltage >= target_voltage_uv) {
        debug_printf("Target voltage reached! (%.3f V >= %.3f V)\r\n",
                     total_voltage / 1000000.0f,
                     target_voltage_uv / 1000000.0f);

        // Check if cells need balancing
        if (cell_delta > BALANCE_THRESHOLD_UV) {
          debug_printf("Cell imbalance detected (%.2f mV), switching to balancing\r\n",
                       cell_delta / 1000.0f);
          current_state = STATE_BALANCING;
          disable_contactor();  // Stop charging while balancing
          apply_cell_balancing();
        } else {
          debug_println("Cells are balanced, charging complete!");
          current_state = STATE_COMPLETE;
          disable_contactor();
          stop_cell_balancing();
        }
      } else {
        debug_printf("Charging... (%.3f V / %.3f V)\r\n",
                     total_voltage / 1000000.0f,
                     target_voltage_uv / 1000000.0f);
      }
      break;

    case STATE_BALANCING:
      debug_println("\n=== State: BALANCING ===");

      // Check if balancing is complete
      if (cell_delta <= BALANCE_THRESHOLD_UV) {
        debug_printf("Balancing complete (delta: %.2f mV)\r\n", cell_delta / 1000.0f);
        stop_cell_balancing();

        // Check if we still need to charge
        if (total_voltage < target_voltage_uv) {
          debug_println("Resuming charging...");
          current_state = STATE_CHARGING;
          enable_contactor();
        } else {
          debug_println("Charging complete!");
          current_state = STATE_COMPLETE;
        }
      } else {
        debug_printf("Balancing... (delta: %.2f mV)\r\n", cell_delta / 1000.0f);
      }
      break;

    case STATE_COMPLETE:
      debug_println("\n=== State: COMPLETE ===");
      debug_printf("Battery charged to %.3f V\r\n", total_voltage / 1000000.0f);
      debug_printf("Cell delta: %.2f mV\r\n", cell_delta / 1000.0f);
      debug_println("Type 'start' to charge again or 'end' for low power mode");
      break;

    case STATE_SLEEP:
      debug_println("\n=== State: SLEEP ===");
      debug_println("BCC in low power mode. Reset device to wake up.");
      break;

    case STATE_ERROR:
      debug_println("\n=== State: ERROR ===");
      debug_println("System in error state. Reset to recover.");
      break;

    default:
      break;
  }
}

void setup() {
  // delay(5000);

  // Initialize Serial
  DebugSerial.begin(115200);
  debug_println("\n\n========================================");
  debug_println("  Simple Single-Device Battery Charger");
  debug_println("========================================");
  debug_println("Target: 21V (3.5V per cell)");
  debug_println("Balance threshold: 10mV");
  debug_println("========================================\n");

  // Initialize contactor control pins
  pinMode(HV_CONTACTOR_2_PIN, OUTPUT);
  pinMode(HV_CONTACTOR_NSLEEP_PIN, OUTPUT);
  pinMode(HV_CONTACTOR_FAULT_PIN, INPUT);

  // Ensure contactors are off
  digitalWrite(HV_CONTACTOR_2_PIN, LOW);
  digitalWrite(HV_CONTACTOR_NSLEEP_PIN, LOW);

  // Initialize PWM for contactor 1
  PinName pinName = digitalPinToPinName(HV_CONTACTOR_1_PIN);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pinName, PinMap_PWM);

  if (Instance != nullptr) {
    contactor_channel = STM_PIN_CHANNEL(pinmap_function(pinName, PinMap_PWM));
    contactor_timer = new HardwareTimer(Instance);
    contactor_timer->setMode(contactor_channel, TIMER_OUTPUT_COMPARE_PWM1, HV_CONTACTOR_1_PIN);
    contactor_timer->setOverflow(CONTACTOR_PWM_FREQ, HERTZ_FORMAT);
    contactor_timer->setCaptureCompare(contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    contactor_timer->pause(); // Start paused
    debug_println("Contactor control initialized with PWM economizer");
  } else {
    debug_println("ERROR: Failed to initialize contactor PWM!");
  }
  debug_println();

  // Create monitoring task
  BaseType_t result = xTaskCreate(
    monitor_task,
    "Monitor",
    2048,
    NULL,
    2,  // Higher priority for monitoring
    &monitor_task_handle
  );

  if (result != pdPASS) {
    debug_println("ERROR: Failed to create monitor task!");
    while (1) delay(1000);
  }

  // Create console task
  result = xTaskCreate(
    console_task,
    "Console",
    2048,
    NULL,
    1,  // Lower priority for console
    &console_task_handle
  );

  if (result != pdPASS) {
    debug_println("ERROR: Failed to create console task!");
    while (1) delay(1000);
  }

  debug_println("Starting FreeRTOS scheduler...");
  debug_println("========================================\n");

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never reach here
  debug_println("ERROR: Scheduler failed to start!");
  while (1);
}

void loop() {
  // Empty - FreeRTOS tasks run instead
}