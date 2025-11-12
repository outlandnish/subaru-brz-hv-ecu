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

#include "Arduino.h"
#include "SPI.h"
#include "TPLSPI.h"
#include "BatteryCellController.h"
#include "bcc/bcc_config.h"
#include "hal/dma_config.h"
#include "hal/hv-ecu-v0-pins.h"
#include <STM32FreeRTOS.h>
#include <HardwareTimer.h>

#define Serial SerialUSB

// Configuration
#define DEVICE_COUNT 1          // Single device on the chain
#define CELL_COUNT 6            // 6S battery
#define TARGET_VOLTAGE_UV 21000000  // 21V in microvolts (3.5V per cell)
#define BALANCE_THRESHOLD_UV 10000  // 10mV in microvolts
#define MEASUREMENT_INTERVAL_MS 1000  // Measure every 1 second
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

// State variables
ChargeState current_state = STATE_INIT;
uint32_t cell_voltages_uv[CELL_COUNT];
uint32_t last_measurement_time = 0;
bool contactor_enabled = false;
bool hardware_initialized = false;
uint32_t last_contactor_cycle_time = 0;  // Track when we last cycled contactors

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
void print_voltages();
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
    Serial.println("Enabling contactor (charging ON)");

    // Enable the H-bridge driver
    digitalWrite(CONTACTOR_NSLEEP_PIN, HIGH);
    delay(10);

    // Set contactor 2 LOW (constant)
    digitalWrite(CONTACTOR_2_PIN, LOW);

    // Start PWM on contactor 1 at 100% to engage (pull-in)
    contactor_timer->setCaptureCompare(contactor_channel, CONTACTOR_ENGAGE_DUTY, PERCENT_COMPARE_FORMAT);
    contactor_timer->resume();

    // Wait for contactor to engage
    delay(CONTACTOR_ENGAGE_TIME_MS);

    // Reduce to hold duty cycle (economizer mode)
    contactor_timer->setCaptureCompare(contactor_channel, CONTACTOR_HOLD_DUTY, PERCENT_COMPARE_FORMAT);
    Serial.printf("Contactor holding at %d%% duty cycle\r\n", CONTACTOR_HOLD_DUTY);

    contactor_enabled = true;
    last_contactor_cycle_time = millis();  // Reset cycle timer when enabling
  }
}

void cycle_contactor() {
  Serial.println("\n=== Cycling Contactor (Resetting Charger) ===");

  // Disable contactor
  contactor_timer->pause();
  contactor_timer->setCaptureCompare(contactor_channel, 0, PERCENT_COMPARE_FORMAT);
  digitalWrite(CONTACTOR_2_PIN, LOW);
  delay(10);
  digitalWrite(CONTACTOR_NSLEEP_PIN, LOW);

  // Pause for 1 second
  Serial.println("Pausing for 1 second...");
  delay(CONTACTOR_CYCLE_PAUSE_MS);

  // Re-enable contactor
  Serial.println("Re-enabling contactor");
  digitalWrite(CONTACTOR_NSLEEP_PIN, HIGH);
  delay(10);
  digitalWrite(CONTACTOR_2_PIN, LOW);

  // Start PWM at 100% to engage
  contactor_timer->setCaptureCompare(contactor_channel, CONTACTOR_ENGAGE_DUTY, PERCENT_COMPARE_FORMAT);
  contactor_timer->resume();
  delay(CONTACTOR_ENGAGE_TIME_MS);

  // Reduce to hold duty cycle
  contactor_timer->setCaptureCompare(contactor_channel, CONTACTOR_HOLD_DUTY, PERCENT_COMPARE_FORMAT);
  Serial.printf("Contactor re-engaged at %d%% duty cycle\r\n", CONTACTOR_HOLD_DUTY);

  last_contactor_cycle_time = millis();
}

void disable_contactor() {
  if (contactor_enabled) {
    Serial.println("Disabling contactor (charging OFF)");

    // Stop PWM
    contactor_timer->pause();
    contactor_timer->setCaptureCompare(contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    digitalWrite(CONTACTOR_2_PIN, LOW);
    delay(10);

    // Disable the H-bridge driver
    digitalWrite(CONTACTOR_NSLEEP_PIN, LOW);

    contactor_enabled = false;
  }
}

void monitor_task(void *pvParameters) {
  // Hardware initialization in task context (like BMS)
  Serial.println("\n=== Initializing BCC0 ===");
  vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for system to stabilize

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

  // Initialize BCC hardware
  bcc_status_t error = bcc0->begin(nullptr);

  if (error == BCC_STATUS_SUCCESS) {
    Serial.println("BCC0: Ready");
    hardware_initialized = true;
    current_state = STATE_IDLE;

    // Load calibration data from fuse mirror
    load_calibration_data();

    // Note: Full calibration dump available via 'dump' command
  } else {
    Serial.printf("BCC0: Init failed (error %d)\r\n", error);
    current_state = STATE_ERROR;
  }

  // Main monitoring loop
  while (true) {
    if (hardware_initialized && current_state != STATE_ERROR && current_state != STATE_SLEEP) {
      // Always measure voltages (like BMS does)
      if (measure_voltages()) {
        // Update charging state if we're actively charging
        if (current_state != STATE_IDLE && current_state != STATE_COMPLETE) {
          uint32_t now = millis();
          if (now - last_measurement_time >= MEASUREMENT_INTERVAL_MS) {
            last_measurement_time = now;
            update_charging_state();
          }
        }

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
      }
    }

    vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
  }
}

void console_task(void *pvParameters) {
  Serial.println("\n=== Simple Charger Console ===");
  Serial.println("Commands: 'start', 'stop', 'end', 'status', 'dump', 'reset'");
  Serial.println();

  String inputBuffer = "";

  while (true) {
    // Check for available serial data
    while (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          inputBuffer.trim();
          inputBuffer.toLowerCase();

          if (inputBuffer == "start") {
            if (hardware_initialized && (current_state == STATE_IDLE || current_state == STATE_COMPLETE)) {
              Serial.println("\nStarting charging sequence...");
              current_state = STATE_CHARGING;
              enable_contactor();
              last_measurement_time = 0; // Force immediate measurement
            } else if (!hardware_initialized) {
              Serial.println("\nERROR: Hardware not initialized yet");
            } else {
              Serial.println("\nERROR: Cannot start from current state");
            }
          } else if (inputBuffer == "stop") {
            Serial.println("\nStopping charging...");
            disable_contactor();
            stop_cell_balancing();
            current_state = STATE_IDLE;
            last_contactor_cycle_time = 0;  // Reset cycle timer
          } else if (inputBuffer == "end") {
            if (hardware_initialized) {
              Serial.println("\nEntering low power mode...");
              disable_contactor();
              stop_cell_balancing();

              // Put BCC into low power mode
              bcc_status_t error = bcc0->enter_low_power_mode();
              if (error == BCC_STATUS_SUCCESS) {
                Serial.println("BCC0 entered low power mode successfully");
                current_state = STATE_SLEEP;
              } else {
                Serial.printf("ERROR: Failed to enter low power mode (error %d)\r\n", error);
                current_state = STATE_ERROR;
              }
              last_contactor_cycle_time = 0;  // Reset cycle timer
            } else {
              Serial.println("\nERROR: Hardware not initialized yet");
            }
          } else if (inputBuffer == "status") {
            if (hardware_initialized) {
              // Just print the latest voltages - don't trigger new measurement
              print_voltages();
            } else {
              Serial.println("\nERROR: Hardware not initialized yet");
            }
          } else if (inputBuffer == "dump") {
            if (hardware_initialized) {
              Serial.println("\nDumping calibration data...");
              dump_calibration_data();
            } else {
              Serial.println("\nERROR: Hardware not initialized yet");
            }
          } else if (inputBuffer == "reset") {
            Serial.println("\nResetting STM32...");
            Serial.flush();  // Ensure message is sent before reset
            delay(100);
            NVIC_SystemReset();
          } else if (inputBuffer.length() > 0) {
            Serial.printf("\nUnknown command: %s\r\n", inputBuffer.c_str());
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
    Serial.printf("ERROR: Failed to start conversion (%d)\r\n", error);
    return false;
  }

  // Wait for conversion to complete
  // Cell voltage ADC conversion takes approximately 20ms
  // Just wait instead of polling to avoid DMA conflicts
  delay(50);

  // Read cell voltages directly
  error = bcc0->get_cell_voltages(BCC_CID_DEV1, cell_voltages_uv);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("ERROR: Failed to read voltages (%d)\r\n", error);
    return false;
  }

  return true;
}

void print_voltages() {
  Serial.println("\n=== Cell Voltages ===");
  uint32_t total_voltage = 0;
  uint32_t total_voltage_cal = 0;
  uint32_t min_v = cell_voltages_uv[0];
  uint32_t max_v = cell_voltages_uv[0];

  for (uint8_t i = 0; i < CELL_COUNT; i++) {
    uint32_t cal_voltage = apply_calibration(cell_voltages_uv[i], i);

    if (cal_data.loaded) {
      Serial.printf("  Cell %d: %.4f V  (cal: %.4f V)\r\n",
                    i + 1,
                    cell_voltages_uv[i] / 1000000.0f,
                    cal_voltage / 1000000.0f);
    } else {
      Serial.printf("  Cell %d: %.4f V\r\n", i + 1, cell_voltages_uv[i] / 1000000.0f);
    }

    total_voltage += cell_voltages_uv[i];
    total_voltage_cal += cal_voltage;
    if (cell_voltages_uv[i] < min_v) min_v = cell_voltages_uv[i];
    if (cell_voltages_uv[i] > max_v) max_v = cell_voltages_uv[i];
  }

  float delta_mv = (max_v - min_v) / 1000.0f;
  Serial.printf("\nTotal: %.3f V  |  Delta: %.2f mV\r\n",
                total_voltage / 1000000.0f, delta_mv);

  if (cal_data.loaded) {
    Serial.printf("Total (cal): %.3f V\r\n", total_voltage_cal / 1000000.0f);
  }

  Serial.printf("Min: %.4f V  |  Max: %.4f V\r\n",
                min_v / 1000000.0f, max_v / 1000000.0f);
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
  Serial.println("\n=== Enabling Cell Balancing ===");

  // First, enable the cell balancing feature globally
  bcc_status_t error = bcc0->enable_cell_balancing(BCC_CID_DEV1, true);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("ERROR: Failed to enable cell balancing feature (%d)\r\n", error);
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
        Serial.printf("  Cell %d: Balancing enabled (%.2f mV above min)\r\n",
                     i + 1, delta / 1000.0f);
        any_balancing = true;
      } else {
        Serial.printf("  Cell %d: Failed to enable balancing\r\n", i + 1);
      }
    }
  }

  if (!any_balancing) {
    Serial.println("  No cells need balancing");
  }
}

void stop_cell_balancing() {
  Serial.println("\n=== Stopping Cell Balancing ===");

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
    Serial.println("\n=== Calibration Data Loaded ===");
    Serial.printf("  VREF Cal:       0x%04X (%d decimal, %d signed)\n",
                  cal_data.vref_cal, cal_data.vref_cal, (int16_t)cal_data.vref_cal);
    Serial.printf("  Cell Offset 0:  0x%04X (%d decimal, %d signed)\n",
                  cal_data.cell_offset_0, cal_data.cell_offset_0, (int16_t)cal_data.cell_offset_0);
    Serial.printf("  Cell Offset 1:  0x%04X (%d decimal, %d signed)\n",
                  cal_data.cell_offset_1, cal_data.cell_offset_1, (int16_t)cal_data.cell_offset_1);
    Serial.printf("  Cell Gain:      0x%04X (%d decimal, %d signed)\n",
                  cal_data.cell_gain, cal_data.cell_gain, (int16_t)cal_data.cell_gain);
    Serial.println("=================================\n");
  } else {
    Serial.println("WARNING: Failed to load calibration data from fuse mirror");
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
  Serial.println("\n========================================");
  Serial.println("BCC Fuse Mirror & EEPROM Dump");
  Serial.println("========================================\n");

  for (uint8_t dev = 0; dev < DEVICE_COUNT; dev++) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);

    // Fuse Mirror Dump
    Serial.printf("=== CID %d Fuse Mirror ===\n", cid);
    for (uint8_t addr = 0x00; addr <= 0x1F; addr++) {
      uint16_t fuseVal;
      bcc_status_t error = bcc0->read_fuse_mirror(cid, addr, &fuseVal);
      if (error == BCC_STATUS_SUCCESS) {
        Serial.printf("0x%02X: 0x%04X\n", addr, fuseVal);
      } else {
        Serial.printf("0x%02X: ERROR %d\n", addr, error);
      }
    }
    Serial.println();

    // EEPROM Dump (128 bytes, 16 bytes per line)
    Serial.printf("=== CID %d EEPROM ===\n", cid);
    for (uint16_t addr = 0x00; addr <= 0x7F; addr++) {
      uint8_t eepromVal;
      bcc_status_t error = bcc0->read_eeprom(cid, (uint8_t)addr, &eepromVal);

      if (addr % 16 == 0) {
        Serial.printf("0x%02X: ", addr);
      }

      if (error == BCC_STATUS_SUCCESS) {
        Serial.printf("%02X ", eepromVal);
      } else {
        Serial.print("XX ");
      }

      if ((addr + 1) % 16 == 0 || addr == 0x7F) {
        Serial.println();
      }
    }
    Serial.println();
  }

  Serial.println("========================================");
  Serial.println("Dump complete");
  Serial.println("========================================\n");
}

void update_charging_state() {
  // Measure voltages
  if (!measure_voltages()) {
    Serial.println("ERROR: Failed to measure voltages");
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
      Serial.println("\n=== State: IDLE ===");
      Serial.println("Type 'start' to begin charging");
      break;

    case STATE_CHARGING:
      Serial.println("\n=== State: CHARGING ===");

      // Check if we've reached target voltage
      if (total_voltage >= TARGET_VOLTAGE_UV) {
        Serial.printf("Target voltage reached! (%.3f V >= %.3f V)\r\n",
                     total_voltage / 1000000.0f,
                     TARGET_VOLTAGE_UV / 1000000.0f);

        // Check if cells need balancing
        if (cell_delta > BALANCE_THRESHOLD_UV) {
          Serial.printf("Cell imbalance detected (%.2f mV), switching to balancing\r\n",
                       cell_delta / 1000.0f);
          current_state = STATE_BALANCING;
          disable_contactor();  // Stop charging while balancing
          apply_cell_balancing();
        } else {
          Serial.println("Cells are balanced, charging complete!");
          current_state = STATE_COMPLETE;
          disable_contactor();
          stop_cell_balancing();
        }
      } else {
        Serial.printf("Charging... (%.3f V / %.3f V)\r\n",
                     total_voltage / 1000000.0f,
                     TARGET_VOLTAGE_UV / 1000000.0f);
      }
      break;

    case STATE_BALANCING:
      Serial.println("\n=== State: BALANCING ===");

      // Check if balancing is complete
      if (cell_delta <= BALANCE_THRESHOLD_UV) {
        Serial.printf("Balancing complete (delta: %.2f mV)\r\n", cell_delta / 1000.0f);
        stop_cell_balancing();

        // Check if we still need to charge
        if (total_voltage < TARGET_VOLTAGE_UV) {
          Serial.println("Resuming charging...");
          current_state = STATE_CHARGING;
          enable_contactor();
        } else {
          Serial.println("Charging complete!");
          current_state = STATE_COMPLETE;
        }
      } else {
        Serial.printf("Balancing... (delta: %.2f mV)\r\n", cell_delta / 1000.0f);
      }
      break;

    case STATE_COMPLETE:
      Serial.println("\n=== State: COMPLETE ===");
      Serial.printf("Battery charged to %.3f V\r\n", total_voltage / 1000000.0f);
      Serial.printf("Cell delta: %.2f mV\r\n", cell_delta / 1000.0f);
      Serial.println("Type 'start' to charge again or 'end' for low power mode");
      break;

    case STATE_SLEEP:
      Serial.println("\n=== State: SLEEP ===");
      Serial.println("BCC in low power mode. Reset device to wake up.");
      break;

    case STATE_ERROR:
      Serial.println("\n=== State: ERROR ===");
      Serial.println("System in error state. Reset to recover.");
      break;

    default:
      break;
  }
}

void setup() {
  delay(5000);

  // Initialize Serial
  Serial.begin(115200);
  Serial.println("\n\n========================================");
  Serial.println("  Simple Single-Device Battery Charger");
  Serial.println("========================================");
  Serial.println("Target: 21V (3.5V per cell)");
  Serial.println("Balance threshold: 10mV");
  Serial.println("========================================\n");

  // Initialize contactor control pins
  pinMode(CONTACTOR_2_PIN, OUTPUT);
  pinMode(CONTACTOR_NSLEEP_PIN, OUTPUT);
  pinMode(CONTACTOR_FAULT_PIN, INPUT);

  // Ensure contactors are off
  digitalWrite(CONTACTOR_2_PIN, LOW);
  digitalWrite(CONTACTOR_NSLEEP_PIN, LOW);

  // Initialize PWM for contactor 1
  PinName pinName = digitalPinToPinName(CONTACTOR_1_PIN);
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pinName, PinMap_PWM);

  if (Instance != nullptr) {
    contactor_channel = STM_PIN_CHANNEL(pinmap_function(pinName, PinMap_PWM));
    contactor_timer = new HardwareTimer(Instance);
    contactor_timer->setMode(contactor_channel, TIMER_OUTPUT_COMPARE_PWM1, CONTACTOR_1_PIN);
    contactor_timer->setOverflow(CONTACTOR_PWM_FREQ, HERTZ_FORMAT);
    contactor_timer->setCaptureCompare(contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    contactor_timer->pause(); // Start paused
    Serial.println("Contactor control initialized with PWM economizer");
  } else {
    Serial.println("ERROR: Failed to initialize contactor PWM!");
  }
  Serial.println();

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
    Serial.println("ERROR: Failed to create monitor task!");
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
    Serial.println("ERROR: Failed to create console task!");
    while (1) delay(1000);
  }

  Serial.println("Starting FreeRTOS scheduler...");
  Serial.println("========================================\n");

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // Should never reach here
  Serial.println("ERROR: Scheduler failed to start!");
  while (1);
}

void loop() {
  // Empty - FreeRTOS tasks run instead
}
