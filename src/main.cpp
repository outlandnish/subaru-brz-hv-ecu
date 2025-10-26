/*
 * BMS Charging System for 6S2P Battery Packs
 *
 * Main application entry point and serial console interface.
 * See README.md for complete documentation, commands, and LED status indicators.
 */

#include "main.h"
#include "bms.h"
#include "dma_config.h"
#include <STM32FreeRTOS.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(STATUS_LED_COUNT, STATUS_LEDS, NEO_GRB + NEO_KHZ800);

#define DEVICE_COUNT 1
#define CELL_COUNT 6

// BMS Configuration for BCC0 (single 6S2P pack)
BatteryCellControllerConfig bcc0_config = {
  .device_count = DEVICE_COUNT,
  .cell_count = CELL_COUNT,
  .enable_pin = BMS0_ENABLE,
  .intb_pin = BMS0_INTB,
  .cs_pin = BMS0_TX_CS,
  .loopback = false
};

// Charging Configuration
BMSChargingConfig charging_config = {
  .target_cell_voltage = 3.6f,      // Target 3.6V per cell
  .balance_threshold_mv = 50.0f,    // Start balancing when cells differ by 50mV
  .balance_target_mv = 10.0f,       // Resume charging when cells differ by <10mV
  .balancing_timer_min = 5,         // Balance for 5 minutes at a time
  .measurement_interval_ms = 1000   // Measure voltages every second
};

BatteryManagementSystem *bms;

// Serial console task for user commands
void console_task(void *pvParameters) {
  Serial.println("\n=== BMS Serial Console ===");
  Serial.println("Type 'help' for available commands");
  Serial.println();

  String inputBuffer = "";

  while (true) {
    // Check for available serial data
    while (Serial.available() > 0) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          // Process command
          inputBuffer.trim();
          inputBuffer.toLowerCase();

          if (inputBuffer == "help") {
            Serial.println("\n=== Available Commands ===");
            Serial.println("help                  - Show this help message");
            Serial.println("status                - Show current BMS status");
            Serial.println("start                 - Start charging");
            Serial.println("stop                  - Stop charging");
            Serial.println("balance               - Force cell balancing");
            Serial.println("voltages              - Show current cell voltages");
            Serial.println("config                - Show charging configuration");
            Serial.println("dump                  - Dump all BCC registers");
            Serial.println("set target <voltage>  - Set target cell voltage (V)");
            Serial.println("set balance_th <mv>   - Set balance threshold (mV)");
            Serial.println("set balance_tgt <mv>  - Set balance target (mV)");
            Serial.println("set interval <ms>     - Set measurement interval (ms)");
            Serial.println();
            Serial.println("=== LED Status Indicators ===");
            Serial.println("State LEDs (0-2):");
            Serial.println("  Purple (solid)      - System initializing");
            Serial.println("  Blue (breathing)    - Idle, ready to charge");
            Serial.println("  Green (chase)       - Charging");
            Serial.println("  Green (solid)       - Charging complete");
            Serial.println("  Orange (pulsing)    - Cell balancing");
            Serial.println("  Red (flashing)      - Error/Fault");
            Serial.println();
            Serial.println("Contactor LEDs (3-4):");
            Serial.println("  LED 3 Yellow        - Negative contactor closed");
            Serial.println("  LED 4 Cyan          - Positive contactor closed");
            Serial.println("  (Off when open)");
            Serial.println();
          }
          else if (inputBuffer == "status") {
            BMS_State state = bms->get_state();
            const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Cooldown", "Sleep", "Error"};
            Serial.println("\n=== BMS Status ===");
            Serial.printf("State: %s\r\n", state_str[state]);
            Serial.println();
          }
          else if (inputBuffer == "start") {
            Serial.println();
            bms->start_charging();
            Serial.println();
          }
          else if (inputBuffer == "stop") {
            Serial.println();
            bms->stop_charging();
            Serial.println();
          }
          else if (inputBuffer == "balance") {
            Serial.println();
            bms->force_balance_cells();
            Serial.println();
          }
          else if (inputBuffer == "voltages") {
            uint32_t voltages[BCC_MAX_CELLS];
            uint8_t count;
            bms->get_cell_voltages(voltages, &count);
            uint32_t stack_v = bms->get_stack_voltage();

            Serial.println("\n=== Voltages ===");
            Serial.printf("Stack: %.3f V\r\n", stack_v / 1000000.0f);
            Serial.println();
            for (uint8_t i = 0; i < count; i++) {
              float voltage = voltages[i] / 1000000.0f;
              Serial.printf("Cell %d: %.4f V\r\n", i + 1, voltage);
            }

            // Calculate and show min/max/diff
            if (count > 0) {
              uint32_t min_v = voltages[0];
              uint32_t max_v = voltages[0];
              for (uint8_t i = 1; i < count; i++) {
                if (voltages[i] < min_v) min_v = voltages[i];
                if (voltages[i] > max_v) max_v = voltages[i];
              }
              float diff_mv = (max_v - min_v) / 1000.0f;
              Serial.printf("\nMin: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
                           min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
            }
            Serial.println();
          }
          else if (inputBuffer == "config") {
            BMSChargingConfig config = bms->get_charging_config();
            Serial.println("\n=== Charging Configuration ===");
            Serial.printf("Target cell voltage:    %.2f V\r\n", config.target_cell_voltage);
            Serial.printf("Balance threshold:      %.1f mV\r\n", config.balance_threshold_mv);
            Serial.printf("Balance target:         %.1f mV\r\n", config.balance_target_mv);
            Serial.printf("Balancing timer:        %d min\r\n", config.balancing_timer_min);
            Serial.printf("Measurement interval:   %d ms\r\n", config.measurement_interval_ms);
            Serial.println();
          }
          else if (inputBuffer == "dump") {
            Serial.println();
            bms->dump_registers();
          }
          else if (inputBuffer.startsWith("set target ")) {
            String value = inputBuffer.substring(11);
            float voltage = value.toFloat();
            if (voltage >= 2.5 && voltage <= 4.2) {
              charging_config.target_cell_voltage = voltage;
              bms->set_charging_config(charging_config);
              Serial.printf("\nTarget voltage set to %.2f V\r\n\n", voltage);
            } else {
              Serial.println("\nError: Voltage must be between 2.5V and 4.2V\n");
            }
          }
          else if (inputBuffer.startsWith("set balance_th ")) {
            String value = inputBuffer.substring(15);
            float threshold = value.toFloat();
            if (threshold >= 1.0 && threshold <= 500.0) {
              charging_config.balance_threshold_mv = threshold;
              bms->set_charging_config(charging_config);
              Serial.printf("\nBalance threshold set to %.1f mV\r\n\n", threshold);
            } else {
              Serial.println("\nError: Threshold must be between 1.0 and 500.0 mV\n");
            }
          }
          else if (inputBuffer.startsWith("set balance_tgt ")) {
            String value = inputBuffer.substring(16);
            float target = value.toFloat();
            if (target >= 1.0 && target <= 100.0) {
              charging_config.balance_target_mv = target;
              bms->set_charging_config(charging_config);
              Serial.printf("\nBalance target set to %.1f mV\r\n\n", target);
            } else {
              Serial.println("\nError: Target must be between 1.0 and 100.0 mV\n");
            }
          }
          else if (inputBuffer.startsWith("set interval ")) {
            String value = inputBuffer.substring(13);
            uint16_t interval = value.toInt();
            if (interval >= 100 && interval <= 10000) {
              charging_config.measurement_interval_ms = interval;
              bms->set_charging_config(charging_config);
              Serial.printf("\nMeasurement interval set to %d ms\r\n\n", interval);
            } else {
              Serial.println("\nError: Interval must be between 100 and 10000 ms\n");
            }
          }
          else {
            Serial.printf("\nUnknown command: %s\r\n", inputBuffer.c_str());
            Serial.println("Type 'help' for available commands\n");
          }

          inputBuffer = "";
        }
      } else {
        inputBuffer += c;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Check serial every 50ms
  }
}

void setup() {
  delay(5000);
  // Initialize Serial FIRST
  Serial.begin(115200);
  Serial.println("=== BMS Charging System for 6S2P Pack ===");
  Serial.println();

  // Initialize NeoPixel strip
  Serial.println("Initializing status LEDs...");
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Create BMS instance (BCC1 disabled - only using BCC0 for single pack)
  Serial.println("Creating BMS instance...");
  bms = new BatteryManagementSystem(&bcc0_config, nullptr);

  // Set status LEDs
  bms->set_status_leds(&strip);

  // Configure charging parameters
  Serial.println("Configuring charging parameters:");
  Serial.printf("  Target cell voltage: %.2f V\r\n", charging_config.target_cell_voltage);
  Serial.printf("  Balance threshold: %.1f mV\r\n", charging_config.balance_threshold_mv);
  Serial.printf("  Balance target: %.1f mV\r\n", charging_config.balance_target_mv);
  Serial.printf("  Measurement interval: %d ms\r\n", charging_config.measurement_interval_ms);
  Serial.println();

  bms->set_charging_config(charging_config);

  // Configure contactor control pins
  Serial.println("Configuring contactor control...");
  bms->set_contactor_pins(
    NEGATIVE_CONTACTOR_CONTROL,
    POSITIVE_CONTACTOR_CONTROL,
    CONTACTOR_CONTROL_ENABLE,
    CONTACTOR_CONTROL_FAULT
  );

  // Initialize BMS (BCC hardware initialization)
  Serial.println("Initializing BMS hardware...");
  if (!bms->initialize(nullptr)) {
    Serial.println("ERROR: BMS initialization failed!");
    Serial.println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("BMS initialized successfully!");
  Serial.println();

  // Start BMS tasks
  Serial.println("Starting BMS tasks...");
  if (!bms->start_tasks()) {
    Serial.println("ERROR: Failed to start BMS tasks!");
    Serial.println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("BMS tasks started successfully!");
  Serial.println();

  // Create console task for user interaction
  Serial.println("Starting serial console...");
  BaseType_t result = xTaskCreate(
    console_task,
    "Console",
    2048,
    NULL,
    1,  // Lower priority than BMS tasks
    NULL
  );

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create console task!");
    Serial.println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println();
  Serial.println("===========================================");
  Serial.println("System ready!");
  Serial.println("Type 'help' for available commands");
  Serial.println("Type 'start' to begin charging");
  Serial.println("===========================================");
  Serial.println();

  // Start the FreeRTOS scheduler
  Serial.println("Starting FreeRTOS scheduler...");
  vTaskStartScheduler();

  // Should never reach here
  Serial.println("ERROR: Scheduler failed to start!");
  while (1);
}

void loop() {
  // Empty - FreeRTOS tasks run instead
}
