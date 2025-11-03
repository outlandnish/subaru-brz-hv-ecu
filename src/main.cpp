/*
 * BMS Charging System for 6S2P Battery Packs
 *
 * Main application entry point and serial console interface.
 * See README.md for complete documentation, commands, and LED status indicators.
 */

#include "main.h"
#include <STM32FreeRTOS.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(STATUS_LED_COUNT, STATUS_LEDS, NEO_GRB + NEO_KHZ800);

// CAN bus instances
CANBus *ipc_can = nullptr;  // IPC CAN for PCS control (500kbps)
CANBus *m3_can = nullptr;   // M3 CAN for external communication
CANBus *hv_can = nullptr;   // HV CAN for IVT shunt and vehicle comms (500kbps)

// EVSE, PCS, and IVT controllers
EVSEController *evse = nullptr;
TeslaM3PCSController *pcs = nullptr;
IVTShunt *ivt_shunt = nullptr;

#define DEVICE_COUNT 1
#define CELL_COUNT 6

// BMS Configuration for BCC0 (single 6S2P pack)
BatteryCellControllerConfig bcc0_config = {
  .device_count = DEVICE_COUNT,
  .cell_count = CELL_COUNT,
  .enable_pin = BCC0_ENABLE,
  .intb_pin = BCC0_INTB,
  .cs_pin = BCC0_TX_CS,
  .loopback = false
};

BatteryCellControllerConfig bcc1_config = {
  .device_count = DEVICE_COUNT,
  .cell_count = CELL_COUNT,
  .enable_pin = BCC1_ENABLE,
  .intb_pin = BCC1_INTB,
  .cs_pin = BCC1_TX_CS,
  .loopback = false
};

// Charging Configuration
BMSChargingConfig charging_config = {
  .target_cell_voltage = 3.6f,      // Target 3.6V per cell
  .balance_threshold_mv = 50.0f,    // Start balancing when cells differ by 50mV
  .balance_target_mv = 10.0f,       // Resume charging when cells differ by <10mV
  .balancing_timer_min = 5,         // Balance for 5 minutes at a time
  .measurement_interval_ms = 20     // Measure voltages every 20ms (50 Hz)
};

BatteryManagementSystem *bms;

// Function to jump directly to STM32 bootloader
void jump_to_bootloader() {
  typedef void (*pFunction)(void);
  pFunction JumpToBootloader;
  uint32_t bootloader_addr = 0x1FFF0000;  // STM32F413VH bootloader address

  // Disable all interrupts first
  __disable_irq();

  // Stop FreeRTOS scheduler if running
  vTaskSuspendAll();

  // Disable SysTick
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL = 0;

  // Clear all pending interrupts
  for (uint8_t i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;  // Disable all interrupts
    NVIC->ICPR[i] = 0xFFFFFFFF;  // Clear all pending flags
  }

  // Disable and deinitialize USB peripheral
  #ifdef USBCON
  // Disable USB peripheral registers
  USB_OTG_FS->GCCFG = 0;
  USB_OTG_FS->GOTGCTL = 0;

  // Disable USB clock
  __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
  #endif

  // Reset all peripherals to default state
  __HAL_RCC_APB1_FORCE_RESET();
  __HAL_RCC_APB1_RELEASE_RESET();
  __HAL_RCC_APB2_FORCE_RESET();
  __HAL_RCC_APB2_RELEASE_RESET();
  __HAL_RCC_AHB1_FORCE_RESET();
  __HAL_RCC_AHB1_RELEASE_RESET();

  // Deinitialize HAL
  HAL_DeInit();

  // Reset clock to default HSI
  HAL_RCC_DeInit();

  // Remap system memory to 0x00000000
  #if defined(__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH)
  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
  #else
  SYSCFG->MEMRMP = 0x01;  // Map system flash at 0x00000000
  #endif

  // Set stack pointer to bootloader stack
  __set_MSP(*(__IO uint32_t*)bootloader_addr);

  // Get bootloader entry point
  JumpToBootloader = (pFunction)(*(__IO uint32_t*)(bootloader_addr + 4));

  // Jump to bootloader
  JumpToBootloader();

  // Should never reach here
  while(1);
}

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
            Serial.println("faults                - Show fault status");
            Serial.println("config                - Show charging configuration");
            Serial.println("evse                  - Show EVSE status");
            Serial.println("pcs                   - Show PCS status");
            Serial.println("ivt                   - Show IVT shunt status");
            Serial.println("dump                  - Dump all BCC registers and fuse mirror");
            Serial.println("sleep                 - Put BCC into low-power sleep mode");
            Serial.println("wakeup                - Wake BCC from sleep mode");
            Serial.println("reboot                - Software reset (restart application)");
            Serial.println("dfu                   - Enter USB DFU bootloader mode");
            Serial.println("set target <voltage>  - Set target cell voltage (V)");
            Serial.println("set balance_th <mv>   - Set balance threshold (mV)");
            Serial.println("set balance_tgt <mv>  - Set balance target (mV)");
            Serial.println("set interval <ms>     - Set measurement interval (ms)");
            Serial.println();
            Serial.println("=== LED Status Indicators ===");
            Serial.println("State LEDs (0-3):");
            Serial.println("  Purple (solid)      - System initializing");
            Serial.println("  Blue (breathing)    - Idle, ready to charge");
            Serial.println("  Green (chase)       - Charging");
            Serial.println("  Green (solid)       - Charging complete");
            Serial.println("  Orange (pulsing)    - Cell balancing");
            Serial.println("  Red (flashing)      - Error/Fault");
            Serial.println();
            Serial.println("Contactor LED:");
            Serial.println("  LED 4 Yellow        - Contactor enabled");
            Serial.println("  LED 4 Off           - Contactor disabled");
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
            uint32_t voltages_raw[BCC_MAX_CELLS];
            uint32_t voltages_filtered[BCC_MAX_CELLS];
            uint8_t count;
            bms->get_cell_voltages(voltages_raw, &count);
            bms->get_cell_voltages_filtered(voltages_filtered, &count);
            uint32_t stack_v_raw = bms->get_stack_voltage();
            uint32_t stack_v_filtered = bms->get_stack_voltage_filtered();

            Serial.println("\n=== Voltages ===");
            Serial.printf("Stack (filtered): %.3f V  (raw: %.3f V)\r\n",
                         stack_v_filtered / 1000000.0f,
                         stack_v_raw / 1000000.0f);
            Serial.println();

            // Display BCC0 cells (0-5)
            Serial.println("--- Pack 0 (BCC0) ---");
            for (uint8_t i = 0; i < 6 && i < count; i++) {
              Serial.printf("Cell %d: %.4f V  (raw: %.4f V)\r\n",
                           i + 1,
                           voltages_filtered[i] / 1000000.0f,
                           voltages_raw[i] / 1000000.0f);
            }

            // Calculate and show min/max/diff for BCC0
            if (count >= 6) {
              uint32_t min_v = voltages_filtered[0];
              uint32_t max_v = voltages_filtered[0];
              for (uint8_t i = 1; i < 6; i++) {
                if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
              }
              float diff_mv = (max_v - min_v) / 1000.0f;
              Serial.printf("Pack 0 - Min: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
                           min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
            }

            // Display BCC1 cells (6-11) if present
            if (count > 6) {
              Serial.println("\n--- Pack 1 (BCC1) ---");
              for (uint8_t i = 6; i < count; i++) {
                Serial.printf("Cell %d: %.4f V  (raw: %.4f V)\r\n",
                             i - 5,  // Display as Cell 1-6 for BCC1
                             voltages_filtered[i] / 1000000.0f,
                             voltages_raw[i] / 1000000.0f);
              }

              // Calculate and show min/max/diff for BCC1
              uint32_t min_v = voltages_filtered[6];
              uint32_t max_v = voltages_filtered[6];
              for (uint8_t i = 7; i < count; i++) {
                if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
              }
              float diff_mv = (max_v - min_v) / 1000.0f;
              Serial.printf("Pack 1 - Min: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
                           min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
            }

            // Overall statistics
            if (count > 0) {
              uint32_t min_v = voltages_filtered[0];
              uint32_t max_v = voltages_filtered[0];
              for (uint8_t i = 1; i < count; i++) {
                if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
              }
              float diff_mv = (max_v - min_v) / 1000.0f;
              Serial.printf("\nOverall - Min: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
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
          else if (inputBuffer == "evse") {
            Serial.println("\n=== EVSE Status ===");
            if (evse != nullptr) {
              const char* state_str[] = {"State A (No Vehicle)", "State B (Connected)", "State C (Ready/Charging)", "State D (Ventilation)", "State E (No Power)", "Fault"};
              const char* vehicle_str[] = {"Not Ready (State B)", "Ready (State C)", "Ventilation (State D)"};
              Serial.printf("EVSE State: %s\r\n", state_str[evse->get_state()]);
              Serial.printf("Vehicle Signaling: %s\r\n", vehicle_str[evse->get_vehicle_state()]);
              Serial.printf("Cable Limit: %d A\r\n", evse->get_cable_limit());
              Serial.printf("Available Current: %d mA\r\n", evse->get_available_current_ma());
              Serial.printf("Max Charge Current: %d mA\r\n", evse->get_max_charge_current_ma());
              Serial.printf("Connected: %s\r\n", evse->is_connected() ? "Yes" : "No");
              Serial.printf("Ready to Charge: %s\r\n", evse->is_ready_to_charge() ? "Yes" : "No");
            } else {
              Serial.println("EVSE not configured");
            }
            Serial.println();
          }
          else if (inputBuffer == "pcs") {
            Serial.println("\n=== PCS Status ===");
            if (pcs != nullptr) {
              const char* state_str[] = {"Init", "Standby", "Charge Prep", "Charging", "Charge Stop", "DCDC Active", "Fault"};
              Serial.printf("State: %s\r\n", state_str[pcs->get_state()]);
              Serial.printf("Mode: 0x%02X\r\n", pcs->get_mode());
              Serial.printf("Target Voltage: %d mV\r\n", pcs->get_target_voltage_mv());
              Serial.printf("Charge Power: %d W\r\n", pcs->get_charge_power_w());
              Serial.printf("Max Charge Power: %d W\r\n", pcs->get_max_charge_power_w());
              Serial.printf("Enabled: %s\r\n", pcs->is_enabled() ? "Yes" : "No");
              Serial.printf("Charging: %s\r\n", pcs->is_charging() ? "Yes" : "No");
              Serial.printf("DCDC Active: %s\r\n", pcs->is_dcdc_active() ? "Yes" : "No");
              Serial.printf("HV Requested: %s\r\n", pcs->is_hv_requested() ? "Yes" : "No");
            } else {
              Serial.println("PCS not configured");
            }
            Serial.println();
          }
          else if (inputBuffer == "ivt") {
            Serial.println("\n=== IVT Shunt Status ===");
            if (ivt_shunt != nullptr) {
              Serial.printf("Status: %s\r\n", ivt_shunt->is_alive() ? "Online" : "OFFLINE");
              Serial.printf("Current: %.2f A\r\n", ivt_shunt->get_current());
              Serial.printf("Voltage: %.2f V\r\n", ivt_shunt->get_voltage());
              Serial.printf("Voltage 2: %.2f V\r\n", ivt_shunt->get_voltage2());
              Serial.printf("Voltage 3: %.2f V\r\n", ivt_shunt->get_voltage3());
              Serial.printf("Power: %.2f kW\r\n", ivt_shunt->get_power());
              Serial.printf("Temperature: %.1f C\r\n", ivt_shunt->get_temperature());
              Serial.printf("Amp-Hours: %.3f Ah\r\n", ivt_shunt->get_amp_hours());
              Serial.printf("Energy: %.3f kWh\r\n", ivt_shunt->get_kilowatt_hours());
              Serial.printf("Frames Received: %lu\r\n", ivt_shunt->get_frame_count());
              Serial.printf("Last Message: %lu ms ago\r\n", millis() - ivt_shunt->get_last_message_time());
            } else {
              Serial.println("IVT shunt not configured");
            }
            Serial.println();
          }
          else if (inputBuffer == "dump") {
            Serial.println();
            bms->dump_registers();
          }
          else if (inputBuffer == "faults") {
            Serial.println();
            bms->print_fault_status();
          }
          else if (inputBuffer == "sleep") {
            Serial.println("\nPutting BCC into sleep mode...");
            BMS_State result = bms->enable_sleep_mode();
            if (result == BMS_Sleep) {
              Serial.println("BCC successfully entered sleep mode.");
              Serial.println("Note: Use 'wakeup' command to resume operation.");
            } else {
              Serial.println("Failed to enter sleep mode.");
            }
            Serial.println();
          }
          else if (inputBuffer == "wakeup") {
            Serial.println("\nWaking up BCC from sleep mode...");
            // The BCC wake_up() function is called during initialization
            // We need to re-initialize the BCC after wakeup
            Serial.println("Re-initializing BCC...");
            // Note: This requires access to the BCC object and device configuration
            // For now, recommend using 'reboot' command for full system restart
            Serial.println("Note: For full functionality, use 'reboot' command.");
            Serial.println("BCC wakeup sequence requires full re-initialization.");
            Serial.println();
          }
          else if (inputBuffer == "reboot") {
            Serial.println("\n=== System Reboot ===");

            // Safety shutdown sequence (same as DFU)
            Serial.println("Stopping charging...");
            bms->stop_charging();
            delay(100);

            Serial.println("Putting BCC into sleep mode...");
            bms->enable_sleep_mode();
            delay(100);

            Serial.println("Performing software reset...");
            Serial.flush();
            delay(100);

            // Perform software reset using NVIC
            NVIC_SystemReset();

            // Should never reach here
            while(1);
          }
          else if (inputBuffer == "dfu") {
            Serial.println("\n=== Entering USB DFU Mode ===");

            // Safety shutdown sequence
            Serial.println("Stopping charging...");
            bms->stop_charging();
            delay(100);

            Serial.println("Putting BCC into sleep mode...");
            bms->enable_sleep_mode();
            delay(100);

            Serial.println("Jumping to bootloader...");
            Serial.flush();
            delay(100);

            // Jump directly to bootloader (no reset needed)
            jump_to_bootloader();

            // Should never reach here
            while(1);
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
  Serial.println("=== BMS Charging System for Dual 6S2P Packs ===");
  Serial.println();

  // Initialize NeoPixel strip
  Serial.println("Initializing status LEDs...");
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Initialize CAN buses
  Serial.println("Initializing CAN buses...");
  ipc_can = new CANBus(IPC_CAN_RX, IPC_CAN_TX);
  if (!ipc_can->begin(CAN_BPS_500K)) {  // 500kbps for IPC CAN
    Serial.println("ERROR: Failed to initialize IPC CAN!");
  } else {
    Serial.println("IPC CAN initialized at 500kbps");
  }

  m3_can = new CANBus(M3_CAN_RX, M3_CAN_TX, M3_CAN_TERM);
  if (!m3_can->begin(CAN_BPS_500K)) {  // 500kbps for M3 CAN
    Serial.println("ERROR: Failed to initialize M3 CAN!");
  } else {
    Serial.println("M3 CAN initialized at 500kbps");
    m3_can->setTermination(true);  // Enable termination resistor
  }

  hv_can = new CANBus(HV_CAN_RX, HV_CAN_TX, HV_CAN_TERM);
  if (!hv_can->begin(CAN_BPS_500K)) {  // 500kbps for HV CAN
    Serial.println("ERROR: Failed to initialize HV CAN!");
  } else {
    Serial.println("HV CAN initialized at 500kbps");
    hv_can->setTermination(true);  // Enable termination resistor
  }
  Serial.println();

  // Initialize EVSE controller
  Serial.println("Initializing EVSE controller...");
  evse = new EVSEController(PROXIMITY_PILOT_INPUT, CONTROL_PILOT_INPUT, CONTROL_PILOT_OUTPUT);
  evse->begin();
  Serial.println("EVSE controller initialized");
  Serial.println();

  // Initialize PCS controller
  Serial.println("Initializing PCS controller...");
  pcs = new TeslaM3PCSController(PCS_ENABLE_CONTROL, PCS_CHARGE_CONTROL, PCS_DCDC_CONTROL);
  pcs->begin(ipc_can, m3_can);
  Serial.println("PCS controller initialized");
  Serial.println();

  // Initialize IVT current shunt
  Serial.println("Initializing IVT current shunt...");
  ivt_shunt = new IVTShunt();
  ivt_shunt->begin(hv_can);
  Serial.println("IVT shunt initialized");
  Serial.println();

  // Create BMS instance with both BCC0 and BCC1 enabled
  Serial.println("Creating BMS instance for dual pack monitoring...");
  bms = new BatteryManagementSystem(&bcc0_config, &bcc1_config);

  // Set status LEDs
  bms->set_status_leds(&strip);

  // Configure charging parameters
  Serial.println("Configuring charging parameters:");
  Serial.printf("  Target cell voltage: %.2f V\r\n", charging_config.target_cell_voltage);
  Serial.printf("  Balance threshold: %.1f mV\r\n", charging_config.balance_threshold_mv);
  Serial.printf("  Balance target: %.1f mV\r\n", charging_config.balance_target_mv);
  Serial.printf("  Measurement interval: %d ms (%.1f Hz)\r\n",
                charging_config.measurement_interval_ms,
                1000.0f / charging_config.measurement_interval_ms);
  Serial.println();

  bms->set_charging_config(charging_config);

  // Configure EVSE, PCS, and IVT shunt
  Serial.println("Configuring EVSE, PCS, and IVT shunt with BMS...");
  bms->set_evse(evse);
  bms->set_pcs(pcs);
  bms->set_ivt_shunt(ivt_shunt);
  bms->set_can_buses(ipc_can, m3_can, hv_can);

  // Configure contactor control pins
  Serial.println("Configuring contactor control...");
  bms->set_contactor_pins(
    CONTACTOR_1_PIN,
    CONTACTOR_2_PIN,
    CONTACTOR_NSLEEP_PIN,
    CONTACTOR_FAULT_PIN
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