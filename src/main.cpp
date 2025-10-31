// /*
//  * BMS Charging System for 6S2P Battery Packs
//  *
//  * Main application entry point and serial console interface.
//  * See README.md for complete documentation, commands, and LED status indicators.
//  */

// #include "main.h"
// #include "bms.h"
// #include "dma_config.h"
// #include <STM32FreeRTOS.h>
// #include <Adafruit_NeoPixel.h>

// Adafruit_NeoPixel strip = Adafruit_NeoPixel(STATUS_LED_COUNT, STATUS_LEDS, NEO_GRB + NEO_KHZ800);

// #define DEVICE_COUNT 1
// #define CELL_COUNT 6

// // BMS Configuration for BCC0 (single 6S2P pack)
// BatteryCellControllerConfig bcc0_config = {
//   .device_count = DEVICE_COUNT,
//   .cell_count = CELL_COUNT,
//   .enable_pin = BMS0_ENABLE,
//   .intb_pin = BMS0_INTB,
//   .cs_pin = BMS0_TX_CS,
//   .loopback = false
// };

// // Charging Configuration
// BMSChargingConfig charging_config = {
//   .target_cell_voltage = 3.6f,      // Target 3.6V per cell
//   .balance_threshold_mv = 50.0f,    // Start balancing when cells differ by 50mV
//   .balance_target_mv = 10.0f,       // Resume charging when cells differ by <10mV
//   .balancing_timer_min = 5,         // Balance for 5 minutes at a time
//   .measurement_interval_ms = 20     // Measure voltages every 20ms (50 Hz)
// };

// BatteryManagementSystem *bms;

// // Function to jump directly to STM32 bootloader
// void jump_to_bootloader() {
//   typedef void (*pFunction)(void);
//   pFunction JumpToBootloader;
//   uint32_t bootloader_addr = 0x1FFF0000;  // STM32F413VH bootloader address

//   // Disable all interrupts first
//   __disable_irq();

//   // Stop FreeRTOS scheduler if running
//   vTaskSuspendAll();

//   // Disable SysTick
//   SysTick->CTRL = 0;
//   SysTick->LOAD = 0;
//   SysTick->VAL = 0;

//   // Clear all pending interrupts
//   for (uint8_t i = 0; i < 8; i++) {
//     NVIC->ICER[i] = 0xFFFFFFFF;  // Disable all interrupts
//     NVIC->ICPR[i] = 0xFFFFFFFF;  // Clear all pending flags
//   }

//   // Disable and deinitialize USB peripheral
//   #ifdef USBCON
//   // Disable USB peripheral registers
//   USB_OTG_FS->GCCFG = 0;
//   USB_OTG_FS->GOTGCTL = 0;

//   // Disable USB clock
//   __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
//   #endif

//   // Reset all peripherals to default state
//   __HAL_RCC_APB1_FORCE_RESET();
//   __HAL_RCC_APB1_RELEASE_RESET();
//   __HAL_RCC_APB2_FORCE_RESET();
//   __HAL_RCC_APB2_RELEASE_RESET();
//   __HAL_RCC_AHB1_FORCE_RESET();
//   __HAL_RCC_AHB1_RELEASE_RESET();

//   // Deinitialize HAL
//   HAL_DeInit();

//   // Reset clock to default HSI
//   HAL_RCC_DeInit();

//   // Remap system memory to 0x00000000
//   #if defined(__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH)
//   __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
//   #else
//   SYSCFG->MEMRMP = 0x01;  // Map system flash at 0x00000000
//   #endif

//   // Set stack pointer to bootloader stack
//   __set_MSP(*(__IO uint32_t*)bootloader_addr);

//   // Get bootloader entry point
//   JumpToBootloader = (pFunction)(*(__IO uint32_t*)(bootloader_addr + 4));

//   // Jump to bootloader
//   JumpToBootloader();

//   // Should never reach here
//   while(1);
// }

// // Serial console task for user commands
// void console_task(void *pvParameters) {
//   Serial.println("\n=== BMS Serial Console ===");
//   Serial.println("Type 'help' for available commands");
//   Serial.println();

//   String inputBuffer = "";

//   while (true) {
//     // Check for available serial data
//     while (Serial.available() > 0) {
//       char c = Serial.read();

//       if (c == '\n' || c == '\r') {
//         if (inputBuffer.length() > 0) {
//           // Process command
//           inputBuffer.trim();
//           inputBuffer.toLowerCase();

//           if (inputBuffer == "help") {
//             Serial.println("\n=== Available Commands ===");
//             Serial.println("help                  - Show this help message");
//             Serial.println("status                - Show current BMS status");
//             Serial.println("start                 - Start charging");
//             Serial.println("stop                  - Stop charging");
//             Serial.println("balance               - Force cell balancing");
//             Serial.println("voltages              - Show current cell voltages");
//             Serial.println("faults                - Show fault status");
//             Serial.println("config                - Show charging configuration");
//             Serial.println("dump                  - Dump all BCC registers and fuse mirror");
//             Serial.println("sleep                 - Put BCC into low-power sleep mode");
//             Serial.println("wakeup                - Wake BCC from sleep mode");
//             Serial.println("reboot                - Software reset (restart application)");
//             Serial.println("dfu                   - Enter USB DFU bootloader mode");
//             Serial.println("set target <voltage>  - Set target cell voltage (V)");
//             Serial.println("set balance_th <mv>   - Set balance threshold (mV)");
//             Serial.println("set balance_tgt <mv>  - Set balance target (mV)");
//             Serial.println("set interval <ms>     - Set measurement interval (ms)");
//             Serial.println();
//             Serial.println("=== LED Status Indicators ===");
//             Serial.println("State LEDs (0-3):");
//             Serial.println("  Purple (solid)      - System initializing");
//             Serial.println("  Blue (breathing)    - Idle, ready to charge");
//             Serial.println("  Green (chase)       - Charging");
//             Serial.println("  Green (solid)       - Charging complete");
//             Serial.println("  Orange (pulsing)    - Cell balancing");
//             Serial.println("  Red (flashing)      - Error/Fault");
//             Serial.println();
//             Serial.println("Contactor LED:");
//             Serial.println("  LED 4 Yellow        - Contactor enabled");
//             Serial.println("  LED 4 Off           - Contactor disabled");
//             Serial.println();
//           }
//           else if (inputBuffer == "status") {
//             BMS_State state = bms->get_state();
//             const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Cooldown", "Sleep", "Error"};
//             Serial.println("\n=== BMS Status ===");
//             Serial.printf("State: %s\r\n", state_str[state]);
//             Serial.println();
//           }
//           else if (inputBuffer == "start") {
//             Serial.println();
//             bms->start_charging();
//             Serial.println();
//           }
//           else if (inputBuffer == "stop") {
//             Serial.println();
//             bms->stop_charging();
//             Serial.println();
//           }
//           else if (inputBuffer == "balance") {
//             Serial.println();
//             bms->force_balance_cells();
//             Serial.println();
//           }
//           else if (inputBuffer == "voltages") {
//             uint32_t voltages_raw[BCC_MAX_CELLS];
//             uint32_t voltages_filtered[BCC_MAX_CELLS];
//             uint8_t count;
//             bms->get_cell_voltages(voltages_raw, &count);
//             bms->get_cell_voltages_filtered(voltages_filtered, &count);
//             uint32_t stack_v_raw = bms->get_stack_voltage();
//             uint32_t stack_v_filtered = bms->get_stack_voltage_filtered();

//             Serial.println("\n=== Voltages ===");
//             Serial.printf("Stack (filtered): %.3f V  (raw: %.3f V)\r\n",
//                          stack_v_filtered / 1000000.0f,
//                          stack_v_raw / 1000000.0f);
//             Serial.println();
//             for (uint8_t i = 0; i < count; i++) {
//               Serial.printf("Cell %d: %.4f V  (raw: %.4f V)\r\n",
//                            i + 1,
//                            voltages_filtered[i] / 1000000.0f,
//                            voltages_raw[i] / 1000000.0f);
//             }

//             // Calculate and show min/max/diff using filtered values
//             if (count > 0) {
//               uint32_t min_v = voltages_filtered[0];
//               uint32_t max_v = voltages_filtered[0];
//               for (uint8_t i = 1; i < count; i++) {
//                 if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
//                 if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
//               }
//               float diff_mv = (max_v - min_v) / 1000.0f;
//               Serial.printf("\nFiltered - Min: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
//                            min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
//             }
//             Serial.println();
//           }
//           else if (inputBuffer == "config") {
//             BMSChargingConfig config = bms->get_charging_config();
//             Serial.println("\n=== Charging Configuration ===");
//             Serial.printf("Target cell voltage:    %.2f V\r\n", config.target_cell_voltage);
//             Serial.printf("Balance threshold:      %.1f mV\r\n", config.balance_threshold_mv);
//             Serial.printf("Balance target:         %.1f mV\r\n", config.balance_target_mv);
//             Serial.printf("Balancing timer:        %d min\r\n", config.balancing_timer_min);
//             Serial.printf("Measurement interval:   %d ms\r\n", config.measurement_interval_ms);
//             Serial.println();
//           }
//           else if (inputBuffer == "dump") {
//             Serial.println();
//             bms->dump_registers();
//           }
//           else if (inputBuffer == "faults") {
//             Serial.println();
//             bms->print_fault_status();
//           }
//           else if (inputBuffer == "sleep") {
//             Serial.println("\nPutting BCC into sleep mode...");
//             BMS_State result = bms->enable_sleep_mode();
//             if (result == BMS_Sleep) {
//               Serial.println("BCC successfully entered sleep mode.");
//               Serial.println("Note: Use 'wakeup' command to resume operation.");
//             } else {
//               Serial.println("Failed to enter sleep mode.");
//             }
//             Serial.println();
//           }
//           else if (inputBuffer == "wakeup") {
//             Serial.println("\nWaking up BCC from sleep mode...");
//             // The BCC wake_up() function is called during initialization
//             // We need to re-initialize the BCC after wakeup
//             Serial.println("Re-initializing BCC...");
//             // Note: This requires access to the BCC object and device configuration
//             // For now, recommend using 'reboot' command for full system restart
//             Serial.println("Note: For full functionality, use 'reboot' command.");
//             Serial.println("BCC wakeup sequence requires full re-initialization.");
//             Serial.println();
//           }
//           else if (inputBuffer == "reboot") {
//             Serial.println("\n=== System Reboot ===");

//             // Safety shutdown sequence (same as DFU)
//             Serial.println("Stopping charging...");
//             bms->stop_charging();
//             delay(100);

//             Serial.println("Putting BCC into sleep mode...");
//             bms->enable_sleep_mode();
//             delay(100);

//             Serial.println("Performing software reset...");
//             Serial.flush();
//             delay(100);

//             // Perform software reset using NVIC
//             NVIC_SystemReset();

//             // Should never reach here
//             while(1);
//           }
//           else if (inputBuffer == "dfu") {
//             Serial.println("\n=== Entering USB DFU Mode ===");

//             // Safety shutdown sequence
//             Serial.println("Stopping charging...");
//             bms->stop_charging();
//             delay(100);

//             Serial.println("Putting BCC into sleep mode...");
//             bms->enable_sleep_mode();
//             delay(100);

//             Serial.println("Jumping to bootloader...");
//             Serial.flush();
//             delay(100);

//             // Jump directly to bootloader (no reset needed)
//             jump_to_bootloader();

//             // Should never reach here
//             while(1);
//           }
//           else if (inputBuffer.startsWith("set target ")) {
//             String value = inputBuffer.substring(11);
//             float voltage = value.toFloat();
//             if (voltage >= 2.5 && voltage <= 4.2) {
//               charging_config.target_cell_voltage = voltage;
//               bms->set_charging_config(charging_config);
//               Serial.printf("\nTarget voltage set to %.2f V\r\n\n", voltage);
//             } else {
//               Serial.println("\nError: Voltage must be between 2.5V and 4.2V\n");
//             }
//           }
//           else if (inputBuffer.startsWith("set balance_th ")) {
//             String value = inputBuffer.substring(15);
//             float threshold = value.toFloat();
//             if (threshold >= 1.0 && threshold <= 500.0) {
//               charging_config.balance_threshold_mv = threshold;
//               bms->set_charging_config(charging_config);
//               Serial.printf("\nBalance threshold set to %.1f mV\r\n\n", threshold);
//             } else {
//               Serial.println("\nError: Threshold must be between 1.0 and 500.0 mV\n");
//             }
//           }
//           else if (inputBuffer.startsWith("set balance_tgt ")) {
//             String value = inputBuffer.substring(16);
//             float target = value.toFloat();
//             if (target >= 1.0 && target <= 100.0) {
//               charging_config.balance_target_mv = target;
//               bms->set_charging_config(charging_config);
//               Serial.printf("\nBalance target set to %.1f mV\r\n\n", target);
//             } else {
//               Serial.println("\nError: Target must be between 1.0 and 100.0 mV\n");
//             }
//           }
//           else if (inputBuffer.startsWith("set interval ")) {
//             String value = inputBuffer.substring(13);
//             uint16_t interval = value.toInt();
//             if (interval >= 100 && interval <= 10000) {
//               charging_config.measurement_interval_ms = interval;
//               bms->set_charging_config(charging_config);
//               Serial.printf("\nMeasurement interval set to %d ms\r\n\n", interval);
//             } else {
//               Serial.println("\nError: Interval must be between 100 and 10000 ms\n");
//             }
//           }
//           else {
//             Serial.printf("\nUnknown command: %s\r\n", inputBuffer.c_str());
//             Serial.println("Type 'help' for available commands\n");
//           }

//           inputBuffer = "";
//         }
//       } else {
//         inputBuffer += c;
//       }
//     }

//     vTaskDelay(pdMS_TO_TICKS(50)); // Check serial every 50ms
//   }
// }

// void setup() {
//   delay(5000);
//   // Initialize Serial FIRST
//   Serial.begin(115200);
//   Serial.println("=== BMS Charging System for 6S2P Pack ===");
//   Serial.println();

//   // Initialize NeoPixel strip
//   Serial.println("Initializing status LEDs...");
//   strip.begin();
//   strip.show(); // Initialize all pixels to 'off'

//   // Create BMS instance (BCC1 disabled - only using BCC0 for single pack)
//   Serial.println("Creating BMS instance...");
//   bms = new BatteryManagementSystem(&bcc0_config, nullptr);

//   // Set status LEDs
//   bms->set_status_leds(&strip);

//   // Configure charging parameters
//   Serial.println("Configuring charging parameters:");
//   Serial.printf("  Target cell voltage: %.2f V\r\n", charging_config.target_cell_voltage);
//   Serial.printf("  Balance threshold: %.1f mV\r\n", charging_config.balance_threshold_mv);
//   Serial.printf("  Balance target: %.1f mV\r\n", charging_config.balance_target_mv);
//   Serial.printf("  Measurement interval: %d ms (%.1f Hz)\r\n",
//                 charging_config.measurement_interval_ms,
//                 1000.0f / charging_config.measurement_interval_ms);
//   Serial.println();

//   bms->set_charging_config(charging_config);

//   // Configure contactor control pins
//   Serial.println("Configuring contactor control...");
//   bms->set_contactor_pins(
//     CONTACTOR_PH_PIN,
//     CONTACTOR_EN_PIN,
//     CONTACTOR_NSLEEP_PIN,
//     CONTACTOR_FAULT_PIN
//   );

//   // Initialize BMS (BCC hardware initialization)
//   Serial.println("Initializing BMS hardware...");
//   if (!bms->initialize(nullptr)) {
//     Serial.println("ERROR: BMS initialization failed!");
//     Serial.println("System halted.");
//     while (1) {
//       delay(1000);
//     }
//   }

//   Serial.println("BMS initialized successfully!");
//   Serial.println();

//   // Start BMS tasks
//   Serial.println("Starting BMS tasks...");
//   if (!bms->start_tasks()) {
//     Serial.println("ERROR: Failed to start BMS tasks!");
//     Serial.println("System halted.");
//     while (1) {
//       delay(1000);
//     }
//   }

//   Serial.println("BMS tasks started successfully!");
//   Serial.println();

//   // Create console task for user interaction
//   Serial.println("Starting serial console...");
//   BaseType_t result = xTaskCreate(
//     console_task,
//     "Console",
//     2048,
//     NULL,
//     1,  // Lower priority than BMS tasks
//     NULL
//   );

//   if (result != pdPASS) {
//     Serial.println("ERROR: Failed to create console task!");
//     Serial.println("System halted.");
//     while (1) {
//       delay(1000);
//     }
//   }

//   Serial.println();
//   Serial.println("===========================================");
//   Serial.println("System ready!");
//   Serial.println("Type 'help' for available commands");
//   Serial.println("Type 'start' to begin charging");
//   Serial.println("===========================================");
//   Serial.println();

//   // Start the FreeRTOS scheduler
//   Serial.println("Starting FreeRTOS scheduler...");
//   vTaskStartScheduler();

//   // Should never reach here
//   Serial.println("ERROR: Scheduler failed to start!");
//   while (1);
// }

// void loop() {
//   // Empty - FreeRTOS tasks run instead
// }

#include "Arduino.h"

#define BMS0_TX_SCK PA5
#define BMS0_TX_CS PA6
#define BMS0_TX_DATA PA7
#define BMS0_RX_SCK PA9
#define BMS0_RX_CS PB9
#define BMS0_RX_DATA PA10
#define BMS0_ENABLE PE8
#define BMS0_INTB PE9

#define BMS1_TX_SCK PE2
#define BMS1_TX_CS PE4
#define BMS1_TX_DATA PE6
#define BMS1_RX_SCK PB3_ALT1   // Force SPI3 instead of SPI1
#define BMS1_RX_CS PA4_ALT1    // SPI3_NSS (PA4 defaults to SPI1, need ALT1 for SPI3)
#define BMS1_RX_DATA PB4_ALT1  // Force SPI3 instead of SPI1, todo: update PCB to use PB5 (MOSI not MISO)
#define BMS1_ENABLE PE10
#define BMS1_INTB PE11

uint32_t interruptCount = 0;

void isr_wrapper() {
  interruptCount++;
  // Serial.println("BCC0 INTB Interrupt Triggered!");
}

void setup() {
  pinMode(BMS0_INTB, INPUT_PULLUP);
  pinMode(BMS0_ENABLE, OUTPUT);
  digitalWrite(BMS0_ENABLE, HIGH); // Enable BCC0

  Serial.begin(115200);
  delay(5000);
  Serial.println("Starting BCC0 INTB test...");

  attachInterrupt(BMS0_INTB, isr_wrapper, CHANGE);

  digitalWrite(BMS0_ENABLE, LOW);
  /* Wait at least 100 us. */
  delayMicroseconds(150);
  digitalWrite(BMS0_ENABLE, HIGH);
}

void loop() {
  // Main loop does nothing, just waits for interrupts
  Serial.printf("Interrupt count: %lu\n", interruptCount);
  delay(1000);
}