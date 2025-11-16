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

// IVT controller
IVTShunt *ivt_shunt = nullptr;

// CHAdeMO controller (Foccci on M3/CP CAN)
CHAdeMOController *chademo = nullptr;

// HV CAN monitoring
bool hv_can_monitor_enabled = false;
uint32_t hv_can_frame_count = 0;

// M3 CAN test message generator
bool m3_can_test_enabled = false;
uint32_t m3_can_test_interval_ms = 1000;  // Default 1Hz
uint32_t m3_can_test_count = 0;

// CAN message queues for thread-safe message passing
QueueHandle_t ipc_can_queue = nullptr;
QueueHandle_t m3_can_queue = nullptr;
QueueHandle_t hv_can_queue = nullptr;

#define CAN_QUEUE_LENGTH 32  // Buffer up to 32 messages per bus

// BMS Configuration - will be populated from parameters in setup()
BatteryCellControllerConfig bcc0_config;
BatteryCellControllerConfig bcc1_config;

BatteryManagementSystem *bms;

// CAN RX polling task - reads from hardware and puts messages into queues
void can_rx_task(void *pvParameters) {
  CAN_FRAME frame;
  static uint32_t last_debug = 0;
  static uint32_t ipc_read_count = 0;
  static uint32_t m3_read_count = 0;
  static uint32_t hv_read_count = 0;

  while (true) {
    // Poll IPC CAN bus
    if (ipc_can && ipc_can->available()) {
      while (ipc_can->read(frame)) {
        ipc_read_count++;
        Serial.printf("IPC RX: 0x%03X\r\n", frame.id);
        xQueueSend(ipc_can_queue, &frame, 0);
      }
    }

    // Poll M3 CAN bus
    if (m3_can && m3_can->available()) {
      while (m3_can->read(frame)) {
        m3_read_count++;
        Serial.printf("M3 RX: 0x%03X [%d] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                      frame.id, frame.length,
                      frame.data.uint8[0], frame.data.uint8[1], frame.data.uint8[2], frame.data.uint8[3],
                      frame.data.uint8[4], frame.data.uint8[5], frame.data.uint8[6], frame.data.uint8[7]);
        xQueueSend(m3_can_queue, &frame, 0);
      }
    }

    // Poll HV CAN bus
    if (hv_can && hv_can->available()) {
      while (hv_can->read(frame)) {
        hv_read_count++;
        Serial.printf("HV RX: 0x%03X [%d] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                      frame.id, frame.length,
                      frame.data.uint8[0], frame.data.uint8[1], frame.data.uint8[2], frame.data.uint8[3],
                      frame.data.uint8[4], frame.data.uint8[5], frame.data.uint8[6], frame.data.uint8[7]);

        // Put frame in queue
        xQueueSend(hv_can_queue, &frame, 0);
      }
    }

    // Poll every 1ms (1000 Hz)
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// IVT processing task - processes messages from HV CAN queue
void ivt_process_task(void *pvParameters) {
  CAN_FRAME frame;

  while (true) {
    // TODO: replace with HV can when the hardware is fixed
    // Wait for M3 CAN message (block for up to 10ms)
    if (xQueueReceive(m3_can_queue, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
      // Check if message is for IVT (0x521-0x528 or 0x511)
      if ((frame.id >= 0x521 && frame.id <= 0x528) || frame.id == 0x511) {
        if (ivt_shunt) {
          ivt_shunt->process_can_frame(&frame);
        }
      }
      // Check if message is for CHAdeMO (0x100, 0x102, 0x108, 0x109)
      else if (frame.id == 0x100 || frame.id == 0x102 || frame.id == 0x108 || frame.id == 0x109) {
        if (chademo) {
          chademo->process_can_message(&frame);
        }
      }
    }
  }
}


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
            bool bcc0_ok = bms->is_bcc0_initialized();
            bool bcc1_ok = bms->is_bcc1_initialized();
            bool any_bcc_ok = bcc0_ok || bcc1_ok;

            Serial.println("\n=== Available Commands ===");
            Serial.println("help                  - Show this help message");
            Serial.println("status                - Show current BMS status");
            Serial.println("bcc                   - Show BCC status");
            Serial.println("ivt                   - Show IVT shunt status");
            Serial.println("chademo               - Show CHAdeMO status");
            Serial.println("hv                    - Show HV system status");
            Serial.println("hv can monitor        - Show HV CAN monitoring status");
            Serial.println("hv can monitor on     - Enable HV CAN traffic monitoring");
            Serial.println("hv can monitor off    - Disable HV CAN traffic monitoring");
            Serial.println("m3 can send <id> <b0> <b1> ... <b7>");
            Serial.println("                      - Send CAN message on M3 bus (hex values)");
            Serial.println("m3 can test           - Show test generator status");
            Serial.println("m3 can test on        - Enable periodic test messages (1Hz IVT)");
            Serial.println("m3 can test off       - Disable test messages");
            Serial.println("m3 can test rate <ms> - Set test message interval (ms)");
            Serial.println("reboot                - Software reset (restart application)");
            Serial.println();

            if (any_bcc_ok) {
              Serial.println("=== BCC (Battery Cell Controller) Commands ===");
              Serial.println("bcc start             - Start charging");
              Serial.println("bcc stop              - Stop charging");
              Serial.println("bcc balance           - Force cell balancing");
              Serial.println("bcc voltages [summary]");
              Serial.println("                      - Show module voltage summary");
              Serial.println("bcc voltages <module#>");
              Serial.println("                      - Show detailed voltages for specific module (0-15)");
              Serial.println("bcc faults            - Show fault status");
              Serial.println("bcc config            - Show charging configuration");
              Serial.println("bcc dump              - Dump all BCC registers and fuse mirror");
              Serial.println("bcc sleep             - Put BCC into low-power sleep mode");
              Serial.println("bcc wakeup            - Wake BCC from sleep mode");
              Serial.println("bcc set target <voltage>");
              Serial.println("                      - Set target cell voltage (V)");
              Serial.println("bcc set balance_th <mv>");
              Serial.println("                      - Set balance threshold (mV)");
              Serial.println("bcc set balance_tgt <mv>");
              Serial.println("                      - Set balance target (mV)");
              Serial.println("bcc set interval <ms> - Set measurement interval (ms)");
              Serial.println();
            }

            Serial.println("=== CHAdeMO (Foccci) Commands ===");
            Serial.println("chademo start <volts> <amps>");
            Serial.println("                      - Start CHAdeMO charge session");
            Serial.println("chademo stop          - Stop CHAdeMO charge session");
            Serial.println("chademo limits <v> <a>");
            Serial.println("                      - Set EVSE capabilities (max voltage/current)");
            Serial.println();
          }
          else if (inputBuffer == "status") {
            BMS_State state = bms->get_state();
            const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Cooldown", "Sleep", "Error"};
            Serial.println("\n=== BMS Status ===");
            Serial.printf("State: %s\r\n", state_str[state]);
            Serial.println();
          }
          else if (inputBuffer == "bcc") {
            Serial.println("\n=== BCC Status ===");
            Serial.printf("BCC0: %s\r\n", bms->is_bcc0_initialized() ? "Initialized" : "NOT INITIALIZED");
            if (bms->is_bcc1_enabled()) {
              Serial.printf("BCC1: %s\r\n", bms->is_bcc1_initialized() ? "Initialized" : "NOT INITIALIZED");
            } else {
              Serial.println("BCC1: Disabled");
            }
            Serial.println();
          }
          else if (inputBuffer == "bcc start") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized. Cannot start charging.\n");
            } else {
              Serial.println();
              bms->start_charging();
              Serial.println();
            }
          }
          else if (inputBuffer == "bcc stop") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized.\n");
            } else {
              Serial.println();
              bms->stop_charging();
              Serial.println();
            }
          }
          else if (inputBuffer == "bcc balance") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized. Cannot force balancing.\n");
            } else {
              Serial.println();
              bms->force_balance_cells();
              Serial.println();
            }
          }
          else if (inputBuffer == "bcc voltages" || inputBuffer.startsWith("bcc voltages ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized. Cannot read voltages.\n");
              continue;
            }

            uint32_t voltages_raw[BCC_MAX_CELLS];
            uint32_t voltages_filtered[BCC_MAX_CELLS];
            uint8_t count;
            bms->get_cell_voltages(voltages_raw, &count);
            bms->get_cell_voltages_filtered(voltages_filtered, &count);

            // Parse command - support "bcc voltages", "bcc voltages summary", or "bcc voltages <module#>"
            bool show_summary = (inputBuffer == "bcc voltages" || inputBuffer == "bcc voltages summary");
            int8_t module_num = -1;

            if (inputBuffer.startsWith("bcc voltages ") && inputBuffer != "bcc voltages summary") {
              String arg = inputBuffer.substring(13);
              module_num = arg.toInt();
            }

            Serial.println("\n=== Cell Voltages ===");

            // Display specific module if requested
            if (module_num >= 0) {
              uint8_t bcc0_device_count = bcc0_config.device_count;
              uint8_t cell_count = bcc0_config.cell_count;
              uint8_t bcc_chain = (module_num < bcc0_device_count) ? 0 : 1;
              uint8_t module_in_chain = module_num % bcc0_device_count;
              uint8_t start_cell = module_num * cell_count;
              uint8_t end_cell = start_cell + cell_count;

              if (start_cell >= count) {
                Serial.printf("Error: Module %d does not exist\r\n\n", module_num);
                continue;
              }

              Serial.printf("--- Module %d (BCC%d Device %d) ---\r\n", module_num, bcc_chain, module_in_chain);

              uint32_t module_voltage = 0;
              uint32_t min_v = voltages_filtered[start_cell];
              uint32_t max_v = voltages_filtered[start_cell];

              for (uint8_t i = start_cell; i < end_cell && i < count; i++) {
                Serial.printf("  Cell %d: %.4f V  (raw: %.4f V)\r\n",
                             (i % cell_count) + 1,
                             voltages_filtered[i] / 1000000.0f,
                             voltages_raw[i] / 1000000.0f);
                module_voltage += voltages_filtered[i];
                if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
              }

              float diff_mv = (max_v - min_v) / 1000.0f;
              Serial.printf("\nModule Voltage: %.3f V\r\n", module_voltage / 1000000.0f);
              Serial.printf("Min: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
                           min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
              Serial.println();
            }
            // Display summary for all modules
            else if (show_summary) {
              uint8_t bcc0_device_count = bcc0_config.device_count;
              uint8_t cell_count = bcc0_config.cell_count;
              uint8_t bcc0_modules = bcc0_device_count;
              uint8_t total_modules = bcc0_device_count * (bms->is_bcc1_initialized() ? 2 : 1);

              uint32_t total_voltage = 0;
              uint32_t bcc0_voltage = 0;
              uint32_t bcc1_voltage = 0;

              // BCC0 modules
              if (bms->is_bcc0_initialized()) {
                Serial.println("--- BCC0 Module Summary ---");
                for (uint8_t mod = 0; mod < bcc0_modules; mod++) {
                  uint8_t start_cell = mod * cell_count;
                  uint8_t end_cell = start_cell + cell_count;

                  uint32_t module_voltage = 0;
                  uint32_t min_v = voltages_filtered[start_cell];
                  uint32_t max_v = voltages_filtered[start_cell];

                  for (uint8_t i = start_cell; i < end_cell && i < count; i++) {
                    module_voltage += voltages_filtered[i];
                    if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                    if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
                  }

                  float diff_mv = (max_v - min_v) / 1000.0f;
                  Serial.printf("  Module %d: %.3f V  (Δ %.2f mV)\r\n",
                               mod, module_voltage / 1000000.0f, diff_mv);
                  bcc0_voltage += module_voltage;
                }
                Serial.printf("BCC0 Total: %.3f V\r\n\n", bcc0_voltage / 1000000.0f);
                total_voltage += bcc0_voltage;
              }

              // BCC1 modules
              if (bms->is_bcc1_initialized()) {
                Serial.println("--- BCC1 Module Summary ---");
                for (uint8_t mod = 0; mod < bcc0_modules; mod++) {
                  uint8_t module_num = mod + bcc0_modules;
                  uint8_t start_cell = module_num * cell_count;
                  uint8_t end_cell = start_cell + cell_count;

                  if (start_cell >= count) break;

                  uint32_t module_voltage = 0;
                  uint32_t min_v = voltages_filtered[start_cell];
                  uint32_t max_v = voltages_filtered[start_cell];

                  for (uint8_t i = start_cell; i < end_cell && i < count; i++) {
                    module_voltage += voltages_filtered[i];
                    if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                    if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
                  }

                  float diff_mv = (max_v - min_v) / 1000.0f;
                  Serial.printf("  Module %d: %.3f V  (Δ %.2f mV)\r\n",
                               module_num, module_voltage / 1000000.0f, diff_mv);
                  bcc1_voltage += module_voltage;
                }
                Serial.printf("BCC1 Total: %.3f V\r\n\n", bcc1_voltage / 1000000.0f);
                total_voltage += bcc1_voltage;
              }

              // Overall statistics
              Serial.println("--- Overall Statistics ---");
              Serial.printf("Total Voltage (both chains): %.3f V\r\n", total_voltage / 1000000.0f);
              Serial.printf("Total Cell Count: %d\r\n", count);

              uint32_t min_v = voltages_filtered[0];
              uint32_t max_v = voltages_filtered[0];
              for (uint8_t i = 1; i < count; i++) {
                if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
              }
              float diff_mv = (max_v - min_v) / 1000.0f;
              Serial.printf("Min Cell: %.4f V, Max Cell: %.4f V, Diff: %.2f mV\r\n",
                           min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
              Serial.println();
            }
          }
          else if (inputBuffer == "bcc config") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized.\n");
            } else {
              BMSChargingConfig config = bms->get_charging_config();
              Serial.println("\n=== Charging Configuration ===");
              Serial.printf("Target cell voltage:    %.2f V\r\n", config.target_cell_voltage);
              Serial.printf("Balance threshold:      %.1f mV\r\n", config.balance_threshold_mv);
              Serial.printf("Balance target:         %.1f mV\r\n", config.balance_target_mv);
              Serial.printf("Balancing timer:        %d min\r\n", config.balancing_timer_min);
              Serial.printf("Measurement interval:   %d ms\r\n", config.measurement_interval_ms);
              Serial.println();
            }
          }
          else if (inputBuffer == "ivt") {
            Serial.println("\n=== IVT Shunt Status ===");
            if (ivt_shunt != nullptr) {
              Serial.printf("Status: %s\r\n", ivt_shunt->is_alive() ? "Online" : "OFFLINE");
              Serial.printf("Current: %.2f A\r\n", ivt_shunt->get_current());
              Serial.printf("Voltage 1 (Pack): %.2f V\r\n", ivt_shunt->get_voltage());
              Serial.printf("Voltage 2 (HV Bus): %.2f V\r\n", ivt_shunt->get_voltage2());
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
          else if (inputBuffer == "chademo") {
            Serial.println("\n=== CHAdeMO Status ===");
            if (chademo != nullptr) {
              const char* state_str[] = {"Idle", "Connected", "Precharge", "Charging", "Ending", "Error", "Timeout"};
              Serial.printf("State: %s\r\n", state_str[chademo->get_state()]);
              Serial.printf("Connected: %s\r\n", chademo->is_connected() ? "Yes" : "No");
              Serial.printf("Charging: %s\r\n", chademo->is_charging() ? "Yes" : "No");
              Serial.printf("EVSE Voltage: %d V\r\n", chademo->get_evse_voltage());
              Serial.printf("EVSE Current: %d A\r\n", chademo->get_evse_current());
              Serial.printf("Time Since Last RX: %lu ms\r\n", chademo->get_time_since_last_rx());
              if (chademo->has_timeout()) {
                Serial.println("WARNING: Communication timeout!");
              }
            } else {
              Serial.println("CHAdeMO not configured");
            }
            Serial.println();
          }
          else if (inputBuffer.startsWith("chademo start ")) {
            if (chademo == nullptr) {
              Serial.println("\nError: CHAdeMO not configured\n");
            } else {
              String args = inputBuffer.substring(14);
              int spaceIdx = args.indexOf(' ');
              if (spaceIdx > 0) {
                uint16_t voltage = args.substring(0, spaceIdx).toInt();
                uint16_t current = args.substring(spaceIdx + 1).toInt();
                Serial.printf("\nStarting CHAdeMO charge: %dV, %dA\r\n", voltage, current);
                chademo->start_charging(voltage, current);
              } else {
                Serial.println("\nError: Usage: chademo start <volts> <amps>\n");
              }
            }
          }
          else if (inputBuffer == "chademo stop") {
            if (chademo == nullptr) {
              Serial.println("\nError: CHAdeMO not configured\n");
            } else {
              Serial.println("\nStopping CHAdeMO charge\n");
              chademo->stop_charging();
            }
          }
          else if (inputBuffer == "hv") {
            Serial.println("\n=== HV System Status ===");
            const char* hv_state_str[] = {"Disabled", "Precharge", "Active", "Fault", "Shutdown"};
            Serial.printf("HV State: %s\r\n", hv_state_str[bms->get_hv_state()]);

            if (ivt_shunt != nullptr && ivt_shunt->is_alive()) {
              float pack_voltage = ivt_shunt->get_voltage();
              float hv_bus_voltage = ivt_shunt->get_voltage2();
              float voltage_diff = abs(pack_voltage - hv_bus_voltage);

              Serial.printf("Pack Voltage (V1): %.2f V\r\n", pack_voltage);
              Serial.printf("HV Bus Voltage (V2): %.2f V\r\n", hv_bus_voltage);
              Serial.printf("Voltage Difference: %.2f V\r\n", voltage_diff);
            } else {
              Serial.println("IVT shunt not available - cannot read voltages");
            }

            // Show contactor states
            Serial.printf("Positive Contactor: %s\r\n",
                         digitalRead(CONTACTOR_1_PIN) == HIGH ? "CLOSED" : "OPEN");
            Serial.printf("Negative Contactor: %s\r\n",
                         digitalRead(CONTACTOR_2_PIN) == HIGH ? "CLOSED" : "OPEN");
            Serial.println();
          }
          else if (inputBuffer == "bcc dump") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized. Cannot dump registers.\n");
            } else {
              Serial.println();
              bms->dump_registers();
            }
          }
          else if (inputBuffer == "bcc faults") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized. Cannot read faults.\n");
            } else {
              Serial.println();
              bms->print_fault_status();
            }
          }
          else if (inputBuffer == "bcc sleep") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized. Cannot enter sleep mode.\n");
            } else {
              Serial.println("\nPutting BCC into sleep mode...");
              BMS_State result = bms->enable_sleep_mode();
              if (result == BMS_Sleep) {
                Serial.println("BCC successfully entered sleep mode.");
                Serial.println("Note: Use 'bcc wakeup' command to resume operation.");
              } else {
                Serial.println("Failed to enter sleep mode.");
              }
              Serial.println();
            }
          }
          else if (inputBuffer == "bcc wakeup") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized. Cannot wake up.\n");
            } else {
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
          else if (inputBuffer.startsWith("bcc set target ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized.\n");
            } else {
              String value = inputBuffer.substring(15);
              float voltage = value.toFloat();
              if (voltage >= 2.5 && voltage <= 4.2) {
                Param::SetFloat(Param::targetCellVolt, voltage);
                Serial.printf("\nTarget voltage set to %.2f V (will take effect after reboot)\r\n\n", voltage);
              } else {
                Serial.println("\nError: Voltage must be between 2.5V and 4.2V\n");
              }
            }
          }
          else if (inputBuffer.startsWith("bcc set balance_th ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized.\n");
            } else {
              String value = inputBuffer.substring(19);
              float threshold = value.toFloat();
              if (threshold >= 1.0 && threshold <= 500.0) {
                Param::SetFloat(Param::balanceThreshold, threshold);
                Serial.printf("\nBalance threshold set to %.1f mV (will take effect after reboot)\r\n\n", threshold);
              } else {
                Serial.println("\nError: Threshold must be between 1.0 and 500.0 mV\n");
              }
            }
          }
          else if (inputBuffer.startsWith("bcc set balance_tgt ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized.\n");
            } else {
              String value = inputBuffer.substring(20);
              float target = value.toFloat();
              if (target >= 1.0 && target <= 100.0) {
                Param::SetFloat(Param::balanceTarget, target);
                Serial.printf("\nBalance target set to %.1f mV (will take effect after reboot)\r\n\n", target);
              } else {
                Serial.println("\nError: Target must be between 1.0 and 100.0 mV\n");
              }
            }
          }
          else if (inputBuffer.startsWith("bcc set interval ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized.\n");
            } else {
              String value = inputBuffer.substring(17);
              uint16_t interval = value.toInt();
              if (interval >= 100 && interval <= 10000) {
                Param::SetInt(Param::measureInterval, interval);
                Serial.printf("\nMeasurement interval set to %d ms (will take effect after reboot)\r\n\n", interval);
              } else {
                Serial.println("\nError: Interval must be between 100 and 10000 ms\n");
              }
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

  // Initialize libopeninv parameter system
  Serial.println("Initializing parameter system...");
  Param::LoadDefaults();
  int param_load_result = parm_load();
  if (param_load_result == 0) {
    Serial.println("Parameters loaded from flash");
  } else {
    Serial.println("No saved parameters found, using defaults");
  }
  Serial.println();

  // Configure BCC hardware from parameters
  Serial.println("Configuring BCC hardware from parameters...");
  bcc0_config.device_count = Param::GetInt(Param::bcc0DeviceCount);
  bcc0_config.cell_count = Param::GetInt(Param::bcc0CellCount);
  bcc0_config.enable_pin = BCC0_ENABLE;
  bcc0_config.intb_pin = BCC0_INTB;
  bcc0_config.cs_pin = BCC0_TX_CS;
  bcc0_config.loopback = false;

  bcc1_config.device_count = Param::GetInt(Param::bcc1DeviceCount);
  bcc1_config.cell_count = Param::GetInt(Param::bcc1CellCount);
  bcc1_config.enable_pin = BCC1_ENABLE;
  bcc1_config.intb_pin = BCC1_INTB;
  bcc1_config.cs_pin = BCC1_TX_CS;
  bcc1_config.loopback = false;

  Serial.printf("  BCC0: %d devices x %d cells = %d total cells\r\n",
                bcc0_config.device_count, bcc0_config.cell_count,
                bcc0_config.device_count * bcc0_config.cell_count);
  Serial.printf("  BCC1: %d devices x %d cells = %d total cells\r\n",
                bcc1_config.device_count, bcc1_config.cell_count,
                bcc1_config.device_count * bcc1_config.cell_count);
  Serial.printf("  Total: %d cells\r\n",
                (bcc0_config.device_count * bcc0_config.cell_count) +
                (bcc1_config.device_count * bcc1_config.cell_count));
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
  ipc_can->watchFor();

  m3_can = new CANBus(M3_CAN_RX, M3_CAN_TX);
  if (!m3_can->begin(CAN_BPS_500K)) {  // 500kbps for M3 CAN
    Serial.println("ERROR: Failed to initialize M3 CAN!");
  } else {
    Serial.println("M3 CAN initialized at 500kbps");
  }
  ipc_can->watchFor();

  hv_can = new CANBus(HV_CAN_RX, HV_CAN_TX);
  if (!hv_can->begin(CAN_BPS_500K)) {  // 500kbps for HV CAN
    Serial.println("ERROR: Failed to initialize HV CAN!");
  } else {
    Serial.println("HV CAN initialized at 500kbps");
  }
  Serial.println();
  hv_can->watchFor();

  // Initialize IVT current shunt
  Serial.println("Initializing IVT current shunt...");
  ivt_shunt = new IVTShunt();
  ivt_shunt->begin(hv_can);
  Serial.println("IVT shunt initialized");
  Serial.println();

  // Initialize CHAdeMO controller on M3/CP CAN
  Serial.println("Initializing CHAdeMO controller (Foccci)...");
  chademo = new CHAdeMOController();
  chademo->begin(m3_can);
  Serial.println("CHAdeMO controller initialized");
  Serial.println();

  // Create BMS instance with both BCC0 and BCC1 enabled
  Serial.println("Creating BMS instance for dual pack monitoring...");
  bms = new BatteryManagementSystem(&bcc0_config, &bcc1_config);

  // Set status LEDs
  bms->set_status_leds(&strip);

  // Display loaded charging parameters
  Serial.println("Charging parameters loaded from parameter system:");
  Serial.printf("  Target cell voltage: %.2f V\r\n", Param::GetFloat(Param::targetCellVolt));
  Serial.printf("  Balance threshold: %.1f mV\r\n", Param::GetFloat(Param::balanceThreshold));
  Serial.printf("  Balance target: %.1f mV\r\n", Param::GetFloat(Param::balanceTarget));
  Serial.printf("  Measurement interval: %d ms (%.1f Hz)\r\n",
                Param::GetInt(Param::measureInterval),
                1000.0f / Param::GetInt(Param::measureInterval));
  Serial.printf("  Battery capacity: %.1f Ah\r\n", Param::GetFloat(Param::batteryCapacity));
  Serial.printf("  Max charge current: %.1f A\r\n", Param::GetFloat(Param::maxChargeCurrent));
  Serial.println();

  // Configure IVT shunt and CHAdeMO
  Serial.println("Configuring IVT shunt and CHAdeMO with BMS...");
  bms->set_ivt_shunt(ivt_shunt);
  bms->set_chademo(chademo);
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

  // Create CAN message queues
  Serial.println("Creating CAN message queues...");
  ipc_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));
  m3_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));
  hv_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));

  if (!ipc_can_queue || !m3_can_queue || !hv_can_queue) {
    Serial.println("ERROR: Failed to create CAN queues!");
    Serial.println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("CAN queues created");
  Serial.println();

  // Create CAN RX polling task (high priority - reads from hardware)
  Serial.println("Starting CAN RX task...");
  BaseType_t result = xTaskCreate(
    can_rx_task,
    "CAN RX",
    1024,
    NULL,
    4,  // Very high priority for hardware polling
    NULL
  );

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create CAN RX task!");
    Serial.println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("CAN RX task started");
  Serial.println();

  // Create IVT processing task (processes HV CAN messages for IVT)
  Serial.println("Starting IVT processing task...");
  result = xTaskCreate(
    ivt_process_task,
    "IVT Proc",
    1536,
    NULL,
    3,  // High priority for message processing
    NULL
  );

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create IVT processing task!");
    Serial.println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("IVT processing task started");
  Serial.println();

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create M3 test task!");
    Serial.println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  // Create console task for user interaction
  Serial.println("Starting serial console...");
  result = xTaskCreate(
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
  Serial.println("Type 'bcc start' to begin charging");
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