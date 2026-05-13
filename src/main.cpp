/*
 * BMS Charging System for 6S2P Battery Packs
 *
 * Main application entry point and serial console interface.
 * See README.md for complete documentation, commands, and LED status indicators.
 */

#include "main.h"
#include <STM32FreeRTOS.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(STATUS_LED_COUNT, STATUS_LEDS, NEO_GRB + NEO_KHZ800);
HardwareSerial DebugSerial(USART1_RX, USART1_TX);  // Use USART1 for debug serial

// CAN bus instances
CANBus *m3_can = nullptr;   // M3 CAN for external communication
CANBus *hv_can = nullptr;   // HV CAN for IVT shunt and vehicle comms (500kbps)

// IVT controller
IVTShunt *ivt_shunt = nullptr;

// CHAdeMO controller (Foccci on M3/CP CAN)
CHAdeMOController *chademo = nullptr;

// libopeninv CanOpen SDO for BMS CAN communication
CanHardwareArduino *hv_can_hardware = nullptr;
CanMap *can_map = nullptr;
CanSdo *can_sdo = nullptr;

// HV CAN monitoring
bool hv_can_monitor_enabled = false;
uint32_t hv_can_frame_count = 0;

// M3 CAN test message generator
bool m3_can_test_enabled = false;
uint32_t m3_can_test_interval_ms = 1000;  // Default 1Hz
uint32_t m3_can_test_count = 0;

QueueHandle_t m3_can_queue = nullptr;
QueueHandle_t hv_can_queue = nullptr;

#define CAN_QUEUE_LENGTH 32  // Buffer up to 32 messages per bus

// BMS Configuration - will be populated from parameters in setup()
BatteryCellControllerConfig bcc0_config;
BatteryCellControllerConfig bcc1_config;

BatteryManagementSystem *bms;

// Pre-generated parameter JSON for web interface
String parameterJson;

// Forward declarations
void send_parameter_json();
void dump_bcc_config(BatteryManagementSystem *bms);

// Direct data source callback for JSON transfer - returns byte at offset or -1 if out of range
int get_json_byte(uint32_t offset) {
  if (offset >= parameterJson.length()) {
    return -1;  // End of data
  }
  return (int)(uint8_t)parameterJson[offset];
}

// CAN RX polling task - reads from hardware and puts messages into queues
void can_rx_task(void *pvParameters) {
  CAN_FRAME frame;
  static uint32_t last_debug = 0;
  static uint32_t m3_read_count = 0;
  static uint32_t hv_read_count = 0;

  while (true)  {
    // Poll M3 CAN bus
    if (m3_can && m3_can->available()) {
      while (m3_can->read(frame)) {
        m3_read_count++;
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
      // Process all messages through CanOpen SDO handler first
      if (hv_can_hardware && can_sdo) {
        uint32_t data[2];
        memcpy(data, frame.data.uint8, 8);
        hv_can_hardware->HandleRx(frame.id, data, frame.length);
      }

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

// Generate parameter JSON once at startup
void build_parameter_json() {
  JsonDocument doc;

  for (int i = 0; i < Param::PARAM_LAST; i++) {
    const Param::Attributes* attr = Param::GetAttrib((Param::PARAM_NUM)i);
    if (!attr) continue;

    JsonObject param = doc[attr->name].to<JsonObject>();
    param["unit"] = attr->unit;
    param["category"] = attr->category;
    // Use native float values directly (no conversion needed)
    param["minimum"] = attr->min;
    param["maximum"] = attr->max;
    param["default"] = attr->def;
    param["id"] = attr->id;  // Add parameter ID for SDO access
    param["isparam"] = (Param::GetType((Param::PARAM_NUM)i) == Param::TYPE_PARAM) ? 1 : 0;
    
    // Version is already stored as major.minor float
    if (strcmp(attr->name, "version") == 0) {
      param["value"] = Param::GetFloat((Param::PARAM_NUM)i);
    }
    // For other spot values, include current value
    else if (Param::GetType((Param::PARAM_NUM)i) == Param::TYPE_SPOTVALUE) {
      param["value"] = Param::GetFloat((Param::PARAM_NUM)i);
    }
  }

  serializeJson(doc, parameterJson);
  Serial.printf("Generated parameter JSON (%d bytes)\r\n", parameterJson.length());
  
  // Update CanSdo with actual JSON size
  if (can_sdo != nullptr) {
    can_sdo->SetJsonSize(parameterJson.length());
  }
}

// Send stored JSON to web interface
void send_parameter_json() {
  if (!can_sdo || parameterJson.length() == 0) return;

  // Send all data without delays - the print buffer and timeout mechanism will handle flow control
  for (unsigned int i = 0; i < parameterJson.length(); i++) {
    can_sdo->PutChar(parameterJson[i]);
  }
}

// CanOpen periodic task - sends mapped CAN messages
void canopen_periodic_task(void *pvParameters) {
  Serial.println("CanOpen Periodic Task: Started");

  while (true) {
    // Send all mapped CAN messages
    if (can_map != nullptr) {
      can_map->SendAll();
    }

    // Trigger SDO timeout handling
    if (can_sdo != nullptr) {
      can_sdo->TriggerTimeout(10);  // 10ms period
    }

    // Periodic delay (100Hz - 10ms for responsive SDO communication)
    vTaskDelay(pdMS_TO_TICKS(10));
  }
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
              Serial.println("bcc config dump       - Dump pack configuration registers");
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

            Serial.println("=== CanOpen SDO Commands ===");
            Serial.println("can sdo read <nodeId> <index> <subIndex>");
            Serial.println("                      - Read SDO parameter from remote node");
            Serial.println("can sdo write <nodeId> <index> <subIndex> <value>");
            Serial.println("                      - Write SDO parameter to remote node");
            Serial.println("can map save          - Save CAN mappings to flash");
            Serial.println("can node <id>         - Set local CAN node ID");
            Serial.println();
          }
          else if (inputBuffer == "status") {
            BMS_State state = bms->get_state();
            const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Cooldown", "Sleep", "Error"};
            Serial.println("\n=== BMS Status ===");
            Serial.printf("State: %s\r\n", state_str[state]);
            Serial.println();
            
            // Show spot values being transmitted
            Serial.println("=== Spot Values (being transmitted on CAN) ===");
            Serial.printf("Pack Voltage: %.2f V\r\n", Param::GetFloat(Param::packVoltage));
            Serial.printf("Pack Current: %.2f A\r\n", Param::GetFloat(Param::packCurrent));
            Serial.printf("Max Cell: %d mV\r\n", Param::GetInt(Param::maxCellVolt));
            Serial.printf("Min Cell: %d mV\r\n", Param::GetInt(Param::minCellVolt));
            Serial.printf("Cell Diff: %d mV\r\n", Param::GetInt(Param::cellVoltDiff));
            Serial.printf("SOC: %d %%\r\n", Param::GetInt(Param::soc));
            Serial.printf("BMS State: %d\r\n", Param::GetInt(Param::bmsState));
            Serial.printf("HV State: %d\r\n", Param::GetInt(Param::hvState));
            Serial.printf("Safe Charge Current: %d A\r\n", Param::GetInt(Param::safeChargeCurrent));
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
          else if (inputBuffer == "bcc config dump") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              Serial.println("\nError: No BCC initialized. Cannot dump configuration.\n");
            } else {
              Serial.println("\nDumping BCC configuration...");
              Serial.println("Copy this configuration array to replace TAYCAN_CONFIG:\n");
              dump_bcc_config(bms);
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
          else if (inputBuffer.startsWith("can sdo read ")) {
            if (can_sdo == nullptr) {
              Serial.println("\nError: CanOpen SDO not configured\n");
            } else {
              // Parse: can sdo read <nodeId> <index> <subIndex>
              String args = inputBuffer.substring(13);
              int space1 = args.indexOf(' ');
              int space2 = args.indexOf(' ', space1 + 1);
              if (space1 > 0 && space2 > space1) {
                uint8_t nodeId = strtol(args.substring(0, space1).c_str(), NULL, 0);
                uint16_t index = strtol(args.substring(space1 + 1, space2).c_str(), NULL, 0);
                uint8_t subIndex = strtol(args.substring(space2 + 1).c_str(), NULL, 0);

                Serial.printf("\nReading SDO: Node %d, Index 0x%04X, SubIndex %d\r\n", nodeId, index, subIndex);
                can_sdo->SDORead(nodeId, index, subIndex);

                // Wait for reply (with timeout)
                uint32_t start = millis();
                uint32_t data = 0;
                while (millis() - start < 1000) {
                  if (can_sdo->SDOReadReply(data)) {
                    Serial.printf("SDO Reply: 0x%08X (%d)\r\n\n", data, (int32_t)data);
                    break;
                  }
                  delay(10);
                }
                if (millis() - start >= 1000) {
                  Serial.println("SDO Read timeout\n");
                }
              } else {
                Serial.println("\nError: Usage: can sdo read <nodeId> <index> <subIndex>\n");
              }
            }
          }
          else if (inputBuffer.startsWith("can sdo write ")) {
            if (can_sdo == nullptr) {
              Serial.println("\nError: CanOpen SDO not configured\n");
            } else {
              // Parse: can sdo write <nodeId> <index> <subIndex> <value>
              String args = inputBuffer.substring(14);
              int space1 = args.indexOf(' ');
              int space2 = args.indexOf(' ', space1 + 1);
              int space3 = args.indexOf(' ', space2 + 1);
              if (space1 > 0 && space2 > space1 && space3 > space2) {
                uint8_t nodeId = strtol(args.substring(0, space1).c_str(), NULL, 0);
                uint16_t index = strtol(args.substring(space1 + 1, space2).c_str(), NULL, 0);
                uint8_t subIndex = strtol(args.substring(space2 + 1, space3).c_str(), NULL, 0);
                uint32_t value = strtol(args.substring(space3 + 1).c_str(), NULL, 0);

                Serial.printf("\nWriting SDO: Node %d, Index 0x%04X, SubIndex %d, Value 0x%08X\r\n", nodeId, index, subIndex, value);
                can_sdo->SDOWrite(nodeId, index, subIndex, value);
                Serial.println("SDO Write sent\n");
              } else {
                Serial.println("\nError: Usage: can sdo write <nodeId> <index> <subIndex> <value>\n");
              }
            }
          }
          else if (inputBuffer == "can map save") {
            if (can_map == nullptr) {
              Serial.println("\nError: CAN mapping not configured\n");
            } else {
              Serial.println("\nSaving CAN mappings to flash...");
              can_map->Save();
              Serial.println("CAN mappings saved\n");
            }
          }
          else if (inputBuffer.startsWith("can node ")) {
            if (can_sdo == nullptr) {
              Serial.println("\nError: CanOpen SDO not configured\n");
            } else {
              String arg = inputBuffer.substring(9);
              uint8_t nodeId = arg.toInt();
              if (nodeId > 0 && nodeId < 128) {
                can_sdo->SetNodeId(nodeId);
                Param::SetInt(Param::canNodeId, nodeId);
                Serial.printf("\nCAN Node ID set to %d\r\n\n", nodeId);
              } else {
                Serial.println("\nError: Node ID must be between 1 and 127\n");
              }
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
                         digitalRead(HV_CONTACTOR_1_PIN) == HIGH ? "CLOSED" : "OPEN");
            Serial.printf("Negative Contactor: %s\r\n",
                         digitalRead(HV_CONTACTOR_2_PIN) == HIGH ? "CLOSED" : "OPEN");
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
                Param::SetFloat(Param::targetCellVolt, voltage * 1000.0f);  // Convert V to mV
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
  // Initialize Serial FIRST
  Serial.begin(115200);
  delay(2000);  // Wait for serial monitor to connect
  Serial.println("=== BMS Charging System for Dual 6S2P Packs ===");
  Serial.println();

  // Initialize libopeninv parameter system
  Serial.println("Initializing parameter system...");
  Param::LoadDefaults();
  int param_load_result = parm_load();
  if (param_load_result == 0) {
    Serial.println("Parameters loaded from flash successfully!");
  } else {
    Serial.println("No saved parameters found, using defaults");
  }

  // Set firmware version (store as major.minor float for web interface)
  float version_float = FW_VERSION_MAJOR + (FW_VERSION_MINOR / 10.0f);
  Param::SetFloat(Param::version, version_float);
  Serial.printf("Firmware version: 0x%08X (v%d.%d.%d.%d) = %.1f\r\n",
                FIRMWARE_VERSION,
                FW_VERSION_MAJOR,
                FW_VERSION_MINOR,
                FW_VERSION_PATCH,
                FW_VERSION_BUILD,
                version_float);

  // Display STM32 unique device ID (used as serial number)
  Serial.printf("Device Serial: %08X-%08X-%08X\r\n",
                STM32_UNIQUE_ID[0],
                STM32_UNIQUE_ID[1],
                STM32_UNIQUE_ID[2]);
  Serial.println();

  // Configure BCC hardware from parameters
  Serial.println("Configuring BCC hardware from parameters...");
  Serial.printf("  bcc0DeviceCount (raw) = %d\r\n", Param::GetInt(Param::bcc0DeviceCount));
  Serial.printf("  bcc0DeviceType (raw) = %d\r\n", Param::GetInt(Param::bcc0DeviceType));
  Serial.printf("  bcc1DeviceCount (raw) = %d\r\n", Param::GetInt(Param::bcc1DeviceCount));
  Serial.printf("  bcc1DeviceType (raw) = %d\r\n", Param::GetInt(Param::bcc1DeviceType));
  Serial.println();

  bcc0_config.device_count = Param::GetInt(Param::bcc0DeviceCount);
  bcc0_config.device_type = (bcc_device_t)Param::GetInt(Param::bcc0DeviceType);
  // Cell count is automatically determined by device type
  bcc0_config.cell_count = (bcc0_config.device_type == BCC_DEVICE_MC33771) ? MC33771_MAX_CELLS : MC33772_MAX_CELLS;
  bcc0_config.enable_pin = BCC0_ENABLE;
  bcc0_config.intb_pin = BCC0_INTB;
  bcc0_config.cs_pin = BCC0_TX_CS;
  bcc0_config.loopback = false;

  bcc1_config.device_count = Param::GetInt(Param::bcc1DeviceCount);
  bcc1_config.device_type = (bcc_device_t)Param::GetInt(Param::bcc1DeviceType);
  // Cell count is automatically determined by device type
  bcc1_config.cell_count = (bcc1_config.device_type == BCC_DEVICE_MC33771) ? MC33771_MAX_CELLS : MC33772_MAX_CELLS;
  bcc1_config.enable_pin = BCC1_ENABLE;
  bcc1_config.intb_pin = BCC1_INTB;
  bcc1_config.cs_pin = BCC1_TX_CS;
  bcc1_config.loopback = false;

  Serial.printf("  BCC0: %d x MC3377%d devices x %d cells = %d total cells%s\r\n",
                bcc0_config.device_count,
                bcc0_config.device_type == BCC_DEVICE_MC33771 ? 1 : 2,
                bcc0_config.cell_count,
                bcc0_config.device_count * bcc0_config.cell_count,
                bcc0_config.device_count == 0 ? " (DISABLED)" : "");
  Serial.printf("  BCC1: %d x MC3377%d devices x %d cells = %d total cells%s\r\n",
                bcc1_config.device_count,
                bcc1_config.device_type == BCC_DEVICE_MC33771 ? 1 : 2,
                bcc1_config.cell_count,
                bcc1_config.device_count * bcc1_config.cell_count,
                bcc1_config.device_count == 0 ? " (DISABLED)" : "");

  uint8_t total_cells = (bcc0_config.device_count * bcc0_config.cell_count) +
                        (bcc1_config.device_count * bcc1_config.cell_count);
  Serial.printf("  Total: %d cells\r\n", total_cells);

  if (total_cells == 0) {
    Serial.println("  ERROR: No BCC devices configured! System cannot operate.");
  }
  Serial.println();

  // Initialize NeoPixel strip
  Serial.println("Initializing status LEDs...");
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // Configure wakeup input
  pinMode(WAKEUP, INPUT);

  // Configure AC contactor pins (safe default: contactor open, driver asleep).
  // BMS does not yet drive these; control will be added with the AC charging flow.
  pinMode(AC_CONTACTOR_1_PIN, OUTPUT);
  pinMode(AC_CONTACTOR_2_PIN, OUTPUT);
  pinMode(AC_CONTACTOR_NSLEEP_PIN, OUTPUT);
  pinMode(AC_CONTACTOR_FAULT_PIN, INPUT);
  digitalWrite(AC_CONTACTOR_1_PIN, LOW);
  digitalWrite(AC_CONTACTOR_2_PIN, LOW);
  digitalWrite(AC_CONTACTOR_NSLEEP_PIN, LOW);

  // Initialize CAN buses
  Serial.println("Initializing CAN buses...");

  m3_can = new CANBus(M3_CAN_RX, M3_CAN_TX);
  if (!m3_can->begin(CAN_BPS_500K)) {  // 500kbps for M3 CAN
    Serial.println("ERROR: Failed to initialize M3 CAN!");
  } else {
    Serial.println("M3 CAN initialized at 500kbps");
  }
  m3_can->watchFor();

  hv_can = new CANBus(HV_CAN_RX, HV_CAN_TX);
  if (!hv_can->begin(CAN_BPS_500K)) {  // 500kbps for HV CAN
    Serial.println("ERROR: Failed to initialize HV CAN!");
  } else {
    Serial.println("HV CAN initialized at 500kbps");
  }
  Serial.println();
  hv_can->watchFor();

  // Initialize libopeninv CanOpen SDO for BMS CAN communication
  Serial.println("Initializing CanOpen SDO for BMS communication...");
  hv_can_hardware = new CanHardwareArduino(m3_can);
  can_map = new CanMap(hv_can_hardware);
  can_sdo = new CanSdo(hv_can_hardware, can_map);
  can_sdo->SetNodeId(Param::GetInt(Param::canNodeId));  // Use node ID from parameters
  
  // Set up direct data source for on-demand JSON delivery (no buffer needed!)
  can_sdo->SetDataSource(get_json_byte);
  
  Serial.printf("CanOpen SDO initialized on M3 CAN (Node ID: %d)\r\n", Param::GetInt(Param::canNodeId));
  Serial.println();

  // Configure CAN message mappings for BMS spot values
  // Serial.println("Configuring CAN message mappings for BMS telemetry...");
  // // Pack voltage and current on 0x420
  // can_map->AddSend(Param::packVoltage, 0x420, 0, 32, 1000.0f);      // V -> mV, bits 0-31
  // can_map->AddSend(Param::packCurrent, 0x420, 32, 32, 1000.0f);     // A -> mA, bits 32-63
  
  // // Cell voltages and SOC on 0x421
  // can_map->AddSend(Param::maxCellVolt, 0x421, 0, 16, 1.0f);         // mV, bits 0-15
  // can_map->AddSend(Param::minCellVolt, 0x421, 16, 16, 1.0f);        // mV, bits 16-31
  // can_map->AddSend(Param::soc, 0x421, 32, 16, 10.0f);               // % * 10, bits 32-47
  // can_map->AddSend(Param::cellVoltDiff, 0x421, 48, 16, 1.0f);       // mV, bits 48-63
  
  // // BMS status and states on 0x422
  // can_map->AddSend(Param::bmsState, 0x422, 0, 8, 1.0f);             // BMS state enum, bits 0-7
  // can_map->AddSend(Param::hvState, 0x422, 8, 8, 1.0f);              // HV state enum, bits 8-15
  // can_map->AddSend(Param::bcc0Initialized, 0x422, 16, 1, 1.0f);     // Boolean, bit 16
  // can_map->AddSend(Param::bcc1Initialized, 0x422, 17, 1, 1.0f);     // Boolean, bit 17
  // can_map->AddSend(Param::faultStatus, 0x422, 24, 16, 1.0f);        // Fault bits, bits 24-39
  // can_map->AddSend(Param::safeChargeCurrent, 0x422, 40, 16, 10.0f); // A * 10, bits 40-55
  
  Serial.println("CAN mappings configured:");
  Serial.println("  0x420: Pack voltage, current");
  Serial.println("  0x421: Cell voltages (min/max/diff), SOC");
  Serial.println("  0x422: BMS/HV state, initialization, faults, safe current");
  
  // Configure CAN receive mappings for IVT shunt messages
  // IVT format: Byte 0=MuxID, Byte 1=counter, Bytes 2-5=32-bit big-endian value
  // Serial.println("Configuring CAN receive mappings for IVT shunt...");
  // can_map->AddRecv(Param::ivtCurrent, 0x521, 16, -32, 0.001f);      // mA -> A, big-endian
  // can_map->AddRecv(Param::ivtVoltage1, 0x522, 16, -32, 0.001f);     // mV -> V, big-endian
  // can_map->AddRecv(Param::ivtVoltage2, 0x523, 16, -32, 0.001f);     // mV -> V, big-endian
  // can_map->AddRecv(Param::ivtVoltage3, 0x524, 16, -32, 0.001f);     // mV -> V, big-endian
  // can_map->AddRecv(Param::ivtTemperature, 0x525, 16, -32, 0.1f);    // deci-deg -> C, big-endian
  // can_map->AddRecv(Param::ivtPower, 0x526, 16, -32, 0.001f);        // W -> kW, big-endian
  
  Serial.println("  0x521: IVT Current (A)");
  Serial.println("  0x522: IVT Voltage 1 (V)");
  Serial.println("  0x523: IVT Voltage 2 (V)");
  Serial.println("  0x524: IVT Voltage 3 (V)");
  Serial.println("  0x525: IVT Temperature (C)");
  Serial.println("  0x526: IVT Power (kW)");
  Serial.println();

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
  Serial.printf("  Target cell voltage: %.2f V\r\n", Param::GetFloat(Param::targetCellVolt) / 1000.0f);  // Convert mV to V
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
  bms->set_can_buses(m3_can, hv_can);

  // Configure contactor control pins
  Serial.println("Configuring contactor control...");
  bms->set_contactor_pins(
    HV_CONTACTOR_1_PIN,
    HV_CONTACTOR_2_PIN,
    HV_CONTACTOR_NSLEEP_PIN,
    HV_CONTACTOR_FAULT_PIN
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
  }

  Serial.println("BMS tasks started successfully!");
  Serial.println();

  // Read cell voltage limits from BCC hardware configuration
  Serial.println("Reading pack voltage limits from BCC hardware...");
  bms->read_and_set_voltage_limits();
  Serial.println();

  // Create CAN message queues
  Serial.println("Creating CAN message queues...");
  m3_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));
  hv_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));

  if (!m3_can_queue || !hv_can_queue) {
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

  // Create CanOpen periodic task (sends mapped CAN messages)
  Serial.println("Starting CanOpen periodic task...");
  result = xTaskCreate(
    canopen_periodic_task,
    "CanOpen",
    1024,
    NULL,
    2,  // Medium priority
    NULL
  );

  if (result != pdPASS) {
    Serial.println("ERROR: Failed to create CanOpen periodic task!");
    Serial.println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("CanOpen periodic task started");
  Serial.println();

  // Generate parameter JSON for web interface
  Serial.println("Generating parameter JSON...");
  build_parameter_json();
  Serial.println();

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

// Dump BCC configuration registers from all connected modules
void dump_bcc_config(BatteryManagementSystem *bms) {
  // Register addresses to dump (configuration registers)
  const uint8_t reg_addrs[] = {
    0x03, 0x04, 0x05, 0x06, 0x07, 0x08,  // SYS_CFG1, SYS_CFG2, SYS_DIAG, ADC_CFG, ADC2_OFFSET_COMP, OV_UV_EN
    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E,  // CB1-6_CFG
    0x0F, 0x10, 0x11, 0x12, 0x13,        // GPIO_CFG1-2, FAULT_MASK1-3
    0x14, 0x15, 0x16,                     // WAKEUP_MASK1-3
    0x4B, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,  // TH_ALL_CT, TH_CT6-1
    0x5A, 0x5B, 0x5C, 0x5D, 0x5E, 0x5F, 0x60,  // TH_AN6-0_OT
    0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,  // TH_AN6-0_UT
    0x68, 0x2E, 0x2F                           // TH_ISENSE_OC, TH_COULOMB_CNT_MSB/LSB
  };

  const char* reg_names[] = {
    "SYS_CFG1", "SYS_CFG2", "SYS_DIAG", "ADC_CFG", "ADC2_OFFSET_COMP", "OV_UV_EN",
    "CB1_CFG", "CB2_CFG", "CB3_CFG", "CB4_CFG", "CB5_CFG", "CB6_CFG",
    "GPIO_CFG1", "GPIO_CFG2", "FAULT_MASK1", "FAULT_MASK2", "FAULT_MASK3",
    "WAKEUP_MASK1", "WAKEUP_MASK2", "WAKEUP_MASK3",
    "TH_ALL_CT", "TH_CT6", "TH_CT5", "TH_CT4", "TH_CT3", "TH_CT2", "TH_CT1",
    "TH_AN6_OT", "TH_AN5_OT", "TH_AN4_OT", "TH_AN3_OT", "TH_AN2_OT", "TH_AN1_OT", "TH_AN0_OT",
    "TH_AN6_UT", "TH_AN5_UT", "TH_AN4_UT", "TH_AN3_UT", "TH_AN2_UT", "TH_AN1_UT", "TH_AN0_UT",
    "TH_ISENSE_OC", "TH_COULOMB_CNT_MSB", "TH_COULOMB_CNT_LSB"
  };

  const uint8_t num_regs = sizeof(reg_addrs) / sizeof(reg_addrs[0]);

  Serial.println("\n=== CSV Format: BCC Configuration Dump ===");

  // Print CSV header
  Serial.print("BCC,CID");
  for (uint8_t i = 0; i < num_regs; i++) {
    Serial.print(",");
    Serial.print(reg_names[i]);
  }
  Serial.println();

  // Read and print configuration for each BCC chain and module
  uint8_t bcc_configs[2] = {
    bms->get_bcc0_total_cell_count() > 0 ? 1 : 0,  // BCC0 enabled?
    bms->is_bcc1_enabled() ? 1 : 0                   // BCC1 enabled?
  };

  for (uint8_t bcc = 0; bcc < 2; bcc++) {
    if (!bcc_configs[bcc]) continue;

    // Get device count for this BCC chain
    uint8_t device_count = (bcc == 0) ?
      Param::GetInt(Param::bcc0DeviceCount) :
      Param::GetInt(Param::bcc1DeviceCount);

    // Read configuration from each module in the chain
    for (uint8_t cid = 1; cid <= device_count; cid++) {
      Serial.print(bcc);
      Serial.print(",");
      Serial.print(cid);

      // Read each register
      for (uint8_t i = 0; i < num_regs; i++) {
        uint16_t value = 0;
        bcc_status_t status = bms->read_bcc_register(bcc, (bcc_cid_t)cid, reg_addrs[i], &value);

        Serial.print(",");
        if (status == BCC_STATUS_SUCCESS) {
          Serial.printf("0x%04X", value);
        } else {
          Serial.print("ERROR");
        }
      }
      Serial.println();

      // Small delay to avoid overwhelming the serial buffer
      delay(10);
    }
  }

  Serial.println("=== Dump Complete ===");
  Serial.println("\nUse 'python3 capture_battery_data.py' to save this data to CSV file.");
}

void loop() {
  // Empty - FreeRTOS tasks run instead
}

