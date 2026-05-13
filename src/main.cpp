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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"

Adafruit_NeoPixel strip = Adafruit_NeoPixel(STATUS_LED_COUNT, STATUS_LEDS, NEO_GRB + NEO_KHZ800);
// DebugSerial is defined in src/debug_serial.cpp; extern declaration via debug_serial.h

// CAN bus instances
CANBus *m3_can = nullptr;   // M3 CAN for external communication
CANBus *hv_can = nullptr;   // HV CAN for IVT shunt and vehicle comms (500kbps)

// IVT controller
IVTShunt *ivt_shunt = nullptr;

// CHAdeMO controller (Foccci on HV CAN)
CHAdeMOController *chademo = nullptr;

// libopeninv CanOpen SDO for BMS CAN communication
CanHardwareArduino *hv_can_hardware = nullptr;
CanMap *can_map = nullptr;
CanSdo *can_sdo = nullptr;

// HV CAN monitoring
bool hv_can_monitor_enabled = false;
uint32_t hv_can_frame_count = 0;

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
        if (hv_can_monitor_enabled) {
          debug_printf("HV RX: 0x%03X [%d] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                        frame.id, frame.length,
                        frame.data.uint8[0], frame.data.uint8[1], frame.data.uint8[2], frame.data.uint8[3],
                        frame.data.uint8[4], frame.data.uint8[5], frame.data.uint8[6], frame.data.uint8[7]);
        }

        // Put frame in queue
        xQueueSend(hv_can_queue, &frame, 0);
      }
    }

    // Poll every 1ms (1000 Hz)
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// HV CAN processing task - dispatches IVT-S and CHAdeMO frames from hv_can_queue
void ivt_process_task(void *pvParameters) {
  CAN_FRAME frame;

  while (true) {
    if (xQueueReceive(hv_can_queue, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
      // Process all messages through CanOpen SDO handler first
      if (hv_can_hardware && can_sdo) {
        uint32_t data[2];
        memcpy(data, frame.data.uint8, 8);
        hv_can_hardware->HandleRx(frame.id, data, frame.length);
      }

      // IVT-S: 0x521-0x528, 0x511
      if ((frame.id >= 0x521 && frame.id <= 0x528) || frame.id == 0x511) {
        if (ivt_shunt) {
          ivt_shunt->process_can_frame(&frame);
        }
      }
      // CHAdeMO (Foccci): 0x100, 0x102, 0x108, 0x109
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
  debug_printf("Generated parameter JSON (%d bytes)\r\n", parameterJson.length());
  
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
  debug_println("CanOpen Periodic Task: Started");

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
  debug_println("\n=== BMS Serial Console ===");
  debug_println("Type 'help' for available commands");
  debug_println();

  String inputBuffer = "";

  while (true) {
    // Check for available serial data
    while (DebugSerial.available() > 0) {
      char c = DebugSerial.read();

      if (c == '\n' || c == '\r') {
        if (inputBuffer.length() > 0) {
          // Process command
          inputBuffer.trim();
          inputBuffer.toLowerCase();

          if (inputBuffer == "help") {
            bool bcc0_ok = bms->is_bcc0_initialized();
            bool bcc1_ok = bms->is_bcc1_initialized();
            bool any_bcc_ok = bcc0_ok || bcc1_ok;

            debug_println("\n=== Available Commands ===");
            debug_println("help                  - Show this help message");
            debug_println("status                - Show current BMS status");
            debug_println("bcc                   - Show BCC status");
            debug_println("ivt                   - Show IVT shunt status");
            debug_println("chademo               - Show CHAdeMO status");
            debug_println("hv                    - Show HV system status");
            debug_println("hv can monitor        - Show HV CAN monitoring status");
            debug_println("hv can monitor on     - Enable HV CAN traffic monitoring");
            debug_println("hv can monitor off    - Disable HV CAN traffic monitoring");
            debug_println("m3 can send <id> <b0> <b1> ... <b7>");
            debug_println("                      - Send CAN message on M3 bus (hex values)");
            debug_println("reboot                - Software reset (restart application)");
            debug_println();

            if (any_bcc_ok) {
              debug_println("=== BCC (Battery Cell Controller) Commands ===");
              debug_println("bcc start             - Start charging");
              debug_println("bcc stop              - Stop charging");
              debug_println("bcc balance           - Force cell balancing");
              debug_println("bcc config dump       - Dump pack configuration registers");
              debug_println("bcc voltages [summary]");
              debug_println("                      - Show module voltage summary");
              debug_println("bcc voltages <module#>");
              debug_println("                      - Show detailed voltages for specific module (0-15)");
              debug_println("bcc faults            - Show fault status");
              debug_println("bcc config            - Show charging configuration");
              debug_println("bcc dump              - Dump all BCC registers and fuse mirror");
              debug_println("bcc sleep             - Put BCC into low-power sleep mode");
              debug_println("bcc wakeup            - Wake BCC from sleep mode");
              debug_println("bcc set target <voltage>");
              debug_println("                      - Set target cell voltage (V)");
              debug_println("bcc set balance_th <mv>");
              debug_println("                      - Set balance threshold (mV)");
              debug_println("bcc set balance_tgt <mv>");
              debug_println("                      - Set balance target (mV)");
              debug_println("bcc set interval <ms> - Set measurement interval (ms)");
              debug_println();
            }

            debug_println("=== CHAdeMO (Foccci) Commands ===");
            debug_println("chademo start <volts> <amps>");
            debug_println("                      - Start CHAdeMO charge session");
            debug_println("chademo stop          - Stop CHAdeMO charge session");
            debug_println("chademo limits <v> <a>");
            debug_println("                      - Set EVSE capabilities (max voltage/current)");
            debug_println();

            debug_println("=== CanOpen SDO Commands ===");
            debug_println("can sdo read <nodeId> <index> <subIndex>");
            debug_println("                      - Read SDO parameter from remote node");
            debug_println("can sdo write <nodeId> <index> <subIndex> <value>");
            debug_println("                      - Write SDO parameter to remote node");
            debug_println("can map save          - Save CAN mappings to flash");
            debug_println("can node <id>         - Set local CAN node ID");
            debug_println();
          }
          else if (inputBuffer == "status") {
            BMS_State state = bms->get_state();
            const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Sleep", "Error"};
            const uint8_t state_str_count = sizeof(state_str) / sizeof(state_str[0]);
            debug_println("\n=== BMS Status ===");
            debug_printf("State: %s\r\n", (state < state_str_count) ? state_str[state] : "Unknown");
            debug_println();
            
            // Show spot values being transmitted
            debug_println("=== Spot Values (being transmitted on CAN) ===");
            debug_printf("Pack Voltage: %.2f V\r\n", Param::GetFloat(Param::packVoltage));
            debug_printf("Pack Current: %.2f A\r\n", Param::GetFloat(Param::packCurrent));
            debug_printf("Max Cell: %d mV\r\n", Param::GetInt(Param::maxCellVolt));
            debug_printf("Min Cell: %d mV\r\n", Param::GetInt(Param::minCellVolt));
            debug_printf("Cell Diff: %d mV\r\n", Param::GetInt(Param::cellVoltDiff));
            debug_printf("SOC: %d %%\r\n", Param::GetInt(Param::soc));
            debug_printf("BMS State: %d\r\n", Param::GetInt(Param::bmsState));
            debug_printf("HV State: %d\r\n", Param::GetInt(Param::hvState));
            debug_printf("Safe Charge Current: %d A\r\n", Param::GetInt(Param::safeChargeCurrent));
            debug_println();
          }
          else if (inputBuffer == "bcc") {
            debug_println("\n=== BCC Status ===");
            debug_printf("BCC0: %s\r\n", bms->is_bcc0_initialized() ? "Initialized" : "NOT INITIALIZED");
            if (bms->is_bcc1_enabled()) {
              debug_printf("BCC1: %s\r\n", bms->is_bcc1_initialized() ? "Initialized" : "NOT INITIALIZED");
            } else {
              debug_println("BCC1: Disabled");
            }
            debug_println();
          }
          else if (inputBuffer == "bcc start") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized. Cannot start charging.\n");
            } else {
              debug_println();
              bms->start_charging();
              debug_println();
            }
          }
          else if (inputBuffer == "bcc stop") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized.\n");
            } else {
              debug_println();
              bms->stop_charging();
              debug_println();
            }
          }
          else if (inputBuffer == "bcc config dump") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized. Cannot dump configuration.\n");
            } else {
              debug_println("\nDumping BCC configuration...");
              debug_println("Copy this configuration array to replace TAYCAN_CONFIG:\n");
              dump_bcc_config(bms);
              debug_println();
            }
          }
          else if (inputBuffer == "bcc balance") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized. Cannot force balancing.\n");
            } else {
              debug_println();
              bms->force_balance_cells();
              debug_println();
            }
          }
          else if (inputBuffer == "bcc voltages" || inputBuffer.startsWith("bcc voltages ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized. Cannot read voltages.\n");
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

            debug_println("\n=== Cell Voltages ===");

            // Display specific module if requested
            if (module_num >= 0) {
              uint8_t bcc0_device_count = bcc0_config.device_count;
              uint8_t cell_count = bcc0_config.cell_count;
              uint8_t bcc_chain = (module_num < bcc0_device_count) ? 0 : 1;
              uint8_t module_in_chain = module_num % bcc0_device_count;
              uint8_t start_cell = module_num * cell_count;
              uint8_t end_cell = start_cell + cell_count;

              if (start_cell >= count) {
                debug_printf("Error: Module %d does not exist\r\n\n", module_num);
                continue;
              }

              debug_printf("--- Module %d (BCC%d Device %d) ---\r\n", module_num, bcc_chain, module_in_chain);

              uint32_t module_voltage = 0;
              uint32_t min_v = voltages_filtered[start_cell];
              uint32_t max_v = voltages_filtered[start_cell];

              for (uint8_t i = start_cell; i < end_cell && i < count; i++) {
                debug_printf("  Cell %d: %.4f V  (raw: %.4f V)\r\n",
                             (i % cell_count) + 1,
                             voltages_filtered[i] / 1000000.0f,
                             voltages_raw[i] / 1000000.0f);
                module_voltage += voltages_filtered[i];
                if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
              }

              float diff_mv = (max_v - min_v) / 1000.0f;
              debug_printf("\nModule Voltage: %.3f V\r\n", module_voltage / 1000000.0f);
              debug_printf("Min: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
                           min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
              debug_println();
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
                debug_println("--- BCC0 Module Summary ---");
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
                  debug_printf("  Module %d: %.3f V  (Δ %.2f mV)\r\n",
                               mod, module_voltage / 1000000.0f, diff_mv);
                  bcc0_voltage += module_voltage;
                }
                debug_printf("BCC0 Total: %.3f V\r\n\n", bcc0_voltage / 1000000.0f);
                total_voltage += bcc0_voltage;
              }

              // BCC1 modules
              if (bms->is_bcc1_initialized()) {
                debug_println("--- BCC1 Module Summary ---");
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
                  debug_printf("  Module %d: %.3f V  (Δ %.2f mV)\r\n",
                               module_num, module_voltage / 1000000.0f, diff_mv);
                  bcc1_voltage += module_voltage;
                }
                debug_printf("BCC1 Total: %.3f V\r\n\n", bcc1_voltage / 1000000.0f);
                total_voltage += bcc1_voltage;
              }

              // Overall statistics
              debug_println("--- Overall Statistics ---");
              debug_printf("Total Voltage (both chains): %.3f V\r\n", total_voltage / 1000000.0f);
              debug_printf("Total Cell Count: %d\r\n", count);

              uint32_t min_v = voltages_filtered[0];
              uint32_t max_v = voltages_filtered[0];
              for (uint8_t i = 1; i < count; i++) {
                if (voltages_filtered[i] < min_v) min_v = voltages_filtered[i];
                if (voltages_filtered[i] > max_v) max_v = voltages_filtered[i];
              }
              float diff_mv = (max_v - min_v) / 1000.0f;
              debug_printf("Min Cell: %.4f V, Max Cell: %.4f V, Diff: %.2f mV\r\n",
                           min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
              debug_println();
            }
          }
          else if (inputBuffer == "bcc config") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized.\n");
            } else {
              BMSChargingConfig config = bms->get_charging_config();
              debug_println("\n=== Charging Configuration ===");
              debug_printf("Target cell voltage:    %.2f V\r\n", config.target_cell_voltage);
              debug_printf("Balance threshold:      %.1f mV\r\n", config.balance_threshold_mv);
              debug_printf("Balance target:         %.1f mV\r\n", config.balance_target_mv);
              debug_printf("Balancing timer:        %d min\r\n", config.balancing_timer_min);
              debug_printf("Measurement interval:   %d ms\r\n", config.measurement_interval_ms);
              debug_println();
            }
          }
          else if (inputBuffer == "ivt") {
            debug_println("\n=== IVT Shunt Status ===");
            if (ivt_shunt != nullptr) {
              debug_printf("Status: %s\r\n", ivt_shunt->is_alive() ? "Online" : "OFFLINE");
              debug_printf("Current: %.2f A\r\n", ivt_shunt->get_current());
              debug_printf("Voltage 1 (Pack): %.2f V\r\n", ivt_shunt->get_voltage());
              debug_printf("Voltage 2 (HV Bus): %.2f V\r\n", ivt_shunt->get_voltage2());
              debug_printf("Voltage 3: %.2f V\r\n", ivt_shunt->get_voltage3());
              debug_printf("Power: %.2f kW\r\n", ivt_shunt->get_power());
              debug_printf("Temperature: %.1f C\r\n", ivt_shunt->get_temperature());
              debug_printf("Amp-Hours: %.3f Ah\r\n", ivt_shunt->get_amp_hours());
              debug_printf("Energy: %.3f kWh\r\n", ivt_shunt->get_kilowatt_hours());
              debug_printf("Frames Received: %lu\r\n", ivt_shunt->get_frame_count());
              debug_printf("Last Message: %lu ms ago\r\n", millis() - ivt_shunt->get_last_message_time());
            } else {
              debug_println("IVT shunt not configured");
            }
            debug_println();
          }
          else if (inputBuffer == "chademo") {
            debug_println("\n=== CHAdeMO Status ===");
            if (chademo != nullptr) {
              const char* state_str[] = {"Idle", "Connected", "Precharge", "Charging", "Ending", "Error", "Timeout"};
              debug_printf("State: %s\r\n", state_str[chademo->get_state()]);
              debug_printf("Connected: %s\r\n", chademo->is_connected() ? "Yes" : "No");
              debug_printf("Charging: %s\r\n", chademo->is_charging() ? "Yes" : "No");
              debug_printf("EVSE Voltage: %d V\r\n", chademo->get_evse_voltage());
              debug_printf("EVSE Current: %d A\r\n", chademo->get_evse_current());
              debug_printf("Time Since Last RX: %lu ms\r\n", chademo->get_time_since_last_rx());
              if (chademo->has_timeout()) {
                debug_println("WARNING: Communication timeout!");
              }
            } else {
              debug_println("CHAdeMO not configured");
            }
            debug_println();
          }
          else if (inputBuffer.startsWith("chademo start ")) {
            if (chademo == nullptr) {
              debug_println("\nError: CHAdeMO not configured\n");
            } else {
              String args = inputBuffer.substring(14);
              int spaceIdx = args.indexOf(' ');
              if (spaceIdx > 0) {
                uint16_t voltage = args.substring(0, spaceIdx).toInt();
                uint16_t current = args.substring(spaceIdx + 1).toInt();
                debug_printf("\nStarting CHAdeMO charge: %dV, %dA\r\n", voltage, current);
                chademo->start_charging(voltage, current);
              } else {
                debug_println("\nError: Usage: chademo start <volts> <amps>\n");
              }
            }
          }
          else if (inputBuffer == "chademo stop") {
            if (chademo == nullptr) {
              debug_println("\nError: CHAdeMO not configured\n");
            } else {
              debug_println("\nStopping CHAdeMO charge\n");
              chademo->stop_charging();
            }
          }
          else if (inputBuffer.startsWith("can sdo read ")) {
            if (can_sdo == nullptr) {
              debug_println("\nError: CanOpen SDO not configured\n");
            } else {
              // Parse: can sdo read <nodeId> <index> <subIndex>
              String args = inputBuffer.substring(13);
              int space1 = args.indexOf(' ');
              int space2 = args.indexOf(' ', space1 + 1);
              if (space1 > 0 && space2 > space1) {
                uint8_t nodeId = strtol(args.substring(0, space1).c_str(), NULL, 0);
                uint16_t index = strtol(args.substring(space1 + 1, space2).c_str(), NULL, 0);
                uint8_t subIndex = strtol(args.substring(space2 + 1).c_str(), NULL, 0);

                debug_printf("\nReading SDO: Node %d, Index 0x%04X, SubIndex %d\r\n", nodeId, index, subIndex);
                can_sdo->SDORead(nodeId, index, subIndex);

                // Wait for reply (with timeout)
                uint32_t start = millis();
                uint32_t data = 0;
                while (millis() - start < 1000) {
                  if (can_sdo->SDOReadReply(data)) {
                    debug_printf("SDO Reply: 0x%08X (%d)\r\n\n", data, (int32_t)data);
                    break;
                  }
                  delay(10);
                }
                if (millis() - start >= 1000) {
                  debug_println("SDO Read timeout\n");
                }
              } else {
                debug_println("\nError: Usage: can sdo read <nodeId> <index> <subIndex>\n");
              }
            }
          }
          else if (inputBuffer.startsWith("can sdo write ")) {
            if (can_sdo == nullptr) {
              debug_println("\nError: CanOpen SDO not configured\n");
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

                debug_printf("\nWriting SDO: Node %d, Index 0x%04X, SubIndex %d, Value 0x%08X\r\n", nodeId, index, subIndex, value);
                can_sdo->SDOWrite(nodeId, index, subIndex, value);
                debug_println("SDO Write sent\n");
              } else {
                debug_println("\nError: Usage: can sdo write <nodeId> <index> <subIndex> <value>\n");
              }
            }
          }
          else if (inputBuffer == "can map save") {
            if (can_map == nullptr) {
              debug_println("\nError: CAN mapping not configured\n");
            } else {
              debug_println("\nSaving CAN mappings to flash...");
              can_map->Save();
              debug_println("CAN mappings saved\n");
            }
          }
          else if (inputBuffer.startsWith("can node ")) {
            if (can_sdo == nullptr) {
              debug_println("\nError: CanOpen SDO not configured\n");
            } else {
              String arg = inputBuffer.substring(9);
              uint8_t nodeId = arg.toInt();
              if (nodeId > 0 && nodeId < 128) {
                can_sdo->SetNodeId(nodeId);
                Param::SetInt(Param::canNodeId, nodeId);
                debug_printf("\nCAN Node ID set to %d\r\n\n", nodeId);
              } else {
                debug_println("\nError: Node ID must be between 1 and 127\n");
              }
            }
          }
          else if (inputBuffer == "hv") {
            debug_println("\n=== HV System Status ===");
            const char* hv_state_str[] = {"Disabled", "Precharge", "Active", "Fault", "Shutdown"};
            debug_printf("HV State: %s\r\n", hv_state_str[bms->get_hv_state()]);

            if (ivt_shunt != nullptr && ivt_shunt->is_alive()) {
              float pack_voltage = ivt_shunt->get_voltage();
              float hv_bus_voltage = ivt_shunt->get_voltage2();
              float voltage_diff = abs(pack_voltage - hv_bus_voltage);

              debug_printf("Pack Voltage (V1): %.2f V\r\n", pack_voltage);
              debug_printf("HV Bus Voltage (V2): %.2f V\r\n", hv_bus_voltage);
              debug_printf("Voltage Difference: %.2f V\r\n", voltage_diff);
            } else {
              debug_println("IVT shunt not available - cannot read voltages");
            }

            // Show contactor states
            debug_printf("Positive Contactor: %s\r\n",
                         digitalRead(HV_CONTACTOR_1_PIN) == HIGH ? "CLOSED" : "OPEN");
            debug_printf("Negative Contactor: %s\r\n",
                         digitalRead(HV_CONTACTOR_2_PIN) == HIGH ? "CLOSED" : "OPEN");
            debug_println();
          }
          else if (inputBuffer == "bcc dump") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized. Cannot dump registers.\n");
            } else {
              debug_println();
              bms->dump_registers();
            }
          }
          else if (inputBuffer == "bcc faults") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized. Cannot read faults.\n");
            } else {
              debug_println();
              bms->print_fault_status();
            }
          }
          else if (inputBuffer == "bcc sleep") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized. Cannot enter sleep mode.\n");
            } else {
              debug_println("\nPutting BCC into sleep mode...");
              BMS_State result = bms->enable_sleep_mode();
              if (result == BMS_Sleep) {
                debug_println("BCC successfully entered sleep mode.");
                debug_println("Note: Use 'bcc wakeup' command to resume operation.");
              } else {
                debug_println("Failed to enter sleep mode.");
              }
              debug_println();
            }
          }
          else if (inputBuffer == "bcc wakeup") {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized. Cannot wake up.\n");
            } else {
              debug_println("\nWaking up BCC from sleep mode...");
              // The BCC wake_up() function is called during initialization
              // We need to re-initialize the BCC after wakeup
              debug_println("Re-initializing BCC...");
              // Note: This requires access to the BCC object and device configuration
              // For now, recommend using 'reboot' command for full system restart
              debug_println("Note: For full functionality, use 'reboot' command.");
              debug_println("BCC wakeup sequence requires full re-initialization.");
              debug_println();
            }
          }
          else if (inputBuffer == "reboot") {
            debug_println("\n=== System Reboot ===");

            // Safety shutdown sequence (same as DFU)
            debug_println("Stopping charging...");
            bms->stop_charging();
            delay(100);

            debug_println("Putting BCC into sleep mode...");
            bms->enable_sleep_mode();
            delay(100);

            debug_println("Performing software reset...");
            DebugSerial.flush();
            delay(100);

            // Perform software reset using NVIC
            NVIC_SystemReset();

            // Should never reach here
            while(1);
          }
          else if (inputBuffer.startsWith("bcc set target ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized.\n");
            } else {
              String value = inputBuffer.substring(15);
              float voltage = value.toFloat();
              if (voltage >= 2.5 && voltage <= 4.2) {
                Param::SetFloat(Param::targetCellVolt, voltage * 1000.0f);  // Convert V to mV
                debug_printf("\nTarget voltage set to %.2f V (will take effect after reboot)\r\n\n", voltage);
              } else {
                debug_println("\nError: Voltage must be between 2.5V and 4.2V\n");
              }
            }
          }
          else if (inputBuffer.startsWith("bcc set balance_th ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized.\n");
            } else {
              String value = inputBuffer.substring(19);
              float threshold = value.toFloat();
              if (threshold >= 1.0 && threshold <= 500.0) {
                Param::SetFloat(Param::balanceThreshold, threshold);
                debug_printf("\nBalance threshold set to %.1f mV (will take effect after reboot)\r\n\n", threshold);
              } else {
                debug_println("\nError: Threshold must be between 1.0 and 500.0 mV\n");
              }
            }
          }
          else if (inputBuffer.startsWith("bcc set balance_tgt ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized.\n");
            } else {
              String value = inputBuffer.substring(20);
              float target = value.toFloat();
              if (target >= 1.0 && target <= 100.0) {
                Param::SetFloat(Param::balanceTarget, target);
                debug_printf("\nBalance target set to %.1f mV (will take effect after reboot)\r\n\n", target);
              } else {
                debug_println("\nError: Target must be between 1.0 and 100.0 mV\n");
              }
            }
          }
          else if (inputBuffer.startsWith("bcc set interval ")) {
            if (!bms->is_bcc0_initialized() && !bms->is_bcc1_initialized()) {
              debug_println("\nError: No BCC initialized.\n");
            } else {
              String value = inputBuffer.substring(17);
              uint16_t interval = value.toInt();
              if (interval >= 100 && interval <= 10000) {
                Param::SetInt(Param::measureInterval, interval);
                debug_printf("\nMeasurement interval set to %d ms (will take effect after reboot)\r\n\n", interval);
              } else {
                debug_println("\nError: Interval must be between 100 and 10000 ms\n");
              }
            }
          }
          else {
            debug_printf("\nUnknown command: %s\r\n", inputBuffer.c_str());
            debug_println("Type 'help' for available commands\n");
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
  // Drive HV contactor pins safe FIRST — before serial, BCC, or task init.
  // nSLEEP=LOW keeps DRV8874 in sleep (outputs Hi-Z). Must be the very first
  // thing so contactors can never close due to a later init exception.
  pinMode(HV_CONTACTOR_1_PIN, OUTPUT);
  pinMode(HV_CONTACTOR_2_PIN, OUTPUT);
  pinMode(HV_CONTACTOR_NSLEEP_PIN, OUTPUT);
  pinMode(HV_CONTACTOR_FAULT_PIN, INPUT);
  digitalWrite(HV_CONTACTOR_1_PIN, LOW);
  digitalWrite(HV_CONTACTOR_2_PIN, LOW);
  digitalWrite(HV_CONTACTOR_NSLEEP_PIN, LOW);

  // Initialize Serial FIRST
  DebugSerial.begin(115200);
  debug_serial_init();
#ifdef DEBUG_WAIT_FOR_SERIAL
  delay(2000);
#endif

  debug_println("=== BMS Charging System for Dual 6S2P Packs ===");
  debug_println();

  // Initialize libopeninv parameter system
  debug_println("Initializing parameter system...");
  Param::LoadDefaults();
  int param_load_result = parm_load();
  if (param_load_result == 0) {
    debug_println("Parameters loaded from flash successfully!");
  } else {
    debug_println("No saved parameters found, using defaults");
  }

  // Set firmware version (store as major.minor float for web interface)
  float version_float = FW_VERSION_MAJOR + (FW_VERSION_MINOR / 10.0f);
  Param::SetFloat(Param::version, version_float);
  debug_printf("Firmware version: 0x%08X (v%d.%d.%d.%d) = %.1f\r\n",
                FIRMWARE_VERSION,
                FW_VERSION_MAJOR,
                FW_VERSION_MINOR,
                FW_VERSION_PATCH,
                FW_VERSION_BUILD,
                version_float);

  // Display STM32 unique device ID (used as serial number)
  debug_printf("Device Serial: %08X-%08X-%08X\r\n",
                STM32_UNIQUE_ID[0],
                STM32_UNIQUE_ID[1],
                STM32_UNIQUE_ID[2]);
  debug_println();

  // Configure BCC hardware from parameters
  debug_println("Configuring BCC hardware from parameters...");
  debug_printf("  bcc0DeviceCount (raw) = %d\r\n", Param::GetInt(Param::bcc0DeviceCount));
  debug_printf("  bcc0DeviceType (raw) = %d\r\n", Param::GetInt(Param::bcc0DeviceType));
  debug_printf("  bcc1DeviceCount (raw) = %d\r\n", Param::GetInt(Param::bcc1DeviceCount));
  debug_printf("  bcc1DeviceType (raw) = %d\r\n", Param::GetInt(Param::bcc1DeviceType));
  debug_println();

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

  debug_printf("  BCC0: %d x MC3377%d devices x %d cells = %d total cells%s\r\n",
                bcc0_config.device_count,
                bcc0_config.device_type == BCC_DEVICE_MC33771 ? 1 : 2,
                bcc0_config.cell_count,
                bcc0_config.device_count * bcc0_config.cell_count,
                bcc0_config.device_count == 0 ? " (DISABLED)" : "");
  debug_printf("  BCC1: %d x MC3377%d devices x %d cells = %d total cells%s\r\n",
                bcc1_config.device_count,
                bcc1_config.device_type == BCC_DEVICE_MC33771 ? 1 : 2,
                bcc1_config.cell_count,
                bcc1_config.device_count * bcc1_config.cell_count,
                bcc1_config.device_count == 0 ? " (DISABLED)" : "");

  uint8_t total_cells = (bcc0_config.device_count * bcc0_config.cell_count) +
                        (bcc1_config.device_count * bcc1_config.cell_count);
  debug_printf("  Total: %d cells\r\n", total_cells);

  // Initialize NeoPixel strip before the hard-fault check so we can blink LEDs.
  strip.begin();
  strip.show();

  if (total_cells == 0) {
    debug_println("  FATAL: No BCC devices configured! System cannot operate.");
    // Blink all LEDs red and halt — contactors are already confirmed open above.
    while (true) {
      for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, 0xFF0000);
      strip.show();
      delay(300);
      strip.clear(); strip.show();
      delay(300);
    }
  }
  debug_println();

  // Status LEDs initialized above; log it here in sequence.
  debug_println("Status LEDs initialized.");

  // Configure wakeup input
  pinMode(WAKEUP, INPUT);

  // HVIL interlock — pulled up; LOW = loop broken (connector removed)
  pinMode(HVIL_DETECT_PIN, INPUT_PULLUP);

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
  debug_println("Initializing CAN buses...");

  m3_can = new CANBus(M3_CAN_RX, M3_CAN_TX);
  if (!m3_can->begin(CAN_BPS_500K)) {
    debug_println("FATAL: Failed to initialize M3 CAN! System halted.");
    while (true) {
      for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, 0xFF0000);
      strip.show(); delay(300);
      strip.clear(); strip.show(); delay(300);
    }
  }
  debug_println("M3 CAN initialized at 500kbps");
  m3_can->watchFor();

  hv_can = new CANBus(HV_CAN_RX, HV_CAN_TX);
  if (!hv_can->begin(CAN_BPS_500K)) {
    debug_println("FATAL: Failed to initialize HV CAN! System halted.");
    while (true) {
      for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, 0xFF0000);
      strip.show(); delay(300);
      strip.clear(); strip.show(); delay(300);
    }
  }
  debug_println("HV CAN initialized at 500kbps");
  debug_println();
  hv_can->watchFor();

  // Initialize libopeninv CanOpen SDO for BMS CAN communication
  debug_println("Initializing CanOpen SDO for BMS communication...");
  hv_can_hardware = new CanHardwareArduino(m3_can);
  can_map = new CanMap(hv_can_hardware);
  can_sdo = new CanSdo(hv_can_hardware, can_map);
  can_sdo->SetNodeId(Param::GetInt(Param::canNodeId));  // Use node ID from parameters
  
  // Set up direct data source for on-demand JSON delivery (no buffer needed!)
  can_sdo->SetDataSource(get_json_byte);
  
  debug_printf("CanOpen SDO initialized on M3 CAN (Node ID: %d)\r\n", Param::GetInt(Param::canNodeId));
  debug_println();

  // Configure CAN message mappings for BMS spot values
  // debug_println("Configuring CAN message mappings for BMS telemetry...");
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
  
  debug_println("CAN mappings configured:");
  debug_println("  0x420: Pack voltage, current");
  debug_println("  0x421: Cell voltages (min/max/diff), SOC");
  debug_println("  0x422: BMS/HV state, initialization, faults, safe current");
  
  // Configure CAN receive mappings for IVT shunt messages
  // IVT format: Byte 0=MuxID, Byte 1=counter, Bytes 2-5=32-bit big-endian value
  // debug_println("Configuring CAN receive mappings for IVT shunt...");
  // can_map->AddRecv(Param::ivtCurrent, 0x521, 16, -32, 0.001f);      // mA -> A, big-endian
  // can_map->AddRecv(Param::ivtVoltage1, 0x522, 16, -32, 0.001f);     // mV -> V, big-endian
  // can_map->AddRecv(Param::ivtVoltage2, 0x523, 16, -32, 0.001f);     // mV -> V, big-endian
  // can_map->AddRecv(Param::ivtVoltage3, 0x524, 16, -32, 0.001f);     // mV -> V, big-endian
  // can_map->AddRecv(Param::ivtTemperature, 0x525, 16, -32, 0.1f);    // deci-deg -> C, big-endian
  // can_map->AddRecv(Param::ivtPower, 0x526, 16, -32, 0.001f);        // W -> kW, big-endian
  
  debug_println("  0x521: IVT Current (A)");
  debug_println("  0x522: IVT Voltage 1 (V)");
  debug_println("  0x523: IVT Voltage 2 (V)");
  debug_println("  0x524: IVT Voltage 3 (V)");
  debug_println("  0x525: IVT Temperature (C)");
  debug_println("  0x526: IVT Power (kW)");
  debug_println();

  // Initialize IVT current shunt
  debug_println("Initializing IVT current shunt...");
  ivt_shunt = new IVTShunt();
  ivt_shunt->begin(hv_can);
  debug_println("IVT shunt initialized");
  debug_println();

  // Initialize CHAdeMO controller on HV CAN
  debug_println("Initializing CHAdeMO controller (Foccci)...");
  chademo = new CHAdeMOController();
  chademo->begin(hv_can);
  debug_println("CHAdeMO controller initialized");
  debug_println();

  // Create BMS instance with both BCC0 and BCC1 enabled
  debug_println("Creating BMS instance for dual pack monitoring...");
  bms = new BatteryManagementSystem(&bcc0_config, &bcc1_config);

  // Set status LEDs
  bms->set_status_leds(&strip);

  // Display loaded charging parameters
  debug_println("Charging parameters loaded from parameter system:");
  debug_printf("  Target cell voltage: %.2f V\r\n", Param::GetFloat(Param::targetCellVolt) / 1000.0f);  // Convert mV to V
  debug_printf("  Balance threshold: %.1f mV\r\n", Param::GetFloat(Param::balanceThreshold));
  debug_printf("  Balance target: %.1f mV\r\n", Param::GetFloat(Param::balanceTarget));
  debug_printf("  Measurement interval: %d ms (%.1f Hz)\r\n",
                Param::GetInt(Param::measureInterval),
                1000.0f / Param::GetInt(Param::measureInterval));
  debug_printf("  Battery capacity: %.1f Ah\r\n", Param::GetFloat(Param::batteryCapacity));
  debug_printf("  Max charge current: %.1f A\r\n", Param::GetFloat(Param::maxChargeCurrent));
  debug_println();

  // Configure IVT shunt and CHAdeMO
  debug_println("Configuring IVT shunt and CHAdeMO with BMS...");
  bms->set_ivt_shunt(ivt_shunt);
  bms->set_chademo(chademo);
  bms->set_can_buses(m3_can, hv_can);

  // Configure contactor control pins
  debug_println("Configuring contactor control...");
  bms->set_contactor_pins(
    HV_CONTACTOR_1_PIN,
    HV_CONTACTOR_2_PIN,
    HV_CONTACTOR_NSLEEP_PIN,
    HV_CONTACTOR_FAULT_PIN
  );

  // Initialize BMS (BCC hardware initialization)
  debug_println("Initializing BMS hardware...");
  if (!bms->initialize(nullptr)) {
    debug_println("ERROR: BMS initialization failed!");
    debug_println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  debug_println("BMS initialized successfully!");
  debug_println();

  // Start BMS tasks
  debug_println("Starting BMS tasks...");
  if (!bms->start_tasks()) {
    debug_println("ERROR: Failed to start BMS tasks!");
    debug_println("System halted.");
  }

  debug_println("BMS tasks started successfully!");
  debug_println();

  // Read cell voltage limits from BCC hardware configuration
  debug_println("Reading pack voltage limits from BCC hardware...");
  bms->read_and_set_voltage_limits();
  debug_println();

  // Create CAN message queues
  debug_println("Creating CAN message queues...");
  m3_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));
  hv_can_queue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CAN_FRAME));

  if (!m3_can_queue || !hv_can_queue) {
    debug_println("ERROR: Failed to create CAN queues!");
    debug_println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  debug_println("CAN queues created");
  debug_println();

  // Create CAN RX polling task (high priority - reads from hardware)
  debug_println("Starting CAN RX task...");
  BaseType_t result = xTaskCreate(
    can_rx_task,
    "CAN RX",
    1024,
    NULL,
    4,  // Very high priority for hardware polling
    NULL
  );

  if (result != pdPASS) {
    debug_println("ERROR: Failed to create CAN RX task!");
    debug_println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  debug_println("CAN RX task started");
  debug_println();

  // Create IVT processing task (processes HV CAN messages for IVT)
  debug_println("Starting IVT processing task...");
  result = xTaskCreate(
    ivt_process_task,
    "IVT Proc",
    1536,
    NULL,
    3,  // High priority for message processing
    NULL
  );

  if (result != pdPASS) {
    debug_println("ERROR: Failed to create IVT processing task!");
    debug_println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  debug_println("IVT processing task started");
  debug_println();

  // Create CanOpen periodic task (sends mapped CAN messages)
  debug_println("Starting CanOpen periodic task...");
  result = xTaskCreate(
    canopen_periodic_task,
    "CanOpen",
    1024,
    NULL,
    2,  // Medium priority
    NULL
  );

  if (result != pdPASS) {
    debug_println("ERROR: Failed to create CanOpen periodic task!");
    debug_println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  debug_println("CanOpen periodic task started");
  debug_println();

  // Generate parameter JSON for web interface
  debug_println("Generating parameter JSON...");
  build_parameter_json();
  debug_println();

  // Create console task for user interaction
  debug_println("Starting serial console...");
  result = xTaskCreate(
    console_task,
    "Console",
    2048,
    NULL,
    1,  // Lower priority than BMS tasks
    NULL
  );

  if (result != pdPASS) {
    debug_println("ERROR: Failed to create console task!");
    debug_println("System halted.");
    while (1) {
      delay(1000);
    }
  }

  debug_println();
  debug_println("===========================================");
  debug_println("System ready!");
  debug_println("Type 'help' for available commands");
  debug_println("Type 'bcc start' to begin charging");
  debug_println("===========================================");
  debug_println();

  // Start the FreeRTOS scheduler.
  // The Arduino STM32 framework calls setup() directly from main() before the
  // scheduler runs (see framework-arduinoststm32/cores/arduino/main.cpp), so
  // this call is required and correct — it is NOT a duplicate invocation.
  debug_println("Starting FreeRTOS scheduler...");
  vTaskStartScheduler();

  // Should never reach here
  debug_println("ERROR: Scheduler failed to start!");
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

  debug_println("\n=== CSV Format: BCC Configuration Dump ===");

  // Print CSV header
  DebugSerial.print("BCC,CID");
  for (uint8_t i = 0; i < num_regs; i++) {
    DebugSerial.print(",");
    DebugSerial.print(reg_names[i]);
  }
  debug_println();

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
      DebugSerial.print(bcc);
      DebugSerial.print(",");
      DebugSerial.print(cid);

      // Read each register
      for (uint8_t i = 0; i < num_regs; i++) {
        uint16_t value = 0;
        bcc_status_t status = bms->read_bcc_register(bcc, (bcc_cid_t)cid, reg_addrs[i], &value);

        DebugSerial.print(",");
        if (status == BCC_STATUS_SUCCESS) {
          debug_printf("0x%04X", value);
        } else {
          DebugSerial.print("ERROR");
        }
      }
      debug_println();

      // Small delay to avoid overwhelming the serial buffer
      delay(10);
    }
  }

  debug_println("=== Dump Complete ===");
  debug_println("\nUse 'python3 capture_battery_data.py' to save this data to CSV file.");
}

void loop() {
  // Empty - FreeRTOS tasks run instead
}

