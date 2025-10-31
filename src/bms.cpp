#include "bms.h"
#include "Arduino.h"

#define Serial SerialUSB

BatteryManagementSystem::BatteryManagementSystem(BatteryCellControllerConfig *config0, BatteryCellControllerConfig *config1) {
  bcc0_config = config0;
  devices_0 = new bcc_device_t[config0->device_count];
  for (uint8_t i = 0; i < config0->device_count; i++) {
    devices_0[i] = BCC_DEVICE_MC33772;
  }

  bcc0_tx_spi = new SPIClass(BMS0_TX_DATA, NC, BMS0_TX_SCK, NC);
  bcc0_rx_spi = new SPIClass(BMS0_RX_DATA, NC, BMS0_RX_SCK, BMS0_RX_CS);

  tpl0 = new TPLSPI(bcc0_tx_spi, bcc0_rx_spi, config0->cs_pin, configureDMA_HV_ECU);
  bcc0 = new BatteryCellController(tpl0, devices_0, config0->device_count, config0->cell_count, config0->enable_pin, config0->intb_pin, config0->loopback);

  bcc1_enabled = false;
  if (config1 != nullptr) {
    bcc1_config = config1;
    devices_1 = new bcc_device_t[config1->device_count];

    bcc1_tx_spi = new SPIClass(BMS1_TX_DATA, NC, BMS1_TX_SCK, NC);
    bcc1_rx_spi = new SPIClass(NC, BMS1_RX_DATA, BMS1_RX_SCK, BMS1_RX_CS);

    tpl1 = new TPLSPI(bcc1_tx_spi, bcc1_rx_spi, BMS1_TX_CS);
    bcc1 = new BatteryCellController(tpl1, devices_1, config1->device_count, config1->cell_count, config1->enable_pin, config1->intb_pin, config1->loopback);
    bcc1_enabled = true;
  } else {
    bcc1_config = nullptr;
    bcc1 = nullptr;
    tpl1 = nullptr;
  }

  current_state = BMS_Initialization;
  contactor_fault = false;
  hardware_initialized = false;
  stack_voltage_uv = 0;
  stack_voltage_filtered_uv = 0;
  voltage_filter_alpha = 0.2f;  // Default: 0.2 @ 50 Hz = smooth filtering with good responsiveness
  status_leds = nullptr;
  led_animation_step = 0;
  last_successful_measurement = 0;
  communication_timeout_ms = 5000; // 5 second timeout
  communication_lost = false;

  // Initialize cell voltage and balancing arrays
  memset(cell_voltages_uv, 0, sizeof(cell_voltages_uv));
  memset(cell_voltages_filtered_uv, 0, sizeof(cell_voltages_filtered_uv));
  memset(cells_to_balance, 0, sizeof(cells_to_balance));

  // Initialize fault tracking
  memset(fault_status, 0, sizeof(fault_status));
  has_overvoltage_fault = false;
  has_undervoltage_fault = false;
  has_temperature_fault = false;
  has_cb_open_fault = false;
  has_cb_short_fault = false;
  last_fault_check = 0;
  fault_check_interval_ms = 5000; // Check faults every 5 seconds

  // Set default charging config
  charging_config.target_cell_voltage = 3.6f;
  charging_config.balance_threshold_mv = 50.0f;
  charging_config.balance_target_mv = 10.0f;
  charging_config.balancing_timer_min = 5;
  charging_config.measurement_interval_ms = 20;  // 20ms = 50 Hz measurement rate
}

bool BatteryManagementSystem::initialize(uint16_t device_configuration[][BCC_INIT_CONF_REG_CNT]) {
  // Hardware initialization now happens in the monitor task after scheduler starts
  // This function is kept for compatibility but doesn't do hardware init anymore
  Serial.println("BMS: Configuration accepted (hardware init will occur after scheduler starts)");
  return true;
}

void BatteryManagementSystem::configure_settings(uint16_t config[][BCC_INIT_CONF_REG_CNT]) {
  // this->bcc0->init_devices(config);
}

void BatteryManagementSystem::set_charging_config(BMSChargingConfig config) {
  charging_config = config;
}

void BatteryManagementSystem::set_contactor_pins(uint8_t ph_pin, uint8_t en_pin, uint8_t nsleep_pin, uint8_t fault_pin) {
  negative_contactor_pin = ph_pin;      // Using existing variable for PH
  positive_contactor_pin = en_pin;      // Using existing variable for EN
  contactor_enable_pin = nsleep_pin;    // Using existing variable for nSLEEP
  contactor_fault_pin = fault_pin;

  pinMode(negative_contactor_pin, OUTPUT);  // PH pin
  pinMode(positive_contactor_pin, OUTPUT);  // EN pin
  pinMode(contactor_enable_pin, OUTPUT);    // nSLEEP pin
  pinMode(contactor_fault_pin, INPUT_PULLUP); // nFAULT pin (active LOW)

  // Initialize DRV8874 to disabled state
  digitalWrite(contactor_enable_pin, HIGH);    // nSLEEP HIGH to wake device
  digitalWrite(negative_contactor_pin, LOW);   // PH = LOW (direction doesn't matter when disabled)
  digitalWrite(positive_contactor_pin, LOW);   // EN = LOW (disabled)
}

void BatteryManagementSystem::set_status_leds(Adafruit_NeoPixel *leds) {
  status_leds = leds;
  if (status_leds) {
    status_leds->begin();
    // Turn off all LEDs
    for (uint8_t i = 0; i < status_leds->numPixels(); i++) {
      status_leds->setPixelColor(i, 0);
    }
    status_leds->show();
  }
}

bool BatteryManagementSystem::start_tasks() {
  Serial.println("BMS: Starting FreeRTOS tasks...");

  // Create master task
  BaseType_t result = xTaskCreate(
    master_task_wrapper,
    "BMS_Master",
    2048,
    this,
    2,
    &master_task_handle
  );

  if (result != pdPASS) {
    Serial.println("BMS: Failed to create master task");
    return false;
  }

  // Create BCC0 monitor task
  result = xTaskCreate(
    bcc0_monitor_task_wrapper,
    "BCC0_Monitor",
    2048,
    this,
    2,
    &bcc0_monitor_task_handle
  );

  if (result != pdPASS) {
    Serial.println("BMS: Failed to create BCC0 monitor task");
    return false;
  }

  // Create BCC1 monitor task if enabled
  if (bcc1_enabled) {
    result = xTaskCreate(
      bcc1_monitor_task_wrapper,
      "BCC1_Monitor",
      2048,
      this,
      2,
      &bcc1_monitor_task_handle
    );

    if (result != pdPASS) {
      Serial.println("BMS: Failed to create BCC1 monitor task");
      return false;
    }
  }

  Serial.println("BMS: All tasks created successfully");
  return true;
}

// Static task wrappers
void BatteryManagementSystem::master_task_wrapper(void *pvParameters) {
  BatteryManagementSystem *bms = static_cast<BatteryManagementSystem*>(pvParameters);
  bms->master_task_loop();
}

void BatteryManagementSystem::bcc0_monitor_task_wrapper(void *pvParameters) {
  BatteryManagementSystem *bms = static_cast<BatteryManagementSystem*>(pvParameters);
  bms->bcc0_monitor_task_loop();
}

void BatteryManagementSystem::bcc1_monitor_task_wrapper(void *pvParameters) {
  BatteryManagementSystem *bms = static_cast<BatteryManagementSystem*>(pvParameters);
  bms->bcc1_monitor_task_loop();
}

// Master task - manages charging state machine
void BatteryManagementSystem::master_task_loop() {
  Serial.println("BMS Master Task: Started");

  // Wait for hardware initialization to complete
  while (!hardware_initialized) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  Serial.println("BMS Master Task: Hardware initialized, starting state machine");

  while (true) {
    // Check for contactor fault
    if (digitalRead(contactor_fault_pin) == LOW) {
      if (!contactor_fault) {
        Serial.println("BMS: Contactor fault detected!");
        contactor_fault = true;
        disable_contactors();
        current_state = BMS_Error;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    switch (current_state) {
      case BMS_Idle:
        // Wait for user to start charging via console
        // Do nothing, just monitor
        break;

      case BMS_Charging: {
        // Use filtered voltages for decision making to avoid noise-induced state changes
        if (has_reached_target_voltage(cell_voltages_filtered_uv, bcc0_config->cell_count)) {
          Serial.println("BMS: Target voltage reached!");
          disable_contactors();
          current_state = BMS_Idle;
          break;
        }

        // Check cell voltage difference using filtered values
        float max_diff_mv = get_max_cell_voltage_diff_mv(cell_voltages_filtered_uv, bcc0_config->cell_count);

        if (max_diff_mv > charging_config.balance_threshold_mv) {
          Serial.printf("BMS: Cell imbalance detected: %.2f mV (threshold: %.2f mV)\r\n",
                       max_diff_mv, charging_config.balance_threshold_mv);
          disable_contactors();
          current_state = BMS_CellBalancing;

          // Calculate which cells need balancing using filtered voltages
          calculate_cell_balance_requirements(cell_voltages_filtered_uv, bcc0_config->cell_count, cells_to_balance);
          apply_cell_balancing(bcc0, cells_to_balance, bcc0_config->cell_count);
        }
        break;
      }

      case BMS_CellBalancing: {
        // Check if cells are balanced enough to resume charging (using filtered values)
        float max_diff_mv = get_max_cell_voltage_diff_mv(cell_voltages_filtered_uv, bcc0_config->cell_count);

        if (max_diff_mv <= charging_config.balance_target_mv) {
          Serial.printf("BMS: Cells balanced: %.2f mV (target: %.2f mV)\r\n",
                       max_diff_mv, charging_config.balance_target_mv);
          stop_cell_balancing(bcc0, bcc0_config->cell_count);
          current_state = BMS_Charging;
          enable_contactors();
        } else {
          Serial.printf("BMS: Balancing... Current difference: %.2f mV\r\n", max_diff_mv);
        }
        break;
      }

      case BMS_Error:
        // Stay in error state until reset
        Serial.println("BMS: In error state");
        disable_contactors();
        break;

      default:
        break;
    }

    // Update LED status
    update_status_leds();

    vTaskDelay(pdMS_TO_TICKS(charging_config.measurement_interval_ms));
  }
}

// BCC0 monitor task - reads cell voltages
void BatteryManagementSystem::bcc0_monitor_task_loop() {
  Serial.println("BCC0 Monitor Task: Started");

  // Perform hardware initialization here (after scheduler starts)
  if (!hardware_initialized) {
    Serial.println("BCC0: Initializing hardware...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for system to stabilize

    pinMode(bcc0_config->cs_pin, OUTPUT);
    digitalWrite(bcc0_config->cs_pin, HIGH);

    bcc_status_t error = bcc0->begin(nullptr);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("BCC0: Initialization failed: %d\r\n", error);
      current_state = BMS_Error;
      hardware_initialized = false;
    } else {
      Serial.println("BCC0: Initialization successful");
      hardware_initialized = true;
      current_state = BMS_Idle;
    }
  }

  static uint32_t print_counter = 0;
  const uint32_t PRINT_INTERVAL = 50; // Print voltages every 50 measurements (1 second @ 50 Hz)

  while (true) {
    if (hardware_initialized && current_state != BMS_Error) {
      bool voltage_ok = measure_cell_voltages(bcc0, cell_voltages_uv);
      bool stack_ok = measure_stack_voltage(bcc0, &stack_voltage_uv);

      if (voltage_ok && stack_ok) {
        // Successful measurement - update timestamp and clear comm lost flag
        last_successful_measurement = millis();
        if (communication_lost) {
          Serial.println("BCC0: Communication restored");
          communication_lost = false;
        }

        // Apply exponential filter to smooth measurements
        apply_exponential_filter();

        // Only print cell voltages periodically to avoid console spam
        print_counter++;
        if (print_counter >= PRINT_INTERVAL) {
          print_counter = 0;

          // Calculate min/max using filtered values for quick overview
          uint32_t min_v = cell_voltages_filtered_uv[0];
          uint32_t max_v = cell_voltages_filtered_uv[0];
          for (uint8_t i = 1; i < bcc0_config->cell_count; i++) {
            if (cell_voltages_filtered_uv[i] < min_v) min_v = cell_voltages_filtered_uv[i];
            if (cell_voltages_filtered_uv[i] > max_v) max_v = cell_voltages_filtered_uv[i];
          }
          float diff_mv = (max_v - min_v) / 1000.0f;

          Serial.printf("[Monitor] Stack: %.3f V | Min: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
                       stack_voltage_filtered_uv / 1000000.0f,
                       min_v / 1000000.0f, max_v / 1000000.0f, diff_mv);
        }
      } else {
        // Failed measurement - check for timeout
        if (!communication_lost && last_successful_measurement > 0) {
          uint32_t time_since_last = millis() - last_successful_measurement;
          if (time_since_last > communication_timeout_ms) {
            Serial.println("BCC0: Communication lost! Entering error state.");
            communication_lost = true;
            current_state = BMS_Error;
          }
        }
      }

      // Periodically check fault status
      uint32_t current_time = millis();
      if (current_time - last_fault_check >= fault_check_interval_ms) {
        if (read_fault_status(bcc0)) {
          check_faults();
          last_fault_check = current_time;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(charging_config.measurement_interval_ms));
  }
}

// BCC1 monitor task - disabled for now but keeps same structure
void BatteryManagementSystem::bcc1_monitor_task_loop() {
  Serial.println("BCC1 Monitor Task: Started (monitoring disabled)");

  while (true) {
    // BCC1 monitoring disabled for single pack charging
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

// Helper functions
bool BatteryManagementSystem::measure_cell_voltages(BatteryCellController *bcc, uint32_t *cell_voltages) {
  bcc_status_t error;
  bool completed;

  // Start conversion
  error = bcc->start_conversion_global_async(0x0717);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("Error starting conversion: %d\r\n", error);
    return false;
  }

  // Wait for conversion
  delayMicroseconds(600);

  // Check if completed
  do {
    error = bcc->is_converting(BCC_CID_DEV1, &completed);
    if (error != BCC_STATUS_SUCCESS) {
      return false;
    }
  } while (!completed);

  // Read cell voltages
  error = bcc->get_cell_voltages(BCC_CID_DEV1, cell_voltages);
  return (error == BCC_STATUS_SUCCESS);
}

bool BatteryManagementSystem::measure_stack_voltage(BatteryCellController *bcc, uint32_t *stack_voltage) {
  bcc_status_t error = bcc->get_stack_voltage(BCC_CID_DEV1, stack_voltage);
  return (error == BCC_STATUS_SUCCESS);
}

void BatteryManagementSystem::apply_exponential_filter() {
  // Apply exponential filter: filtered = alpha * new + (1 - alpha) * old
  // Alpha closer to 1 = less filtering (faster response)
  // Alpha closer to 0 = more filtering (smoother but slower response)

  // Filter stack voltage
  if (stack_voltage_filtered_uv == 0) {
    // First measurement - initialize filter with raw value
    stack_voltage_filtered_uv = stack_voltage_uv;
  } else {
    stack_voltage_filtered_uv = (uint32_t)(
      voltage_filter_alpha * stack_voltage_uv +
      (1.0f - voltage_filter_alpha) * stack_voltage_filtered_uv
    );
  }

  // Filter each cell voltage
  for (uint8_t i = 0; i < bcc0_config->cell_count; i++) {
    if (cell_voltages_filtered_uv[i] == 0) {
      // First measurement - initialize filter with raw value
      cell_voltages_filtered_uv[i] = cell_voltages_uv[i];
    } else {
      cell_voltages_filtered_uv[i] = (uint32_t)(
        voltage_filter_alpha * cell_voltages_uv[i] +
        (1.0f - voltage_filter_alpha) * cell_voltages_filtered_uv[i]
      );
    }
  }
}

bool BatteryManagementSystem::read_fault_status(BatteryCellController *bcc) {
  bcc_status_t error = bcc->get_fault_status(BCC_CID_DEV1, fault_status);
  return (error == BCC_STATUS_SUCCESS);
}

void BatteryManagementSystem::check_faults() {
  // Check for overvoltage faults (CELL_OV_FLT register, index 0)
  has_overvoltage_fault = (fault_status[BCC_FS_CELL_OV] != 0);

  // Check for undervoltage faults (CELL_UV_FLT register, index 1)
  has_undervoltage_fault = (fault_status[BCC_FS_CELL_UV] != 0);

  // Check for cell balancing open faults (CB_OPEN_FLT register, index 2)
  has_cb_open_fault = (fault_status[BCC_FS_CB_OPEN] != 0);

  // Check for cell balancing short faults (CB_SHORT_FLT register, index 3)
  has_cb_short_fault = (fault_status[BCC_FS_CB_SHORT] != 0);

  // Check for temperature faults (AN_OT_UT_FLT register, index 5)
  has_temperature_fault = (fault_status[BCC_FS_AN_OT_UT] != 0);
}

void BatteryManagementSystem::calculate_cell_balance_requirements(uint32_t *cell_voltages, uint8_t cell_count,
                                                                  uint8_t *cells_to_balance) {
  // Find minimum voltage
  uint32_t min_voltage = cell_voltages[0];
  for (uint8_t i = 1; i < cell_count; i++) {
    if (cell_voltages[i] < min_voltage) {
      min_voltage = cell_voltages[i];
    }
  }

  // Mark cells that are significantly higher than minimum
  for (uint8_t i = 0; i < cell_count; i++) {
    uint32_t diff_uv = cell_voltages[i] - min_voltage;
    float diff_mv = diff_uv / 1000.0f;

    if (diff_mv > charging_config.balance_target_mv) {
      cells_to_balance[i] = 1;
      Serial.printf("BMS: Cell %d needs balancing (%.2f mV above min)\r\n", i + 1, diff_mv);
    } else {
      cells_to_balance[i] = 0;
    }
  }
}

void BatteryManagementSystem::apply_cell_balancing(BatteryCellController *bcc, uint8_t *cells_to_balance,
                                                   uint8_t cell_count) {
  Serial.println("BMS: Applying cell balancing...");

  // Enable cell balancing module
  bcc->enable_cell_balancing(BCC_CID_DEV1, true);

  // Set balancing for each cell
  for (uint8_t i = 0; i < cell_count; i++) {
    if (cells_to_balance[i]) {
      bcc->set_cell_balancing(BCC_CID_DEV1, i, true, charging_config.balancing_timer_min);
      Serial.printf("BMS: Balancing cell %d enabled\r\n", i + 1);
    }
  }
}

void BatteryManagementSystem::stop_cell_balancing(BatteryCellController *bcc, uint8_t cell_count) {
  Serial.println("BMS: Stopping cell balancing...");

  // Disable balancing for all cells
  for (uint8_t i = 0; i < cell_count; i++) {
    bcc->set_cell_balancing(BCC_CID_DEV1, i, false, 0);
  }

  // Disable cell balancing module
  bcc->enable_cell_balancing(BCC_CID_DEV1, false);
}

float BatteryManagementSystem::get_max_cell_voltage_diff_mv(uint32_t *cell_voltages, uint8_t cell_count) {
  if (cell_count == 0) return 0.0f;

  uint32_t min_voltage = cell_voltages[0];
  uint32_t max_voltage = cell_voltages[0];

  for (uint8_t i = 1; i < cell_count; i++) {
    if (cell_voltages[i] < min_voltage) min_voltage = cell_voltages[i];
    if (cell_voltages[i] > max_voltage) max_voltage = cell_voltages[i];
  }

  return (max_voltage - min_voltage) / 1000.0f; // Convert uV to mV
}

bool BatteryManagementSystem::has_reached_target_voltage(uint32_t *cell_voltages, uint8_t cell_count) {
  uint32_t target_uv = (uint32_t)(charging_config.target_cell_voltage * 1000000.0f);

  for (uint8_t i = 0; i < cell_count; i++) {
    if (cell_voltages[i] < target_uv) {
      return false;
    }
  }

  return true;
}

void BatteryManagementSystem::enable_contactors() {
  if (contactor_fault) {
    Serial.println("BMS: Cannot enable contactor - fault detected");
    return;
  }

  Serial.println("BMS: Enabling contactor");
  // DRV8874 in independent half-bridge mode (PMODE floating)
  // To drive current through contactor: PH/IN2=HIGH, EN/IN1=LOW
  digitalWrite(contactor_enable_pin, HIGH);    // nSLEEP = HIGH (device awake)
  digitalWrite(negative_contactor_pin, HIGH);  // PH/IN2 = HIGH (OUT2 enabled)
  digitalWrite(positive_contactor_pin, LOW);   // EN/IN1 = LOW (OUT1 low)
}

void BatteryManagementSystem::disable_contactors() {
  Serial.println("BMS: Disabling contactor");
  // DRV8874 in independent half-bridge mode: both outputs LOW
  digitalWrite(positive_contactor_pin, LOW);   // EN/IN1 = LOW (OUT1 disabled)
  digitalWrite(negative_contactor_pin, LOW);   // PH/IN2 = LOW (OUT2 low)
  // Note: nSLEEP (contactor_enable_pin) stays HIGH to keep device awake
}

void BatteryManagementSystem::control_contactors(bool enable_negative, bool enable_positive) {
  // Note: This function is kept for compatibility but with DRV8874 H-bridge driver
  // we only have one contactor, so we enable if either signal is true
  if (contactor_fault) {
    digitalWrite(positive_contactor_pin, LOW);  // EN/IN1 = LOW (disabled)
    digitalWrite(negative_contactor_pin, LOW);  // PH/IN2 = LOW
    return;
  }

  if (enable_negative || enable_positive) {
    // Enable: PH/IN2=HIGH, EN/IN1=LOW (current flows OUT2→OUT1)
    digitalWrite(negative_contactor_pin, HIGH);  // PH/IN2 = HIGH
    digitalWrite(positive_contactor_pin, LOW);   // EN/IN1 = LOW
  } else {
    // Disable: both LOW
    digitalWrite(positive_contactor_pin, LOW);   // EN/IN1 = LOW
    digitalWrite(negative_contactor_pin, LOW);   // PH/IN2 = LOW
  }
}

void BatteryManagementSystem::start_charging() {
  if (current_state == BMS_Error) {
    Serial.println("BMS: Cannot start charging - system in error state");
    return;
  }

  if (current_state == BMS_Idle || current_state == BMS_Cooldown) {
    Serial.println("BMS: Starting charging cycle");
    enable_contactors();
    current_state = BMS_Charging;
  } else {
    const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Cooldown", "Sleep", "Error"};
    Serial.printf("BMS: Already in state: %s\r\n", state_str[current_state]);
  }
}

void BatteryManagementSystem::stop_charging() {
  Serial.println("BMS: User requested charging stop");
  disable_contactors();
  stop_cell_balancing(bcc0, bcc0_config->cell_count);
  current_state = BMS_Cooldown;
}

void BatteryManagementSystem::force_balance_cells() {
  if (current_state == BMS_Error) {
    Serial.println("BMS: Cannot balance - system in error state");
    return;
  }

  Serial.println("BMS: User requested cell balancing");
  disable_contactors();
  calculate_cell_balance_requirements(cell_voltages_uv, bcc0_config->cell_count, cells_to_balance);
  apply_cell_balancing(bcc0, cells_to_balance, bcc0_config->cell_count);
  current_state = BMS_CellBalancing;
}

void BatteryManagementSystem::get_cell_voltages(uint32_t *voltages, uint8_t *count) {
  *count = bcc0_config->cell_count;
  memcpy(voltages, cell_voltages_uv, bcc0_config->cell_count * sizeof(uint32_t));
}

void BatteryManagementSystem::get_cell_voltages_filtered(uint32_t *voltages, uint8_t *count) {
  *count = bcc0_config->cell_count;
  memcpy(voltages, cell_voltages_filtered_uv, bcc0_config->cell_count * sizeof(uint32_t));
}

void BatteryManagementSystem::set_voltage_filter_alpha(float alpha) {
  // Clamp alpha to valid range [0.01, 1.0]
  if (alpha < 0.01f) alpha = 0.01f;
  if (alpha > 1.0f) alpha = 1.0f;
  voltage_filter_alpha = alpha;
}

void BatteryManagementSystem::get_fault_status(uint16_t *faults) {
  memcpy(faults, fault_status, sizeof(fault_status));
}

bool BatteryManagementSystem::has_faults() const {
  return has_overvoltage_fault || has_undervoltage_fault ||
         has_temperature_fault || has_cb_open_fault || has_cb_short_fault;
}

void BatteryManagementSystem::dump_registers() {
  if (!hardware_initialized) {
    Serial.println("Error: BCC hardware not initialized");
    return;
  }

  Serial.println("\n========================================");
  Serial.println("BCC Configuration Register Dump");
  Serial.println("========================================\n");

  // Dump BCC0
  for (uint8_t dev = 0; dev < bcc0_config->device_count; dev++) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);
    bcc_device_t device_type = bcc0_config->devices[dev];

    Serial.printf("###############################################\n");
    Serial.printf("# BCC0 - CID %d (MC3377%s)\n", cid,
            (device_type == BCC_DEVICE_MC33771) ? "1" : "2");
    Serial.printf("###############################################\n\n");

    // Read INIT register
    uint16_t regVal;
    bcc_status_t error = bcc0->read_register(cid, BCC_REG_INIT_ADDR, 1U, &regVal);
    if (error == BCC_STATUS_SUCCESS) {
      Serial.printf("  %-25s | 0x%04X | 0x%02X%02X\n", "INIT", BCC_REG_INIT_ADDR,
                   regVal >> 8, regVal & 0xFFU);
    }

    Serial.println("  -------------------------------");
    Serial.println("  Register Name            | Addr   | Value");
    Serial.println("  -------------------------------");

    // Read all configuration registers based on device type
    if (device_type == BCC_DEVICE_MC33771) {
      for (uint8_t i = 0; i < REG_CONF_CNT_MC33771; i++) {
        error = bcc0->read_register(cid, BCC_REGISTERS_DATA_MC33771[i].address, 1U, &regVal);
        if (error == BCC_STATUS_SUCCESS) {
          Serial.printf("  %-25s | 0x%04X | 0x%02X%02X\n",
                       BCC_REGISTERS_DATA_MC33771[i].name,
                       BCC_REGISTERS_DATA_MC33771[i].address,
                       regVal >> 8, regVal & 0xFFU);
        } else {
          Serial.printf("  %-25s | 0x%04X | ERROR %d\n",
                       BCC_REGISTERS_DATA_MC33771[i].name,
                       BCC_REGISTERS_DATA_MC33771[i].address,
                       error);
        }
      }
    } else {
      for (uint8_t i = 0; i < REG_CONF_CNT_MC33772; i++) {
        error = bcc0->read_register(cid, BCC_REGISTERS_DATA_MC33772[i].address, 1U, &regVal);
        if (error == BCC_STATUS_SUCCESS) {
          Serial.printf("  %-25s | 0x%04X | 0x%02X%02X\n",
                       BCC_REGISTERS_DATA_MC33772[i].name,
                       BCC_REGISTERS_DATA_MC33772[i].address,
                       regVal >> 8, regVal & 0xFFU);
        } else {
          Serial.printf("  %-25s | 0x%04X | ERROR %d\n",
                       BCC_REGISTERS_DATA_MC33772[i].name,
                       BCC_REGISTERS_DATA_MC33772[i].address,
                       error);
        }
      }
    }

    Serial.println("  -------------------------------\n");

    // Read GUID
    uint64_t guid;
    error = bcc0->read_guid(cid, &guid);
    if (error == BCC_STATUS_SUCCESS) {
      Serial.printf("  Device GUID: 0x%02X%04X%04X\n",
              (uint16_t)((guid >> 32) & 0x001FU),
              (uint16_t)((guid >> 16) & 0xFFFFU),
              (uint16_t)(guid & 0xFFFFU));
    }

    Serial.println();
  }

  Serial.println("========================================");
  Serial.println("Fuse Mirror Data");
  Serial.println("========================================\n");

  // Dump fuse mirror data for BCC0
  for (uint8_t dev = 0; dev < bcc0_config->device_count; dev++) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);
    bcc_device_t device_type = bcc0_config->devices[dev];

    Serial.printf("###############################################\n");
    Serial.printf("# BCC0 - CID %d Fuse Mirror\n", cid);
    Serial.printf("###############################################\n\n");

    Serial.println("  -------------------------------");
    Serial.println("  Fuse Address         | Value");
    Serial.println("  -------------------------------");

    // Read all fuse mirror addresses (0x00 - 0x1F)
    // MC33771C has fewer fuse addresses than MC33772C
    uint8_t max_fuse_addr = (device_type == BCC_DEVICE_MC33771) ? 0x17 : 0x1F;

    for (uint8_t addr = 0x00; addr <= max_fuse_addr; addr++) {
      uint16_t fuseVal;
      bcc_status_t fuse_error = bcc0->read_fuse_mirror(cid, addr, &fuseVal);
      if (fuse_error == BCC_STATUS_SUCCESS) {
        Serial.printf("  0x%02X                 | 0x%04X\n", addr, fuseVal);
      } else {
        Serial.printf("  0x%02X                 | ERROR %d\n", addr, fuse_error);
      }
    }

    Serial.println("  -------------------------------\n");
  }

  Serial.println("========================================");
  Serial.println("Dump complete");
  Serial.println("========================================\n");
}

void BatteryManagementSystem::print_fault_status() {
  Serial.println("\n=== Fault Status ===");

  // Overall status
  Serial.printf("Overall Status: %s\r\n", has_faults() ? "FAULTS DETECTED" : "OK");
  Serial.println();

  // Cell overvoltage faults
  Serial.printf("Cell Overvoltage:  0x%04X", fault_status[BCC_FS_CELL_OV]);
  if (has_overvoltage_fault) {
    Serial.print(" [FAULT]");
    // Print which cells have OV fault (each bit represents a cell)
    Serial.print(" - Cells: ");
    for (uint8_t i = 0; i < bcc0_config->cell_count; i++) {
      if (fault_status[BCC_FS_CELL_OV] & (1 << i)) {
        Serial.printf("%d ", i + 1);
      }
    }
  }
  Serial.println();

  // Cell undervoltage faults
  Serial.printf("Cell Undervoltage: 0x%04X", fault_status[BCC_FS_CELL_UV]);
  if (has_undervoltage_fault) {
    Serial.print(" [FAULT]");
    // Print which cells have UV fault
    Serial.print(" - Cells: ");
    for (uint8_t i = 0; i < bcc0_config->cell_count; i++) {
      if (fault_status[BCC_FS_CELL_UV] & (1 << i)) {
        Serial.printf("%d ", i + 1);
      }
    }
  }
  Serial.println();

  // Temperature faults
  Serial.printf("Temperature Faults: 0x%04X", fault_status[BCC_FS_AN_OT_UT]);
  if (has_temperature_fault) {
    Serial.print(" [FAULT]");
    // Print which ANx pins have temperature fault
    Serial.print(" - AN pins: ");
    for (uint8_t i = 0; i < 7; i++) {
      if (fault_status[BCC_FS_AN_OT_UT] & (1 << i)) {
        Serial.printf("AN%d ", i);
      }
    }
  }
  Serial.println();

  // Cell balancing faults
  Serial.printf("CB Open Fault:     0x%04X", fault_status[BCC_FS_CB_OPEN]);
  if (fault_status[BCC_FS_CB_OPEN] != 0) Serial.print(" [FAULT]");
  Serial.println();

  Serial.printf("CB Short Fault:    0x%04X", fault_status[BCC_FS_CB_SHORT]);
  if (fault_status[BCC_FS_CB_SHORT] != 0) Serial.print(" [FAULT]");
  Serial.println();

  // GPIO status
  Serial.printf("GPIO Status:       0x%04X", fault_status[BCC_FS_GPIO_STATUS]);
  Serial.println();

  // Communication status
  Serial.printf("Comm Status:       0x%04X", fault_status[BCC_FS_COMM]);
  if (fault_status[BCC_FS_COMM] != 0) Serial.print(" [ERRORS]");
  Serial.println();

  // General fault status registers
  Serial.printf("Fault1 Status:     0x%04X", fault_status[BCC_FS_FAULT1]);
  if (fault_status[BCC_FS_FAULT1] != 0) Serial.print(" [FAULT]");
  Serial.println();

  Serial.printf("Fault2 Status:     0x%04X", fault_status[BCC_FS_FAULT2]);
  if (fault_status[BCC_FS_FAULT2] != 0) Serial.print(" [FAULT]");
  Serial.println();

  Serial.printf("Fault3 Status:     0x%04X", fault_status[BCC_FS_FAULT3]);
  if (fault_status[BCC_FS_FAULT3] != 0) Serial.print(" [FAULT]");
  Serial.println();

  Serial.println();
}

BMS_State BatteryManagementSystem::enable_sleep_mode() {
  auto result = this->bcc0->enter_low_power_mode() == BCC_STATUS_SUCCESS;
  if (result)
    current_state = BMS_Sleep;

  return current_state;
}

// LED Control Functions
uint32_t BatteryManagementSystem::color_rgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void BatteryManagementSystem::set_led_color(uint8_t led, uint32_t color) {
  if (status_leds && led < status_leds->numPixels()) {
    status_leds->setPixelColor(led, color);
  }
}

void BatteryManagementSystem::set_state_leds(uint32_t color) {
  if (!status_leds) return;
  // LEDs 0-3 for state indication
  for (uint8_t i = 0; i < 4; i++) {
    status_leds->setPixelColor(i, color);
  }
}

void BatteryManagementSystem::update_contactor_leds() {
  if (!status_leds) return;

  // Single contactor with DRV8874 H-bridge driver
  // Contactor is enabled when PH/IN2 pin is HIGH
  bool ph_high = digitalRead(negative_contactor_pin) == HIGH;  // PH/IN2 pin

  // LED 4: Contactor status (Yellow when enabled, off when disabled)
  set_led_color(4, ph_high ? color_rgb(255, 255, 0) : color_rgb(0, 0, 0));
}

void BatteryManagementSystem::led_pattern_idle() {
  // Soft blue breathing pattern on state LEDs (0-3)
  led_animation_step = (led_animation_step + 1) % 100;
  uint8_t brightness = (led_animation_step < 50) ? (led_animation_step * 2) : ((100 - led_animation_step) * 2);
  brightness = brightness / 4; // Keep it dim
  set_state_leds(color_rgb(0, 0, brightness));
}

void BatteryManagementSystem::led_pattern_charging() {
  // Green wave/chase pattern showing charging progress on state LEDs (0-3)
  led_animation_step = (led_animation_step + 1) % 4;

  for (uint8_t i = 0; i < 4; i++) {
    if (i == led_animation_step) {
      set_led_color(i, color_rgb(0, 255, 0)); // Bright green
    } else if (i == (led_animation_step + 3) % 4) {
      set_led_color(i, color_rgb(0, 64, 0)); // Dim green trailing
    } else {
      set_led_color(i, color_rgb(0, 16, 0)); // Very dim green
    }
  }
}

void BatteryManagementSystem::led_pattern_balancing() {
  // Yellow/Orange pulsing pattern on state LEDs (0-3)
  led_animation_step = (led_animation_step + 1) % 100;
  uint8_t brightness = (led_animation_step < 50) ? (led_animation_step * 5) : ((100 - led_animation_step) * 5);
  set_state_leds(color_rgb(brightness, brightness / 2, 0)); // Orange
}

void BatteryManagementSystem::led_pattern_complete() {
  // Solid green on state LEDs (0-3)
  set_state_leds(color_rgb(0, 128, 0)); // Medium green
}

void BatteryManagementSystem::led_pattern_error() {
  // Flashing red on state LEDs (0-3)
  led_animation_step = (led_animation_step + 1) % 60;
  if (led_animation_step < 30) {
    set_state_leds(color_rgb(255, 0, 0)); // Bright red
  } else {
    set_state_leds(color_rgb(0, 0, 0)); // Off
  }
}

void BatteryManagementSystem::update_status_leds() {
  if (!status_leds) return;

  // Update state LEDs (0-3) based on BMS state
  switch (current_state) {
    case BMS_Initialization:
      // Purple - system initializing
      set_state_leds(color_rgb(128, 0, 128));
      break;

    case BMS_Idle:
    case BMS_Cooldown:
      led_pattern_idle();
      break;

    case BMS_Charging:
      // Check if we're close to target using filtered voltages
      if (has_reached_target_voltage(cell_voltages_filtered_uv, bcc0_config->cell_count)) {
        led_pattern_complete();
      } else {
        led_pattern_charging();
      }
      break;

    case BMS_CellBalancing:
      led_pattern_balancing();
      break;

    case BMS_Error:
      led_pattern_error();
      break;

    case BMS_Sleep:
      // All state LEDs off
      set_state_leds(0);
      break;

    default:
      break;
  }

  // Update contactor LEDs (3-4)
  update_contactor_leds();

  status_leds->show();
}