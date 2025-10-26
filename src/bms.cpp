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
  status_leds = nullptr;
  led_animation_step = 0;
  last_successful_measurement = 0;
  communication_timeout_ms = 5000; // 5 second timeout
  communication_lost = false;

  // Initialize cell voltage and balancing arrays
  memset(cell_voltages_uv, 0, sizeof(cell_voltages_uv));
  memset(cells_to_balance, 0, sizeof(cells_to_balance));

  // Set default charging config
  charging_config.target_cell_voltage = 3.6f;
  charging_config.balance_threshold_mv = 50.0f;
  charging_config.balance_target_mv = 10.0f;
  charging_config.balancing_timer_min = 5;
  charging_config.measurement_interval_ms = 1000;
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

void BatteryManagementSystem::set_contactor_pins(uint8_t negative, uint8_t positive, uint8_t enable, uint8_t fault) {
  negative_contactor_pin = negative;
  positive_contactor_pin = positive;
  contactor_enable_pin = enable;
  contactor_fault_pin = fault;

  pinMode(negative_contactor_pin, OUTPUT);
  pinMode(positive_contactor_pin, OUTPUT);
  pinMode(contactor_enable_pin, OUTPUT);
  pinMode(contactor_fault_pin, INPUT_PULLUP);

  digitalWrite(negative_contactor_pin, LOW);
  digitalWrite(positive_contactor_pin, LOW);
  digitalWrite(contactor_enable_pin, LOW);
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
        // Check if we've reached target voltage
        if (has_reached_target_voltage(cell_voltages_uv, bcc0_config->cell_count)) {
          Serial.println("BMS: Target voltage reached!");
          disable_contactors();
          current_state = BMS_Idle;
          break;
        }

        // Check cell voltage difference
        float max_diff_mv = get_max_cell_voltage_diff_mv(cell_voltages_uv, bcc0_config->cell_count);

        if (max_diff_mv > charging_config.balance_threshold_mv) {
          Serial.printf("BMS: Cell imbalance detected: %.2f mV (threshold: %.2f mV)\r\n",
                       max_diff_mv, charging_config.balance_threshold_mv);
          disable_contactors();
          current_state = BMS_CellBalancing;

          // Calculate which cells need balancing
          calculate_cell_balance_requirements(cell_voltages_uv, bcc0_config->cell_count, cells_to_balance);
          apply_cell_balancing(bcc0, cells_to_balance, bcc0_config->cell_count);
        }
        break;
      }

      case BMS_CellBalancing: {
        // Check if cells are balanced enough to resume charging
        float max_diff_mv = get_max_cell_voltage_diff_mv(cell_voltages_uv, bcc0_config->cell_count);

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
  const uint32_t PRINT_INTERVAL = 10; // Print voltages every 10 measurements

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

        // Only print cell voltages periodically to avoid console spam
        print_counter++;
        if (print_counter >= PRINT_INTERVAL) {
          print_counter = 0;

          // Calculate min/max for quick overview
          uint32_t min_v = cell_voltages_uv[0];
          uint32_t max_v = cell_voltages_uv[0];
          for (uint8_t i = 1; i < bcc0_config->cell_count; i++) {
            if (cell_voltages_uv[i] < min_v) min_v = cell_voltages_uv[i];
            if (cell_voltages_uv[i] > max_v) max_v = cell_voltages_uv[i];
          }
          float diff_mv = (max_v - min_v) / 1000.0f;

          Serial.printf("[Monitor] Stack: %.3f V | Min: %.4f V, Max: %.4f V, Diff: %.2f mV\r\n",
                       stack_voltage_uv / 1000000.0f,
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
    Serial.println("BMS: Cannot enable contactors - fault detected");
    return;
  }

  Serial.println("BMS: Enabling contactors");
  digitalWrite(contactor_enable_pin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(50)); // Small delay
  digitalWrite(negative_contactor_pin, HIGH);
  digitalWrite(positive_contactor_pin, HIGH);
}

void BatteryManagementSystem::disable_contactors() {
  Serial.println("BMS: Disabling contactors");
  digitalWrite(negative_contactor_pin, LOW);
  digitalWrite(positive_contactor_pin, LOW);
  digitalWrite(contactor_enable_pin, LOW);
}

void BatteryManagementSystem::control_contactors(bool enable_negative, bool enable_positive) {
  if (contactor_fault) {
    digitalWrite(negative_contactor_pin, LOW);
    digitalWrite(positive_contactor_pin, LOW);
    return;
  }

  digitalWrite(negative_contactor_pin, enable_negative ? HIGH : LOW);
  digitalWrite(positive_contactor_pin, enable_positive ? HIGH : LOW);
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
  // LEDs 0-2 for state indication
  for (uint8_t i = 0; i < 3; i++) {
    status_leds->setPixelColor(i, color);
  }
}

void BatteryManagementSystem::update_contactor_leds() {
  if (!status_leds) return;

  // LED 3: Negative contactor (Yellow when closed, off when open)
  bool neg_closed = digitalRead(negative_contactor_pin) == HIGH;
  set_led_color(3, neg_closed ? color_rgb(128, 128, 0) : color_rgb(0, 0, 0));

  // LED 4: Positive contactor (Cyan when closed, off when open)
  bool pos_closed = digitalRead(positive_contactor_pin) == HIGH;
  set_led_color(4, pos_closed ? color_rgb(0, 128, 128) : color_rgb(0, 0, 0));
}

void BatteryManagementSystem::led_pattern_idle() {
  // Soft blue breathing pattern on state LEDs (0-2)
  led_animation_step = (led_animation_step + 1) % 100;
  uint8_t brightness = (led_animation_step < 50) ? (led_animation_step * 2) : ((100 - led_animation_step) * 2);
  brightness = brightness / 4; // Keep it dim
  set_state_leds(color_rgb(0, 0, brightness));
}

void BatteryManagementSystem::led_pattern_charging() {
  // Green wave/chase pattern showing charging progress on state LEDs (0-2)
  led_animation_step = (led_animation_step + 1) % 3;

  for (uint8_t i = 0; i < 3; i++) {
    if (i == led_animation_step) {
      set_led_color(i, color_rgb(0, 255, 0)); // Bright green
    } else if (i == (led_animation_step + 2) % 3) {
      set_led_color(i, color_rgb(0, 64, 0)); // Dim green trailing
    } else {
      set_led_color(i, color_rgb(0, 16, 0)); // Very dim green
    }
  }
}

void BatteryManagementSystem::led_pattern_balancing() {
  // Yellow/Orange pulsing pattern on state LEDs (0-2)
  led_animation_step = (led_animation_step + 1) % 100;
  uint8_t brightness = (led_animation_step < 50) ? (led_animation_step * 5) : ((100 - led_animation_step) * 5);
  set_state_leds(color_rgb(brightness, brightness / 2, 0)); // Orange
}

void BatteryManagementSystem::led_pattern_complete() {
  // Solid green on state LEDs (0-2)
  set_state_leds(color_rgb(0, 128, 0)); // Medium green
}

void BatteryManagementSystem::led_pattern_error() {
  // Flashing red on state LEDs (0-2)
  led_animation_step = (led_animation_step + 1) % 60;
  if (led_animation_step < 30) {
    set_state_leds(color_rgb(255, 0, 0)); // Bright red
  } else {
    set_state_leds(color_rgb(0, 0, 0)); // Off
  }
}

void BatteryManagementSystem::update_status_leds() {
  if (!status_leds) return;

  // Update state LEDs (0-2) based on BMS state
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
      // Check if we're close to target
      if (has_reached_target_voltage(cell_voltages_uv, bcc0_config->cell_count)) {
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