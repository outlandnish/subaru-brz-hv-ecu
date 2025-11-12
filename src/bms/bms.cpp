#include "bms.h"
#include "Arduino.h"

#define Serial SerialUSB

BatteryManagementSystem::BatteryManagementSystem(BatteryCellControllerConfig *config0, BatteryCellControllerConfig *config1) {
  bcc0_config = config0;
  devices_0 = new bcc_device_t[config0->device_count];
  for (uint8_t i = 0; i < config0->device_count; i++) {
    devices_0[i] = BCC_DEVICE_MC33772;
  }

  bcc0_tx_spi = new SPIClass(BCC0_TX_DATA, NC, BCC0_TX_SCK, NC);
  bcc0_rx_spi = new SPIClass(BCC0_RX_DATA, NC, BCC0_RX_SCK, BCC0_RX_CS);

  tpl0 = new TPLSPI(bcc0_tx_spi, bcc0_rx_spi, config0->cs_pin, configureDMA_HV_ECU);
  bcc0 = new BatteryCellController(tpl0, devices_0, config0->device_count, config0->cell_count, config0->enable_pin, config0->intb_pin, config0->loopback);

  bcc1_config = config1;
  devices_1 = new bcc_device_t[config1->device_count];
  for (uint8_t i = 0; i < config1->device_count; i++) {
    devices_1[i] = BCC_DEVICE_MC33772;
  }

  bcc1_tx_spi = new SPIClass(BCC1_TX_DATA, NC, BCC1_TX_SCK, NC);
  bcc1_rx_spi = new SPIClass(BCC1_RX_DATA, NC, BCC1_RX_SCK, BCC1_RX_CS);

  tpl1 = new TPLSPI(bcc1_tx_spi, bcc1_rx_spi, config1->cs_pin, configureDMA_HV_ECU);
  bcc1 = new BatteryCellController(tpl1, devices_1, config1->device_count, config1->cell_count, config1->enable_pin, config1->intb_pin, config1->loopback);

  current_state = BMS_Initialization;
  hv_state = HV_Disabled;
  hv_mode = HV_MODE_CHARGING;  // Default to charging mode
  hv_state_entry_time = 0;
  precharge_start_time = 0;
  contactor_fault = false;
  hardware_initialized = false;
  bcc0_initialized = false;
  bcc1_initialized = false;
  stack_voltage_uv = 0;
  stack_voltage_filtered_uv = 0;
  voltage_filter_alpha = 0.2f;  // Default: 0.2 @ 50 Hz = smooth filtering with good responsiveness
  status_leds = nullptr;
  led_animation_step = 0;
  last_successful_measurement = 0;
  communication_timeout_ms = 5000; // 5 second timeout
  communication_lost = false;

  // Initialize EVSE and IVT pointers
  evse = nullptr;
  ivt_shunt = nullptr;
  ipc_can = nullptr;
  m3_can = nullptr;
  hv_can = nullptr;

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

  // Set default HV connection config
  hv_config.precharge_voltage_margin_v = 10.0f;  // 10V margin for precharge completion
  hv_config.precharge_timeout_ms = 5000;         // 5 second timeout
  hv_config.precharge_check_interval_ms = 100;   // Check every 100ms

  // Initialize PWM contactor control
  positive_contactor_timer = nullptr;
  negative_contactor_timer = nullptr;
  contactors_use_pwm = false;
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

void BatteryManagementSystem::set_contactor_pins(uint8_t contactor1_pin, uint8_t contactor2_pin, uint8_t nsleep_pin, uint8_t fault_pin) {
  positive_contactor_pin = contactor1_pin;  // IN1 controls OUT1 for contactor 1
  negative_contactor_pin = contactor2_pin;  // IN2 controls OUT2 for contactor 2
  contactor_enable_pin = nsleep_pin;        // nSLEEP
  contactor_fault_pin = fault_pin;

  pinMode(contactor_enable_pin, OUTPUT);       // nSLEEP pin
  pinMode(contactor_fault_pin, INPUT_PULLUP);  // nFAULT pin (active LOW)

  // Initialize DRV8874 to disabled state
  digitalWrite(contactor_enable_pin, HIGH);    // nSLEEP HIGH to wake device

  // Initialize PWM for both contactors
  PinName pos_pin = digitalPinToPinName(positive_contactor_pin);
  TIM_TypeDef *pos_instance = (TIM_TypeDef *)pinmap_peripheral(pos_pin, PinMap_PWM);

  PinName neg_pin = digitalPinToPinName(negative_contactor_pin);
  TIM_TypeDef *neg_instance = (TIM_TypeDef *)pinmap_peripheral(neg_pin, PinMap_PWM);

  if (pos_instance != nullptr && neg_instance != nullptr) {
    // Setup positive contactor PWM
    positive_contactor_channel = STM_PIN_CHANNEL(pinmap_function(pos_pin, PinMap_PWM));
    positive_contactor_timer = new HardwareTimer(pos_instance);
    positive_contactor_timer->setMode(positive_contactor_channel, TIMER_OUTPUT_COMPARE_PWM1, positive_contactor_pin);
    positive_contactor_timer->setOverflow(CONTACTOR_PWM_FREQ, HERTZ_FORMAT);
    positive_contactor_timer->setCaptureCompare(positive_contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    positive_contactor_timer->pause();

    // Setup negative contactor PWM
    negative_contactor_channel = STM_PIN_CHANNEL(pinmap_function(neg_pin, PinMap_PWM));
    negative_contactor_timer = new HardwareTimer(neg_instance);
    negative_contactor_timer->setMode(negative_contactor_channel, TIMER_OUTPUT_COMPARE_PWM1, negative_contactor_pin);
    negative_contactor_timer->setOverflow(CONTACTOR_PWM_FREQ, HERTZ_FORMAT);
    negative_contactor_timer->setCaptureCompare(negative_contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    negative_contactor_timer->pause();

    contactors_use_pwm = true;
    Serial.println("BMS: Contactor PWM economizer enabled");
  } else {
    // Fall back to digital control
    pinMode(positive_contactor_pin, OUTPUT);
    pinMode(negative_contactor_pin, OUTPUT);
    digitalWrite(positive_contactor_pin, LOW);
    digitalWrite(negative_contactor_pin, LOW);
    contactors_use_pwm = false;
    Serial.println("BMS: Using digital contactor control (PWM not available)");
  }
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

void BatteryManagementSystem::set_evse(EVSEController *evse_controller) {
  evse = evse_controller;
}


void BatteryManagementSystem::set_ivt_shunt(IVTShunt *shunt) {
  ivt_shunt = shunt;
}

void BatteryManagementSystem::set_can_buses(CANBus *ipc_can_bus, CANBus *m3_can_bus, CANBus *hv_can_bus) {
  ipc_can = ipc_can_bus;
  m3_can = m3_can_bus;
  hv_can = hv_can_bus;
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

  // Create BCC1 monitor task
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

  // Create HV CAN task
  result = xTaskCreate(
    hv_can_task_wrapper,
    "HV_CAN",
    2048,
    this,
    1,  // Lower priority than monitor tasks
    &hv_can_task_handle
  );

  if (result != pdPASS) {
    Serial.println("BMS: Failed to create HV CAN task");
    return false;
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

void BatteryManagementSystem::hv_can_task_wrapper(void *pvParameters) {
  BatteryManagementSystem *bms = static_cast<BatteryManagementSystem*>(pvParameters);
  bms->hv_can_task_loop();
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
    // Update EVSE status if configured
    if (evse != nullptr) {
      evse->update();
    }

    // Update HV state machine
    update_hv_state();

    // Check for contactor fault
    if (digitalRead(contactor_fault_pin) == LOW) {
      if (!contactor_fault) {
        Serial.println("BMS: Contactor fault detected!");
        contactor_fault = true;
        hv_disconnect();  // This will disable contactors and stop PCS
        current_state = BMS_Error;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    switch (current_state) {
      case BMS_Idle:
        // Wait for user to start charging via console
        // Check if EVSE is connected and ready
        if (evse != nullptr && evse->is_ready_to_charge()) {
          uint16_t available_current = evse->get_max_charge_current_ma();
          Serial.printf("BMS: EVSE ready - %d mA available\r\n", available_current);
        }
        break;

      case BMS_Charging: {
        // Check if EVSE is still connected (if configured)
        if (evse != nullptr && !evse->is_ready_to_charge()) {
          Serial.println("BMS: EVSE disconnected - stopping charge");
          hv_disconnect();  // This will disable contactors and stop PCS
          current_state = BMS_Idle;
          break;
        }

        // Use filtered voltages for decision making to avoid noise-induced state changes
        if (has_reached_target_voltage(cell_voltages_filtered_uv, bcc0_config->cell_count)) {
          Serial.println("BMS: Target voltage reached!");
          hv_disconnect();  // This will disable contactors and stop PCS
          current_state = BMS_Idle;
          break;
        }

        // Update PCS with current battery voltage and power request
        // Set target voltage slightly above current voltage for CV charging
        uint32_t target_v_mv = (uint32_t)(charging_config.target_cell_voltage * bcc0_config->cell_count * 1000.0f);
        PCSController::set_hv_voltage_async(target_v_mv / 1000);  // Convert mV to V

        // Calculate charge power based on available current from EVSE
        if (evse != nullptr) {
          uint16_t available_current_ma = evse->get_max_charge_current_ma();
          uint16_t stack_voltage_v = stack_voltage_filtered_uv / 1000000;
          uint16_t charge_power_w = (available_current_ma * stack_voltage_v) / 1000;  // P = I * V
          PCSController::set_charge_power_async(charge_power_w);
        }

        // Check cell voltage difference using filtered values
        float max_diff_mv = get_max_cell_voltage_diff_mv(cell_voltages_filtered_uv, bcc0_config->cell_count);

        if (max_diff_mv > charging_config.balance_threshold_mv) {
          Serial.printf("BMS: Cell imbalance detected: %.2f mV (threshold: %.2f mV)\r\n",
                       max_diff_mv, charging_config.balance_threshold_mv);
          hv_disconnect();  // This will disable contactors and stop PCS
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
  // Perform hardware initialization here (after scheduler starts)
  if (!hardware_initialized) {
    Serial.println("BCC0: Initializing...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for system to stabilize

    pinMode(bcc0_config->cs_pin, OUTPUT);
    digitalWrite(bcc0_config->cs_pin, HIGH);

    bcc_status_t error = bcc0->begin(nullptr);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("BCC0: Init failed (error %d)\r\n", error);
      current_state = BMS_Error;
      hardware_initialized = false;
      bcc0_initialized = false;
    } else {
      Serial.println("BCC0: Ready");
      hardware_initialized = true;
      bcc0_initialized = true;
      current_state = BMS_Idle;
    }
  }

  while (true) {
    if (hardware_initialized && current_state != BMS_Error) {
      bool voltage_ok = measure_cell_voltages(bcc0, cell_voltages_uv);
      bool stack_ok = measure_stack_voltage(bcc0, &stack_voltage_uv);

      if (voltage_ok && stack_ok) {
        // Successful measurement - update timestamp and clear comm lost flag
        last_successful_measurement = millis();
        if (communication_lost) {
          Serial.println("BCC0: Comm restored");
          communication_lost = false;
        }

        // Apply exponential filter to smooth measurements
        apply_exponential_filter();
      } else {
        // Failed measurement - check for timeout
        if (!communication_lost && last_successful_measurement > 0) {
          uint32_t time_since_last = millis() - last_successful_measurement;
          if (time_since_last > communication_timeout_ms) {
            Serial.println("BCC0: Comm lost!");
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

// BCC1 monitor task
void BatteryManagementSystem::bcc1_monitor_task_loop() {
  // Perform hardware initialization here (after scheduler starts)
  if (!bcc1_initialized) {
    Serial.println("BCC1: Waiting for BCC0...");
    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for BCC0 to init first

    Serial.println("BCC1: Initializing...");
    pinMode(bcc1_config->cs_pin, OUTPUT);
    digitalWrite(bcc1_config->cs_pin, HIGH);

    bcc_status_t error = bcc1->begin(nullptr);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("BCC1: Init failed (error %d)\r\n", error);
      bcc1_initialized = false;
    } else {
      Serial.println("BCC1: Ready");
      bcc1_initialized = true;
    }
  }

  const uint8_t CELL_OFFSET = 6; // BCC1 cells stored at positions 6-11

  while (true) {
    if (bcc1_initialized && hardware_initialized && current_state != BMS_Error) {
      uint32_t bcc1_cell_voltages[BCC_MAX_CELLS];
      uint32_t bcc1_stack_voltage;

      bool voltage_ok = measure_cell_voltages(bcc1, bcc1_cell_voltages);
      bool stack_ok = measure_stack_voltage(bcc1, &bcc1_stack_voltage);

      if (voltage_ok && stack_ok) {
        // Store BCC1 cells at offset 6 in the main arrays
        for (uint8_t i = 0; i < bcc1_config->cell_count; i++) {
          cell_voltages_uv[CELL_OFFSET + i] = bcc1_cell_voltages[i];
        }

        // Apply exponential filter for BCC1 cells
        for (uint8_t i = 0; i < bcc1_config->cell_count; i++) {
          uint8_t idx = CELL_OFFSET + i;
          if (cell_voltages_filtered_uv[idx] == 0) {
            // First measurement - initialize filter
            cell_voltages_filtered_uv[idx] = cell_voltages_uv[idx];
          } else {
            // Apply exponential filter
            cell_voltages_filtered_uv[idx] =
              (uint32_t)(voltage_filter_alpha * cell_voltages_uv[idx] +
                        (1.0f - voltage_filter_alpha) * cell_voltages_filtered_uv[idx]);
          }
        }
      }
    }

    // Run at configured measurement interval
    vTaskDelay(pdMS_TO_TICKS(charging_config.measurement_interval_ms));
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

// HV Connection State Machine Implementation
void BatteryManagementSystem::hv_connect(HV_Mode mode) {
  if (contactor_fault) {
    Serial.println("BMS: Cannot connect HV - contactor fault detected");
    hv_state = HV_Fault;
    return;
  }

  if (ivt_shunt == nullptr || !ivt_shunt->is_alive()) {
    Serial.println("BMS: Cannot connect HV - IVT shunt not available");
    hv_state = HV_Fault;
    return;
  }

  hv_mode = mode;  // Store the requested mode
  const char* mode_str = (mode == HV_MODE_CHARGING) ? "CHARGING" : "DRIVE";
  Serial.printf("BMS: Starting HV connection sequence (mode: %s)\r\n", mode_str);

  hv_state = HV_Precharge;
  hv_state_entry_time = millis();
  precharge_start_time = millis();

  // Step 1: Close negative contactor (IN2/OUT2) - begins HV precharge
  Serial.println("BMS: Step 1 - Closing negative contactor (precharge begins)");
  digitalWrite(contactor_enable_pin, HIGH);      // nSLEEP = HIGH (device awake)
  digitalWrite(negative_contactor_pin, HIGH);    // IN2 = HIGH (OUT2 energizes negative contactor)
  digitalWrite(positive_contactor_pin, LOW);     // IN1 = LOW (precharge active, positive contactor open)

  // Step 2: Start PCS initialization sequence based on mode
  bool pcs_started = false;
  if (mode == HV_MODE_CHARGING) {
    pcs_started = PCSController::start_charging_async();
    Serial.println("BMS: Step 2 - PCS charging initialization started");
  } else {
    pcs_started = PCSController::start_drive_mode_async();
    Serial.println("BMS: Step 2 - PCS drive mode initialization started");
  }

  if (!pcs_started) {
    Serial.println("BMS: ERROR - Failed to start PCS initialization");
    hv_disconnect();
    hv_state = HV_Fault;
  }
}

void BatteryManagementSystem::hv_disconnect() {
  Serial.println("BMS: Disconnecting HV system");
  hv_state = HV_Shutdown;
  hv_state_entry_time = millis();

  // Stop PCS gracefully
  PCSController::stop_async();

  // Open both contactors immediately
  digitalWrite(positive_contactor_pin, LOW);   // IN1 = LOW (OUT1 disabled)
  digitalWrite(negative_contactor_pin, LOW);   // IN2 = LOW (OUT2 disabled)

  // Verify disconnection using IVT-S
  if (ivt_shunt != nullptr && ivt_shunt->is_alive()) {
    delay(100);  // Wait for contactors to open
    float hv_bus_voltage = ivt_shunt->get_voltage2();
    if (hv_bus_voltage < 10.0f) {
      Serial.println("BMS: HV bus discharged successfully");
    } else {
      Serial.printf("BMS: Warning - HV bus still at %.1f V after disconnect\r\n", hv_bus_voltage);
    }
  }

  hv_state = HV_Disabled;
  Serial.println("BMS: HV system disabled");
}

bool BatteryManagementSystem::is_precharge_complete() {
  if (ivt_shunt == nullptr || !ivt_shunt->is_alive()) {
    return false;
  }

  // IVT-S Voltage 1 = Pack voltage
  // IVT-S Voltage 2 = HV bus voltage
  float pack_voltage = ivt_shunt->get_voltage();
  float hv_bus_voltage = ivt_shunt->get_voltage2();

  float voltage_diff = abs(pack_voltage - hv_bus_voltage);

  Serial.printf("BMS: Precharge check - Pack: %.1f V, HV Bus: %.1f V, Diff: %.1f V\r\n",
                pack_voltage, hv_bus_voltage, voltage_diff);

  return (voltage_diff <= hv_config.precharge_voltage_margin_v);
}

void BatteryManagementSystem::update_hv_state() {
  uint32_t current_time = millis();
  uint32_t time_in_state = current_time - hv_state_entry_time;

  switch (hv_state) {
    case HV_Disabled:
      // Nothing to do, waiting for connection request
      break;

    case HV_Precharge: {
      // Check for timeout
      uint32_t precharge_time = current_time - precharge_start_time;
      if (precharge_time > hv_config.precharge_timeout_ms) {
        Serial.println("BMS: Precharge timeout!");
        PCSController::stop_async();  // Stop PCS on timeout
        hv_disconnect();
        hv_state = HV_Fault;
        break;
      }

      // Check if both PCS and voltage precharge are complete
      if (time_in_state >= hv_config.precharge_check_interval_ms) {
        bool pcs_ready = PCSController::is_precharge_complete();
        bool voltage_ready = is_precharge_complete();

        if (pcs_ready && voltage_ready) {
          Serial.println("BMS: Precharge complete (PCS ready + voltage matched)");
          Serial.println("BMS: Step 3 - Closing positive contactor");
          // Step 3: Close positive contactor (IN1/OUT1)
          digitalWrite(positive_contactor_pin, HIGH);  // IN1 = HIGH (OUT1 energizes positive contactor)

          // Step 4: Precharge disabled by external circuit when positive contactor closes
          Serial.println("BMS: HV system active");
          hv_state = HV_Active;
          hv_state_entry_time = current_time;
        } else {
          // Log status for debugging
          if (!pcs_ready) {
            Serial.printf("BMS: Waiting for PCS (state=%d)\r\n", PCSController::get_state());
          }
          if (!voltage_ready) {
            // Voltage status already printed by is_precharge_complete()
          }
          // Update check timestamp
          hv_state_entry_time = current_time;
        }
      }
      break;
    }

    case HV_Active:
      // Verify contactors are still closed and voltages are stable
      if (ivt_shunt != nullptr && ivt_shunt->is_alive()) {
        float pack_voltage = ivt_shunt->get_voltage();
        float hv_bus_voltage = ivt_shunt->get_voltage2();
        float voltage_diff = abs(pack_voltage - hv_bus_voltage);

        // If voltage difference is too large, something is wrong
        if (voltage_diff > hv_config.precharge_voltage_margin_v * 2.0f) {
          Serial.printf("BMS: HV voltage mismatch detected! Pack: %.1f V, Bus: %.1f V\r\n",
                       pack_voltage, hv_bus_voltage);
          hv_disconnect();
          hv_state = HV_Fault;
        }
      }
      break;

    case HV_Shutdown:
      // Handled by hv_disconnect()
      break;

    case HV_Fault:
      // Stay in fault state until reset
      disable_contactors();
      break;

    default:
      break;
  }
}

// Legacy function - now redirects to HV state machine
void BatteryManagementSystem::enable_contactors() {
  hv_connect(HV_MODE_CHARGING);  // Default to charging mode for legacy calls
}

// Legacy enable_contactors implementation (commented out, replaced by hv_connect)
/*
void BatteryManagementSystem::enable_contactors() {
  if (contactor_fault) {
    Serial.println("BMS: Cannot enable contactors - fault detected");
    return;
  }

  Serial.println("BMS: Enabling contactors");
  // DRV8874 in independent half-bridge mode (PMODE floating)
  // IN1 and IN2 independently control OUT1 and OUT2 for two separate contactors
  digitalWrite(contactor_enable_pin, HIGH);    // nSLEEP = HIGH (device awake)
  digitalWrite(positive_contactor_pin, HIGH);  // IN1 = HIGH (OUT1 energizes contactor 1)
  digitalWrite(negative_contactor_pin, HIGH);  // IN2 = HIGH (OUT2 energizes contactor 2)
}
*/

// Legacy function - now redirects to HV state machine
void BatteryManagementSystem::disable_contactors() {
  hv_disconnect();
}

// Legacy disable_contactors implementation (commented out, replaced by hv_disconnect)
/*
void BatteryManagementSystem::disable_contactors() {
  Serial.println("BMS: Disabling contactors");
  // DRV8874 in independent half-bridge mode: set both IN1 and IN2 LOW to disable both contactors
  digitalWrite(positive_contactor_pin, LOW);   // IN1 = LOW (OUT1 disabled, contactor 1 off)
  digitalWrite(negative_contactor_pin, LOW);   // IN2 = LOW (OUT2 disabled, contactor 2 off)
  // Note: nSLEEP (contactor_enable_pin) stays HIGH to keep device awake
}
*/

void BatteryManagementSystem::control_contactors(bool enable_contactor1, bool enable_contactor2) {
  // DRV8874 in independent half-bridge mode with two separate contactors
  // IN1 controls OUT1 for contactor 1, IN2 controls OUT2 for contactor 2
  if (contactor_fault) {
    if (contactors_use_pwm) {
      positive_contactor_timer->pause();
      negative_contactor_timer->pause();
    } else {
      digitalWrite(positive_contactor_pin, LOW);
      digitalWrite(negative_contactor_pin, LOW);
    }
    return;
  }

  if (contactors_use_pwm) {
    // PWM economizer mode for both contactors

    // Contactor 1
    if (enable_contactor1) {
      // Engage with 100% duty
      positive_contactor_timer->setCaptureCompare(positive_contactor_channel, CONTACTOR_ENGAGE_DUTY, PERCENT_COMPARE_FORMAT);
      positive_contactor_timer->resume();
      delay(CONTACTOR_ENGAGE_TIME_MS);
      // Drop to hold duty
      positive_contactor_timer->setCaptureCompare(positive_contactor_channel, CONTACTOR_HOLD_DUTY, PERCENT_COMPARE_FORMAT);
    } else {
      positive_contactor_timer->pause();
      positive_contactor_timer->setCaptureCompare(positive_contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    }

    // Contactor 2
    if (enable_contactor2) {
      // Engage with 100% duty
      negative_contactor_timer->setCaptureCompare(negative_contactor_channel, CONTACTOR_ENGAGE_DUTY, PERCENT_COMPARE_FORMAT);
      negative_contactor_timer->resume();
      delay(CONTACTOR_ENGAGE_TIME_MS);
      // Drop to hold duty
      negative_contactor_timer->setCaptureCompare(negative_contactor_channel, CONTACTOR_HOLD_DUTY, PERCENT_COMPARE_FORMAT);
    } else {
      negative_contactor_timer->pause();
      negative_contactor_timer->setCaptureCompare(negative_contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    }
  } else {
    // Digital control fallback
    digitalWrite(positive_contactor_pin, enable_contactor1 ? HIGH : LOW);
    digitalWrite(negative_contactor_pin, enable_contactor2 ? HIGH : LOW);
  }
}

void BatteryManagementSystem::start_charging() {
  if (current_state == BMS_Error) {
    Serial.println("BMS: Cannot start charging - system in error state");
    return;
  }

  // Check if EVSE is ready (if configured)
  if (evse != nullptr && !evse->is_ready_to_charge()) {
    Serial.println("BMS: Cannot start charging - EVSE not ready");
    return;
  }

  if (current_state == BMS_Idle) {
    Serial.println("BMS: Starting charging cycle");

    // Signal EVSE we're ready to charge
    if (evse != nullptr) {
      evse->set_ready_to_charge(true);
    }

    // Set initial target voltage for PCS
    PCSController::set_hv_voltage_async(get_target_stack_voltage());

    // Start HV connection with PCS in charging mode
    hv_connect(HV_MODE_CHARGING);
    current_state = BMS_Charging;
  } else {
    const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Sleep", "Error"};
    Serial.printf("BMS: Already in state: %s\r\n", state_str[current_state]);
  }
}

void BatteryManagementSystem::start_drive_mode() {
  if (current_state == BMS_Error) {
    Serial.println("BMS: Cannot start drive mode - system in error state");
    return;
  }

  if (current_state == BMS_Idle) {
    Serial.println("BMS: Starting drive mode (DCDC only)");

    // Set DCDC voltage target
    PCSController::set_dcdc_voltage_async(13.8f);  // Standard 12V system voltage

    // Start HV connection with PCS in drive mode
    hv_connect(HV_MODE_DRIVE);
    // Note: BMS state stays Idle since we're not charging
  } else {
    const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Sleep", "Error"};
    Serial.printf("BMS: Already in state: %s\r\n", state_str[current_state]);
  }
}

void BatteryManagementSystem::stop_charging() {
  Serial.println("BMS: User requested charging stop");

  // Signal EVSE we're not ready
  if (evse != nullptr) {
    evse->set_ready_to_charge(false);
  }

  // Stop HV system (which will stop PCS gracefully)
  hv_disconnect();

  stop_cell_balancing(bcc0, bcc0_config->cell_count);
  current_state = BMS_Idle;
}

void BatteryManagementSystem::stop_hv_system() {
  Serial.println("BMS: Stopping HV system");
  hv_disconnect();
  current_state = BMS_Idle;
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
  uint8_t total_cells = bcc0_config->cell_count + bcc1_config->cell_count;
  *count = total_cells;
  memcpy(voltages, cell_voltages_uv, total_cells * sizeof(uint32_t));
}

void BatteryManagementSystem::get_cell_voltages_filtered(uint32_t *voltages, uint8_t *count) {
  uint8_t total_cells = bcc0_config->cell_count + bcc1_config->cell_count;
  *count = total_cells;
  memcpy(voltages, cell_voltages_filtered_uv, total_cells * sizeof(uint32_t));
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

  // Dump BCC1 if enabled
  if (bcc1_enabled && bcc1 != nullptr) {
    Serial.println("\n========================================");
    Serial.println("BCC1 Configuration Register Dump");
    Serial.println("========================================\n");

    for (uint8_t dev = 0; dev < bcc1_config->device_count; dev++) {
      bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);
      bcc_device_t device_type = bcc1_config->devices[dev];

      Serial.printf("###############################################\n");
      Serial.printf("# BCC1 - CID %d (MC3377%s)\n", cid,
              (device_type == BCC_DEVICE_MC33771) ? "1" : "2");
      Serial.printf("###############################################\n\n");

      // Read INIT register
      uint16_t regVal;
      bcc_status_t error = bcc1->read_register(cid, BCC_REG_INIT_ADDR, 1U, &regVal);
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
          error = bcc1->read_register(cid, BCC_REGISTERS_DATA_MC33771[i].address, 1U, &regVal);
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
          error = bcc1->read_register(cid, BCC_REGISTERS_DATA_MC33772[i].address, 1U, &regVal);
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
      error = bcc1->read_guid(cid, &guid);
      if (error == BCC_STATUS_SUCCESS) {
        Serial.printf("  Device GUID: 0x%02X%04X%04X\n",
                (uint16_t)((guid >> 32) & 0x001FU),
                (uint16_t)((guid >> 16) & 0xFFFFU),
                (uint16_t)(guid & 0xFFFFU));
      }

      Serial.println();
    }

    Serial.println("========================================");
    Serial.println("BCC1 Fuse Mirror Data");
    Serial.println("========================================\n");

    // Dump fuse mirror data for BCC1
    for (uint8_t dev = 0; dev < bcc1_config->device_count; dev++) {
      bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);
      bcc_device_t device_type = bcc1_config->devices[dev];

      Serial.printf("###############################################\n");
      Serial.printf("# BCC1 - CID %d Fuse Mirror\n", cid);
      Serial.printf("###############################################\n\n");

      Serial.println("  -------------------------------");
      Serial.println("  Fuse Address         | Value");
      Serial.println("  -------------------------------");

      // Read all fuse mirror addresses (0x00 - 0x1F)
      uint8_t max_fuse_addr = (device_type == BCC_DEVICE_MC33771) ? 0x17 : 0x1F;

      for (uint8_t addr = 0x00; addr <= max_fuse_addr; addr++) {
        uint16_t fuseVal;
        bcc_status_t fuse_error = bcc1->read_fuse_mirror(cid, addr, &fuseVal);
        if (fuse_error == BCC_STATUS_SUCCESS) {
          Serial.printf("  0x%02X                 | 0x%04X\n", addr, fuseVal);
        } else {
          Serial.printf("  0x%02X                 | ERROR %d\n", addr, fuse_error);
        }
      }

      Serial.println("  -------------------------------\n");
    }
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
  // LEDs 0-1 for BMS state indication (LED 2 is now dedicated to HV state)
  for (uint8_t i = 0; i < 2; i++) {
    status_leds->setPixelColor(i, color);
  }
}

void BatteryManagementSystem::update_pcs_led() {
  if (!status_leds) return;

  // LED 4: PCS (Power Conversion System) status

  PCSState pcs_state = PCSController::get_state();
  bool charging = PCSController::is_charge_enabled();
  bool dcdc_active = PCSController::is_dcdc_enabled();
  uint8_t r = 0, g = 0, b = 0;
  uint8_t brightness;

  switch (pcs_state) {
    case PCS_STATE_INIT:
      // Purple - initializing
      set_led_color(4, color_rgb(128, 0, 128));
      break;

    case PCS_STATE_OFF:
      // Dim blue - off/standby
      set_led_color(4, color_rgb(0, 0, 64));
      break;

    case PCS_STATE_WAITSTART:
    case PCS_STATE_PRECHARGE:
      // Cyan - precharging
      set_led_color(4, color_rgb(0, 128, 128));
      break;

    case PCS_STATE_ACTIVATE:
    case PCS_STATE_CHARGING:
      // Pulsing green - charging active
      led_animation_step = (led_animation_step + 1) % 100;
      g = (led_animation_step < 50) ? (led_animation_step * 5) : ((100 - led_animation_step) * 5);
      set_led_color(4, color_rgb(0, g, 0));
      break;

    case PCS_STATE_DRIVE:
      // Pulsing blue - DCDC active (drive mode)
      led_animation_step = (led_animation_step + 1) % 100;
      b = (led_animation_step < 50) ? (led_animation_step * 5) : ((100 - led_animation_step) * 5);
      set_led_color(4, color_rgb(0, 0, b));
      break;

    case PCS_STATE_STOP:
      // Yellow - shutting down
      set_led_color(4, color_rgb(128, 128, 0));
      break;

    case PCS_STATE_FAULT:
      // Flashing red - PCS fault
      led_animation_step = (led_animation_step + 1) % 60;
      if (led_animation_step < 30) {
        set_led_color(4, color_rgb(255, 0, 0));  // Bright red
      } else {
        set_led_color(4, color_rgb(0, 0, 0));    // Off
      }
      break;

    default:
      set_led_color(4, color_rgb(0, 0, 0));
      break;
  }
}

void BatteryManagementSystem::update_evse_led() {
  if (!status_leds || !evse) return;

  // LED 3: EVSE proximity and control pilot status
  EVSEState state = evse->get_state();
  EVSECableLimit cable_limit = evse->get_cable_limit();

  switch (state) {
    case EVSE_STATE_A:
      // No vehicle connected - Off
      set_led_color(3, color_rgb(0, 0, 0));
      break;

    case EVSE_STATE_B:
      // Vehicle connected, not ready - Cyan (blue-green)
      set_led_color(3, color_rgb(0, 128, 128));
      break;

    case EVSE_STATE_C:
      // Vehicle ready to charge / charging - Green
      set_led_color(3, color_rgb(0, 255, 0));
      break;

    case EVSE_STATE_D:
      // Vehicle with ventilation required - Magenta
      set_led_color(3, color_rgb(255, 0, 255));
      break;

    case EVSE_STATE_E:
      // No power / fault - Red flashing
      led_animation_step = (led_animation_step + 1) % 60;
      if (led_animation_step < 30) {
        set_led_color(3, color_rgb(255, 0, 0));
      } else {
        set_led_color(3, color_rgb(0, 0, 0));
      }
      break;

    case EVSE_FAULT:
      // Fault - Red solid
      set_led_color(3, color_rgb(255, 0, 0));
      break;

    default:
      set_led_color(3, color_rgb(0, 0, 0));
      break;
  }
}

void BatteryManagementSystem::update_hv_led() {
  if (!status_leds) return;
  uint8_t brightness;

  // LED 2: HV system state
  switch (hv_state) {
    case HV_Disabled:
      // Off - HV system disabled
      set_led_color(2, color_rgb(0, 0, 0));
      break;

    case HV_Precharge:
      // Yellow pulsing - precharging
      led_animation_step = (led_animation_step + 1) % 100;
      brightness = (led_animation_step < 50) ? (led_animation_step * 5) : ((100 - led_animation_step) * 5);
      set_led_color(2, color_rgb(brightness, brightness, 0));  // Yellow pulse
      break;

    case HV_Active:
      // Solid green - HV active and stable
      set_led_color(2, color_rgb(0, 255, 0));
      break;

    case HV_Fault:
      // Flashing red - HV fault
      led_animation_step = (led_animation_step + 1) % 60;
      if (led_animation_step < 30) {
        set_led_color(2, color_rgb(255, 0, 0));  // Bright red
      } else {
        set_led_color(2, color_rgb(0, 0, 0));    // Off
      }
      break;

    case HV_Shutdown:
      // Orange - shutting down
      set_led_color(2, color_rgb(255, 128, 0));
      break;

    default:
      set_led_color(2, color_rgb(0, 0, 0));
      break;
  }
}

void BatteryManagementSystem::led_pattern_idle() {
  // Soft blue breathing pattern on state LEDs (0-1)
  led_animation_step = (led_animation_step + 1) % 100;
  uint8_t brightness = (led_animation_step < 50) ? (led_animation_step * 2) : ((100 - led_animation_step) * 2);
  brightness = brightness / 4; // Keep it dim
  set_state_leds(color_rgb(0, 0, brightness));
}

void BatteryManagementSystem::led_pattern_charging() {
  // Green wave/chase pattern showing charging progress on state LEDs (0-1)
  led_animation_step = (led_animation_step + 1) % 2;

  for (uint8_t i = 0; i < 2; i++) {
    if (i == led_animation_step) {
      set_led_color(i, color_rgb(0, 255, 0)); // Bright green
    } else {
      set_led_color(i, color_rgb(0, 64, 0)); // Dim green
    }
  }
}

void BatteryManagementSystem::led_pattern_balancing() {
  // Yellow/Orange pulsing pattern on state LEDs (0-1)
  led_animation_step = (led_animation_step + 1) % 100;
  uint8_t brightness = (led_animation_step < 50) ? (led_animation_step * 5) : ((100 - led_animation_step) * 5);
  set_state_leds(color_rgb(brightness, brightness / 2, 0)); // Orange
}

void BatteryManagementSystem::led_pattern_complete() {
  // Solid green on state LEDs (0-1)
  set_state_leds(color_rgb(0, 128, 0)); // Medium green
}

void BatteryManagementSystem::led_pattern_error() {
  // Flashing red on state LEDs (0-1)
  led_animation_step = (led_animation_step + 1) % 60;
  if (led_animation_step < 30) {
    set_state_leds(color_rgb(255, 0, 0)); // Bright red
  } else {
    set_state_leds(color_rgb(0, 0, 0)); // Off
  }
}

void BatteryManagementSystem::update_status_leds() {
  if (!status_leds) return;

  // Update BMS state LEDs (0-1)
  switch (current_state) {
    case BMS_Initialization:
      // Purple - system initializing
      set_state_leds(color_rgb(128, 0, 128));
      break;

    case BMS_Idle:
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

  // Update HV state LED (2)
  update_hv_led();

  // Update EVSE LED (3)
  update_evse_led();

  // Update PCS LED (4)
  update_pcs_led();

  status_leds->show();
}

// HV CAN task - periodically broadcasts HV system state and BMS state
void BatteryManagementSystem::hv_can_task_loop() {
  Serial.println("HV CAN Task: Started");

  // Wait for hardware initialization to complete
  while (!hardware_initialized) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  Serial.println("HV CAN Task: Hardware initialized, starting CAN broadcasts");

  const uint32_t HV_STATUS_CAN_ID = 0x400;  // CAN ID for HV status message
  const uint32_t BROADCAST_INTERVAL_MS = 100;  // Send status every 100ms (10 Hz)

  while (true) {
    if (hv_can != nullptr) {
      // Prepare HV status message
      // Byte 0: HV State
      // Byte 1: BMS State
      // Byte 2-3: Stack voltage (MSB first, in 0.1V units)
      // Byte 4-7: Reserved for future use

      uint8_t data[8] = {0};

      // Byte 0: HV State
      data[0] = static_cast<uint8_t>(hv_state);

      // Byte 1: BMS State
      data[1] = static_cast<uint8_t>(current_state);

      // Byte 2-3: Stack voltage in 0.1V units (e.g., 240 = 24.0V)
      uint16_t stack_voltage_dv = static_cast<uint16_t>(stack_voltage_filtered_uv / 100000);  // Convert uV to 0.1V
      data[2] = (stack_voltage_dv >> 8) & 0xFF;  // MSB
      data[3] = stack_voltage_dv & 0xFF;         // LSB

      // Byte 4: Fault flags (bit-packed)
      data[4] = 0;
      if (has_overvoltage_fault) data[4] |= (1 << 0);
      if (has_undervoltage_fault) data[4] |= (1 << 1);
      if (has_temperature_fault) data[4] |= (1 << 2);
      if (has_cb_open_fault) data[4] |= (1 << 3);
      if (has_cb_short_fault) data[4] |= (1 << 4);
      if (contactor_fault) data[4] |= (1 << 5);
      if (communication_lost) data[4] |= (1 << 6);

      // Byte 5: Additional status
      data[5] = 0;
      if (evse != nullptr && evse->is_connected()) data[5] |= (1 << 0);
      if (evse != nullptr && evse->is_ready_to_charge()) data[5] |= (1 << 1);
      if (PCSController::is_charge_enabled) data[5] |= (1 << 2);

      // Bytes 6-7: Reserved
      data[6] = 0;
      data[7] = 0;

      // Send CAN message
      hv_can->sendMessage(HV_STATUS_CAN_ID, data, 8);
    }

    // Wait for next broadcast interval
    vTaskDelay(pdMS_TO_TICKS(BROADCAST_INTERVAL_MS));
  }
}