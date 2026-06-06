#include "bms.h"
#include <Adafruit_NeoPixel.h>
#include "Arduino.h"
#include "debug_serial.h"

BatteryManagementSystem::BatteryManagementSystem(BatteryCellControllerConfig *config0, BatteryCellControllerConfig *config1) {
  bcc0_config = config0;
  bcc0_initialized = false;
  bcc1_initialized = false;

  devices_0 = new bcc_device_t[config0->device_count];
  for (uint8_t i = 0; i < config0->device_count; i++) {
    devices_0[i] = config0->device_type;
  }

  bcc0_tx_spi = new SPIClass(BCC0_TX_DATA, NC, BCC0_TX_SCK, NC);
  bcc0_rx_spi = new SPIClass(BCC0_RX_DATA, NC, BCC0_RX_SCK, BCC0_RX_CS);

  tpl0 = new TPLSPI(bcc0_tx_spi, bcc0_rx_spi, config0->cs_pin, configureDMA_HV_ECU);
  bcc0 = new BatteryCellController(tpl0, devices_0, config0->device_count, config0->cell_count, config0->enable_pin, config0->intb_pin, config0->loopback);
  // begin() deferred to bcc0_monitor_task_loop() — must run after scheduler starts so interrupts are live

  bcc1_config = config1;

  // Only create BCC1 objects if device count > 0
  if (config1->device_count > 0) {
    devices_1 = new bcc_device_t[config1->device_count];
    for (uint8_t i = 0; i < config1->device_count; i++) {
      devices_1[i] = config1->device_type;
    }

    bcc1_tx_spi = new SPIClass(BCC1_TX_DATA, NC, BCC1_TX_SCK, NC);
    bcc1_rx_spi = new SPIClass(BCC1_RX_DATA, NC, BCC1_RX_SCK, BCC1_RX_CS);

    tpl1 = new TPLSPI(bcc1_tx_spi, bcc1_rx_spi, config1->cs_pin, configureDMA_HV_ECU);
    bcc1 = new BatteryCellController(tpl1, devices_1, config1->device_count, config1->cell_count, config1->enable_pin, config1->intb_pin, config1->loopback);
    // begin() deferred to bcc1_monitor_task_loop() — must run after scheduler starts so interrupts are live
  } else {
    devices_1 = nullptr;
    bcc1_tx_spi = nullptr;
    bcc1_rx_spi = nullptr;
    tpl1 = nullptr;
    bcc1 = nullptr;
    bcc1_initialized = true;
  }

  current_state = BMS_Initialization;
  hv_state = HV_Disabled;
  hv_mode = HV_MODE_CHARGING;  // Default to charging mode
  hv_state_entry_time = 0;
  precharge_start_time = 0;
  contactor_fault = false;
  hvil_open_count = 0;
  hardware_initialized = false;
  // bcc0_initialized and bcc1_initialized already set by begin() calls above
  stack_voltage_uv = 0;
  stack_voltage_bcc1_uv = 0;
  cell_voltage_mutex = xSemaphoreCreateMutex();
  status_leds = nullptr;
  led_animation_step = 0;
  led_display_mode = 0;
  led_mode_switch_time = 0;
  last_successful_measurement = 0;
  communication_timeout_ms = Param::GetInt(Param::commTimeout);
  communication_lost = false;

  // Initialize IVT and CHAdeMO pointers
  ivt_shunt = nullptr;
  chademo = nullptr;
  hv_can = nullptr;
#ifdef BMS_M3_CAN
  m3_can = nullptr;
  m3_mgr = nullptr;
#endif

  // Initialize SOC tracking
  current_soc_percent = Param::GetFloat(Param::initSocPercent);
  accumulated_charge_ah = 0.0f;
  last_soc_update_time = 0;
  soc_initialized = false;

  // Initialize cell voltage and balancing arrays
  memset(cell_voltages_uv, 0, sizeof(cell_voltages_uv));
  memset(cells_to_balance, 0, sizeof(cells_to_balance));

  // Initialize fault tracking
  memset(fault_status, 0, sizeof(fault_status));
  has_overvoltage_fault = false;
  has_undervoltage_fault = false;
  has_temperature_fault = false;
  has_cb_open_fault = false;
  has_cb_short_fault = false;
  bcc0_temp_an3_c = -1000.0f;
  bcc0_temp_an4_c = -1000.0f;
  bcc1_temp_an3_c = -1000.0f;
  bcc1_temp_an4_c = -1000.0f;
  last_fault_check = 0;
  fault_check_interval_ms = Param::GetInt(Param::faultCheckInt);

  // Load charging config from parameters
  charging_config.target_cell_voltage   = Param::GetInt(Param::ovpThresholdMv) / 1000.0f;
  charging_config.balance_threshold_mv  = (float)Param::GetInt(Param::balanceDeltaMv);
  charging_config.balance_target_mv     = (float)Param::GetInt(Param::balanceAbsMv);
  charging_config.balancing_timer_min   = (uint16_t)Param::GetInt(Param::balanceTimerMin);
  charging_config.measurement_interval_ms = (uint16_t)Param::GetInt(Param::measureInterval);
  charging_config.battery_capacity_ah   = Param::GetFloat(Param::batteryCapacity);
  charging_config.max_charge_current_a  = Param::GetInt(Param::ocpChargeMa) / 1000.0f;
  charging_config.min_soc_percent       = Param::GetFloat(Param::minSocPercent);
  charging_config.max_soc_percent       = Param::GetFloat(Param::maxSocPercent);

  // Load HV connection config from parameters
  hv_config.precharge_voltage_margin_v  = Param::GetInt(Param::prechargeCompletionMv) / 1000.0f;
  hv_config.precharge_timeout_ms        = Param::GetInt(Param::prechargeTimeoutMs);
  hv_config.precharge_check_interval_ms = Param::GetInt(Param::prechargeCheckInt);

  // Initialize PWM contactor control
  positive_contactor_timer = nullptr;
  negative_contactor_timer = nullptr;
  contactors_use_pwm = false;

  // Determine if BCC interfaces should be enabled based on device count
  bcc1_enabled = (config1->device_count > 0);

  debug_printf("BMS: BCC0 %d devices, BCC1 %d devices\r\n",
                config0->device_count, config1->device_count);
}

bool BatteryManagementSystem::initialize(uint16_t device_configuration[][BCC_INIT_CONF_REG_CNT]) {
  (void)device_configuration;
  // BCC begin() runs in task context — nothing to do here pre-scheduler
  debug_println("BMS: Initialized (BCC begin deferred to tasks)");
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
    positive_contactor_timer->setOverflow(Param::GetInt(Param::pwmFrequency), HERTZ_FORMAT);
    positive_contactor_timer->setCaptureCompare(positive_contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    positive_contactor_timer->pause();

    // Setup negative contactor PWM
    negative_contactor_channel = STM_PIN_CHANNEL(pinmap_function(neg_pin, PinMap_PWM));
    negative_contactor_timer = new HardwareTimer(neg_instance);
    negative_contactor_timer->setMode(negative_contactor_channel, TIMER_OUTPUT_COMPARE_PWM1, negative_contactor_pin);
    negative_contactor_timer->setOverflow(Param::GetInt(Param::pwmFrequency), HERTZ_FORMAT);
    negative_contactor_timer->setCaptureCompare(negative_contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    negative_contactor_timer->pause();

    contactors_use_pwm = true;
    debug_println("BMS: Contactor PWM economizer enabled");
  } else {
    // Fall back to digital control
    pinMode(positive_contactor_pin, OUTPUT);
    pinMode(negative_contactor_pin, OUTPUT);
    digitalWrite(positive_contactor_pin, LOW);
    digitalWrite(negative_contactor_pin, LOW);
    contactors_use_pwm = false;
    debug_println("BMS: Using digital contactor control (PWM not available)");
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

void BatteryManagementSystem::set_ivt_shunt(IVTShunt *shunt) {
  ivt_shunt = shunt;
}

void BatteryManagementSystem::set_chademo(CHAdeMOController *chademo_controller) {
  chademo = chademo_controller;
}

void BatteryManagementSystem::set_hv_can(CANBus *hv_can_bus) {
  hv_can = hv_can_bus;
}

#ifdef BMS_M3_CAN
void BatteryManagementSystem::set_can_buses(CANBus *m3_can_bus, CANBus *hv_can_bus) {
  m3_can = m3_can_bus;
  hv_can = hv_can_bus;
}

void BatteryManagementSystem::set_m3_can_manager(M3CANManager *mgr) {
  m3_mgr = mgr;
}
#endif

bool BatteryManagementSystem::start_tasks() {
  debug_println("BMS: Starting FreeRTOS tasks...");

  // Create master task
  BaseType_t result = xTaskCreate(
    master_task_wrapper,
    "BMS_Master",
    512,
    this,
    2,
    &master_task_handle
  );

  if (result != pdPASS) {
    debug_println("BMS: Failed to create master task");
    return false;
  }

  // Create BCC0 monitor task
  if (bcc0_config->device_count > 0) {
    result = xTaskCreate(
      bcc0_monitor_task_wrapper,
      "BCC0_Monitor",
      512,
      this,
      2,
      &bcc0_monitor_task_handle
    );

    if (result != pdPASS) {
      debug_println("BMS: Failed to create BCC0 monitor task");
      return false;
    }
    debug_println("BMS: BCC0 monitor task created");
  } else {
    debug_println("BMS: BCC0 disabled, skipping monitor task creation");
    bcc0_monitor_task_handle = nullptr;
  }

  // Create BCC1 monitor task (only if enabled)
  if (bcc1_enabled) {
    result = xTaskCreate(
      bcc1_monitor_task_wrapper,
      "BCC1_Monitor",
      512,
      this,
      2,
      &bcc1_monitor_task_handle
    );

    if (result != pdPASS) {
      debug_println("BMS: Failed to create BCC1 monitor task");
      return false;
    }
    debug_println("BMS: BCC1 monitor task created");
  } else {
    debug_println("BMS: BCC1 disabled, skipping monitor task creation");
    bcc1_monitor_task_handle = nullptr;
  }

  // Create HV CAN task
  result = xTaskCreate(
    hv_can_task_wrapper,
    "HV_CAN",
    512,
    this,
    1,  // Lower priority than monitor tasks
    &hv_can_task_handle
  );

  if (result != pdPASS) {
    debug_println("BMS: Failed to create HV CAN task");
    return false;
  }

  debug_println("BMS: All tasks created successfully");
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
  debug_println("BMS Master Task: Started");

  // Wait for hardware initialization to complete
  while (!hardware_initialized) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  debug_println("BMS Master Task: Hardware initialized, starting state machine");

  while (true) {
    // Snapshot shared voltage state at top of each iteration
    uint32_t snap_voltages[BCC_MAX_CELLS];
    uint32_t snap_stack_uv = 0, snap_stack_bcc1_uv = 0;
    uint8_t snap_cell_count = bcc0_config->device_count * bcc0_config->cell_count
                            + bcc1_config->device_count * bcc1_config->cell_count;
    if (xSemaphoreTake(cell_voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      memcpy(snap_voltages, cell_voltages_uv, snap_cell_count * sizeof(uint32_t));
      snap_stack_uv = stack_voltage_uv;
      snap_stack_bcc1_uv = stack_voltage_bcc1_uv;
      xSemaphoreGive(cell_voltage_mutex);
    }

    // Update SOC using coulomb counting from IVT
    update_soc();

    // Update CHAdeMO controller if configured
    if (chademo != nullptr) {
      chademo->update();

      // Update CHAdeMO with current battery status
      uint8_t soc = get_soc();  // Get actual SOC from coulomb counting
      uint16_t current_voltage = (snap_stack_uv + snap_stack_bcc1_uv) / 1000000;
      uint16_t requested_current = calculate_safe_charge_current();  // Calculate based on cell conditions

      chademo->update_battery_status(current_voltage, soc, requested_current);

      // Monitor CHAdeMO state and handle transitions
      if (chademo->is_charging() && current_state == BMS_Charging) {
        // Active charging - update current request dynamically
        uint16_t new_current = calculate_safe_charge_current();
        if (new_current == 0 || has_reached_target_voltage(snap_voltages, snap_cell_count)) {
          // Stop charging if current reaches 0 or target voltage reached
          debug_println("BMS: Charge complete or current limit reached");
          stop_chademo_charging();
        }
      } else if (chademo->has_timeout()) {
        // CHAdeMO timeout - stop charging
        debug_println("BMS: CHAdeMO timeout detected");
        stop_charging();
        current_state = BMS_Error;
      }
    }

    // Update HV state machine
    update_hv_state();

    // Check for contactor fault
    if (digitalRead(contactor_fault_pin) == LOW) {
      if (!contactor_fault) {
        debug_println("BMS: Contactor fault detected!");
        contactor_fault = true;
        hv_disconnect();  // This will disable contactors and stop PCS
        current_state = BMS_Error;
      }
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // HVIL interlock check — require 3 consecutive open reads (~300ms) before faulting
    if (digitalRead(HVIL_DETECT_PIN) == LOW) {
      hvil_open_count = 0;
    } else {
      hvil_open_count++;
      if (hvil_open_count >= 3) {
        hvil_open_count = 3;  // Saturate so it doesn't wrap
        if (hv_state != HV_Fault && hv_state != HV_Disabled) {
          debug_println("BMS: HVIL interlock open — disconnecting HV!");
          hv_disconnect();
          hv_state = HV_Fault;
          current_state = BMS_Error;
        }
      }
    }

    switch (current_state) {
      case BMS_Idle:
        // Wait for user to start charging via console
        break;

      case BMS_Charging: {
        if (has_reached_target_voltage(snap_voltages, snap_cell_count)) {
          debug_println("BMS: Target voltage reached!");
          hv_disconnect();
          current_state = BMS_Idle;
          break;
        }

        float max_diff_mv = get_max_cell_voltage_diff_mv(snap_voltages, snap_cell_count);

        if (max_diff_mv > charging_config.balance_threshold_mv) {
          debug_printf("BMS: Cell imbalance detected: %.2f mV (threshold: %.2f mV)\r\n",
                       max_diff_mv, charging_config.balance_threshold_mv);
          hv_disconnect();
          current_state = BMS_CellBalancing;

          calculate_cell_balance_requirements(snap_voltages, snap_cell_count, cells_to_balance);
          apply_cell_balancing(bcc0, cells_to_balance, snap_cell_count);
        }
        break;
      }

      case BMS_CellBalancing: {
        float max_diff_mv = get_max_cell_voltage_diff_mv(snap_voltages, snap_cell_count);

        if (max_diff_mv <= charging_config.balance_target_mv) {
          debug_printf("BMS: Cells balanced: %.2f mV (target: %.2f mV)\r\n",
                       max_diff_mv, charging_config.balance_target_mv);
          stop_cell_balancing(bcc0, snap_cell_count);
          current_state = BMS_Charging;
          enable_contactors();
        } else {
          debug_printf("BMS: Balancing... Current difference: %.2f mV\r\n", max_diff_mv);
        }
        break;
      }

      case BMS_Error:
        // Stay in error state until reset
        debug_println("BMS: In error state");
        disable_contactors();
        break;

      default:
        break;
    }

    // IVT offline safety: if shunt goes silent while charging, stop immediately.
    if (ivt_shunt && !ivt_shunt->is_alive()) {
      if (hv_state == HV_Active || current_state == BMS_Charging) {
        debug_println("BMS: IVT shunt offline during operation — stopping charging!");
        Param::SetInt(Param::safeChargeCurrent, 0);
        stop_charging();
        current_state = BMS_Error;
      }
    }

    // 1Hz serial status dump
    static uint32_t last_status_print = 0;
    uint32_t now = millis();
    if (now - last_status_print >= 1000) {
      last_status_print = now;
      uint32_t total_stack_uv = snap_stack_uv + snap_stack_bcc1_uv;
      debug_printf("--- BMS status ---\r\n");
      debug_printf("  State: %d  SOC: %.1f%%  Stack: %.3fV\r\n",
        current_state, (double)get_soc(), (double)(total_stack_uv / 1000000.0f));
      for (uint8_t i = 0; i < snap_cell_count; i++) {
        debug_printf("  Cell%02d: %.3fV\r\n", i + 1, (double)(snap_voltages[i] / 1000000.0f));
      }
      debug_printf("  Faults: OV=%d UV=%d Temp=%d(AN_OT_UT=0x%04X) CBOpen=%d CBShort=%d\r\n",
        has_overvoltage_fault, has_undervoltage_fault,
        has_temperature_fault, fault_status[BCC_FS_AN_OT_UT],
        has_cb_open_fault, has_cb_short_fault);
      uint32_t an_uv[BCC_GPIO_INPUT_CNT] = {};
      if (bcc0->get_an_voltages(BCC_CID_DEV1, an_uv) == BCC_STATUS_SUCCESS) {
        debug_printf("  BCC0 AN: ");
        for (uint8_t i = 0; i < BCC_GPIO_INPUT_CNT; i++)
          debug_printf("AN%d=%.3fV ", i, (double)(an_uv[i] / 1000000.0f));
        debug_printf("\r\n");
      }
      if (bcc1_initialized && bcc1 != nullptr) {
        uint16_t bcc1_faults[11] = {};
        bcc1->get_fault_status(BCC_CID_DEV1, bcc1_faults);
        debug_printf("  BCC1 Faults: OV=%d UV=%d Temp=%d(AN_OT_UT=0x%04X)\r\n",
          (bcc1_faults[BCC_FS_CELL_OV] != 0),
          (bcc1_faults[BCC_FS_CELL_UV] != 0),
          (bcc1_faults[BCC_FS_AN_OT_UT] != 0),
          bcc1_faults[BCC_FS_AN_OT_UT]);
        uint32_t an1_uv[BCC_GPIO_INPUT_CNT] = {};
        if (bcc1->get_an_voltages(BCC_CID_DEV1, an1_uv) == BCC_STATUS_SUCCESS) {
          debug_printf("  BCC1 AN: ");
          for (uint8_t i = 0; i < BCC_GPIO_INPUT_CNT; i++)
            debug_printf("AN%d=%.3fV ", i, (double)(an1_uv[i] / 1000000.0f));
          debug_printf("\r\n");
        }
      }
      if (ivt_shunt) {
        debug_printf("  IVT: %.2fA  %.2fV  alive=%d\r\n",
          (double)ivt_shunt->get_current(),
          (double)ivt_shunt->get_voltage(),
          ivt_shunt->is_alive());
      }
    }

    // Update LED status
    update_status_leds();

    // Update libopeninv spot values (read-only parameters)
    update_spot_values();

    vTaskDelay(pdMS_TO_TICKS(charging_config.measurement_interval_ms));
  }
}

// BCC0 monitor task - reads cell voltages
void BatteryManagementSystem::bcc0_monitor_task_loop() {
  debug_println("BCC0 Monitor Task: Started");

  debug_println("BCC0: Initializing...");
  pinMode(bcc0_config->cs_pin, OUTPUT);
  digitalWrite(bcc0_config->cs_pin, HIGH);
  bcc_status_t err = bcc0->begin(nullptr);
  if (err != BCC_STATUS_SUCCESS) {
    debug_printf("BCC0: Init failed (%d)\r\n", err);
    current_state = BMS_Error;
    vTaskDelete(nullptr);
    return;
  }
  debug_println("BCC0: Ready");
  configure_an_thresholds(bcc0);
  bcc0_initialized = true;
  hardware_initialized = true;
  current_state = BMS_Idle;

  // Dump fuse mirror once at startup
  debug_println("BCC0 fuse mirror:");
  for (uint8_t addr = 0x00; addr <= 0x1F; addr++) {
    uint16_t val = 0;
    bcc_status_t err2 = bcc0->read_fuse_mirror(BCC_CID_DEV1, addr, &val);
    if (err2 == BCC_STATUS_SUCCESS) {
      debug_printf("  [0x%02X] = 0x%04X\r\n", addr, val);
    } else {
      debug_printf("  [0x%02X] error %d\r\n", addr, err2);
      break;
    }
  }

  while (true) {
    if (hardware_initialized && current_state != BMS_Error) {
      uint32_t tmp_voltages[BCC_MAX_CELLS];
      uint32_t tmp_stack;
      bool voltage_ok = measure_cell_voltages(bcc0, tmp_voltages);
      bool stack_ok = measure_stack_voltage(bcc0, &tmp_stack);

      if (voltage_ok && stack_ok) {
        if (xSemaphoreTake(cell_voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          memcpy(cell_voltages_uv, tmp_voltages,
                 bcc0_config->device_count * bcc0_config->cell_count * sizeof(uint32_t));
          stack_voltage_uv = tmp_stack;
          xSemaphoreGive(cell_voltage_mutex);
        }
        // Successful measurement - update timestamp and clear comm lost flag
        last_successful_measurement = millis();
        if (communication_lost) {
          debug_println("BCC0: Comm restored");
          communication_lost = false;
        }

        // Voltage filtering removed - using raw measurements directly
      } else {
        // Failed measurement - check for timeout
        if (!communication_lost && last_successful_measurement > 0) {
          uint32_t time_since_last = millis() - last_successful_measurement;
          if (time_since_last > communication_timeout_ms) {
            debug_println("BCC0: Comm lost!");
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
  debug_println("BCC1 Monitor Task: Started");
  // Wait for BCC0 to finish init before starting BCC1
  while (!bcc0_initialized) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  debug_println("BCC1: Initializing...");
  pinMode(bcc1_config->cs_pin, OUTPUT);
  digitalWrite(bcc1_config->cs_pin, HIGH);
  bcc_status_t err = bcc1->begin(nullptr);
  if (err != BCC_STATUS_SUCCESS) {
    debug_printf("BCC1: Init failed (%d)\r\n", err);
    vTaskDelete(nullptr);
    return;
  }
  debug_println("BCC1: Ready");
  configure_an_thresholds(bcc1);
  bcc1_initialized = true;

  const uint8_t CELL_OFFSET = bcc0_config->device_count * bcc0_config->cell_count;

  while (true) {
    if (bcc1_initialized && hardware_initialized && current_state != BMS_Error) {
      uint32_t bcc1_cell_voltages[BCC_MAX_CELLS];
      uint32_t bcc1_stack_voltage;

      bool voltage_ok = measure_cell_voltages(bcc1, bcc1_cell_voltages);
      bool stack_ok = measure_stack_voltage(bcc1, &bcc1_stack_voltage);

      if (voltage_ok && stack_ok) {
        if (xSemaphoreTake(cell_voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          uint8_t n = bcc1_config->device_count * bcc1_config->cell_count;
          for (uint8_t i = 0; i < n; i++) {
            cell_voltages_uv[CELL_OFFSET + i] = bcc1_cell_voltages[i];
          }
          stack_voltage_bcc1_uv = bcc1_stack_voltage;
          xSemaphoreGive(cell_voltage_mutex);
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

void BatteryManagementSystem::configure_an_thresholds(BatteryCellController *bcc) {
  // Disable OT/UT fault monitoring on all AN pins until NTC is characterised.
  // Default OT=1.16V trips on thermistor pins (AN3/AN4 ~0.45V at room temp).
  // TODO: restore proper thresholds once NTC B-value and pullup resistor are known.
  for (uint8_t i = 0; i <= 6; i++) {
    bcc->set_temperature_thresholds(BCC_CID_DEV1, i, 0x0000, 0x3FFF);
  }
}

// Stub NTC conversion — returns raw voltage in °C-equivalent until characterised
// AN voltage is ratiometric to VCOM (~3.3V); AN3/AN4 ~0.457V at room temp
// TODO: replace with Steinhart-Hart once NTC part and pullup resistor are identified
float BatteryManagementSystem::an_voltage_to_temp_c(uint32_t an_uv) {
  (void)an_uv;
  return -1000.0f;  // sentinel: not yet decoded
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
      debug_printf("BMS: Cell %d needs balancing (%.2f mV above min)\r\n", i + 1, diff_mv);
    } else {
      cells_to_balance[i] = 0;
    }
  }
}

void BatteryManagementSystem::apply_cell_balancing(BatteryCellController *bcc, uint8_t *cells_to_balance,
                                                   uint8_t cell_count) {
  debug_println("BMS: Applying cell balancing...");

  // Enable cell balancing module
  bcc->enable_cell_balancing(BCC_CID_DEV1, true);

  // Set balancing for each cell
  for (uint8_t i = 0; i < cell_count; i++) {
    if (cells_to_balance[i]) {
      bcc->set_cell_balancing(BCC_CID_DEV1, i, true, charging_config.balancing_timer_min);
      debug_printf("BMS: Balancing cell %d enabled\r\n", i + 1);
    }
  }
}

void BatteryManagementSystem::stop_cell_balancing(BatteryCellController *bcc, uint8_t cell_count) {
  debug_println("BMS: Stopping cell balancing...");

  // Disable balancing for all cells
  for (uint8_t i = 0; i < cell_count; i++) {
    bcc->set_cell_balancing(BCC_CID_DEV1, i, false, 0);
  }

  // Disable cell balancing module
  bcc->enable_cell_balancing(BCC_CID_DEV1, false);
}

float BatteryManagementSystem::get_max_cell_voltage_diff_mv(const uint32_t *cell_voltages, uint8_t cell_count) const {
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
  if (hv_state == HV_Fault) {
    debug_println("BMS: Cannot connect HV - system in fault state (reset required)");
    return;
  }

  if (contactor_fault) {
    debug_println("BMS: Cannot connect HV - contactor fault detected");
    hv_state = HV_Fault;
    return;
  }

  if (ivt_shunt == nullptr || !ivt_shunt->is_alive()) {
    debug_println("BMS: Cannot connect HV - IVT shunt not available");
    hv_state = HV_Fault;
    return;
  }

  hv_mode = mode;  // Store the requested mode
  const char* mode_str = (mode == HV_MODE_CHARGING) ? "CHARGING" : "DRIVE";
  debug_printf("BMS: Starting HV connection sequence (mode: %s)\r\n", mode_str);

  hv_state = HV_Precharge;
  hv_state_entry_time = millis();
  precharge_start_time = millis();

  // Step 1: Close negative contactor (IN2/OUT2) - begins HV precharge
  debug_println("BMS: Step 1 - Closing negative contactor (precharge begins)");
  digitalWrite(contactor_enable_pin, HIGH);  // nSLEEP = HIGH (device awake)
  control_contactors(false, true);           // positive=open, negative=closed
}

void BatteryManagementSystem::hv_disconnect() {
  debug_println("BMS: Disconnecting HV system");
  hv_state = HV_Shutdown;
  hv_state_entry_time = millis();

  // Open both contactors immediately
  control_contactors(false, false);

  // Verify disconnection using IVT-S
  if (ivt_shunt != nullptr && ivt_shunt->is_alive()) {
    delay(100);  // Wait for contactors to open
    float hv_bus_voltage = ivt_shunt->get_voltage2();
    if (hv_bus_voltage < 10.0f) {
      debug_println("BMS: HV bus discharged successfully");
    } else {
      debug_printf("BMS: Warning - HV bus still at %.1f V after disconnect\r\n", hv_bus_voltage);
    }
  }

  hv_state = HV_Disabled;
  debug_println("BMS: HV system disabled");
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

  debug_printf("BMS: Precharge check - Pack: %.1f V, HV Bus: %.1f V, Diff: %.1f V\r\n",
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
        debug_println("BMS: Precharge timeout!");
        hv_disconnect();
        hv_state = HV_Fault;
        current_state = BMS_Error;
        break;
      }

      // Check if voltage precharge is complete
      if (time_in_state >= hv_config.precharge_check_interval_ms) {
        bool voltage_ready = is_precharge_complete();

        if (voltage_ready) {
          debug_println("BMS: Precharge complete (voltage matched)");
          debug_println("BMS: Step 3 - Closing positive contactor");
          control_contactors(true, true);  // Close both: positive now joins negative

          // Step 4: Precharge disabled by external circuit when positive contactor closes
          debug_println("BMS: HV system active");
          hv_state = HV_Active;
          hv_state_entry_time = current_time;
        } else {
          // Voltage status already printed by is_precharge_complete()
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
          debug_printf("BMS: HV voltage mismatch detected! Pack: %.1f V, Bus: %.1f V\r\n",
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
    debug_println("BMS: Cannot enable contactors - fault detected");
    return;
  }

  debug_println("BMS: Enabling contactors");
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
  debug_println("BMS: Disabling contactors");
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
    uint8_t engage_duty = Param::GetInt(Param::engageDuty);
    uint16_t engage_time_ms = Param::GetInt(Param::engageTime);
    uint8_t hold0 = Param::GetInt(Param::holdDuty0);
    uint8_t hold1 = Param::GetInt(Param::holdDuty1);

    // Contactor 1
    if (enable_contactor1) {
      // Engage with configured duty
      positive_contactor_timer->setCaptureCompare(positive_contactor_channel, engage_duty, PERCENT_COMPARE_FORMAT);
      positive_contactor_timer->resume();
      delay(engage_time_ms);
      // Drop to hold duty
      positive_contactor_timer->setCaptureCompare(positive_contactor_channel, hold0, PERCENT_COMPARE_FORMAT);
    } else {
      positive_contactor_timer->pause();
      positive_contactor_timer->setCaptureCompare(positive_contactor_channel, 0, PERCENT_COMPARE_FORMAT);
    }

    // Contactor 2
    if (enable_contactor2) {
      // Engage with configured duty
      negative_contactor_timer->setCaptureCompare(negative_contactor_channel, engage_duty, PERCENT_COMPARE_FORMAT);
      negative_contactor_timer->resume();
      delay(engage_time_ms);
      // Drop to hold duty
      negative_contactor_timer->setCaptureCompare(negative_contactor_channel, hold1, PERCENT_COMPARE_FORMAT);
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
    debug_println("BMS: Cannot start charging - system in error state");
    return;
  }

  if (hv_state == HV_Fault) {
    debug_println("BMS: Cannot start charging - HV in fault state (reset required)");
    return;
  }

  if (current_state == BMS_Idle) {
    debug_println("BMS: Starting charging cycle");

    // Start HV connection in charging mode
    hv_connect(HV_MODE_CHARGING);
    current_state = BMS_Charging;
  } else {
    const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Sleep", "Error"};
    debug_printf("BMS: Already in state: %s\r\n", state_str[current_state]);
  }
}

void BatteryManagementSystem::start_drive_mode() {
  if (current_state == BMS_Error) {
    debug_println("BMS: Cannot start drive mode - system in error state");
    return;
  }

  if (current_state == BMS_Idle) {
    debug_println("BMS: Starting drive mode");

    // Start HV connection in drive mode
    hv_connect(HV_MODE_DRIVE);
    // Note: BMS state stays Idle since we're not charging
  } else {
    const char* state_str[] = {"Initialization", "Idle", "Charging", "Cell Balancing", "Sleep", "Error"};
    debug_printf("BMS: Already in state: %s\r\n", state_str[current_state]);
  }
}

void BatteryManagementSystem::stop_charging() {
  debug_println("BMS: User requested charging stop");

  // Stop HV system
  hv_disconnect();

  uint8_t n_cells = bcc0_config->device_count * bcc0_config->cell_count
                  + bcc1_config->device_count * bcc1_config->cell_count;
  stop_cell_balancing(bcc0, n_cells);
  current_state = BMS_Idle;
}

void BatteryManagementSystem::stop_hv_system() {
  debug_println("BMS: Stopping HV system");
  hv_disconnect();
  current_state = BMS_Idle;
}

void BatteryManagementSystem::force_balance_cells() {
  if (current_state == BMS_Error) {
    debug_println("BMS: Cannot balance - system in error state");
    return;
  }

  debug_println("BMS: User requested cell balancing");
  disable_contactors();
  uint8_t n_cells = bcc0_config->device_count * bcc0_config->cell_count
                  + bcc1_config->device_count * bcc1_config->cell_count;
  calculate_cell_balance_requirements(cell_voltages_uv, n_cells, cells_to_balance);
  apply_cell_balancing(bcc0, cells_to_balance, n_cells);
  current_state = BMS_CellBalancing;
}

void BatteryManagementSystem::get_cell_voltages(uint32_t *voltages, uint8_t *count) {
  uint8_t total_cells = bcc0_config->device_count * bcc0_config->cell_count
                      + bcc1_config->device_count * bcc1_config->cell_count;
  *count = total_cells;
  memcpy(voltages, cell_voltages_uv, total_cells * sizeof(uint32_t));
}

void BatteryManagementSystem::get_cell_voltages_filtered(uint32_t *voltages, uint8_t *count) {
  uint8_t total_cells = bcc0_config->device_count * bcc0_config->cell_count
                      + bcc1_config->device_count * bcc1_config->cell_count;
  *count = total_cells;
  memcpy(voltages, cell_voltages_uv, total_cells * sizeof(uint32_t));
}


void BatteryManagementSystem::get_fault_status(uint16_t *faults) {
  memcpy(faults, fault_status, sizeof(fault_status));
}

void BatteryManagementSystem::read_and_set_voltage_limits() {
  uint16_t min_mv = 0, max_mv = 0;

  // Read from BCC0 (always present)
  uint16_t th_all_ct = 0;
  bcc_status_t status = bcc0->read_register(BCC_CID_DEV1, 0x4B, 1, &th_all_ct);

  if (status == BCC_STATUS_SUCCESS) {
    // Decode thresholds using MC33772 formula: voltage_mv = (value * 195) / 10
    uint8_t uv_threshold = (th_all_ct & 0x00FF);       // Lower 8 bits
    uint8_t ov_threshold = (th_all_ct & 0xFF00) >> 8;  // Upper 8 bits

    min_mv = (uv_threshold * 195) / 10;  // Undervoltage threshold
    max_mv = (ov_threshold * 195) / 10;  // Overvoltage threshold

    debug_printf("Pack voltage limits from BCC hardware: min=%.3fV (0x%02X), max=%.3fV (0x%02X)\r\n",
                  min_mv / 1000.0f, uv_threshold, max_mv / 1000.0f, ov_threshold);
  } else {
    // Fallback to safe defaults if read fails
    min_mv = 2500;  // 2.5V
    max_mv = 4200;  // 4.2V
    debug_println("WARNING: Could not read voltage limits from BCC, using defaults");
  }

  // Set as spot values
  Param::SetFloat(Param::cellVoltMin, min_mv);
  Param::SetFloat(Param::cellVoltMax, max_mv);
}

bcc_status_t BatteryManagementSystem::read_bcc_register(uint8_t bcc_num, bcc_cid_t cid, uint8_t reg_addr, uint16_t *value) {
  if (bcc_num == 0 && bcc0) {
    return bcc0->read_register(cid, reg_addr, 1, value);
  } else if (bcc_num == 1 && bcc1 && bcc1_enabled) {
    return bcc1->read_register(cid, reg_addr, 1, value);
  }
  return BCC_STATUS_PARAM_RANGE;
}

bool BatteryManagementSystem::has_faults() const {
  return has_overvoltage_fault || has_undervoltage_fault ||
         has_temperature_fault || has_cb_open_fault || has_cb_short_fault;
}

void BatteryManagementSystem::dump_registers() {
  if (!hardware_initialized) {
    debug_println("Error: BCC hardware not initialized");
    return;
  }

  debug_println("\n========================================");
  debug_println("BCC Configuration Register Dump");
  debug_println("========================================\n");

  // Dump BCC0
  for (uint8_t dev = 0; dev < bcc0_config->device_count; dev++) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);
    bcc_device_t device_type = bcc0_config->devices[dev];

    debug_printf("###############################################\n");
    debug_printf("# BCC0 - CID %d (MC3377%s)\n", cid,
            (device_type == BCC_DEVICE_MC33771) ? "1" : "2");
    debug_printf("###############################################\n\n");

    // Read INIT register
    uint16_t regVal;
    bcc_status_t error = bcc0->read_register(cid, BCC_REG_INIT_ADDR, 1U, &regVal);
    if (error == BCC_STATUS_SUCCESS) {
      debug_printf("  %-25s | 0x%04X | 0x%02X%02X\n", "INIT", BCC_REG_INIT_ADDR,
                   regVal >> 8, regVal & 0xFFU);
    }

    debug_println("  -------------------------------");
    debug_println("  Register Name            | Addr   | Value");
    debug_println("  -------------------------------");

    // Read all configuration registers based on device type
    if (device_type == BCC_DEVICE_MC33771) {
      for (uint8_t i = 0; i < REG_CONF_CNT_MC33771; i++) {
        error = bcc0->read_register(cid, BCC_REGISTERS_DATA_MC33771[i].address, 1U, &regVal);
        if (error == BCC_STATUS_SUCCESS) {
          debug_printf("  %-25s | 0x%04X | 0x%02X%02X\n",
                       BCC_REGISTERS_DATA_MC33771[i].name,
                       BCC_REGISTERS_DATA_MC33771[i].address,
                       regVal >> 8, regVal & 0xFFU);
        } else {
          debug_printf("  %-25s | 0x%04X | ERROR %d\n",
                       BCC_REGISTERS_DATA_MC33771[i].name,
                       BCC_REGISTERS_DATA_MC33771[i].address,
                       error);
        }
      }
    } else {
      for (uint8_t i = 0; i < REG_CONF_CNT_MC33772; i++) {
        error = bcc0->read_register(cid, BCC_REGISTERS_DATA_MC33772[i].address, 1U, &regVal);
        if (error == BCC_STATUS_SUCCESS) {
          debug_printf("  %-25s | 0x%04X | 0x%02X%02X\n",
                       BCC_REGISTERS_DATA_MC33772[i].name,
                       BCC_REGISTERS_DATA_MC33772[i].address,
                       regVal >> 8, regVal & 0xFFU);
        } else {
          debug_printf("  %-25s | 0x%04X | ERROR %d\n",
                       BCC_REGISTERS_DATA_MC33772[i].name,
                       BCC_REGISTERS_DATA_MC33772[i].address,
                       error);
        }
      }
    }

    debug_println("  -------------------------------\n");

    // Read GUID
    uint64_t guid;
    error = bcc0->read_guid(cid, &guid);
    if (error == BCC_STATUS_SUCCESS) {
      debug_printf("  Device GUID: 0x%02X%04X%04X\n",
              (uint16_t)((guid >> 32) & 0x001FU),
              (uint16_t)((guid >> 16) & 0xFFFFU),
              (uint16_t)(guid & 0xFFFFU));
    }

    debug_println();
  }

  debug_println("========================================");
  debug_println("Fuse Mirror Data");
  debug_println("========================================\n");

  // Dump fuse mirror data for BCC0
  for (uint8_t dev = 0; dev < bcc0_config->device_count; dev++) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);
    bcc_device_t device_type = bcc0_config->devices[dev];

    debug_printf("###############################################\n");
    debug_printf("# BCC0 - CID %d Fuse Mirror\n", cid);
    debug_printf("###############################################\n\n");

    debug_println("  -------------------------------");
    debug_println("  Fuse Address         | Value");
    debug_println("  -------------------------------");

    // Read all fuse mirror addresses (0x00 - 0x1F)
    // MC33771C has fewer fuse addresses than MC33772C
    uint8_t max_fuse_addr = (device_type == BCC_DEVICE_MC33771) ? 0x17 : 0x1F;

    for (uint8_t addr = 0x00; addr <= max_fuse_addr; addr++) {
      uint16_t fuseVal;
      bcc_status_t fuse_error = bcc0->read_fuse_mirror(cid, addr, &fuseVal);
      if (fuse_error == BCC_STATUS_SUCCESS) {
        debug_printf("  0x%02X                 | 0x%04X\n", addr, fuseVal);
      } else {
        debug_printf("  0x%02X                 | ERROR %d\n", addr, fuse_error);
      }
    }

    debug_println("  -------------------------------\n");
  }

  // Dump BCC1 if enabled
  if (bcc1_enabled && bcc1 != nullptr) {
    debug_println("\n========================================");
    debug_println("BCC1 Configuration Register Dump");
    debug_println("========================================\n");

    for (uint8_t dev = 0; dev < bcc1_config->device_count; dev++) {
      bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);
      bcc_device_t device_type = bcc1_config->devices[dev];

      debug_printf("###############################################\n");
      debug_printf("# BCC1 - CID %d (MC3377%s)\n", cid,
              (device_type == BCC_DEVICE_MC33771) ? "1" : "2");
      debug_printf("###############################################\n\n");

      // Read INIT register
      uint16_t regVal;
      bcc_status_t error = bcc1->read_register(cid, BCC_REG_INIT_ADDR, 1U, &regVal);
      if (error == BCC_STATUS_SUCCESS) {
        debug_printf("  %-25s | 0x%04X | 0x%02X%02X\n", "INIT", BCC_REG_INIT_ADDR,
                     regVal >> 8, regVal & 0xFFU);
      }

      debug_println("  -------------------------------");
      debug_println("  Register Name            | Addr   | Value");
      debug_println("  -------------------------------");

      // Read all configuration registers based on device type
      if (device_type == BCC_DEVICE_MC33771) {
        for (uint8_t i = 0; i < REG_CONF_CNT_MC33771; i++) {
          error = bcc1->read_register(cid, BCC_REGISTERS_DATA_MC33771[i].address, 1U, &regVal);
          if (error == BCC_STATUS_SUCCESS) {
            debug_printf("  %-25s | 0x%04X | 0x%02X%02X\n",
                         BCC_REGISTERS_DATA_MC33771[i].name,
                         BCC_REGISTERS_DATA_MC33771[i].address,
                         regVal >> 8, regVal & 0xFFU);
          } else {
            debug_printf("  %-25s | 0x%04X | ERROR %d\n",
                         BCC_REGISTERS_DATA_MC33771[i].name,
                         BCC_REGISTERS_DATA_MC33771[i].address,
                         error);
          }
        }
      } else {
        for (uint8_t i = 0; i < REG_CONF_CNT_MC33772; i++) {
          error = bcc1->read_register(cid, BCC_REGISTERS_DATA_MC33772[i].address, 1U, &regVal);
          if (error == BCC_STATUS_SUCCESS) {
            debug_printf("  %-25s | 0x%04X | 0x%02X%02X\n",
                         BCC_REGISTERS_DATA_MC33772[i].name,
                         BCC_REGISTERS_DATA_MC33772[i].address,
                         regVal >> 8, regVal & 0xFFU);
          } else {
            debug_printf("  %-25s | 0x%04X | ERROR %d\n",
                         BCC_REGISTERS_DATA_MC33772[i].name,
                         BCC_REGISTERS_DATA_MC33772[i].address,
                         error);
          }
        }
      }

      debug_println("  -------------------------------\n");

      // Read GUID
      uint64_t guid;
      error = bcc1->read_guid(cid, &guid);
      if (error == BCC_STATUS_SUCCESS) {
        debug_printf("  Device GUID: 0x%02X%04X%04X\n",
                (uint16_t)((guid >> 32) & 0x001FU),
                (uint16_t)((guid >> 16) & 0xFFFFU),
                (uint16_t)(guid & 0xFFFFU));
      }

      debug_println();
    }

    debug_println("========================================");
    debug_println("BCC1 Fuse Mirror Data");
    debug_println("========================================\n");

    // Dump fuse mirror data for BCC1
    for (uint8_t dev = 0; dev < bcc1_config->device_count; dev++) {
      bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);
      bcc_device_t device_type = bcc1_config->devices[dev];

      debug_printf("###############################################\n");
      debug_printf("# BCC1 - CID %d Fuse Mirror\n", cid);
      debug_printf("###############################################\n\n");

      debug_println("  -------------------------------");
      debug_println("  Fuse Address         | Value");
      debug_println("  -------------------------------");

      // Read all fuse mirror addresses (0x00 - 0x1F)
      uint8_t max_fuse_addr = (device_type == BCC_DEVICE_MC33771) ? 0x17 : 0x1F;

      for (uint8_t addr = 0x00; addr <= max_fuse_addr; addr++) {
        uint16_t fuseVal;
        bcc_status_t fuse_error = bcc1->read_fuse_mirror(cid, addr, &fuseVal);
        if (fuse_error == BCC_STATUS_SUCCESS) {
          debug_printf("  0x%02X                 | 0x%04X\n", addr, fuseVal);
        } else {
          debug_printf("  0x%02X                 | ERROR %d\n", addr, fuse_error);
        }
      }

      debug_println("  -------------------------------\n");
    }
  }

  debug_println("========================================");
  debug_println("Dump complete");
  debug_println("========================================\n");
}

void BatteryManagementSystem::print_fault_status() {
  debug_println("\n=== Fault Status ===");

  // Overall status
  debug_printf("Overall Status: %s\r\n", has_faults() ? "FAULTS DETECTED" : "OK");
  debug_println();

  // Cell overvoltage faults
  debug_printf("Cell Overvoltage:  0x%04X", fault_status[BCC_FS_CELL_OV]);
  if (has_overvoltage_fault) {
    DebugSerial.print(" [FAULT]");
    // Print which cells have OV fault (each bit represents a cell)
    DebugSerial.print(" - Cells: ");
    for (uint8_t i = 0; i < bcc0_config->cell_count; i++) {
      if (fault_status[BCC_FS_CELL_OV] & (1 << i)) {
        debug_printf("%d ", i + 1);
      }
    }
  }
  debug_println();

  // Cell undervoltage faults
  debug_printf("Cell Undervoltage: 0x%04X", fault_status[BCC_FS_CELL_UV]);
  if (has_undervoltage_fault) {
    DebugSerial.print(" [FAULT]");
    // Print which cells have UV fault
    DebugSerial.print(" - Cells: ");
    for (uint8_t i = 0; i < bcc0_config->cell_count; i++) {
      if (fault_status[BCC_FS_CELL_UV] & (1 << i)) {
        debug_printf("%d ", i + 1);
      }
    }
  }
  debug_println();

  // Temperature faults
  debug_printf("Temperature Faults: 0x%04X", fault_status[BCC_FS_AN_OT_UT]);
  if (has_temperature_fault) {
    DebugSerial.print(" [FAULT]");
    // Print which ANx pins have temperature fault
    DebugSerial.print(" - AN pins: ");
    for (uint8_t i = 0; i < 7; i++) {
      if (fault_status[BCC_FS_AN_OT_UT] & (1 << i)) {
        debug_printf("AN%d ", i);
      }
    }
  }
  debug_println();

  // Cell balancing faults
  debug_printf("CB Open Fault:     0x%04X", fault_status[BCC_FS_CB_OPEN]);
  if (fault_status[BCC_FS_CB_OPEN] != 0) DebugSerial.print(" [FAULT]");
  debug_println();

  debug_printf("CB Short Fault:    0x%04X", fault_status[BCC_FS_CB_SHORT]);
  if (fault_status[BCC_FS_CB_SHORT] != 0) DebugSerial.print(" [FAULT]");
  debug_println();

  // GPIO status
  debug_printf("GPIO Status:       0x%04X", fault_status[BCC_FS_GPIO_STATUS]);
  debug_println();

  // Communication status
  debug_printf("Comm Status:       0x%04X", fault_status[BCC_FS_COMM]);
  if (fault_status[BCC_FS_COMM] != 0) DebugSerial.print(" [ERRORS]");
  debug_println();

  // General fault status registers
  debug_printf("Fault1 Status:     0x%04X", fault_status[BCC_FS_FAULT1]);
  if (fault_status[BCC_FS_FAULT1] != 0) DebugSerial.print(" [FAULT]");
  debug_println();

  debug_printf("Fault2 Status:     0x%04X", fault_status[BCC_FS_FAULT2]);
  if (fault_status[BCC_FS_FAULT2] != 0) DebugSerial.print(" [FAULT]");
  debug_println();

  debug_printf("Fault3 Status:     0x%04X", fault_status[BCC_FS_FAULT3]);
  if (fault_status[BCC_FS_FAULT3] != 0) DebugSerial.print(" [FAULT]");
  debug_println();

  debug_println();
}

BMS_State BatteryManagementSystem::enable_sleep_mode() {
  // Open contactors before powering down BCC — callers may not do this themselves.
  hv_disconnect();

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
  // Paint all LEDs with the state color (state mode owns the whole strip).
  for (uint8_t i = 0; i < status_leds->numPixels(); i++) {
    status_leds->setPixelColor(i, color);
  }
}

void BatteryManagementSystem::update_hv_led() {
  if (!status_leds) return;
  uint8_t brightness;

  // In state mode the HV LED is the last pixel on the strip (most visible
  // "summary" LED next to the SOC bar when it's shown).
  uint8_t hv_idx = status_leds->numPixels() - 1;

  switch (hv_state) {
    case HV_Disabled:
      set_led_color(hv_idx, color_rgb(0, 0, 0));
      break;

    case HV_Precharge:
      led_animation_step = (led_animation_step + 1) % 100;
      brightness = (led_animation_step < 50) ? (led_animation_step * 5) : ((100 - led_animation_step) * 5);
      set_led_color(hv_idx, color_rgb(brightness, brightness, 0));  // Yellow pulse
      break;

    case HV_Active:
      set_led_color(hv_idx, color_rgb(0, 255, 0));
      break;

    case HV_Fault:
      led_animation_step = (led_animation_step + 1) % 60;
      if (led_animation_step < 30) {
        set_led_color(hv_idx, color_rgb(255, 0, 0));
      } else {
        set_led_color(hv_idx, color_rgb(0, 0, 0));
      }
      break;

    case HV_Shutdown:
      set_led_color(hv_idx, color_rgb(255, 128, 0));
      break;

    default:
      set_led_color(hv_idx, color_rgb(0, 0, 0));
      break;
  }
}

// SOC bar across all 10 LEDs. Each LED = 10% SOC. Color shifts with SOC.
void BatteryManagementSystem::led_pattern_soc() {
  if (!status_leds) return;
  uint8_t soc = get_soc();
  uint8_t count = status_leds->numPixels();
  if (count == 0) return;

  uint32_t color;
  if (soc < 20)      color = color_rgb(255, 0, 0);    // Red
  else if (soc < 50) color = color_rgb(255, 128, 0);  // Orange
  else if (soc < 80) color = color_rgb(255, 255, 0);  // Yellow
  else               color = color_rgb(0, 255, 0);    // Green

  // Each LED represents 100/count percent.
  uint16_t per_led = 100 / count;  // 10 for 10 LEDs
  uint8_t full = soc / per_led;
  uint8_t remainder = soc % per_led;

  for (uint8_t i = 0; i < count; i++) {
    if (i < full) {
      set_led_color(i, color);
    } else if (i == full && remainder > 0) {
      uint8_t scale = (remainder * 255) / per_led;
      uint8_t r = (((color >> 16) & 0xFF) * scale) / 255;
      uint8_t g = (((color >> 8) & 0xFF) * scale) / 255;
      uint8_t b = ((color & 0xFF) * scale) / 255;
      set_led_color(i, color_rgb(r, g, b));
    } else {
      set_led_color(i, 0);
    }
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
  if (!status_leds) return;
  // Green chase across all LEDs.
  uint8_t count = status_leds->numPixels();
  led_animation_step = (led_animation_step + 1) % count;

  for (uint8_t i = 0; i < count; i++) {
    if (i == led_animation_step) {
      set_led_color(i, color_rgb(0, 255, 0)); // Bright green head
    } else {
      set_led_color(i, color_rgb(0, 64, 0));  // Dim green trail
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

  // Toggle between state view and SOC view every 3 seconds, but only after
  // the SOC has been initialized so we don't show a stale 0% bar.
  const uint32_t LED_MODE_PERIOD_MS = 3000;
  uint32_t now = millis();
  if (soc_initialized && (now - led_mode_switch_time >= LED_MODE_PERIOD_MS)) {
    led_display_mode ^= 1;
    led_mode_switch_time = now;
  }

  if (soc_initialized && led_display_mode == 1) {
    led_pattern_soc();
    update_hv_led();
    status_leds->show();
    return;
  }

  switch (current_state) {
    case BMS_Initialization:
      set_state_leds(color_rgb(128, 0, 128));
      break;

    case BMS_Idle:
      led_pattern_idle();
      break;

    case BMS_Charging: {
      uint8_t n_cells = bcc0_config->device_count * bcc0_config->cell_count
                      + bcc1_config->device_count * bcc1_config->cell_count;
      if (has_reached_target_voltage(cell_voltages_uv, n_cells)) {
        led_pattern_complete();
      } else {
        led_pattern_charging();
      }
      break;
    }

    case BMS_CellBalancing:
      led_pattern_balancing();
      break;

    case BMS_Error:
      led_pattern_error();
      break;

    case BMS_Sleep:
      set_state_leds(0);
      break;

    default:
      break;
  }

  // HV summary LED overrides the last pixel during state mode.
  update_hv_led();

  status_leds->show();
}

// SOC Tracking Implementation
void BatteryManagementSystem::initialize_soc_from_voltage() {
  if (soc_initialized) return;

  // Estimate SOC from average cell voltage (rough approximation)
  // For NMC/NCA chemistry: ~3.0V = 0%, ~3.7V = 50%, ~4.2V = 100%
  // Note: This is a simplified linear approximation. Real NMC discharge curves are non-linear.

  uint8_t cell_count = bcc0_config->device_count * bcc0_config->cell_count
                     + bcc1_config->device_count * bcc1_config->cell_count;
  if (cell_count == 0) return;

  uint32_t total_voltage = 0;
  for (uint8_t i = 0; i < cell_count; i++) {
    total_voltage += cell_voltages_uv[i];
  }
  float avg_cell_voltage = total_voltage / (float)cell_count / 1000000.0f;

  // NMC voltage mapping: cellVoltMin = 0% SOC, target voltage = 100% SOC
  float min_voltage = Param::GetFloat(Param::cellVoltMin) / 1000.0f;  // Convert mV to V
  float max_voltage = charging_config.target_cell_voltage;

  current_soc_percent = ((avg_cell_voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0f;

  // Clamp to valid range
  if (current_soc_percent < 0.0f) current_soc_percent = 0.0f;
  if (current_soc_percent > 100.0f) current_soc_percent = 100.0f;

  soc_initialized = true;
  last_soc_update_time = millis();

  debug_printf("BMS: SOC initialized from voltage: %.1f%% (avg cell: %.3fV, NMC chemistry)\r\n",
                current_soc_percent, avg_cell_voltage);
}

void BatteryManagementSystem::update_soc() {
  if (!ivt_shunt || !ivt_shunt->is_alive()) {
    // No current measurement available, can't update SOC
    return;
  }

  // Initialize SOC from voltage if not done yet
  if (!soc_initialized) {
    initialize_soc_from_voltage();
    return;
  }

  uint32_t current_time = millis();
  if (last_soc_update_time == 0) {
    last_soc_update_time = current_time;
    return;
  }

  // Calculate time delta in hours
  float delta_time_ms = current_time - last_soc_update_time;
  float delta_time_hours = delta_time_ms / 3600000.0f;  // Convert ms to hours

  // Get current from IVT (positive = charging, negative = discharging)
  float current_a = ivt_shunt->get_current();

  // Integrate current to get charge in Ah
  float charge_delta_ah = current_a * delta_time_hours;
  accumulated_charge_ah += charge_delta_ah;

  // Update SOC based on accumulated charge
  float soc_delta = (charge_delta_ah / charging_config.battery_capacity_ah) * 100.0f;
  current_soc_percent += soc_delta;

  // Clamp SOC to valid range
  if (current_soc_percent < charging_config.min_soc_percent) {
    current_soc_percent = charging_config.min_soc_percent;
  }
  if (current_soc_percent > charging_config.max_soc_percent) {
    current_soc_percent = charging_config.max_soc_percent;
  }

  last_soc_update_time = current_time;
}

void BatteryManagementSystem::set_soc(float soc_percent) {
  current_soc_percent = soc_percent;

  // Clamp to valid range
  if (current_soc_percent < 0.0f) current_soc_percent = 0.0f;
  if (current_soc_percent > 100.0f) current_soc_percent = 100.0f;

  soc_initialized = true;
  last_soc_update_time = millis();
  accumulated_charge_ah = 0.0f;  // Reset accumulator

  debug_printf("BMS: SOC manually set to %.1f%%\r\n", current_soc_percent);
}

uint16_t BatteryManagementSystem::calculate_safe_charge_current() const {
  if (!soc_initialized) return 0;

  float max_current = charging_config.max_charge_current_a;

  uint8_t cell_count = bcc0_config->device_count * bcc0_config->cell_count
                     + bcc1_config->device_count * bcc1_config->cell_count;

  // Factor 1: Cell voltage imbalance - reduce current if cells are imbalanced
  float max_diff_mv = get_max_cell_voltage_diff_mv(cell_voltages_uv, cell_count);
  if (max_diff_mv > charging_config.balance_threshold_mv) {
    float reduction_factor = 1.0f - (max_diff_mv - charging_config.balance_threshold_mv) / 100.0f;
    if (reduction_factor < 0.3f) reduction_factor = 0.3f;
    max_current *= reduction_factor;
  }

  // Factor 2: Taper current as we approach target voltage
  if (cell_count > 0) {
    uint32_t total_voltage = 0;
    for (uint8_t i = 0; i < cell_count; i++) {
      total_voltage += cell_voltages_uv[i];
    }
    float avg_cell_voltage = total_voltage / (float)cell_count / 1000000.0f;
    float target_voltage = charging_config.target_cell_voltage;

    // Start tapering at 95% of target voltage
    float taper_start_voltage = target_voltage * 0.95f;
    if (avg_cell_voltage > taper_start_voltage) {
      float taper_factor = (target_voltage - avg_cell_voltage) / (target_voltage - taper_start_voltage);
      if (taper_factor < 0.2f) taper_factor = 0.2f;  // Minimum 20% current during taper
      max_current *= taper_factor;
    }
  }

  // Factor 3: SOC-based current limiting (taper at high SOC)
  if (current_soc_percent > 90.0f) {
    float soc_factor = (100.0f - current_soc_percent) / 10.0f;  // Linear reduction from 90-100%
    if (soc_factor < 0.2f) soc_factor = 0.2f;
    max_current *= soc_factor;
  }

  // Factor 4: Temperature derating (if we have temperature faults)
  if (has_temperature_fault) {
    max_current *= 0.5f;  // Reduce to 50% if temperature fault
  }

  // Minimum current to request is 5A (below that, stop charging)
  if (max_current < 5.0f) {
    return 0;
  }

  return (uint16_t)max_current;
}

// CHAdeMO Integration
bool BatteryManagementSystem::is_chademo_ready() const {
  if (!chademo) return false;
  if (!ivt_shunt || !ivt_shunt->is_alive()) return false;
  if (!soc_initialized) return false;
  if (has_faults()) return false;
  if (contactor_fault) return false;

  // Check EVSE capabilities are sufficient
  uint16_t evse_max_voltage = chademo->get_evse_max_voltage();
  uint16_t evse_max_current = chademo->get_evse_max_current();
  uint16_t target_voltage = get_target_stack_voltage();

  // EVSE max voltage should be at least our target voltage
  if (evse_max_voltage > 0 && evse_max_voltage < target_voltage) {
    debug_printf("BMS: EVSE max voltage (%dV) below target (%dV)\r\n",
                  evse_max_voltage, target_voltage);
    return false;
  }

  // EVSE should support at least minimum charging current
  if (evse_max_current > 0 && evse_max_current < 5) {
    debug_printf("BMS: EVSE max current (%dA) too low (min 5A)\r\n", evse_max_current);
    return false;
  }

  return true;
}

void BatteryManagementSystem::start_chademo_charging() {
  if (!chademo) {
    debug_println("BMS: CHAdeMO controller not configured");
    return;
  }

  if (!is_chademo_ready()) {
    debug_println("BMS: CHAdeMO not ready to charge");
    return;
  }

  // Get target voltage from BMS config
  uint16_t target_voltage = get_target_stack_voltage();

  // Get maximum safe current based on battery conditions
  uint16_t max_current = calculate_safe_charge_current();
  if (max_current == 0) {
    debug_println("BMS: Cannot start charging - safe current is 0A");
    return;
  }

  // Cap current to EVSE maximum capability
  uint16_t evse_max_current = chademo->get_evse_max_current();
  if (evse_max_current > 0 && max_current > evse_max_current) {
    debug_printf("BMS: Limiting charge current from %dA to EVSE max %dA\r\n",
                  max_current, evse_max_current);
    max_current = evse_max_current;
  }

  debug_printf("BMS: Starting CHAdeMO charging session (target %dV, max %dA)\r\n",
                target_voltage, max_current);

  // Start CHAdeMO charging session
  chademo->start_charging(target_voltage, max_current);

  // Transition BMS to charging state
  start_charging();
}

void BatteryManagementSystem::stop_chademo_charging() {
  if (!chademo) return;

  debug_println("BMS: Stopping CHAdeMO charging session");

  chademo->stop_charging();
  stop_charging();
}

// HV CAN task - periodically broadcasts HV system state and BMS state
void BatteryManagementSystem::hv_can_task_loop() {
  debug_println("HV CAN Task: Started");

  // Wait for hardware initialization to complete
  while (!hardware_initialized) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  debug_println("HV CAN Task: Hardware initialized, starting CAN broadcasts");

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
      uint16_t stack_voltage_dv = static_cast<uint16_t>(stack_voltage_uv / 100000);  // Convert uV to 0.1V
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

      // Bytes 5-7: Reserved
      data[5] = 0;
      data[6] = 0;
      data[7] = 0;

      // Send CAN message
      hv_can->sendMessage(HV_STATUS_CAN_ID, data, 8);
    }

    // Wait for next broadcast interval
    vTaskDelay(pdMS_TO_TICKS(BROADCAST_INTERVAL_MS));
  }
}

// Update libopeninv spot values (read-only parameters)
void BatteryManagementSystem::update_spot_values() {
  // Snapshot shared voltage state under mutex
  uint32_t local_voltages[BCC_MAX_CELLS];
  uint32_t local_stack_uv = 0, local_stack_bcc1_uv = 0;
  uint8_t cell_count = bcc0_config->device_count * bcc0_config->cell_count
                     + bcc1_config->device_count * bcc1_config->cell_count;

  if (xSemaphoreTake(cell_voltage_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    memcpy(local_voltages, cell_voltages_uv, cell_count * sizeof(uint32_t));
    local_stack_uv = stack_voltage_uv;
    local_stack_bcc1_uv = stack_voltage_bcc1_uv;
    xSemaphoreGive(cell_voltage_mutex);
  }

  // Pack voltages (convert uV to V) — sum both BCC chains
  float pack_voltage_v = (local_stack_uv + local_stack_bcc1_uv) / 1000000.0f;
  Param::SetFloat(Param::packVoltage, pack_voltage_v);
  Param::SetFloat(Param::packVoltFilt, pack_voltage_v);

  // Pack current from IVT shunt
  if (ivt_shunt && ivt_shunt->is_alive()) {
    Param::SetFloat(Param::packCurrent, ivt_shunt->get_current());
  } else {
    Param::SetFloat(Param::packCurrent, 0.0f);
  }

  // SOC values
  Param::SetInt(Param::soc, (int)current_soc_percent);
  Param::SetFloat(Param::socPrecise, current_soc_percent);

  // BMS and HV states
  Param::SetInt(Param::bmsState, (int)current_state);
  Param::SetInt(Param::hvState, (int)hv_state);

  // Cell voltage statistics
  if (cell_count > 0) {
    uint32_t min_cell_uv = local_voltages[0];
    uint32_t max_cell_uv = local_voltages[0];

    for (uint8_t i = 1; i < cell_count; i++) {
      if (local_voltages[i] < min_cell_uv) min_cell_uv = local_voltages[i];
      if (local_voltages[i] > max_cell_uv) max_cell_uv = local_voltages[i];
    }

    // Convert uV to mV
    Param::SetInt(Param::maxCellVolt, max_cell_uv / 1000);
    Param::SetInt(Param::minCellVolt, min_cell_uv / 1000);
    Param::SetInt(Param::cellVoltDiff, (max_cell_uv - min_cell_uv) / 1000);
  }

  // Safe charge current
  Param::SetInt(Param::safeChargeCurrent, calculate_safe_charge_current());

  // Initialization status
  Param::SetInt(Param::bcc0Initialized, bcc0_initialized ? 1 : 0);
  Param::SetInt(Param::bcc1Initialized, bcc1_initialized ? 1 : 0);

  // Fault status (bit-packed)
  uint16_t fault_bits = 0;
  if (has_overvoltage_fault) fault_bits |= (1 << 0);
  if (has_undervoltage_fault) fault_bits |= (1 << 1);
  if (has_temperature_fault) fault_bits |= (1 << 2);
  if (has_cb_open_fault) fault_bits |= (1 << 3);
  if (has_cb_short_fault) fault_bits |= (1 << 4);
  Param::SetInt(Param::faultStatus, fault_bits);

  // Communication and contactor status
  Param::SetInt(Param::commLost, communication_lost ? 1 : 0);
  Param::SetInt(Param::contactorFault, contactor_fault ? 1 : 0);
}