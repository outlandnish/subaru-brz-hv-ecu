#pragma once
#include "hal/hv-ecu-v0-pins.h"
#include <STM32FreeRTOS.h>
#include "BatteryCellController.h"
#include "SPI.h"
#include "hal/dma_config.h"
#include <Adafruit_NeoPixel.h>
#include "evse/evse.h"
#include "pcs/pcs.h"
#include "can.h"
#include "ivt-s/ivt_shunt.h"

// HV System States
enum HV_State : uint8_t {
  HV_Disabled,      // HV system disabled, contactors open
  HV_Precharge,     // Precharge sequence in progress
  HV_Active,        // HV system active, contactors closed
  HV_Fault,         // HV system fault condition
  HV_Shutdown       // HV system shutting down
};

// HV Connection Mode (what we're using HV power for)
enum HV_Mode : uint8_t {
  HV_MODE_CHARGING, // Charging mode (AC charger active)
  HV_MODE_DRIVE     // Drive mode (DCDC only for 12V system)
};

// HV Connection Configuration
struct HVConnectionConfig {
  float precharge_voltage_margin_v;  // Acceptable voltage difference for precharge completion (V)
  uint32_t precharge_timeout_ms;     // Maximum time to wait for precharge (ms)
  uint32_t precharge_check_interval_ms; // How often to check precharge status (ms)
};

// BMS Operating States
enum BMS_State : uint8_t {
  BMS_Initialization,
  BMS_Idle,
  BMS_Charging,
  BMS_CellBalancing,
  BMS_Sleep,
  BMS_Error
};

struct BatteryCellControllerConfig{
  uint8_t device_count;
  uint8_t cell_count;
  bcc_device_t devices[BCC_DEVICE_CNT_MAX];
  uint8_t enable_pin;
  uint8_t intb_pin;
  uint8_t cs_pin;
  bool loopback;
};

struct BMSChargingConfig {
  float target_cell_voltage;      // Target voltage per cell (e.g., 3.6V)
  float balance_threshold_mv;     // Cell voltage difference threshold to start balancing (mV)
  float balance_target_mv;        // Cell voltage difference to resume charging (mV)
  uint16_t balancing_timer_min;   // Balancing timer duration in minutes
  uint16_t measurement_interval_ms; // How often to measure voltages
};

class BatteryManagementSystem {
  TPLSPI *tpl0;
  BatteryCellController *bcc0;

  TPLSPI *tpl1;
  BatteryCellController *bcc1;

  BMS_State current_state;
  HV_State hv_state;
  HV_Mode hv_mode;  // Current HV mode (charging or drive)
  uint32_t hv_state_entry_time;  // Time when current HV state was entered
  uint32_t precharge_start_time;  // Time when precharge started

  BatteryCellControllerConfig *bcc0_config, *bcc1_config;
  SPIClass *bcc0_tx_spi, *bcc0_rx_spi;
  SPIClass *bcc1_tx_spi, *bcc1_rx_spi;
  bcc_device_t *devices_0, *devices_1;

  BMSChargingConfig charging_config;
  HVConnectionConfig hv_config;

  // Cell voltage tracking (raw and filtered)
  uint32_t cell_voltages_uv[BCC_MAX_CELLS];           // Raw measurements
  uint32_t cell_voltages_filtered_uv[BCC_MAX_CELLS];  // Exponentially filtered
  uint32_t stack_voltage_uv;                          // Raw measurement
  uint32_t stack_voltage_filtered_uv;                 // Exponentially filtered
  float voltage_filter_alpha;                         // Exponential filter coefficient (0-1)
  uint8_t cells_to_balance[BCC_MAX_CELLS];

  // Fault status tracking
  uint16_t fault_status[11];  // Array to store all fault registers
  bool has_overvoltage_fault;
  bool has_undervoltage_fault;
  bool has_temperature_fault;
  bool has_cb_open_fault;
  bool has_cb_short_fault;
  uint32_t last_fault_check;
  uint32_t fault_check_interval_ms;

  // Initialization flag and communication tracking
  bool hardware_initialized;
  bool bcc0_initialized;
  bool bcc1_initialized;
  uint32_t last_successful_measurement;
  uint32_t communication_timeout_ms;
  bool communication_lost;

  // Contactor control pins
  // Note: Variable names kept for compatibility, but now represent:
  // positive_contactor_pin = IN1 (controls OUT1 for contactor 1)
  // negative_contactor_pin = IN2 (controls OUT2 for contactor 2)
  uint8_t negative_contactor_pin;
  uint8_t positive_contactor_pin;
  uint8_t contactor_enable_pin;
  uint8_t contactor_fault_pin;

  bool contactor_fault;
  bool bcc1_enabled;

  // EVSE Controller
  EVSEController *evse;

  // IVT Current Shunt
  IVTShunt *ivt_shunt;

  // CAN buses
  CANBus *ipc_can;
  CANBus *m3_can;
  CANBus *hv_can;

  // NeoPixel status LEDs
  Adafruit_NeoPixel *status_leds;
  uint8_t led_animation_step;

  // Task handles
  TaskHandle_t master_task_handle;
  TaskHandle_t bcc0_monitor_task_handle;
  TaskHandle_t bcc1_monitor_task_handle;
  TaskHandle_t hv_can_task_handle;

  // Private methods
  void master_task_loop();
  void bcc0_monitor_task_loop();
  void bcc1_monitor_task_loop();
  void hv_can_task_loop();

  bool measure_cell_voltages(BatteryCellController *bcc, uint32_t *cell_voltages);
  bool measure_stack_voltage(BatteryCellController *bcc, uint32_t *stack_voltage);
  void apply_exponential_filter();
  bool read_fault_status(BatteryCellController *bcc);
  void check_faults();
  void calculate_cell_balance_requirements(uint32_t *cell_voltages, uint8_t cell_count,
                                           uint8_t *cells_to_balance);
  void apply_cell_balancing(BatteryCellController *bcc, uint8_t *cells_to_balance,
                           uint8_t cell_count);
  void stop_cell_balancing(BatteryCellController *bcc, uint8_t cell_count);
  float get_max_cell_voltage_diff_mv(uint32_t *cell_voltages, uint8_t cell_count);
  bool has_reached_target_voltage(uint32_t *cell_voltages, uint8_t cell_count);

  // HV system control
  void hv_connect(HV_Mode mode);  // Start HV connection sequence with specified mode
  void hv_disconnect();           // Disconnect HV system
  void update_hv_state();         // Update HV state machine
  bool is_precharge_complete();   // Check if precharge voltage reached

  // Legacy contactor control (will be replaced by HV state machine)
  void enable_contactors();
  void disable_contactors();
  void control_contactors(bool enable_contactor1, bool enable_contactor2);

  // LED control
  void update_status_leds();
  void update_pcs_led();             // Update LED 4 for PCS state
  void update_evse_led();
  void update_hv_led();              // Update LED 2 for HV state
  void set_led_color(uint8_t led, uint32_t color);
  void set_state_leds(uint32_t color);  // Set LEDs 0-1 for BMS state indication
  void led_pattern_idle();
  void led_pattern_charging();
  void led_pattern_balancing();
  void led_pattern_complete();
  void led_pattern_error();
  uint32_t color_rgb(uint8_t r, uint8_t g, uint8_t b);

  // Static task wrappers for FreeRTOS
  static void master_task_wrapper(void *pvParameters);
  static void bcc0_monitor_task_wrapper(void *pvParameters);
  static void bcc1_monitor_task_wrapper(void *pvParameters);
  static void hv_can_task_wrapper(void *pvParameters);

  public:
    BatteryManagementSystem(BatteryCellControllerConfig *config0,
                           BatteryCellControllerConfig *config1);

    bool initialize(uint16_t device_configuration[][BCC_INIT_CONF_REG_CNT]);
    void configure_settings(uint16_t config[][BCC_INIT_CONF_REG_CNT]);
    void set_charging_config(BMSChargingConfig config);
    void set_contactor_pins(uint8_t contactor1, uint8_t contactor2, uint8_t enable, uint8_t fault);
    void set_status_leds(Adafruit_NeoPixel *leds);

    // EVSE and IVT configuration
    void set_evse(EVSEController *evse_controller);
    void set_ivt_shunt(IVTShunt *shunt);
    void set_can_buses(CANBus *ipc_can_bus, CANBus *m3_can_bus, CANBus *hv_can_bus);

    // Start the BMS tasks
    bool start_tasks();

    // HV operation control
    void start_charging();      // Start charging mode (precharge + charging)
    void start_drive_mode();    // Start drive mode (precharge + DCDC only)
    void stop_charging();       // Stop charging
    void stop_hv_system();      // Stop HV system (any mode)
    void force_balance_cells();

    // State and config getters
    BMS_State get_state() const { return current_state; }
    HV_State get_hv_state() const { return hv_state; }
    void set_hv_state(HV_State state) { hv_state = state; }
    BMSChargingConfig get_charging_config() const { return charging_config; }
    bool is_bcc0_initialized() const { return bcc0_initialized; }
    bool is_bcc1_initialized() const { return bcc1_initialized; }
    bool is_bcc1_enabled() const { return bcc1_enabled; }
    void get_cell_voltages(uint32_t *voltages, uint8_t *count);
    void get_cell_voltages_filtered(uint32_t *voltages, uint8_t *count);
    uint32_t get_stack_voltage() const { return stack_voltage_uv; }
    uint32_t get_stack_voltage_filtered() const { return stack_voltage_filtered_uv; }
    void set_voltage_filter_alpha(float alpha);
    void get_fault_status(uint16_t *faults);
    bool has_faults() const;

    uint16_t get_bcc0_total_cell_count() const {
      return bcc0_config->cell_count * bcc0_config->device_count;
    }

    uint16_t get_bcc1_total_cell_count() const {
      return bcc1_config->cell_count * bcc1_config->device_count;
    }

    uint16_t get_target_stack_voltage() const {
      const uint16_t total_bcc0_cells = get_bcc0_total_cell_count();
      const uint16_t total_bcc1_cells = get_bcc1_total_cell_count();

      return (uint16_t)(charging_config.target_cell_voltage *
                        (total_bcc0_cells + total_bcc1_cells));
    }

    // EVSE and IVT status
    EVSEController* get_evse() const { return evse; }
    IVTShunt* get_ivt_shunt() const { return ivt_shunt; }

    // Register dump
    void dump_registers();
    void print_fault_status();

    BMS_State enable_sleep_mode();
};