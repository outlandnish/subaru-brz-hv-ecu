#pragma once
#include <STM32FreeRTOS.h>
#include "BatteryCellController.h"
#include "SPI.h"
#include "dma_config.h"
#include <Adafruit_NeoPixel.h>

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
#define BMS1_RX_SCK PB3
#define BMS1_RX_CS PA4
#define BMS1_RX_DATA PB4
#define BMS1_ENABLE PE10
#define BMS1_INTB PE11

enum BMS_State : uint8_t {
  BMS_Initialization,
  BMS_Idle,
  BMS_Charging,
  BMS_CellBalancing,
  BMS_Cooldown,
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

  BatteryCellControllerConfig *bcc0_config, *bcc1_config;
  SPIClass *bcc0_tx_spi, *bcc0_rx_spi;
  SPIClass *bcc1_tx_spi, *bcc1_rx_spi;
  bcc_device_t *devices_0, *devices_1;

  BMSChargingConfig charging_config;

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
  uint32_t last_successful_measurement;
  uint32_t communication_timeout_ms;
  bool communication_lost;

  // Contactor control pins
  uint8_t negative_contactor_pin;
  uint8_t positive_contactor_pin;
  uint8_t contactor_enable_pin;
  uint8_t contactor_fault_pin;

  bool contactor_fault;
  bool bcc1_enabled;

  // NeoPixel status LEDs
  Adafruit_NeoPixel *status_leds;
  uint8_t led_animation_step;

  // Task handles
  TaskHandle_t master_task_handle;
  TaskHandle_t bcc0_monitor_task_handle;
  TaskHandle_t bcc1_monitor_task_handle;

  // Private methods
  void master_task_loop();
  void bcc0_monitor_task_loop();
  void bcc1_monitor_task_loop();

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

  void enable_contactors();
  void disable_contactors();
  void control_contactors(bool enable_negative, bool enable_positive);

  // LED control
  void update_status_leds();
  void update_contactor_leds();
  void set_led_color(uint8_t led, uint32_t color);
  void set_state_leds(uint32_t color);  // Set LEDs 0-2 for state indication
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

  public:
    BatteryManagementSystem(BatteryCellControllerConfig *config0,
                           BatteryCellControllerConfig *config1 = nullptr);

    bool initialize(uint16_t device_configuration[][BCC_INIT_CONF_REG_CNT]);
    void configure_settings(uint16_t config[][BCC_INIT_CONF_REG_CNT]);
    void set_charging_config(BMSChargingConfig config);
    void set_contactor_pins(uint8_t negative, uint8_t positive, uint8_t enable, uint8_t fault);
    void set_status_leds(Adafruit_NeoPixel *leds);

    // Start the BMS tasks
    bool start_tasks();

    // Charging control
    void start_charging();
    void stop_charging();
    void force_balance_cells();

    // State and config getters
    BMS_State get_state() const { return current_state; }
    BMSChargingConfig get_charging_config() const { return charging_config; }
    void get_cell_voltages(uint32_t *voltages, uint8_t *count);
    void get_cell_voltages_filtered(uint32_t *voltages, uint8_t *count);
    uint32_t get_stack_voltage() const { return stack_voltage_uv; }
    uint32_t get_stack_voltage_filtered() const { return stack_voltage_filtered_uv; }
    void set_voltage_filter_alpha(float alpha);
    void get_fault_status(uint16_t *faults);
    bool has_faults() const;

    // Register dump
    void dump_registers();
    void print_fault_status();

    BMS_State enable_sleep_mode();
};