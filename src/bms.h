#pragma once
#include "BatteryCellController.h"
#include "SPI.h"

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

  public:
    BatteryManagementSystem(BatteryCellControllerConfig *config0, BatteryCellControllerConfig *config1 = nullptr);

    bool initialize(uint16_t device_configuration[][BCC_INIT_CONF_REG_CNT]);
    void configure_settings(uint16_t config[][BCC_INIT_CONF_REG_CNT]);
    bool read_measurements(uint32_t timeout_ms);
    void enable_charging();
    void disable_charging();
    BMS_State enable_sleep_mode();
};