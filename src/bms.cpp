#include "bms.h"

BatteryManagementSystem::BatteryManagementSystem(BatteryCellControllerConfig *config0, BatteryCellControllerConfig *config1) {
  bcc0_config = config0;
  devices_0 = new bcc_device_t[config0->device_count];

  bcc0_tx_spi = new SPIClass(BMS0_TX_DATA, NC, BMS0_TX_SCK, NC);
  bcc0_rx_spi = new SPIClass(BMS0_RX_DATA, NC, BMS0_RX_SCK, BMS0_RX_CS);

  tpl0 = new TPLSPI(bcc0_tx_spi, bcc0_rx_spi, config0->cs_pin);
  bcc0 = new BatteryCellController(tpl0, devices_0, config0->device_count, config0->cell_count, config0->enable_pin, config0->intb_pin, config0->loopback);

  if (config1 == nullptr) {
    bcc1_config = config1;
    devices_1 = new bcc_device_t[config1->device_count];

    bcc1_tx_spi = new SPIClass(BMS1_TX_DATA, NC, BMS1_TX_SCK, NC);
    bcc1_rx_spi = new SPIClass(NC, BMS1_RX_DATA, BMS1_RX_SCK, BMS1_RX_CS);

    tpl1 = new TPLSPI(bcc1_tx_spi, bcc1_rx_spi, BMS1_TX_CS);
    bcc1 = new BatteryCellController(tpl1, devices_1, config1->device_count, config1->cell_count, config1->enable_pin, config1->intb_pin, config1->loopback);
  }

  current_state = BMS_Initialization;
}

bool BatteryManagementSystem::initialize(uint16_t device_configuration[][BCC_INIT_CONF_REG_CNT]) {
  bcc_status_t error = this->bcc0->begin(device_configuration);
  return error == BCC_STATUS_SUCCESS;
}

void BatteryManagementSystem::configure_settings(uint16_t config[][BCC_INIT_CONF_REG_CNT]) {
  // this->bcc0->init_devices(config);
}

bool BatteryManagementSystem::read_measurements(uint32_t timeout_ms) {
 auto error = this->bcc0->start_conversion_global_async(BCC_CONF1_ADC_CFG_VALUE);

  if (error != BCC_STATUS_SUCCESS) {  
    Serial.printf("Error starting global conversion: %d\r\n", error);
    return false;
  }

  uint32_t start_time = millis();
  bool completed = false;
  while (!completed) {
    error = this->bcc0->is_converting(BCC_CID_DEV1, &completed);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("Error checking conversion status: %d\r\n", error);
      return false;
    }

    if (millis() - start_time > timeout_ms) {
      Serial.println("Timeout waiting for conversion to complete");
      return false;
    }
  }

  return true;
}

BMS_State BatteryManagementSystem::enable_sleep_mode() {
  auto result = this->bcc0->enter_low_power_mode() == BCC_STATUS_SUCCESS;
  if (result)
    current_state = BMS_Sleep;
  
  return current_state;
}