#include <STM32FreeRTOS.h>
#include "Arduino.h"
#include "SPI.h"
#include "TPLSPI.h"
#include "BatteryCellController.h"

#define Serial SerialUSB

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
#define BMS1_RX_SCK PB3_ALT1   // Force SPI3 instead of SPI1
#define BMS1_RX_CS PA4_ALT1    // SPI3_NSS (PA4 defaults to SPI1, need ALT1 for SPI3)
#define BMS1_RX_DATA PB4_ALT1  // Force SPI3 instead of SPI1, todo: update PCB to use PB5 (MOSI not MISO)
#define BMS1_ENABLE PE10
#define BMS1_INTB PE11

#define STATUS_LEDS PD5
#define STATUS_LED_COUNT 5

#define NEGATIVE_CONTACTOR_CONTROL PC0
#define POSITIVE_CONTACTOR_CONTROL PC1
#define CONTACTOR_CONTROL_ENABLE PC2
#define CONTACTOR_CONTROL_FAULT PC3

#define M3_CAN_RX PD0
#define M3_CAN_TX PD1
#define M3_CAN_TERMINATION_ENABLE PD2

#define HV_CAN_RX PA8
#define HV_CAN_TX PB15
#define HV_CAN_TERMINATION_ENABLE PD3

#define IPC_CAN_RX PB12
#define IPC_CAN_TX PB13

#define PCS_ENABLE_CONTROL PD12
#define PCS_CHARGE_CONTROL PD13
#define PCS_DCDC_CONTROL PD14

#define PROXIMITY_PILOT_INPUT PC0
#define CONTROL_PILOT_INPUT PC1

bcc_status_t get_measurements(bcc_cid_t cid, uint16_t measurements[]);
bcc_status_t printInitialSettings(bcc_cid_t cid);
void printStackVoltages();

bool contactor_fault = false;
void enable_contactors();
void disable_contactors();
void control_contactors(bool enable_negative, bool enable_positive);