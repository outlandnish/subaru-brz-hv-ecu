#pragma once

#define STATUS_LEDS PD5
#define STATUS_LED_COUNT 5

// DRV8874 H-bridge contactor driver control (PMODE floating = independent half-bridge mode)
// IN1 and IN2 independently control OUT1 and OUT2 for two separate contactors
#define CONTACTOR_1_PIN PC6             // DRV8874 IN1 (controls OUT1, HIGH when energizing)
#define CONTACTOR_2_PIN PC5             // DRV8874 IN2 (controls OUT2, HIGH when energizing)
#define CONTACTOR_NSLEEP_PIN PC2        // DRV8874 nSLEEP (must be HIGH to operate)
#define CONTACTOR_FAULT_PIN PC3         // DRV8874 nFAULT (Fault input, active LOW)

#define PCS_ENABLE_CONTROL PD12
#define PCS_CHARGE_CONTROL PD13
#define PCS_DCDC_CONTROL PD14

#define PROXIMITY_PILOT_INPUT PC0
#define CONTROL_PILOT_INPUT PC1
#define CONTROL_PILOT_OUTPUT PC7

// SPI1
#define BCC0_TX_SCK PA5
#define BCC0_TX_CS PA6
#define BCC0_TX_DATA PA7

// SPI2
#define BCC0_RX_SCK PA9
#define BCC0_RX_CS PB9
#define BCC0_RX_DATA PA10

#define BCC0_ENABLE PE8
#define BCC0_INTB PE9

// SPI4
#define BCC1_TX_SCK PE2
#define BCC1_TX_CS PE4
#define BCC1_TX_DATA PE6

// SPI3
#define BCC1_RX_SCK PB3_ALT1   // Force SPI3 instead of SPI1
#define BCC1_RX_CS PA4_ALT1    // SPI3_NSS (PA4 defaults to SPI1, need ALT1 for SPI3)
#define BCC1_RX_DATA PB5_ALT1

#define BCC1_ENABLE PE10
#define BCC1_INTB PE11

#define M3_CAN_RX PD0
#define M3_CAN_TX PD1
#define M3_CAN_TERM PD2

#define HV_CAN_RX PA8
#define HV_CAN_TX PA15
#define HV_CAN_TERM PD3

#define IPC_CAN_RX PB12
#define IPC_CAN_TX PB13