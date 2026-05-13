#pragma once

#define WAKEUP PA0

#define STATUS_LEDS PB0
#define STATUS_LED_COUNT 10

// DRV8874 H-bridge contactor driver control (PMODE floating = independent half-bridge mode)
// IN1 and IN2 independently control OUT1 and OUT2 for two separate contactors
#define HV_CONTACTOR_1_PIN PD12             // DRV8874 IN1 (controls OUT1, HIGH when energizing)
#define HV_CONTACTOR_2_PIN PD13             // DRV8874 IN2 (controls OUT2, HIGH when energizing)
#define HV_CONTACTOR_NSLEEP_PIN PD10        // DRV8874 nSLEEP (must be HIGH to operate)
#define HV_CONTACTOR_FAULT_PIN PC0         // DRV8874 nFAULT (Fault input, active LOW)

#define AC_CONTACTOR_1_PIN PD14             // DRV8874 IN1 (controls OUT1, HIGH when energizing)
#define AC_CONTACTOR_2_PIN PD15             // DRV8874 IN2 (controls OUT2, HIGH when energizing)
#define AC_CONTACTOR_NSLEEP_PIN PD11        // DRV8874 nSLEEP (must be HIGH to operate)
#define AC_CONTACTOR_FAULT_PIN PC1         // DRV8874 nFAULT (Fault input, active LOW)

#define HVIL_DETECT_PIN PC2 

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

#define M3_CAN_RX PB12
#define M3_CAN_TX PB13

#define HV_CAN_RX PA11
#define HV_CAN_TX PA12

#define USART1_RX PB7
#define USART1_TX PA15_ALT1

#include "HardwareSerial.h"
extern HardwareSerial DebugSerial;
