#include <STM32FreeRTOS.h>
#include "Arduino.h"
#include "SPI.h"
#include "TPLSPI.h"
#include "BatteryCellController.h"
#include "bms/bms.h"
#include "hal/dma_config.h"
#include "hal/hv-ecu-v0-pins.h"
#include "can.h"
#include "taycan.h"
#include "params.h"
#include "param_save.h"
#include "canhardware_arduino.h"
#include "canmap.h"
#include "cansdo.h"

#define Serial SerialUSB

// Firmware version encoding: 0xMMmmppbb
// MM = Major (0-255), mm = Minor (0-255), pp = Patch (0-255), bb = Build (0-255)
// Example: Version 1.2.3 = 0x01020300, Version 1.12.5 = 0x010C0500
#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 5
#define FW_VERSION_PATCH 0
#define FW_VERSION_BUILD 0
#define FIRMWARE_VERSION ((FW_VERSION_MAJOR << 24) | (FW_VERSION_MINOR << 16) | (FW_VERSION_PATCH << 8) | FW_VERSION_BUILD)

// STM32 Unique Device ID (128 bits at fixed address)
#define STM32_UNIQUE_ID_BASE 0x1FFF7A10UL
#define STM32_UNIQUE_ID ((uint32_t*)STM32_UNIQUE_ID_BASE)

bcc_status_t get_measurements(bcc_cid_t cid, uint16_t measurements[]);
bcc_status_t printInitialSettings(bcc_cid_t cid);
void printStackVoltages();

bool contactor_fault = false;
void enable_contactors();
void disable_contactors();
void control_contactors(bool enable_contactor1, bool enable_contactor2);