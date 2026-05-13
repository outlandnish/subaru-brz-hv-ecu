#include <STM32FreeRTOS.h>
#include "Arduino.h"
#include "SPI.h"
#include "TPLSPI.h"
#include "BatteryCellController.h"
#include "bms/bms.h"
#include "hal/dma_config.h"
#include "hal/hv-ecu-v1-pins.h"
#include "can.h"
#include "taycan.h"
#include "params.h"
#include "param_save.h"
#include "canhardware_arduino.h"
#include "canmap.h"
#include "cansdo.h"
#include "debug_serial.h"

#include "version_gen.h"
#define FIRMWARE_VERSION ((FW_VERSION_MAJOR << 24) | (FW_VERSION_MINOR << 16) | (FW_VERSION_PATCH << 8) | 0)

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