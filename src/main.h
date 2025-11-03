#include <STM32FreeRTOS.h>
#include "Arduino.h"
#include "SPI.h"
#include "TPLSPI.h"
#include "BatteryCellController.h"
#include "taycan.h"
#include "uds.h"
#include "hv-ecu-v0-pins.h"

#define Serial SerialUSB

bcc_status_t get_measurements(bcc_cid_t cid, uint16_t measurements[]);
bcc_status_t printInitialSettings(bcc_cid_t cid);
void printStackVoltages();

bool contactor_fault = false;
void enable_contactors();
void disable_contactors();
void control_contactors(bool enable_contactor1, bool enable_contactor2);