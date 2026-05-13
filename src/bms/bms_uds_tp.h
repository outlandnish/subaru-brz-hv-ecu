#pragma once
/*
 * UDS ISO-TP transport shim — bridges iso14229 (UDS_TP_ISOTP_C) to the project's
 * CANBus abstraction.  The iso14229 library calls the three extern-C hooks below;
 * we forward them to the singleton CANBus pointer stored here.
 *
 * Usage:
 *   1. Call bms_uds_tp_init(can_bus, uds_queue) once before UDSServerInit().
 *      uds_queue must be a FreeRTOS queue of CAN_FRAME carrying only 0x7E0/0x7DF
 *      frames, populated by the CAN RX task.
 *   2. Call bms_uds_tp_poll() each server tick to drain uds_queue into ISO-TP.
 *   3. Pass &g_bms_uds_tp.hdl as srv.tp.
 */

#include "iso14229.h"
#include "can.h"
#include <STM32FreeRTOS.h>

// Addresses per EXTERNAL_BMS_PLAN §10
#define UDS_REQ_ID   0x7E0U
#define UDS_RESP_ID  0x7E8U
#define UDS_FUNC_ID  0x7DFU  // functional addressing (optional; mapped to NOOP)

extern UDSISOTpC_t g_bms_uds_tp;

void bms_uds_tp_init(CANBus *can, QueueHandle_t uds_queue);
void bms_uds_tp_poll();  // drain uds_can_queue and feed into ISO-TP
