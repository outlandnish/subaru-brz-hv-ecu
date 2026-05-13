#include "bms_uds_tp.h"
#include "Arduino.h"

UDSISOTpC_t g_bms_uds_tp;
static CANBus       *s_can       = nullptr;
static QueueHandle_t s_uds_queue = nullptr;

// ── iso14229 extern-C hooks ───────────────────────────────────────────────────

extern "C" uint32_t isotp_user_get_us(void) {
    return (uint32_t)(millis() * 1000UL);
}

extern "C" int isotp_user_send_can(uint32_t arb_id, const uint8_t *data,
                                    const uint8_t size, void * /*ud*/) {
    if (!s_can) return -1;
    CAN_FRAME frame;
    frame.id       = arb_id;
    frame.length   = size;
    frame.extended = false;
    frame.rtr      = false;
    memcpy(frame.data.uint8, data, size);
    return s_can->sendFrame(frame) ? size : -1;
}

extern "C" void isotp_user_debug(const char * /*fmt*/, ...) {}

// ── Public API ────────────────────────────────────────────────────────────────

void bms_uds_tp_init(CANBus *can, QueueHandle_t uds_queue) {
    s_can       = can;
    s_uds_queue = uds_queue;

    const UDSISOTpCConfig_t cfg = {
        .source_addr      = UDS_RESP_ID,
        .target_addr      = UDS_REQ_ID,
        .source_addr_func = UDS_RESP_ID,
        .target_addr_func = UDS_TP_NOOP_ADDR,
    };
    UDSISOTpCInit(&g_bms_uds_tp, &cfg);
}

void bms_uds_tp_poll() {
    if (!s_uds_queue) return;

    CAN_FRAME frame;
    while (xQueueReceive(s_uds_queue, &frame, 0) == pdTRUE) {
        if (frame.id == UDS_REQ_ID) {
            isotp_on_can_message(&g_bms_uds_tp.phys_link,
                                  frame.data.uint8, frame.length);
        } else if (frame.id == UDS_FUNC_ID) {
            if (g_bms_uds_tp.phys_link.receive_status == ISOTP_RECEIVE_STATUS_IDLE)
                isotp_on_can_message(&g_bms_uds_tp.phys_link,
                                      frame.data.uint8, frame.length);
        }
    }
}
