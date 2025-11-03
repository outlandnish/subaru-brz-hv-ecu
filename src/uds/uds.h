#ifndef UDS_H
#define UDS_H

#include "iso14229.h"
#include "can.h"
#include <Arduino.h>

#ifdef __cplusplus

// ============================================================================
// UDS Transport Layer Class (uses built-in ISO-TP)
// ============================================================================

class UDSTransport {
public:
    UDSTransport();
    ~UDSTransport();

    // Initialize the transport layer with CAN bus and addresses
    void init(CANBus* can_bus, uint32_t source_addr, uint32_t target_addr,
              uint32_t source_addr_func, uint32_t target_addr_func);

    // Get the C-compatible transport handle for passing to iso14229 client/server
    UDSTp_t* getHandle() { return &m_tp.hdl; }

    // Call this when a CAN frame is received (called from CAN RX callback)
    void onCanMessage(uint32_t arbitration_id, const uint8_t* data, uint8_t len);

    // Call this periodically to handle ISO-TP timing
    void poll();

    // Get the associated CAN bus
    CANBus* getCANBus() { return m_can_bus; }

    // Static instance pointer for C callback
    static UDSTransport* getInstance() { return s_instance; }
    static void setInstance(UDSTransport* instance) { s_instance = instance; }

private:
    UDSISOTpC_t m_tp;        // Built-in ISO-TP transport layer
    CANBus* m_can_bus;       // CAN bus instance to use for sending

    static UDSTransport* s_instance;  // Singleton for C callbacks
};

extern "C" {
#endif

// Note: The following functions are declared in iso14229.h,
// but you must implement them in uds.cpp:
// - uint32_t UDSMillis(void);
// - uint32_t isotp_user_get_us(void);
// - int isotp_user_send_can(...);
// - void isotp_user_debug(...);

#ifdef __cplusplus
}
#endif

#endif // UDS_H