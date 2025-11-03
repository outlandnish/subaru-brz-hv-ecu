#include "uds.h"
#include <stdarg.h>

// Define static members
UDSTransport* UDSTransport::s_instance = nullptr;

// ============================================================================
// Platform-specific timing functions
// ============================================================================
// Note: UDSMillis() is provided by iso14229.c for STM32 (uses HAL_GetTick())

extern "C" uint32_t isotp_user_get_us(void) {
    return micros();
}

// ============================================================================
// Low-level CAN send function (uses CANBus library)
// ============================================================================

extern "C" int isotp_user_send_can(const uint32_t arbitration_id,
                                     const uint8_t* data, const uint8_t size
#if ISO_TP_USER_SEND_CAN_ARG
                                     , void *arg
#endif
                                     ) {
    // Get the UDS transport instance
    UDSTransport* transport = UDSTransport::getInstance();
    if (!transport) {
        return -1; // No instance configured
    }

    // Get the associated CAN bus
    CANBus* can_bus = transport->getCANBus();
    if (!can_bus) {
        return -1; // No CAN bus configured
    }

    // Send the CAN frame
    if (can_bus->sendMessage(arbitration_id, (uint8_t*)data, size)) {
        return 0; // ISOTP_RET_OK
    }

    return -1; // ISOTP_RET_ERROR
}

// ============================================================================
// Debug function (optional)
// ============================================================================

extern "C" void isotp_user_debug(const char* message, ...) {
#ifndef BOOTLOADER_BUILD
    char buf[256];
    va_list args;
    va_start(args, message);
    vsnprintf(buf, sizeof(buf), message, args);
    va_end(args);
    Serial.print("[ISO-TP] ");
    Serial.println(buf);
#else
    (void)message;  // Suppress unused parameter warning
#endif
}

// ============================================================================
// UDSTransport Class Implementation
// ============================================================================

UDSTransport::UDSTransport() : m_can_bus(nullptr) {
    memset(&m_tp, 0, sizeof(m_tp));
}

UDSTransport::~UDSTransport() {
    // Note: UDSISOTpCDeinit is declared but not implemented in iso14229
    if (s_instance == this) {
        s_instance = nullptr;
    }
}

void UDSTransport::init(CANBus* can_bus, uint32_t source_addr, uint32_t target_addr,
                        uint32_t source_addr_func, uint32_t target_addr_func) {
    m_can_bus = can_bus;

    // Set this as the singleton instance for C callbacks
    s_instance = this;

    // Configure the ISO-TP transport with CAN addresses
    UDSISOTpCConfig_t cfg = {
        .source_addr = source_addr,
        .target_addr = target_addr,
        .source_addr_func = source_addr_func,
        .target_addr_func = target_addr_func
    };

    UDSErr_t err = UDSISOTpCInit(&m_tp, &cfg);
    if (err != UDS_OK) {
#ifndef BOOTLOADER_BUILD
        Serial.print("UDSISOTpCInit failed: ");
        Serial.println(err);
#endif
        return;
    }

#ifndef BOOTLOADER_BUILD
    Serial.println("UDS Transport initialized");
    Serial.printf("  Source: 0x%03X, Target: 0x%03X\n", source_addr, target_addr);
    Serial.printf("  Func Source: 0x%03X, Func Target: 0x%03X\n", source_addr_func, target_addr_func);
#endif
}

void UDSTransport::onCanMessage(uint32_t arbitration_id, const uint8_t* data, uint8_t len) {
    // Feed received CAN frame into ISO-TP
    isotp_on_can_message(&m_tp.phys_link, (uint8_t*)data, len);
    isotp_on_can_message(&m_tp.func_link, (uint8_t*)data, len);
}

void UDSTransport::poll() {
    // Poll both physical and functional links
    isotp_poll(&m_tp.phys_link);
    isotp_poll(&m_tp.func_link);
}