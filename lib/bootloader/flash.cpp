#include "flash.h"
#include "boot_shared.h"

// CRC-32/ISO-HDLC: polynomial 0xEDB88320, init 0xFFFFFFFF, final XOR 0xFFFFFFFF.
// Matches the algorithm used in param_save.cpp.
uint32_t flash_crc32(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else         crc = crc >> 1;
        }
    }
    return ~crc;
}

bool flash_app_crc_valid(void) {
    uint32_t stored_size = *(volatile uint32_t *)APP_SIZE_FLASH_ADDR;
    uint32_t stored_crc  = *(volatile uint32_t *)APP_CRC_FLASH_ADDR;

    if (stored_size == 0xFFFFFFFF || stored_size == 0 || stored_size > APP_SIZE_MAX)
        return false;
    if (stored_crc == 0xFFFFFFFF || stored_crc == 0)
        return false;

    uint32_t computed = flash_crc32((const uint8_t *)APP_START_ADDR, stored_size);
    return computed == stored_crc;
}
