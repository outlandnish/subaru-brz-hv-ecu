#pragma once
#include <stdint.h>
#include <stdbool.h>

uint32_t flash_crc32(const uint8_t *data, uint32_t len);
bool     flash_app_crc_valid(void);
