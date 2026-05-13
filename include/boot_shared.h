#pragma once
#include <stdint.h>

#define BOOTLOADER_MAGIC     0xDEADBEEFUL
#define BOOTLOADER_FLAG_ADDR 0x2004FFF0UL  // last 16 bytes of 320KB RAM; read as raw pointer

#define APP_START_ADDR       0x08010000UL
#define APP_SIZE_MAX         (1472UL * 1024UL)
#define APP_SIZE_FLASH_ADDR  0x0800FFF8UL
#define APP_CRC_FLASH_ADDR   0x0800FFFCUL
