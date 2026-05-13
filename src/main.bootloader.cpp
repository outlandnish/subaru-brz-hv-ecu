#include "boot_shared.h"
#include "flash.h"
#include "boot_jump.h"
#include "hal/hv-ecu-v1-pins.h"
#include <Arduino.h>

HardwareSerial BootSerial(USART1_RX, USART1_TX);

#define BOOTLOADER_STAY_MS  5000U
#define HEARTBEAT_MS        500U

static bool     s_app_crc_valid;
static bool     s_stay_flag;
static uint32_t s_stay_start_ms;

void setup() {
    BootSerial.begin(115200);
    BootSerial.println("\r\n=== BOOTLOADER START ===");

    volatile uint32_t *flag = (volatile uint32_t *)BOOTLOADER_FLAG_ADDR;
    uint32_t raw_flag = *flag;
    s_stay_flag = (raw_flag == BOOTLOADER_MAGIC);
    *flag = 0;

    BootSerial.print("BL: flag=0x"); BootSerial.print(raw_flag, HEX);
    BootSerial.print(" stay="); BootSerial.println(s_stay_flag ? "YES" : "NO");

    uint32_t stored_size = *(volatile uint32_t *)APP_SIZE_FLASH_ADDR;
    uint32_t stored_crc  = *(volatile uint32_t *)APP_CRC_FLASH_ADDR;
    BootSerial.print("BL: stored size=0x"); BootSerial.print(stored_size, HEX);
    BootSerial.print(" crc=0x"); BootSerial.println(stored_crc, HEX);

    s_app_crc_valid = flash_app_crc_valid();
    BootSerial.print("BL: app CRC valid=");
    BootSerial.println(s_app_crc_valid ? "YES" : "NO");

    if (s_app_crc_valid && !s_stay_flag) {
        BootSerial.println("BL: jumping to app...");
        BootSerial.flush();
        boot_jump_to_app();
    }

    s_stay_start_ms = millis();

    if (!s_app_crc_valid) {
        BootSerial.println("BL: no valid app CRC - staying");
    } else {
        BootSerial.println("BL: stay flag set - jumping back in 5s");
    }
}

void loop() {
    static uint32_t last_heartbeat = 0;
    uint32_t now = millis();

    if (now - last_heartbeat >= HEARTBEAT_MS) {
        last_heartbeat = now;
        BootSerial.print("BL: t="); BootSerial.print(now); BootSerial.println("ms");
    }

    if (s_stay_flag && s_app_crc_valid && (now - s_stay_start_ms >= BOOTLOADER_STAY_MS)) {
        BootSerial.println("BL: timeout - jumping to app");
        BootSerial.flush();
        boot_jump_to_app();
    }
}
