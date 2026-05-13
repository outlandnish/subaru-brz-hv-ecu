#include "boot_jump.h"
#include "boot_shared.h"
#include <stdint.h>
#include <Arduino.h>

extern HardwareSerial BootSerial;

#ifdef __cplusplus
extern "C" {
#endif

void boot_jump_to_app(void) {
    uint32_t app_sp = *(volatile uint32_t *)APP_START_ADDR;
    uint32_t app_pc = *(volatile uint32_t *)(APP_START_ADDR + 4);

    // Sanity check: stack pointer must be in SRAM range
    if ((app_sp & 0xFF000000) != 0x20000000) {
        while (1);
    }

    // Confirm SP and PC before jumping
    BootSerial.print("BL: app_sp=0x"); BootSerial.print(app_sp, HEX);
    BootSerial.print(" app_pc=0x"); BootSerial.println(app_pc, HEX);
    BootSerial.flush();

    __disable_irq();

    // Disable and clear all NVIC interrupts
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // Disable SysTick
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Relocate vector table to application
    SCB->VTOR = APP_START_ADDR;

    // Set the main stack pointer and jump to the app reset handler
    __set_MSP(app_sp);
    __enable_irq();
    ((void (*)(void))app_pc)();

    while (1);
}

#ifdef __cplusplus
}
#endif
