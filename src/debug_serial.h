#pragma once
#include <stdarg.h>
#include "HardwareSerial.h"
#include "hal/hv-ecu-v1-pins.h"

extern HardwareSerial DebugSerial;

// Must be called once from setup() before any tasks are created.
void debug_serial_init();

// Thread-safe printf/println wrappers (10 ms mutex timeout).
void debug_printf(const char *fmt, ...);
void debug_println(const char *str);
void debug_println();
