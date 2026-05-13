#include "debug_serial.h"
#include <STM32FreeRTOS.h>
#include <stdio.h>

HardwareSerial DebugSerial(USART1_RX, USART1_TX);

static SemaphoreHandle_t s_mutex = nullptr;

void debug_serial_init() {
  s_mutex = xSemaphoreCreateMutex();
}

void debug_printf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    DebugSerial.print(buf);
    xSemaphoreGive(s_mutex);
  } else {
    DebugSerial.print(buf);
  }
}

void debug_println(const char *str) {
  if (s_mutex && xSemaphoreTake(s_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    DebugSerial.println(str);
    xSemaphoreGive(s_mutex);
  } else {
    DebugSerial.println(str);
  }
}

void debug_println() {
  debug_println("");
}
