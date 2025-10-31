/**
 * Dual-Mode Bootloader for STM32F413VH
 *
 * Features:
 * - 3 CAN buses (M3, HV, IPC)
 * - UDS diagnostic services on HV CAN
 * - USB DFU (Device Firmware Update)
 * - Firmware update via UDS (0x34 RequestDownload, 0x36 TransferData, 0x37 RequestTransferExit)
 * - Firmware update via USB DFU (compatible with dfu-util)
 * - Jump to application or stay in bootloader based on UDS session or USB activity
 */

#include <Arduino.h>
#include "can.h"
#include "uds.h"
#include "iso14229.h"
// Note: iso14229.h is already included via uds.h

// Debug output control - disable to save space
#define BOOTLOADER_DEBUG_ENABLED 0

#if BOOTLOADER_DEBUG_ENABLED
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(...)
#endif

// boot_deinit.c
#include <stdint.h>
#include "stm32f4xx_hal.h" // STM32Duino/PlatformIO core headers
#include "core_cm4.h"

// --- Configure these for your board (optional) ---
// If your board drives the USB pull-up (D+) via a GPIO, define it here.
// Otherwise leave undefined and the code will attempt a peripheral reset instead.
#define USB_DP_PORT GPIOA
#define USB_DP_PIN GPIO_PIN_12
// #define USB_DP_ACTIVE_HIGH 1   // set if driving D+ high to enable pull-up

// IRQ names — adjust if your core/variant differs
#ifndef OTG_FS_IRQn
#define OTG_FS_IRQn OTG_FS_IRQn
#endif

// Forward declarations
static void deinit_usb_stack_and_hw(void);
static void deinit_can_stack_and_hw(void);
static void gpio_reset_to_analog(GPIO_TypeDef *port, uint16_t pinmask);
static void jump_to_application(void);
static void bootloader_deinit_peripherals_for_jump(void);

// ============================================================================
// Configuration
// ============================================================================

#define APP_ADDRESS 0x08010000UL // example: app placed at 0x4000 offset

typedef void (*pAppEntry_t)(void);

static int is_valid_stack(uint32_t stack_pointer)
{
  return (stack_pointer >= 0x20000000U && stack_pointer <= 0x20050000U);
}

// Magic value for bootloader entry
// Location: Last 8 bytes of 320KB RAM
#define BOOTLOADER_MAGIC_ADDR 0x2004FFFC
#define BOOTLOADER_MAGIC_VALUE 0xDEADBEEF
#define BOOT_ATTEMPT_ADDR 0x2004FFF8 // Counter for boot attempts

// CAN Bus instances
// HV CAN: RX=PA8 (CAN2_RX), TX=PA15 (CAN2_TX), Termination=PD3
CANBus hvCan(PA8, PA15, PD3);

// UDS on HV CAN
UDSTransport udsTransport;
UDSServer_t udsServer;

// USB DFU
// USBDFU usbDfu;

// Bootloader state
bool inProgrammingSession = false;
uint32_t downloadAddress = 0;
uint32_t downloadSize = 0;
uint32_t bytesReceived = 0;

// ============================================================================
// Bootloader Magic Value
// ============================================================================

bool checkBootloaderMagic()
{
  volatile uint32_t *magicPtr = (volatile uint32_t *)BOOTLOADER_MAGIC_ADDR;
  return (*magicPtr == BOOTLOADER_MAGIC_VALUE);
}

void clearBootloaderMagic()
{
  volatile uint32_t *magicPtr = (volatile uint32_t *)BOOTLOADER_MAGIC_ADDR;
  *magicPtr = 0x00000000;
}

void setBootloaderMagic()
{
  volatile uint32_t *magicPtr = (volatile uint32_t *)BOOTLOADER_MAGIC_ADDR;
  *magicPtr = BOOTLOADER_MAGIC_VALUE;
}

// ============================================================================
// Flash Operations
// ============================================================================

bool flashUnlock()
{
  HAL_FLASH_Unlock();
  return true;
}

bool flashLock()
{
  HAL_FLASH_Lock();
  return true;
}

bool flashEraseSector(uint32_t sector)
{
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;

  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  eraseInit.Sector = sector;
  eraseInit.NbSectors = 1;

  return HAL_FLASHEx_Erase(&eraseInit, &sectorError) == HAL_OK;
}

bool flashWrite(uint32_t address, uint8_t *data, uint32_t length)
{
  for (uint32_t i = 0; i < length; i += 4)
  {
    uint32_t word = *(uint32_t *)(data + i);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i, word) != HAL_OK)
    {
      return false;
    }
  }
  return true;
}

// ============================================================================
// Jump to Application
// ============================================================================

void jump_to_application(void)
{
  uint32_t app_stack = *((uint32_t *)APP_ADDRESS);
  uint32_t app_reset = *((uint32_t *)(APP_ADDRESS + 4U));

  if (!is_valid_stack(app_stack))
  {
    Serial.println("No valid application stack pointer");
    // no valid app present
    return;
  }
  if ((app_reset & 1U) == 1U)
  {
    Serial.println("Valid application reset address");
    // Thumb check OK (LSB should be set)
  }
  else
  {
    Serial.println("Invalid application reset address");
    return; // not a valid thumb address
  }

  Serial.println("Valid application found - preparing to jump...");
  delay(200);

  // Disconnect USB/Serial
  Serial.end();
  delay(100);

  // Deinit CAN
  hvCan.end();

  // Disable global interrupts
  __disable_irq();

  // Disable all NVIC interrupts
  for (int i = 0; i < 8; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }

  // Disable SysTick
  SysTick->CTRL = 0U;
  SysTick->VAL = 0U;

  // Set stack pointer
  __set_MSP(app_stack);

  // Memory barriers
  __DSB();
  __ISB();

  // Jump to application
  pAppEntry_t app_entry = (pAppEntry_t)(app_reset & ~1U);
  app_entry();
}

// ============================================================================
// UDS Service Handler
// ============================================================================

UDSErr_t udsServerCallback(UDSServer_t *srv, UDSEvent_t ev, void *arg)
{
  switch (ev)
  {
  case UDS_EVT_DiagSessCtrl:
  {
    UDSDiagSessCtrlArgs_t *args = (UDSDiagSessCtrlArgs_t *)arg;
    Serial.printf("Session Control: Type=0x%02X\n", args->type);

    if (args->type == 0x02)
    { // Programming session
      inProgrammingSession = true;
      Serial.println("Entered programming session");
      return UDS_OK;
    }
    else if (args->type == 0x01)
    { // Default session
      inProgrammingSession = false;
      clearBootloaderMagic();
      Serial.println("Returned to default session - jumping to application...");
      delay(100);
      jump_to_application();
      return UDS_OK; // Should never reach here
    }
    return UDS_NRC_SubFunctionNotSupported;
  }

  case UDS_EVT_EcuReset:
  {
    UDSECUResetArgs_t *args = (UDSECUResetArgs_t *)arg;
    Serial.printf("ECU Reset: Type=0x%02X\n", args->type);

    if (args->type == 0x01)
    { // Hard reset
      delay(100);
      NVIC_SystemReset();
    }
    return UDS_OK;
  }

  case UDS_EVT_RequestDownload:
  {
    if (!inProgrammingSession)
    {
      return UDS_NRC_ServiceNotSupportedInActiveSession;
    }

    UDSRequestDownloadArgs_t *args = (UDSRequestDownloadArgs_t *)arg;
    downloadAddress = (uint32_t)args->addr;
    downloadSize = args->size;
    bytesReceived = 0;

    Serial.printf("Request Download: Addr=0x%08X, Size=%u bytes\n", downloadAddress, downloadSize);

    // Unlock flash
    flashUnlock();

    // Erase sectors (simplified - erase all application sectors)
    Serial.println("Erasing flash sectors...");
    for (uint8_t sector = 4; sector < 12; sector++)
    { // Sectors 4-11 (64KB to end)
      if (!flashEraseSector(sector))
      {
        Serial.printf("ERROR: Failed to erase sector %d\n", sector);
        flashLock();
        return UDS_NRC_ConditionsNotCorrect;
      }
    }

    Serial.println("Flash erased, ready for download");

    // Set max block length (2048 bytes)
    args->maxNumberOfBlockLength = 2048;

    return UDS_OK;
  }

  case UDS_EVT_TransferData:
  {
    if (!inProgrammingSession)
    {
      return UDS_NRC_ServiceNotSupportedInActiveSession;
    }

    UDSTransferDataArgs_t *args = (UDSTransferDataArgs_t *)arg;

    Serial.printf("Transfer Data: Len=%d\n", args->len);

    // Write to flash (align to 4 bytes for STM32)
    uint32_t alignedLen = (args->len + 3) & ~3;
    uint8_t alignedData[alignedLen];
    memcpy(alignedData, args->data, args->len);
    // Pad with 0xFF if needed
    for (uint32_t i = args->len; i < alignedLen; i++)
    {
      alignedData[i] = 0xFF;
    }

    if (!flashWrite(downloadAddress + bytesReceived, alignedData, alignedLen))
    {
      Serial.println("ERROR: Flash write failed");
      flashLock();
      return UDS_NRC_GeneralProgrammingFailure;
    }

    bytesReceived += args->len;
    Serial.printf("Progress: %u / %u bytes (%.1f%%)\n",
                  bytesReceived, downloadSize, (bytesReceived * 100.0f) / downloadSize);

    return UDS_OK;
  }

  case UDS_EVT_RequestTransferExit:
  {
    if (!inProgrammingSession)
    {
      return UDS_NRC_ServiceNotSupportedInActiveSession;
    }

    Serial.println("Transfer Exit");

    // Lock flash
    flashLock();

    if (bytesReceived == downloadSize)
    {
      Serial.println("Firmware update complete!");
      return UDS_OK;
    }
    else
    {
      Serial.printf("ERROR: Size mismatch! Expected %u, got %u\n", downloadSize, bytesReceived);
      return UDS_NRC_GeneralProgrammingFailure;
    }
  }

  default:
    return UDS_NRC_ServiceNotSupported;
  }
}

// ============================================================================
// CAN RX Callbacks
// ============================================================================

void onHvCanMessage(uint32_t id, uint8_t *data, uint8_t len)
{
  udsTransport.onCanMessage(id, data, len);
}

void onM3CanMessage(uint32_t id, uint8_t *data, uint8_t len)
{
  // Handle M3 CAN messages if needed
}

void onIpcCanMessage(uint32_t id, uint8_t *data, uint8_t len)
{
  // Handle IPC CAN messages if needed
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.println("\n\n========================================");
  Serial.println("Dual-Mode Bootloader v2.0");
  Serial.println("Subaru BRZ HV ECU");
  Serial.println("========================================\n");

  // Check bootloader magic value
  bool magicPresent = checkBootloaderMagic();

  if (magicPresent)
  {
    Serial.println("Bootloader magic detected - staying in bootloader mode");
    clearBootloaderMagic();
  }
  else
  {
    Serial.println("No bootloader magic - checking for valid application...");

    // Validate stack pointer and entry point
    jump_to_application();
  }

  // Initialize HV CAN (500 kbps)
  if (hvCan.begin(500000))
  {
    hvCan.setTermination(true);
    hvCan.enableRxInterrupt(onHvCanMessage);
    Serial.println("✓ HV CAN initialized (500 kbps)");
  }
  else
  {
    Serial.println("✗ HV CAN initialization failed");
  }

  // Initialize UDS on HV CAN
  // Using addresses: Bootloader=0x7E0, Tester=0x7E8
  udsTransport.init(&hvCan, 0x7E0, 0x7E8, 0x7DF, 0x7E0);

  // Initialize UDS server
  UDSServerInit(&udsServer);
  udsServer.tp = udsTransport.getHandle();
  udsServer.fn = udsServerCallback; // Register callback

  Serial.println("✓ UDS Server initialized on HV CAN");

  Serial.println("\nBootloader ready. Waiting for commands...");
  Serial.println("--- UDS over CAN ---");
  Serial.println("Use DiagnosticSessionControl (0x10 0x02) to enter programming mode\n");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop()
{
  // Poll UDS
  UDSServerPoll(&udsServer);
  udsTransport.poll();

  // Heartbeat - show status periodically
  static uint32_t lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000)
  {
    lastHeartbeat = millis();

    if (inProgrammingSession)
    {
      Serial.println("In UDS programming session - waiting for firmware...");
    }
    else
    {
      Serial.println("Bootloader idle - waiting for UDS commands...");
    }
  }
}

// Call this before resetting MSP & VTOR & jumping
void bootloader_deinit_peripherals_for_jump(void)
{
  // 1) Disable interrupts and SysTick (call upstream before this if you prefer)
  __disable_irq();
  SysTick->CTRL = 0U;
  SysTick->VAL = 0U;

  // 2) Stop Arduino timers / tasks if any (user responsibility)
  // e.g. stopTicker(); stopSerialTimers();

  // 3) Deinit USB (CDC/DFU) stack & hardware
  deinit_usb_stack_and_hw();

  // 4) Deinit CAN stack & hardware
  deinit_can_stack_and_hw();

  // 5) HAL deinit / RCC deinit best-effort
#ifdef HAL_RCC_DeInit
  HAL_RCC_DeInit();
  HAL_DeInit();
#endif

  // 6) Disable NVIC interrupts (cover many IRQs)
  for (int i = 0; i < 8; ++i)
  {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }
}

/* ===== USB deinit ===== */
static void deinit_usb_stack_and_hw(void)
{
  // Try high-level Cube/USB stack deinit if symbols are present:
  // Many projects use the Cube USB Device stack and expose USBD_HandleTypeDef hUsbDeviceFS
  // and USBD_DeInit(). If present, call it first to cleanly stop the stack.
#if defined(USBD_DeInit)
  extern USBD_HandleTypeDef hUsbDeviceFS; // if symbol exists
  if (&hUsbDeviceFS)
  {
    USBD_DeInit(&hUsbDeviceFS);
  }
#endif

  // If using HAL PCD (OTG) API, try HAL_PCD_DeInit if available
#ifdef HAL_PCD_MODULE_ENABLED
  // You may have a handle named hpcd_USB_OTG_FS depending on your build
  extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
  if (&hpcd_USB_OTG_FS)
  {
    HAL_PCD_DeInit(&hpcd_USB_OTG_FS);
  }
#endif

  // Disable USB IRQ(s) — common IRQ name for OTG FS:
#if defined(OTG_FS_IRQn)
  NVIC_DisableIRQ(OTG_FS_IRQn);
#endif

  // Force USB OTG FS peripheral reset via RCC (robust fallback)
#if defined(__HAL_RCC_USB_OTG_FS_FORCE_RESET)
  __HAL_RCC_USB_OTG_FS_FORCE_RESET();
  __HAL_RCC_USB_OTG_FS_RELEASE_RESET();
#else
  // If function macros not available, toggle APB reset register directly (less portable).
  // Example for F4 family (USB OTG FS on AHB2):
  // RCC->AHB1RSTR |= RCC_AHB1RSTR_OTGFSRST;  // Not portable; prefer HAL macros above
  // RCC->AHB1RSTR &= ~RCC_AHB1RSTR_OTGFSRST;
#endif

  // If your board drives the USB pull-up (D+) via a GPIO, explicitly disable it here
#ifdef USB_DP_PORT
  // Drive the D+ pin low / float so the host sees disconnect.
  // Configure as input (floating) or as analog to reduce noise.
  gpio_reset_to_analog(USB_DP_PORT, USB_DP_PIN);
#endif

  // As a final fallback, set USB pins to analog to release them
  // USB FS uses PA11 (DM), PA12 (DP) on many F4-based boards
  gpio_reset_to_analog(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);
}

/* ===== CAN deinit ===== */
static void deinit_can_stack_and_hw(void)
{
  // If HAL CAN handle exists, try to DeInit it
#if defined(HAL_CAN_MODULE_ENABLED)
  hvCan.end();
#endif

  // Disable CAN IRQs — names may vary: CAN1_TX_IRQn, CAN1_RX0_IRQn etc.
#if defined(CAN1_TX_IRQn)
  NVIC_DisableIRQ(CAN1_TX_IRQn);
#endif
#if defined(CAN1_RX0_IRQn)
  NVIC_DisableIRQ(CAN1_RX0_IRQn);
#endif
#if defined(CAN1_RX1_IRQn)
  NVIC_DisableIRQ(CAN1_RX1_IRQn);
#endif

  // Force CAN peripheral reset via RCC (robust fallback)
#if defined(__HAL_RCC_CAN1_FORCE_RESET)
  __HAL_RCC_CAN1_FORCE_RESET();
  __HAL_RCC_CAN1_RELEASE_RESET();
#elif defined(__HAL_RCC_CAN_FORCE_RESET)
  __HAL_RCC_CAN_FORCE_RESET();
  __HAL_RCC_CAN_RELEASE_RESET();
#endif

  // Reset CAN-related pins to analog (PA8/PA15)
  gpio_reset_to_analog(GPIOA, GPIO_PIN_8 | GPIO_PIN_15);
}

/* Utility: set pins to analog (lowest-power / released state) */
static void gpio_reset_to_analog(GPIO_TypeDef *port, uint16_t pinmask)
{
  if (pinmask == 0)
    return;
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = pinmask;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}
