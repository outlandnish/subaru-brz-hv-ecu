#include "main.h"
#include "dma_config.h"
#include <STM32FreeRTOS.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(STATUS_LED_COUNT, STATUS_LEDS, NEO_GRB + NEO_KHZ800);

#define DEVICE_COUNT 1
#define CELL_COUNT 6
bcc_device_t devices[] = {
  BCC_DEVICE_MC33772,
};

SPIClass bcc0_tx_spi(BMS0_TX_DATA, NC, BMS0_TX_SCK, NC);
SPIClass bcc0_rx_spi(BMS0_RX_DATA, NC, BMS0_RX_SCK, BMS0_RX_CS);
#if defined(BRZ_HV_ECU)
TPLSPI bcc0_tpl(&bcc0_tx_spi, &bcc0_rx_spi, BMS0_TX_CS, configureDMA_HV_ECU);
#else
TPLSPI bcc0_tpl(&bcc0_tx_spi, &bcc0_rx_spi, BMS0_TX_CS);
#endif
BatteryCellController *bcc0;

// BCC1 setup
#if defined(BRZ_HV_ECU)
SPIClass bcc1_tx_spi(BMS1_TX_DATA, NC, BMS1_TX_SCK, NC);
// BMS1 RX: PB4_ALT1 is SPI3_MISO - pass as MISO (2nd param) for RX-only slave mode
SPIClass bcc1_rx_spi(NC, BMS1_RX_DATA, BMS1_RX_SCK, BMS1_RX_CS);
TPLSPI bcc1_tpl(&bcc1_tx_spi, &bcc1_rx_spi, BMS1_TX_CS, configureDMA_HV_ECU);
BatteryCellController *bcc1;
#endif

bcc_cid_t cid;
bcc_status_t error;

bool init_succeeded = false;
#if defined(BRZ_HV_ECU)
bool init_succeeded_bcc1 = false;
#endif

// Function declarations
bcc_status_t get_measurements(bcc_cid_t cid, uint16_t measurements[]);
bcc_status_t printInitialSettings(bcc_cid_t cid);
void printStackVoltages();

#if defined(BRZ_HV_ECU)
bcc_status_t printInitialSettings_bcc1(bcc_cid_t cid);
void printStackVoltages_bcc1();
#endif

// BMS Task - handles all BCC initialization and communication
void bms_task(void *pvParameters) {
  delay(5000);
  Serial.println("BMS Task: Starting BMS controller...");

  // STM32 initialization
  pinMode(BMS0_TX_CS, OUTPUT);
  digitalWrite(BMS0_TX_CS, HIGH);

  bcc0 = new BatteryCellController(&bcc0_tpl, devices, DEVICE_COUNT, CELL_COUNT, BMS0_ENABLE, BMS0_INTB, false);

  Serial.println("BMS Task: Starting BCC initialization (preserving existing config)...");
  error = bcc0->begin(nullptr);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("BMS Task: BatteryCellController initialization failed: %d\r\n", error);
    while (true) {
      Serial.println("BMS Task: Initialization failed...");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  init_succeeded = true;

  int startCid = static_cast<int>(BCC_CID_DEV1);
  int endCid = startCid + DEVICE_COUNT - 1;

  // Read and display all configuration registers from the battery
  Serial.println("\n========================================");
  Serial.println("Reading existing battery configuration...");
  Serial.println("========================================\n");

  for (int i = startCid; i <= endCid; ++i) {
    cid = static_cast<bcc_cid_t>(i);
    error = printInitialSettings(cid);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("Error reading config for CID %d: %d\r\n", cid, error);
    }
  }

  Serial.println("BMS Task: Initialization complete, starting measurements...\n");

  // Main BMS loop
  unsigned long startTime = millis();
  bool hasSentSleep = false;

  while (true) {
    unsigned long elapsedTime = millis() - startTime;

    if (elapsedTime >= 10000 && !hasSentSleep) {
      // 10 seconds elapsed - put BCC to sleep
      Serial.println("\n==============================================");
      Serial.println("10 seconds elapsed - sending BCC to sleep...");
      Serial.println("==============================================\n");

      // First, disable cyclic timer to prevent ongoing measurements
      Serial.println("Disabling cyclic timer...");
      error = bcc0->update_register(
        BCC_CID_DEV1,
        MC33771C_SYS_CFG1_OFFSET,
        MC33771C_SYS_CFG1_CYCLIC_TIMER_MASK,
        MC33771C_SYS_CFG1_CYCLIC_TIMER(MC33771C_SYS_CFG1_CYCLIC_TIMER_DISABLED_ENUM_VAL)
      );
      if (error != BCC_STATUS_SUCCESS) {
        Serial.printf("Error disabling cyclic timer: %d\r\n", error);
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      Serial.println("Cyclic timer disabled.");

      // Small delay to ensure the change takes effect
      vTaskDelay(pdMS_TO_TICKS(100));

      // Now send the sleep command
      Serial.println("Sending GO2SLEEP command...");
      error = bcc0->sleep();
      if (error != BCC_STATUS_SUCCESS) {
        Serial.printf("Error sending BCC to sleep: %d\r\n", error);
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      Serial.println("\nWaiting for BCC to enter sleep mode...");
      Serial.println("According to datasheet: device resets after 1 second of no communication");
      Serial.println("VCOM LED should turn off...");

      // Per datasheet: wait >1 second with no communication
      vTaskDelay(pdMS_TO_TICKS(1500));

      Serial.println("\n==============================================");
      Serial.println("BCC should now be in sleep/idle mode.");
      Serial.println("VCOM LED (red) should be OFF.");
      Serial.println("If LED is still on, device may auto-sleep after 60 seconds.");
      Serial.println("Safe to disconnect TPL connection.");
      Serial.println("==============================================\n");

      bcc0->disable_tpl();

      hasSentSleep = true;
    }

    if (hasSentSleep) {
      // Already sent sleep command, just wait
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Read and print stack voltages
    printStackVoltages();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

#if defined(BRZ_HV_ECU)
// BMS1 Task - handles second BCC initialization and communication
void bms1_task(void *pvParameters) {
  // Wait for BMS0 to finish initialization
  Serial.println("BMS1 Task: Waiting for BMS0 to complete initialization...");
  while (!init_succeeded) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  Serial.println("BMS1 Task: BMS0 initialized, waiting 2 seconds before starting BMS1...");
  vTaskDelay(pdMS_TO_TICKS(2000));

  Serial.println("BMS1 Task: Starting BMS controller...");

  // STM32 initialization
  pinMode(BMS1_TX_CS, OUTPUT);
  digitalWrite(BMS1_TX_CS, HIGH);

  bcc1 = new BatteryCellController(&bcc1_tpl, devices, DEVICE_COUNT, CELL_COUNT, BMS1_ENABLE, BMS1_INTB, false);

  Serial.println("BMS1 Task: Starting BCC initialization (preserving existing config)...");
  bcc_status_t error = bcc1->begin(nullptr);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("BMS1 Task: BatteryCellController initialization failed: %d\r\n", error);
    while (true) {
      Serial.println("BMS1 Task: Initialization failed...");
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  init_succeeded_bcc1 = true;

  int startCid = static_cast<int>(BCC_CID_DEV1);
  int endCid = startCid + DEVICE_COUNT - 1;

  // Read and display all configuration registers from the battery
  Serial.println("\n========================================");
  Serial.println("BMS1: Reading existing battery configuration...");
  Serial.println("========================================\n");

  for (int i = startCid; i <= endCid; ++i) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(i);
    error = printInitialSettings_bcc1(cid);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("BMS1: Error reading config for CID %d: %d\r\n", cid, error);
    }
  }

  Serial.println("BMS1 Task: Initialization complete, starting measurements...\n");

  // Main BMS loop
  unsigned long startTime = millis();
  bool hasSentSleep = false;

  while (true) {
    unsigned long elapsedTime = millis() - startTime;

    if (elapsedTime >= 10000 && !hasSentSleep) {
      // 10 seconds elapsed - put BCC to sleep
      Serial.println("\n==============================================");
      Serial.println("BMS1: 10 seconds elapsed - sending BCC to sleep...");
      Serial.println("==============================================\n");

      // First, disable cyclic timer to prevent ongoing measurements
      Serial.println("BMS1: Disabling cyclic timer...");
      error = bcc1->update_register(
        BCC_CID_DEV1,
        MC33771C_SYS_CFG1_OFFSET,
        MC33771C_SYS_CFG1_CYCLIC_TIMER_MASK,
        MC33771C_SYS_CFG1_CYCLIC_TIMER(MC33771C_SYS_CFG1_CYCLIC_TIMER_DISABLED_ENUM_VAL)
      );
      if (error != BCC_STATUS_SUCCESS) {
        Serial.printf("BMS1: Error disabling cyclic timer: %d\r\n", error);
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      Serial.println("BMS1: Cyclic timer disabled.");

      // Small delay to ensure the change takes effect
      vTaskDelay(pdMS_TO_TICKS(100));

      // Now send the sleep command
      Serial.println("BMS1: Sending GO2SLEEP command...");
      error = bcc1->sleep();
      if (error != BCC_STATUS_SUCCESS) {
        Serial.printf("BMS1: Error sending BCC to sleep: %d\r\n", error);
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }

      Serial.println("\nBMS1: Waiting for BCC to enter sleep mode...");
      Serial.println("BMS1: According to datasheet: device resets after 1 second of no communication");

      // Per datasheet: wait >1 second with no communication
      vTaskDelay(pdMS_TO_TICKS(1500));

      Serial.println("\n==============================================");
      Serial.println("BMS1: BCC should now be in sleep/idle mode.");
      Serial.println("BMS1: Safe to disconnect TPL connection.");
      Serial.println("==============================================\n");

      bcc1->disable_tpl();

      hasSentSleep = true;
    }

    if (hasSentSleep) {
      // Already sent sleep command, just wait
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

    // Read and print stack voltages
    printStackVoltages_bcc1();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
#endif

void contactor_fault_isr() {
  contactor_fault = digitalRead(CONTACTOR_CONTROL_FAULT) == LOW;
  disable_contactors();
}

void enable_contactors() {
  digitalWrite(CONTACTOR_CONTROL_ENABLE, HIGH);
}

void disable_contactors() {
  digitalWrite(CONTACTOR_CONTROL_ENABLE, LOW);
}

void control_contactors(bool enable_negative, bool enable_positive) {
  if (contactor_fault) {
    // Fault detected, disable both contactors
    digitalWrite(NEGATIVE_CONTACTOR_CONTROL, LOW);
    digitalWrite(POSITIVE_CONTACTOR_CONTROL, LOW);
    return;
  }

  digitalWrite(NEGATIVE_CONTACTOR_CONTROL, enable_negative ? HIGH : LOW);
  digitalWrite(POSITIVE_CONTACTOR_CONTROL, enable_positive ? HIGH : LOW);
}

void setup() {
  // Initialize Serial FIRST
  Serial.begin(115200);
  Serial.println("=== BMS System with FreeRTOS ===");

  Serial.println("Creating BMS task...");

  pinMode(NEGATIVE_CONTACTOR_CONTROL, OUTPUT);
  pinMode(POSITIVE_CONTACTOR_CONTROL, OUTPUT);
  pinMode(CONTACTOR_CONTROL_ENABLE, OUTPUT);
  pinMode(CONTACTOR_CONTROL_FAULT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CONTACTOR_CONTROL_FAULT), contactor_fault_isr, CHANGE);

  // Create BMS task with sufficient stack
  xTaskCreate(
    bms_task,
    "BMS_Task",
    2048,  // Stack size in words (8KB) - BCC needs more stack
    NULL,
    2,     // Priority
    NULL
  );

// #if defined(BRZ_HV_ECU)
//   Serial.println("Creating BMS1 task...");

//   // Create BMS1 task with sufficient stack
//   xTaskCreate(
//     bms1_task,
//     "BMS1_Task",
//     2048,  // Stack size in words (8KB) - BCC needs more stack
//     NULL,
//     2,     // Priority
//     NULL
//   );
// #endif

  Serial.println("Starting FreeRTOS scheduler...");
  Serial.println("Note: Serial output will continue from BMS tasks\n");

  // Start the scheduler - THIS NEVER RETURNS
  vTaskStartScheduler();

  // Should never reach here
  Serial.println("ERROR: Scheduler failed to start!");
  while (1);
}

void loop() {
  // Empty - FreeRTOS tasks run instead
}

bcc_status_t get_measurements(bcc_cid_t cid, uint16_t measurements[])
{
  bool completed;
  bcc_status_t error;

  /* Start conversion. */
  error = bcc0->start_conversion_async(cid);
  if (error != BCC_STATUS_SUCCESS)
    return error;

  /* Wait for completion. */
  delayMicroseconds(600);

  /* Check the conversion is complete. */
  do
  {
    error = bcc0->is_converting(cid, &completed);
    if (error != BCC_STATUS_SUCCESS)
      return error;
  } while (!completed);

  /* Read measured values. */
  return bcc0->get_raw_values(cid, measurements);
}

bcc_status_t printInitialSettings(bcc_cid_t cid)
{
  uint64_t guid;
  uint16_t regVal;
  uint8_t i;
  static char* printPattern = "  | %-18s | 0x%02X%02X |\r\n";
  bcc_status_t error;

  Serial.printf("###############################################\r\n");
  Serial.printf("# CID %d (MC3377%s): Initial value of registers\r\n", cid,
          (devices[cid - 1] == BCC_DEVICE_MC33771) ?
                  "1" : "2");
  Serial.printf("###############################################\r\n\r\n");

  Serial.printf("  -------------------------------\r\n");
  Serial.printf("  | Register           | Value  |\r\n");
  Serial.printf("  -------------------------------\r\n");

  error = bcc0->read_register(cid, BCC_REG_INIT_ADDR, 1U, &regVal);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("Error reading register INIT (CID %d): %d\r\n", cid, error);
    return error;
  }

  Serial.printf(printPattern, "INIT", regVal >> 8, regVal & 0xFFU);

  if (devices[cid - 1] == BCC_DEVICE_MC33771)
  {
    for (i = 0U; i < REG_CONF_CNT_MC33771; i++)
    {
      error = bcc0->read_register(cid, BCC_REGISTERS_DATA_MC33771[i].address, 1U, &regVal);
      if (error != BCC_STATUS_SUCCESS) {
        Serial.printf("Error reading register %s (CID %d): %d\r\n",
                      BCC_REGISTERS_DATA_MC33771[i].name, cid, error);
        return error;
      }
      Serial.printf(printPattern, BCC_REGISTERS_DATA_MC33771[i].name, regVal >> 8, regVal & 0xFFU);
    }
  }
  else
  {
    for (i = 0U; i < REG_CONF_CNT_MC33772; i++)
    {
      error = bcc0->read_register(cid, BCC_REGISTERS_DATA_MC33772[i].address, 1U, &regVal);
      if (error != BCC_STATUS_SUCCESS)
      {
        Serial.printf("Error reading register %s (CID %d): %d\r\n",
                      BCC_REGISTERS_DATA_MC33772[i].name, cid, error);
        return error;
      }
      Serial.printf(printPattern, BCC_REGISTERS_DATA_MC33772[i].name, regVal >> 8, regVal & 0xFFU);
    }
  }

  Serial.printf("  ----------------------------------\r\n");
  Serial.printf("\r\n");

  Serial.printf("  ------------------------\r\n");
  Serial.printf("  | Fuse Mirror | Value  |\r\n");
  Serial.printf("  |  Register   |        |\r\n");
  Serial.printf("  ------------------------\r\n");
  for (i = 0U; i <= ((devices[cid - 1] == BCC_DEVICE_MC33771) ?
    BCC_LAST_FUSE_ADDR_MC33771B : BCC_LAST_FUSE_ADDR_MC33772B); i++)
  {
    error = bcc0->read_fuse_mirror(cid, i, &regVal);
    if (error != BCC_STATUS_SUCCESS)
      return error;

    Serial.printf("  | $%02X\t| 0x%02X%02X |\r\n", i, regVal >> 8, regVal & 0xFFU);
  }
  Serial.printf("  ------------------------\r\n");
  Serial.printf("\r\n");

  error = bcc0->read_guid(cid, &guid);
  if (error != BCC_STATUS_SUCCESS)
    return error;

  Serial.printf("  Device GUID: %02X%04X%04X\r\n",
          (uint16_t)((guid >> 32) & 0x001FU),
          (uint16_t)((guid >> 16) & 0xFFFFU),
          (uint16_t)(guid & 0xFFFFU));
  Serial.printf("\r\n");

  return BCC_STATUS_SUCCESS;
}

void printStackVoltages() {
  float stackVoltages[DEVICE_COUNT];
  uint16_t stackMeasurements[BCC_MEAS_CNT];
  bcc_status_t error;
  bool completed;

  // Start conversion on all devices simultaneously using global write
  error = bcc0->start_conversion_global_async(0x0717);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("Error starting global conversion: %d\r\n", error);
    return;
  }

  // Wait for conversion to complete (typical time is 520us)
  delayMicroseconds(600);

  // Check if conversion completed on first device (they all convert simultaneously)
  do {
    error = bcc0->is_converting(BCC_CID_DEV1, &completed);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("Error checking conversion status: %d\r\n", error);
      return;
    }
  } while (!completed);

  // Read measurements from each device
  for (uint8_t i = 1; i <= DEVICE_COUNT; i++) {
    cid = static_cast<bcc_cid_t>(i);
    error = bcc0->get_raw_values(cid, stackMeasurements);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("Error getting measurements for CID %d: %d\r\n", i, error);
      stackVoltages[i - 1] = 0.0f;
      continue;
    }

    uint16_t rawValue = stackMeasurements[BCC_MSR_STACK_VOLT];
    uint32_t voltageMicrovolts = BCC_GET_STACK_VOLT(rawValue);
    stackVoltages[i - 1] = voltageMicrovolts / 1000000.0f;  // Convert uV to V
  }

  // Print results
  Serial.println("Stack Voltages:");
  for (uint8_t i = 0; i < DEVICE_COUNT; i++) {
    Serial.printf("CID %d: %.3f V\r\n", i + 1, stackVoltages[i]);
  }
}

#if defined(BRZ_HV_ECU)
bcc_status_t printInitialSettings_bcc1(bcc_cid_t cid)
{
  uint64_t guid;
  uint16_t regVal;
  uint8_t i;
  static char* printPattern = "  | %-18s | 0x%02X%02X |\r\n";
  bcc_status_t error;

  Serial.printf("###############################################\r\n");
  Serial.printf("# BMS1 - CID %d (MC3377%s): Initial value of registers\r\n", cid,
          (devices[cid - 1] == BCC_DEVICE_MC33771) ?
                  "1" : "2");
  Serial.printf("###############################################\r\n\r\n");

  Serial.printf("  -------------------------------\r\n");
  Serial.printf("  | Register           | Value  |\r\n");
  Serial.printf("  -------------------------------\r\n");

  error = bcc1->read_register(cid, BCC_REG_INIT_ADDR, 1U, &regVal);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("BMS1: Error reading register INIT (CID %d): %d\r\n", cid, error);
    return error;
  }

  Serial.printf(printPattern, "INIT", regVal >> 8, regVal & 0xFFU);

  if (devices[cid - 1] == BCC_DEVICE_MC33771)
  {
    for (i = 0U; i < REG_CONF_CNT_MC33771; i++)
    {
      error = bcc1->read_register(cid, BCC_REGISTERS_DATA_MC33771[i].address, 1U, &regVal);
      if (error != BCC_STATUS_SUCCESS) {
        Serial.printf("BMS1: Error reading register %s (CID %d): %d\r\n",
                      BCC_REGISTERS_DATA_MC33771[i].name, cid, error);
        return error;
      }
      Serial.printf(printPattern, BCC_REGISTERS_DATA_MC33771[i].name, regVal >> 8, regVal & 0xFFU);
    }
  }
  else
  {
    for (i = 0U; i < REG_CONF_CNT_MC33772; i++)
    {
      error = bcc1->read_register(cid, BCC_REGISTERS_DATA_MC33772[i].address, 1U, &regVal);
      if (error != BCC_STATUS_SUCCESS)
      {
        Serial.printf("BMS1: Error reading register %s (CID %d): %d\r\n",
                      BCC_REGISTERS_DATA_MC33772[i].name, cid, error);
        return error;
      }
      Serial.printf(printPattern, BCC_REGISTERS_DATA_MC33772[i].name, regVal >> 8, regVal & 0xFFU);
    }
  }

  Serial.printf("  ----------------------------------\r\n");
  Serial.printf("\r\n");

  Serial.printf("  ------------------------\r\n");
  Serial.printf("  | Fuse Mirror | Value  |\r\n");
  Serial.printf("  |  Register   |        |\r\n");
  Serial.printf("  ------------------------\r\n");
  for (i = 0U; i <= ((devices[cid - 1] == BCC_DEVICE_MC33771) ?
    BCC_LAST_FUSE_ADDR_MC33771B : BCC_LAST_FUSE_ADDR_MC33772B); i++)
  {
    error = bcc1->read_fuse_mirror(cid, i, &regVal);
    if (error != BCC_STATUS_SUCCESS)
      return error;

    Serial.printf("  | $%02X\t| 0x%02X%02X |\r\n", i, regVal >> 8, regVal & 0xFFU);
  }
  Serial.printf("  ------------------------\r\n");
  Serial.printf("\r\n");

  error = bcc1->read_guid(cid, &guid);
  if (error != BCC_STATUS_SUCCESS)
    return error;

  Serial.printf("  Device GUID: %02X%04X%04X\r\n",
          (uint16_t)((guid >> 32) & 0x001FU),
          (uint16_t)((guid >> 16) & 0xFFFFU),
          (uint16_t)(guid & 0xFFFFU));
  Serial.printf("\r\n");

  return BCC_STATUS_SUCCESS;
}

void printStackVoltages_bcc1() {
  float stackVoltages[DEVICE_COUNT];
  uint16_t stackMeasurements[BCC_MEAS_CNT];
  bcc_status_t error;
  bool completed;

  // Start conversion on all devices simultaneously using global write
  error = bcc1->start_conversion_global_async(0x0717);
  if (error != BCC_STATUS_SUCCESS) {
    Serial.printf("BMS1: Error starting global conversion: %d\r\n", error);
    return;
  }

  // Wait for conversion to complete (typical time is 520us)
  delayMicroseconds(600);

  // Check if conversion completed on first device (they all convert simultaneously)
  do {
    error = bcc1->is_converting(BCC_CID_DEV1, &completed);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("BMS1: Error checking conversion status: %d\r\n", error);
      return;
    }
  } while (!completed);

  // Read measurements from each device
  for (uint8_t i = 1; i <= DEVICE_COUNT; i++) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(i);
    error = bcc1->get_raw_values(cid, stackMeasurements);
    if (error != BCC_STATUS_SUCCESS) {
      Serial.printf("BMS1: Error getting measurements for CID %d: %d\r\n", i, error);
      stackVoltages[i - 1] = 0.0f;
      continue;
    }

    uint16_t rawValue = stackMeasurements[BCC_MSR_STACK_VOLT];
    uint32_t voltageMicrovolts = BCC_GET_STACK_VOLT(rawValue);
    stackVoltages[i - 1] = voltageMicrovolts / 1000000.0f;  // Convert uV to V
  }

  // Print results
  Serial.println("BMS1 Stack Voltages:");
  for (uint8_t i = 0; i < DEVICE_COUNT; i++) {
    Serial.printf("BMS1 CID %d: %.3f V\r\n", i + 1, stackVoltages[i]);
  }
}
#endif