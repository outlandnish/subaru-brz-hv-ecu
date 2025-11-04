/*
 * Battery Characterization Tool
 *
 * Reads GUID, configuration registers, and fuse mirror data from up to 8 batteries
 * on each BCC chain (BCC0 and BCC1) and outputs to CSV format.
 *
 * Usage:
 *   1. Build with: pio run -e battery-char
 *   2. Upload with: pio run -e battery-char -t upload
 *   3. Monitor with: pio device monitor
 *   4. Save output to file by redirecting serial output
 */

#include "Arduino.h"
#include "SPI.h"
#include "TPLSPI.h"
#include "BatteryCellController.h"
#include "bcc/bcc_config.h"
#include "hal/dma_config.h"
#include "hal/hv-ecu-v0-pins.h"

#define Serial SerialUSB

// Configuration
#define MAX_DEVICES 8
#define CELL_COUNT 6

// Global objects
TPLSPI *tpl0, *tpl1;
BatteryCellController *bcc0, *bcc1;
SPIClass *bcc0_tx_spi, *bcc0_rx_spi;
SPIClass *bcc1_tx_spi, *bcc1_rx_spi;
bcc_device_t devices_0[MAX_DEVICES];
bcc_device_t devices_1[MAX_DEVICES];

void print_csv_header() {
  Serial.println("\n=== Battery Characterization Data ===");
  Serial.println("CSV Format: BCC,CID,Field,Address,Value,Description");
  Serial.println("---");
}

void dump_device_guid(uint8_t bcc_num, bcc_cid_t cid, BatteryCellController *bcc) {
  uint64_t guid;
  bcc_status_t error = bcc->read_guid(cid, &guid);

  if (error == BCC_STATUS_SUCCESS) {
    Serial.printf("%d,%d,GUID,N/A,0x%02X%04X%04X,Device GUID\r\n",
                 bcc_num, cid,
                 (uint16_t)((guid >> 32) & 0x001FU),
                 (uint16_t)((guid >> 16) & 0xFFFFU),
                 (uint16_t)(guid & 0xFFFFU));
  } else {
    Serial.printf("%d,%d,GUID,N/A,ERROR_%d,Failed to read GUID\r\n",
                 bcc_num, cid, error);
  }
}

void dump_device_init_register(uint8_t bcc_num, bcc_cid_t cid, BatteryCellController *bcc) {
  uint16_t regVal;
  bcc_status_t error = bcc->read_register(cid, BCC_REG_INIT_ADDR, 1U, &regVal);

  if (error == BCC_STATUS_SUCCESS) {
    Serial.printf("%d,%d,INIT,0x%04X,0x%04X,Initialization register\r\n",
                 bcc_num, cid, BCC_REG_INIT_ADDR, regVal);
  } else {
    Serial.printf("%d,%d,INIT,0x%04X,ERROR_%d,Failed to read\r\n",
                 bcc_num, cid, BCC_REG_INIT_ADDR, error);
  }
}

void dump_device_config_registers(uint8_t bcc_num, bcc_cid_t cid, BatteryCellController *bcc, bcc_device_t device_type) {
  uint16_t regVal;

  if (device_type == BCC_DEVICE_MC33771) {
    for (uint8_t i = 0; i < REG_CONF_CNT_MC33771; i++) {
      bcc_status_t error = bcc->read_register(cid, BCC_REGISTERS_DATA_MC33771[i].address, 1U, &regVal);

      if (error == BCC_STATUS_SUCCESS) {
        Serial.printf("%d,%d,%s,0x%04X,0x%04X,Configuration register\r\n",
                     bcc_num, cid,
                     BCC_REGISTERS_DATA_MC33771[i].name,
                     BCC_REGISTERS_DATA_MC33771[i].address,
                     regVal);
      } else {
        Serial.printf("%d,%d,%s,0x%04X,ERROR_%d,Failed to read\r\n",
                     bcc_num, cid,
                     BCC_REGISTERS_DATA_MC33771[i].name,
                     BCC_REGISTERS_DATA_MC33771[i].address,
                     error);
      }
      delay(1); // Small delay between reads
    }
  } else {
    for (uint8_t i = 0; i < REG_CONF_CNT_MC33772; i++) {
      bcc_status_t error = bcc->read_register(cid, BCC_REGISTERS_DATA_MC33772[i].address, 1U, &regVal);

      if (error == BCC_STATUS_SUCCESS) {
        Serial.printf("%d,%d,%s,0x%04X,0x%04X,Configuration register\r\n",
                     bcc_num, cid,
                     BCC_REGISTERS_DATA_MC33772[i].name,
                     BCC_REGISTERS_DATA_MC33772[i].address,
                     regVal);
      } else {
        Serial.printf("%d,%d,%s,0x%04X,ERROR_%d,Failed to read\r\n",
                     bcc_num, cid,
                     BCC_REGISTERS_DATA_MC33772[i].name,
                     BCC_REGISTERS_DATA_MC33772[i].address,
                     error);
      }
      delay(1); // Small delay between reads
    }
  }
}

void dump_device_fuse_mirror(uint8_t bcc_num, bcc_cid_t cid, BatteryCellController *bcc, bcc_device_t device_type) {
  // MC33771C has fuses 0x00-0x17, MC33772C has 0x00-0x1F
  uint8_t max_fuse_addr = (device_type == BCC_DEVICE_MC33771) ? 0x17 : 0x1F;

  for (uint8_t addr = 0x00; addr <= max_fuse_addr; addr++) {
    uint16_t fuseVal;
    bcc_status_t error = bcc->read_fuse_mirror(cid, addr, &fuseVal);

    if (error == BCC_STATUS_SUCCESS) {
      Serial.printf("%d,%d,FUSE_%02X,0x%02X,0x%04X,Fuse mirror data\r\n",
                   bcc_num, cid, addr, addr, fuseVal);
    } else {
      Serial.printf("%d,%d,FUSE_%02X,0x%02X,ERROR_%d,Failed to read\r\n",
                   bcc_num, cid, addr, addr, error);
    }
    delay(1); // Small delay between reads
  }
}

void characterize_device(uint8_t bcc_num, bcc_cid_t cid, BatteryCellController *bcc, bcc_device_t device_type) {
  Serial.printf("\n# Characterizing BCC%d Device %d (MC3377%s)\r\n",
               bcc_num, cid,
               (device_type == BCC_DEVICE_MC33771) ? "1" : "2");

  // Read GUID
  dump_device_guid(bcc_num, cid, bcc);

  // Read INIT register
  dump_device_init_register(bcc_num, cid, bcc);

  // Read all configuration registers
  dump_device_config_registers(bcc_num, cid, bcc, device_type);

  // Read all fuse mirror data
  dump_device_fuse_mirror(bcc_num, cid, bcc, device_type);
}

bool initialize_bcc0() {
  Serial.println("\n=== Initializing BCC0 ===");

  // Setup device types (assuming MC33772C for 6-cell batteries)
  for (uint8_t i = 0; i < MAX_DEVICES; i++) {
    devices_0[i] = BCC_DEVICE_MC33772;
  }

  // Initialize SPI
  bcc0_tx_spi = new SPIClass(BCC0_TX_DATA, NC, BCC0_TX_SCK, NC);
  bcc0_rx_spi = new SPIClass(BCC0_RX_DATA, NC, BCC0_RX_SCK, BCC0_RX_CS);

  // Initialize TPL and BCC
  tpl0 = new TPLSPI(bcc0_tx_spi, bcc0_rx_spi, BCC0_TX_CS, configureDMA_HV_ECU);
  bcc0 = new BatteryCellController(tpl0, devices_0, MAX_DEVICES, CELL_COUNT,
                                   BCC0_ENABLE, BCC0_INTB, false);

  pinMode(BCC0_TX_CS, OUTPUT);
  digitalWrite(BCC0_TX_CS, HIGH);

  bcc_status_t error = bcc0->begin(nullptr);

  if (error == BCC_STATUS_SUCCESS) {
    Serial.println("BCC0: Initialization successful");
    return true;
  } else {
    Serial.printf("BCC0: Initialization failed with error %d\r\n", error);
    return false;
  }
}

bool initialize_bcc1() {
  Serial.println("\n=== Initializing BCC1 ===");

  // Setup device types (assuming MC33772C for 6-cell batteries)
  for (uint8_t i = 0; i < MAX_DEVICES; i++) {
    devices_1[i] = BCC_DEVICE_MC33772;
  }

  // Initialize SPI
  bcc1_tx_spi = new SPIClass(BCC1_TX_DATA, NC, BCC1_TX_SCK, NC);
  bcc1_rx_spi = new SPIClass(BCC1_RX_DATA, NC, BCC1_RX_SCK, BCC1_RX_CS);

  // Initialize TPL and BCC
  tpl1 = new TPLSPI(bcc1_tx_spi, bcc1_rx_spi, BCC1_TX_CS);
  bcc1 = new BatteryCellController(tpl1, devices_1, MAX_DEVICES, CELL_COUNT,
                                   BCC1_ENABLE, BCC1_INTB, false);

  pinMode(BCC1_TX_CS, OUTPUT);
  digitalWrite(BCC1_TX_CS, HIGH);

  bcc_status_t error = bcc1->begin(nullptr);

  if (error == BCC_STATUS_SUCCESS) {
    Serial.println("BCC1: Initialization successful");
    return true;
  } else {
    Serial.printf("BCC1: Initialization failed with error %d\r\n", error);
    return false;
  }
}

uint8_t detect_devices(BatteryCellController *bcc, uint8_t bcc_num) {
  Serial.printf("\nDetecting devices on BCC%d...\r\n", bcc_num);
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_DEVICES; i++) {
    bcc_cid_t cid = static_cast<bcc_cid_t>(i + 1);
    uint64_t guid;
    bcc_status_t error = bcc->read_guid(cid, &guid);

    if (error == BCC_STATUS_SUCCESS) {
      Serial.printf("  Device found at CID %d (GUID: 0x%02X%04X%04X)\r\n",
                   cid,
                   (uint16_t)((guid >> 32) & 0x001FU),
                   (uint16_t)((guid >> 16) & 0xFFFFU),
                   (uint16_t)(guid & 0xFFFFU));
      count++;
    } else {
      Serial.printf("  No device at CID %d\r\n", cid);
    }
    delay(10);
  }

  Serial.printf("Total devices detected: %d\r\n", count);
  return count;
}

void setup() {
  delay(2000);

  // Initialize Serial
  Serial.begin(115200);
  Serial.println("\n\n========================================");
  Serial.println("   Battery Characterization Tool");
  Serial.println("========================================");
  Serial.println("This tool reads GUID, registers, and fuse");
  Serial.println("mirror data from up to 8 batteries per BCC");
  Serial.println("chain and outputs to CSV format.");
  Serial.println("========================================\n");

  delay(1000);

  // Initialize BCC0
  bool bcc0_ok = initialize_bcc0();
  uint8_t bcc0_device_count = 0;
  if (bcc0_ok) {
    bcc0_device_count = detect_devices(bcc0, 0);
  }

  delay(500);

  // Initialize BCC1
  bool bcc1_ok = initialize_bcc1();
  uint8_t bcc1_device_count = 0;
  if (bcc1_ok) {
    bcc1_device_count = detect_devices(bcc1, 1);
  }

  delay(1000);

  // Print CSV header
  print_csv_header();

  // Characterize BCC0 devices
  if (bcc0_ok && bcc0_device_count > 0) {
    Serial.println("\n========================================");
    Serial.println("Characterizing BCC0 Devices");
    Serial.println("========================================");

    for (uint8_t i = 0; i < bcc0_device_count; i++) {
      bcc_cid_t cid = static_cast<bcc_cid_t>(i + 1);
      characterize_device(0, cid, bcc0, devices_0[i]);
      delay(100);
    }
  }

  // Characterize BCC1 devices
  if (bcc1_ok && bcc1_device_count > 0) {
    Serial.println("\n========================================");
    Serial.println("Characterizing BCC1 Devices");
    Serial.println("========================================");

    for (uint8_t i = 0; i < bcc1_device_count; i++) {
      bcc_cid_t cid = static_cast<bcc_cid_t>(i + 1);
      characterize_device(1, cid, bcc1, devices_1[i]);
      delay(100);
    }
  }

  // Done
  Serial.println("\n========================================");
  Serial.println("Characterization Complete!");
  Serial.println("========================================");
  Serial.println("\nTo save this data:");
  Serial.println("1. Copy the CSV data above");
  Serial.println("2. Paste into a text file");
  Serial.println("3. Save as .csv file");
  Serial.println("\nPress reset button to run again.");
}

void loop() {
  // Nothing to do in loop
  delay(1000);
}
