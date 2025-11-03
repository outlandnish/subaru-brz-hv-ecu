#pragma once

#include "Arduino.h"
#include "SPI.h"

// Generic DMA configuration function for TPLSPI
// Automatically detects which SPI peripherals are being used and configures
// the appropriate DMA streams, channels, and interrupt handlers
//
// Supports SPI1-SPI5 with zero-overhead interrupt handlers
bool configureDMA_HV_ECU(SPIClass *spi_tx, SPIClass *spi_rx);
