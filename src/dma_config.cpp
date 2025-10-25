#include "dma_config.h"

#if defined(BRZ_HV_ECU)

#define Serial SerialUSB

// Per-SPI instance DMA handles (separate for each SPI peripheral)
static DMA_HandleTypeDef hdma_spi1_tx;
static DMA_HandleTypeDef hdma_spi1_rx;
static DMA_HandleTypeDef hdma_spi2_tx;
static DMA_HandleTypeDef hdma_spi2_rx;
static DMA_HandleTypeDef hdma_spi3_tx;
static DMA_HandleTypeDef hdma_spi3_rx;
static DMA_HandleTypeDef hdma_spi4_tx;
static DMA_HandleTypeDef hdma_spi4_rx;
static DMA_HandleTypeDef hdma_spi5_tx;
static DMA_HandleTypeDef hdma_spi5_rx;

// Per-SPI instance handles for zero-overhead IRQ handlers
static SPI_HandleTypeDef *hspi_spi1 = nullptr;
static SPI_HandleTypeDef *hspi_spi2 = nullptr;
static SPI_HandleTypeDef *hspi_spi3 = nullptr;
static SPI_HandleTypeDef *hspi_spi4 = nullptr;
static SPI_HandleTypeDef *hspi_spi5 = nullptr;

// Generic DMA configuration based on SPI handles
bool configureDMA_HV_ECU(SPIClass *spi_tx, SPIClass *spi_rx) {
  SPI_TypeDef *tx_peripheral = spi_tx->getHandle()->Instance;
  SPI_TypeDef *rx_peripheral = spi_rx->getHandle()->Instance;

  // Store handles in per-instance pointers for zero-overhead IRQ handlers
  if (tx_peripheral == SPI1 || rx_peripheral == SPI1) hspi_spi1 = (tx_peripheral == SPI1) ? spi_tx->getHandle() : spi_rx->getHandle();
  if (tx_peripheral == SPI2 || rx_peripheral == SPI2) hspi_spi2 = (tx_peripheral == SPI2) ? spi_tx->getHandle() : spi_rx->getHandle();
  if (tx_peripheral == SPI3 || rx_peripheral == SPI3) hspi_spi3 = (tx_peripheral == SPI3) ? spi_tx->getHandle() : spi_rx->getHandle();
  if (tx_peripheral == SPI4 || rx_peripheral == SPI4) hspi_spi4 = (tx_peripheral == SPI4) ? spi_tx->getHandle() : spi_rx->getHandle();
  if (tx_peripheral == SPI5 || rx_peripheral == SPI5) hspi_spi5 = (tx_peripheral == SPI5) ? spi_tx->getHandle() : spi_rx->getHandle();

  // Configure TX DMA based on which SPI instance
  Serial.printf("Configuring DMA for SPI TX (Instance: 0x%08X)\n", (uint32_t)tx_peripheral);

  DMA_HandleTypeDef *hdma_tx = nullptr;

  if (tx_peripheral == SPI1) {
    // SPI1_TX: DMA2 Stream 3 Channel 3 (or Stream 5 Channel 3)
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_tx = &hdma_spi1_tx;
    hdma_tx->Instance = DMA2_Stream3;
    hdma_tx->Init.Channel = DMA_CHANNEL_3;

    // Enable interrupts
    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  } else if (tx_peripheral == SPI2) {
    // SPI2_TX: DMA1 Stream 4 Channel 0
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_tx = &hdma_spi2_tx;
    hdma_tx->Instance = DMA1_Stream4;
    hdma_tx->Init.Channel = DMA_CHANNEL_0;

    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  } else if (tx_peripheral == SPI3) {
    // SPI3_TX: DMA1 Stream 5 Channel 0 (or Stream 7 Channel 0)
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_tx = &hdma_spi3_tx;
    hdma_tx->Instance = DMA1_Stream5;
    hdma_tx->Init.Channel = DMA_CHANNEL_0;

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_SetPriority(SPI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  } else if (tx_peripheral == SPI4) {
    // SPI4_TX: DMA2 Stream 1 Channel 4 (or Stream 4 Channel 5)
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_tx = &hdma_spi4_tx;
    hdma_tx->Instance = DMA2_Stream1;
    hdma_tx->Init.Channel = DMA_CHANNEL_4;

    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    HAL_NVIC_SetPriority(SPI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI4_IRQn);
  } else if (tx_peripheral == SPI5) {
    // SPI5_TX: DMA2 Stream 6 Channel 7 (or Stream 4 Channel 2)
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_tx = &hdma_spi5_tx;
    hdma_tx->Instance = DMA2_Stream6;
    hdma_tx->Init.Channel = DMA_CHANNEL_7;

    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    HAL_NVIC_SetPriority(SPI5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI5_IRQn);
  } else {
    Serial.printf("Unsupported TX SPI instance: 0x%08X\n", (uint32_t)tx_peripheral);
    return false;
  }

  // Common TX DMA settings
  hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
  hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_tx->Init.Mode = DMA_NORMAL;
  hdma_tx->Init.Priority = DMA_PRIORITY_HIGH;
  hdma_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  hdma_tx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_tx->Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_tx->Init.PeriphBurst = DMA_PBURST_SINGLE;

  if (HAL_DMA_Init(hdma_tx) != HAL_OK) {
    Serial.println("Failed to initialize TX DMA");
    return false;
  }

  __HAL_LINKDMA(spi_tx->getHandle(), hdmatx, *hdma_tx);
  SET_BIT(spi_tx->getHandle()->Instance->CR2, SPI_CR2_TXDMAEN);

  // Configure RX DMA based on which SPI instance
  Serial.printf("Configuring DMA for SPI RX (Instance: 0x%08X)\n", (uint32_t)rx_peripheral);

  DMA_HandleTypeDef *hdma_rx = nullptr;

  if (rx_peripheral == SPI1) {
    // SPI1_RX: DMA2 Stream 0 Channel 3 (or Stream 2 Channel 3)
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_rx = &hdma_spi1_rx;
    hdma_rx->Instance = DMA2_Stream0;
    hdma_rx->Init.Channel = DMA_CHANNEL_3;

    HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  } else if (rx_peripheral == SPI2) {
    // SPI2_RX: DMA1 Stream 3 Channel 0
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_rx = &hdma_spi2_rx;
    hdma_rx->Instance = DMA1_Stream3;
    hdma_rx->Init.Channel = DMA_CHANNEL_0;

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);
  } else if (rx_peripheral == SPI3) {
    // SPI3_RX: DMA1 Stream 0 Channel 0 (or Stream 2 Channel 0)
    __HAL_RCC_DMA1_CLK_ENABLE();
    hdma_rx = &hdma_spi3_rx;
    hdma_rx->Instance = DMA1_Stream0;
    hdma_rx->Init.Channel = DMA_CHANNEL_0;

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(SPI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI3_IRQn);
  } else if (rx_peripheral == SPI4) {
    // SPI4_RX: DMA2 Stream 4 Channel 5 (alternate to avoid conflicts)
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_rx = &hdma_spi4_rx;
    hdma_rx->Instance = DMA2_Stream4;
    hdma_rx->Init.Channel = DMA_CHANNEL_5;

    HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    HAL_NVIC_SetPriority(SPI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI4_IRQn);
  } else if (rx_peripheral == SPI5) {
    // SPI5_RX: DMA2 Stream 5 Channel 7 (alternate to avoid conflicts)
    __HAL_RCC_DMA2_CLK_ENABLE();
    hdma_rx = &hdma_spi5_rx;
    hdma_rx->Instance = DMA2_Stream5;
    hdma_rx->Init.Channel = DMA_CHANNEL_7;

    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
    HAL_NVIC_SetPriority(SPI5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI5_IRQn);
  } else {
    Serial.printf("Unsupported RX SPI instance: 0x%08X\n", (uint32_t)rx_peripheral);
    return false;
  }

  // Common RX DMA settings
  hdma_rx->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_rx->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_rx->Init.MemInc = DMA_MINC_ENABLE;
  hdma_rx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_rx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_rx->Init.Mode = DMA_NORMAL;
  hdma_rx->Init.Priority = DMA_PRIORITY_VERY_HIGH;  // VERY_HIGH for time-critical SPI RX
  hdma_rx->Init.FIFOMode = DMA_FIFOMODE_ENABLE;     // Enable FIFO to buffer incoming data
  hdma_rx->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;  // Trigger after 1 byte
  hdma_rx->Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_rx->Init.PeriphBurst = DMA_PBURST_SINGLE;

  if (HAL_DMA_Init(hdma_rx) != HAL_OK) {
    Serial.println("Failed to initialize RX DMA");
    return false;
  }

  __HAL_LINKDMA(spi_rx->getHandle(), hdmarx, *hdma_rx);

  // Clear any stale error flags and flush RX buffer (important when reprogramming without reset)
  SPI_HandleTypeDef *hspi_rx = spi_rx->getHandle();
  __HAL_SPI_CLEAR_OVRFLAG(hspi_rx);           // Clear overrun flag
  __HAL_SPI_CLEAR_MODFFLAG(hspi_rx);          // Clear mode fault flag
  __HAL_SPI_CLEAR_FREFLAG(hspi_rx);           // Clear frame error flag

  // Flush any stale data in RX buffer by reading DR until RXNE is clear
  volatile uint32_t dummy;
  while (__HAL_SPI_GET_FLAG(hspi_rx, SPI_FLAG_RXNE)) {
    dummy = hspi_rx->Instance->DR;
  }
  (void)dummy;  // Prevent unused variable warning

  // Enable RX DMA requests
  SET_BIT(spi_rx->getHandle()->Instance->CR2, SPI_CR2_RXDMAEN|SPI_CR2_SSOE|SPI_CR2_FRF|SPI_CR2_ERRIE);

  return true;
}

// DMA Interrupt Handlers - these are called based on which streams are configured
// SPI1 TX uses DMA2 Stream 3
extern "C" void DMA2_Stream3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

// SPI1 RX uses DMA2 Stream 0
extern "C" void DMA2_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

// SPI2 TX uses DMA1 Stream 4
extern "C" void DMA1_Stream4_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi2_tx);
}

// SPI2 RX uses DMA1 Stream 3
extern "C" void DMA1_Stream3_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi2_rx);
}

// SPI3 TX uses DMA1 Stream 5
extern "C" void DMA1_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
}

// SPI3 RX uses DMA1 Stream 0
extern "C" void DMA1_Stream0_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
}

// SPI4 TX uses DMA2 Stream 1
extern "C" void DMA2_Stream1_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi4_tx);
}

// SPI4 RX uses DMA2 Stream 4
extern "C" void DMA2_Stream4_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi4_rx);
}

// SPI5 TX uses DMA2 Stream 6
extern "C" void DMA2_Stream6_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi5_tx);
}

// SPI5 RX uses DMA2 Stream 5
extern "C" void DMA2_Stream5_IRQHandler(void) {
  HAL_DMA_IRQHandler(&hdma_spi5_rx);
}

// Zero-overhead SPI Interrupt Handlers - absolute minimum latency
// No branching, no null checks - interrupts only enabled after configuration
// Each handler is a single function call (typically 2-4 CPU cycles)

extern "C" void SPI1_IRQHandler(void) {
  HAL_SPI_IRQHandler(hspi_spi1);
}

extern "C" void SPI2_IRQHandler(void) {
  HAL_SPI_IRQHandler(hspi_spi2);
}

extern "C" void SPI3_IRQHandler(void) {
  HAL_SPI_IRQHandler(hspi_spi3);
}

extern "C" void SPI4_IRQHandler(void) {
  HAL_SPI_IRQHandler(hspi_spi4);
}

extern "C" void SPI5_IRQHandler(void) {
  HAL_SPI_IRQHandler(hspi_spi5);
}

// Helper function to log DMA stream status
static void logDMAStatus(DMA_Stream_TypeDef *dma_stream, const char *label) {
  if (dma_stream == nullptr) return;

  // Get the DMA controller (DMA1 or DMA2)
  DMA_TypeDef *dma = (dma_stream < DMA2_Stream0) ? DMA1 : DMA2;
  uint32_t stream_num = ((uint32_t)dma_stream - (uint32_t)dma) / sizeof(DMA_Stream_TypeDef);

  // Calculate ISR register offset based on stream number
  volatile uint32_t *isr_reg = (stream_num < 4) ? &dma->LISR : &dma->HISR;
  uint32_t isr_shift = (stream_num % 4) * 6 + ((stream_num % 4) > 1 ? 10 : 0);

  uint32_t flags = (*isr_reg >> isr_shift) & 0x3D;

  Serial.printf("%s DMA: CR=0x%08lX, NDTR=%lu, Flags=0x%02lX ", label,
                dma_stream->CR, dma_stream->NDTR, flags);
  if (flags & 0x01) Serial.print("FEIF ");
  if (flags & 0x04) Serial.print("DMEIF ");
  if (flags & 0x08) Serial.print("TEIF ");
  if (flags & 0x10) Serial.print("HTIF ");
  if (flags & 0x20) Serial.print("TCIF ");
  Serial.println();
}

// HAL SPI callbacks for interrupt-driven transfers
extern "C" {
  // Called when SPI TX transmission completes
  void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Serial.println("TX Complete callback fired!");
    // if (hspi->hdmatx != nullptr) {
    //   logDMAStatus(hspi->hdmatx->Instance, "TX");
    // }
    // HAL automatically transitions state to READY
  }

  // Called when SPI RX reception completes
  void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    // Serial.println("RX Complete callback fired!");
    // if (hspi->hdmarx != nullptr) {
    //   logDMAStatus(hspi->hdmarx->Instance, "RX");
    // }
    // HAL automatically transitions state to READY
  }

  // Called if there's an SPI error during interrupt-driven transfer
  void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
    Serial.printf("SPI Error callback! ErrorCode: 0x%08lX ", hspi->ErrorCode);
    if (hspi->ErrorCode & HAL_SPI_ERROR_OVR) Serial.print("OVR ");
    if (hspi->ErrorCode & HAL_SPI_ERROR_MODF) Serial.print("MODF ");
    if (hspi->ErrorCode & HAL_SPI_ERROR_CRC) Serial.print("CRC ");
    if (hspi->ErrorCode & HAL_SPI_ERROR_FRE) Serial.print("FRE ");
    if (hspi->ErrorCode & HAL_SPI_ERROR_DMA) Serial.print("DMA ");
    Serial.printf("(TxXferCount: %d, RxXferCount: %d)\n", hspi->TxXferCount, hspi->RxXferCount);

    // Log SPI state and flags
    Serial.printf("  SPI State: %d, SR: 0x%08lX, CR1: 0x%08lX, CR2: 0x%08lX\n",
                  hspi->State, hspi->Instance->SR, hspi->Instance->CR1, hspi->Instance->CR2);

    // Log DMA state if available
    if (hspi->hdmarx != nullptr) {
      Serial.printf("  RX DMA State: %d, NDTR: %lu\n",
                    hspi->hdmarx->State, hspi->hdmarx->Instance->NDTR);
    }
    if (hspi->hdmatx != nullptr) {
      Serial.printf("  TX DMA State: %d, NDTR: %lu\n",
                    hspi->hdmatx->State, hspi->hdmatx->Instance->NDTR);
    }

    // Clear the error and continue - we may have gotten some data
    hspi->ErrorCode = HAL_SPI_ERROR_NONE;
  }
}

#endif
