#include "can.h"

// Define static members
CAN_HandleTypeDef CANBus::hcan1 = {0};
CAN_HandleTypeDef CANBus::hcan2 = {0};
CAN_HandleTypeDef CANBus::hcan3 = {0};

CANRxCallback CANBus::callback1 = nullptr;
CANRxCallback CANBus::callback2 = nullptr;
CANRxCallback CANBus::callback3 = nullptr;

CANBus* CANBus::instance1 = nullptr;
CANBus* CANBus::instance2 = nullptr;
CANBus* CANBus::instance3 = nullptr;

CANBus::CANBus(uint32_t rx_pin, uint32_t tx_pin, int8_t term_pin)
  : CAN_COMMON(14),  // STM32 CAN has 14 filter banks
    rx_pin(rx_pin), tx_pin(tx_pin), term_pin(term_pin),
    can_instance(nullptr), hcan(nullptr) {
}

bool CANBus::initGPIO() {
  // Convert Arduino pin numbers to STM32 pin names
  PinName rx_pinname = digitalPinToPinName(rx_pin);
  PinName tx_pinname = digitalPinToPinName(tx_pin);

  if (rx_pinname == NC || tx_pinname == NC) {
    return false;
  }

  // Determine which CAN peripheral these pins are connected to
  CAN_TypeDef *can_rx = (CAN_TypeDef *)pinmap_peripheral(rx_pinname, PinMap_CAN_RD);
  CAN_TypeDef *can_tx = (CAN_TypeDef *)pinmap_peripheral(tx_pinname, PinMap_CAN_TD);

  // Verify both pins belong to the same CAN instance
  can_instance = (CAN_TypeDef *)pinmap_merge_peripheral(can_rx, can_tx);

  if (can_instance == NP) {
    return false; // Pins don't belong to same CAN instance
  }

  // Enable CAN clock based on which instance we're using
  if (can_instance == CAN1) {
    __HAL_RCC_CAN1_CLK_ENABLE();
    hcan = &hcan1;
    instance1 = this;  // Store instance pointer for interrupt handling
  }
#ifdef CAN2
  else if (can_instance == CAN2) {
    __HAL_RCC_CAN1_CLK_ENABLE();  // CAN2 needs CAN1 clock
    __HAL_RCC_CAN2_CLK_ENABLE();
    hcan = &hcan2;
    instance2 = this;  // Store instance pointer for interrupt handling
  }
#endif
#ifdef CAN3
  else if (can_instance == CAN3) {
    __HAL_RCC_CAN3_CLK_ENABLE();
    hcan = &hcan3;
    instance3 = this;  // Store instance pointer for interrupt handling
  }
#endif
  else {
    return false;
  }

  // Configure RX and TX pins using pinmap
  pinmap_pinout(rx_pinname, PinMap_CAN_RD);
  pinmap_pinout(tx_pinname, PinMap_CAN_TD);

  // Configure termination pin if provided
  if (term_pin >= 0) {
    pinMode(term_pin, OUTPUT);
    digitalWrite(term_pin, LOW);
  }

  return true;
}

bool CANBus::begin(uint32_t baudrate) {
  // Initialize GPIO pins first
  if (!initGPIO()) {
    return false;
  }

  // Configure CAN peripheral
  hcan->Instance = can_instance;

  // Configure CAN parameters
  // Assuming 45MHz CAN clock for STM32F4
  uint32_t prescaler;
  uint32_t bs1, bs2;

  switch(baudrate) {
    case CAN_BPS_125K:  prescaler = 18; bs1 = CAN_BS1_13TQ; bs2 = CAN_BS2_2TQ; break;
    case CAN_BPS_250K:  prescaler = 9;  bs1 = CAN_BS1_13TQ; bs2 = CAN_BS2_2TQ; break;
    case CAN_BPS_500K:  prescaler = 6;  bs1 = CAN_BS1_11TQ; bs2 = CAN_BS2_3TQ; break;
    case CAN_BPS_1000K: prescaler = 3;  bs1 = CAN_BS1_11TQ; bs2 = CAN_BS2_3TQ; break;
    default: return false;
  }

  hcan->Init.Prescaler = prescaler;
  hcan->Init.Mode = CAN_MODE_NORMAL;
  hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan->Init.TimeSeg1 = bs1;
  hcan->Init.TimeSeg2 = bs2;
  hcan->Init.TimeTriggeredMode = DISABLE;
  hcan->Init.AutoBusOff = ENABLE;
  hcan->Init.AutoWakeUp = DISABLE;
  hcan->Init.AutoRetransmission = ENABLE;
  hcan->Init.ReceiveFifoLocked = DISABLE;
  hcan->Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(hcan) != HAL_OK) {
    return false;
  }

  // Configure default filter to accept all messages
  CAN_FilterTypeDef filter = {0};
  uint8_t filter_bank = getFilterBank();

  filter.FilterBank = filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK) {
    return false;
  }

  if (HAL_CAN_Start(hcan) != HAL_OK) {
    return false;
  }

  return true;
}

void CANBus::end() {
  if (hcan) {
    disableRxInterrupt();
    HAL_CAN_Stop(hcan);
    HAL_CAN_DeInit(hcan);
  }

  // Disable termination resistor
  if (term_pin >= 0) {
    digitalWrite(term_pin, LOW);
  }
}

bool CANBus::sendMessage(uint32_t id, uint8_t* data, uint8_t len) {
  if (!hcan) return false;

  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;

  txHeader.StdId = id;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = len;
  txHeader.TransmitGlobalTime = DISABLE;

  return (HAL_CAN_AddTxMessage(hcan, &txHeader, data, &txMailbox) == HAL_OK);
}

// CAN_COMMON interface implementation
bool CANBus::sendFrame(CAN_FRAME& txFrame) {
  if (!hcan) return false;

  CAN_TxHeaderTypeDef txHeader;
  uint32_t txMailbox;

  txHeader.StdId = txFrame.id;
  txHeader.ExtId = txFrame.id;
  txHeader.IDE = txFrame.extended ? CAN_ID_EXT : CAN_ID_STD;
  txHeader.RTR = txFrame.rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
  txHeader.DLC = txFrame.length;
  txHeader.TransmitGlobalTime = DISABLE;

  return (HAL_CAN_AddTxMessage(hcan, &txHeader, txFrame.data.uint8, &txMailbox) == HAL_OK);
}

bool CANBus::rx_avail() {
  if (!hcan) return false;
  return (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0);
}

uint16_t CANBus::available() {
  if (!hcan) return 0;
  return HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0);
}

uint32_t CANBus::get_rx_buff(CAN_FRAME &msg) {
  if (!hcan) return 0;

  CAN_RxHeaderTypeDef rxHeader;

  if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0) {
    return 0;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, msg.data.uint8) != HAL_OK) {
    return 0;
  }

  msg.id = rxHeader.IDE == CAN_ID_EXT ? rxHeader.ExtId : rxHeader.StdId;
  msg.extended = (rxHeader.IDE == CAN_ID_EXT) ? 1 : 0;
  msg.rtr = (rxHeader.RTR == CAN_RTR_REMOTE) ? 1 : 0;
  msg.length = rxHeader.DLC;
  msg.timestamp = millis();

  return 1;
}

uint32_t CANBus::init(uint32_t ul_baudrate) {
  return begin(ul_baudrate) ? ul_baudrate : 0;
}

uint32_t CANBus::beginAutoSpeed() {
  // Try common baudrates
  uint32_t rates[] = {CAN_BPS_500K, CAN_BPS_250K, CAN_BPS_1000K, CAN_BPS_125K};
  for (uint8_t i = 0; i < 4; i++) {
    if (begin(rates[i])) {
      busSpeed = rates[i];
      return rates[i];
    }
  }
  return 0;
}

uint32_t CANBus::set_baudrate(uint32_t ul_baudrate) {
  // Would need to stop and reconfigure - for now just return current
  return busSpeed;
}

void CANBus::setListenOnlyMode(bool state) {
  // Not implemented - would require reconfiguration
}

void CANBus::enable() {
  if (hcan) {
    HAL_CAN_Start(hcan);
  }
}

void CANBus::disable() {
  if (hcan) {
    HAL_CAN_Stop(hcan);
  }
}

int CANBus::_setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended) {
  return setFilter(id, mask, mailbox) ? 0 : -1;
}

int CANBus::_setFilter(uint32_t id, uint32_t mask, bool extended) {
  return setFilter(id, mask, 0) ? 0 : -1;
}

bool CANBus::receiveMessage(uint32_t &id, uint8_t* data, uint8_t &len) {
  if (!hcan) return false;

  CAN_RxHeaderTypeDef rxHeader;

  if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0) {
    return false;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK) {
    return false;
  }

  id = rxHeader.StdId;
  len = rxHeader.DLC;

  return true;
}

void CANBus::setTermination(bool enabled) {
  if (term_pin >= 0) {
    digitalWrite(term_pin, enabled ? HIGH : LOW);
  }
}

bool CANBus::setFilter(uint32_t filter_id, uint32_t filter_mask, uint32_t filter_bank) {
  if (!hcan) return false;

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = getFilterBank() + filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (filter_id << 5) >> 16;
  filter.FilterIdLow = (filter_id << 5) & 0xFFFF;
  filter.FilterMaskIdHigh = (filter_mask << 5) >> 16;
  filter.FilterMaskIdLow = (filter_mask << 5) & 0xFFFF;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  return (HAL_CAN_ConfigFilter(hcan, &filter) == HAL_OK);
}

bool CANBus::setFilterRange(uint32_t id_low, uint32_t id_high, uint32_t filter_bank) {
  if (!hcan) return false;

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = getFilterBank() + filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (id_low << 5) >> 16;
  filter.FilterIdLow = (id_low << 5) & 0xFFFF;
  filter.FilterMaskIdHigh = (id_high << 5) >> 16;
  filter.FilterMaskIdLow = (id_high << 5) & 0xFFFF;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  return (HAL_CAN_ConfigFilter(hcan, &filter) == HAL_OK);
}

bool CANBus::disableFilter(uint32_t filter_bank) {
  if (!hcan) return false;

  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = getFilterBank() + filter_bank;
  filter.FilterActivation = DISABLE;
  filter.SlaveStartFilterBank = 14;

  return (HAL_CAN_ConfigFilter(hcan, &filter) == HAL_OK);
}

bool CANBus::enableRxInterrupt(CANRxCallback callback) {
  if (!hcan || !callback) return false;

  // Store callback for this CAN instance
  if (hcan->Instance == CAN1) {
    callback1 = callback;
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
#ifdef CAN2
  else if (hcan->Instance == CAN2) {
    callback2 = callback;
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  }
#endif
#ifdef CAN3
  else if (hcan->Instance == CAN3) {
    callback3 = callback;
    HAL_NVIC_SetPriority(CAN3_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN3_RX0_IRQn);
  }
#endif
  else {
    return false;
  }

  // Enable FIFO 0 message pending interrupt
  return (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) == HAL_OK);
}

void CANBus::disableRxInterrupt() {
  if (!hcan) return;

  HAL_CAN_DeactivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  if (hcan->Instance == CAN1) {
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    callback1 = nullptr;
  }
#ifdef CAN2
  else if (hcan->Instance == CAN2) {
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    callback2 = nullptr;
  }
#endif
#ifdef CAN3
  else if (hcan->Instance == CAN3) {
    HAL_NVIC_DisableIRQ(CAN3_RX0_IRQn);
    callback3 = nullptr;
  }
#endif
}

void CANBus::handleRxInterrupt(CAN_HandleTypeDef* hcan) {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t data[8];

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) == HAL_OK) {
    // Get the CANBus instance and legacy callback for this CAN peripheral
    CANBus *instance = nullptr;
    CANRxCallback callback = nullptr;

    if (hcan->Instance == CAN1) {
      instance = instance1;
      callback = callback1;
    }
#ifdef CAN2
    else if (hcan->Instance == CAN2) {
      instance = instance2;
      callback = callback2;
    }
#endif
#ifdef CAN3
    else if (hcan->Instance == CAN3) {
      instance = instance3;
      callback = callback3;
    }
#endif

    // Call legacy callback if set
    if (callback) {
      uint32_t id = rxHeader.IDE == CAN_ID_EXT ? rxHeader.ExtId : rxHeader.StdId;
      callback(id, data, rxHeader.DLC);
    }

    // Process can_common callbacks if instance exists
    if (instance) {
      CAN_FRAME frame;
      frame.id = rxHeader.IDE == CAN_ID_EXT ? rxHeader.ExtId : rxHeader.StdId;
      frame.extended = (rxHeader.IDE == CAN_ID_EXT) ? 1 : 0;
      frame.rtr = (rxHeader.RTR == CAN_RTR_REMOTE) ? 1 : 0;
      frame.length = rxHeader.DLC;
      frame.timestamp = millis();
      memcpy(frame.data.uint8, data, rxHeader.DLC);

      // Notify attached listeners via can_common
      for (int i = 0; i < SIZE_LISTENERS; i++) {
        if (instance->listener[i] != nullptr) {
          instance->listener[i]->gotFrame(&frame, 0);
        }
      }

      // Call general callback if set
      if (instance->cbGeneral) {
        instance->cbGeneral(&frame);
      }
    }
  }
}

CAN_HandleTypeDef* CANBus::getCAN() {
  return hcan;
}

uint8_t CANBus::getFilterBank() {
  if (!can_instance) return 0;

  if (can_instance == CAN1) {
    return 0;
  }
#ifdef CAN2
  else if (can_instance == CAN2) {
    return 14;
  }
#endif
#ifdef CAN3
  else if (can_instance == CAN3) {
    return 21;
  }
#endif
  return 0;
}

// Interrupt handlers - these need to be defined outside the class
extern "C" {
  void CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&CANBus::hcan1);
  }

#ifdef CAN2
  void CAN2_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&CANBus::hcan2);
  }
#endif

#ifdef CAN3
  void CAN3_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&CANBus::hcan3);
  }
#endif

  // HAL callback
  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CANBus::handleRxInterrupt(hcan);
  }
}
