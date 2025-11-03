#pragma once
#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "PeripheralPins.h"
#include "pinmap.h"
#include "Arduino.h"
#include "can_common.h"

// Callback function type for RX interrupts (legacy)
typedef void (*CANRxCallback)(uint32_t id, uint8_t* data, uint8_t len);

class CANBus : public CAN_COMMON {
  static CANRxCallback         callback1;
  static CANRxCallback         callback2;
  static CANRxCallback         callback3;

  // Static instance pointers for interrupt handling
  static CANBus*               instance1;
  static CANBus*               instance2;
  static CANBus*               instance3;

  public:
    // Constructor now takes Arduino pin numbers for RX, TX, and optional termination control
    CANBus(uint32_t rx_pin, uint32_t tx_pin, int8_t term_pin = -1);

    // CAN_COMMON pure virtual method implementations
    int _setFilterSpecific(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended) override;
    int _setFilter(uint32_t id, uint32_t mask, bool extended) override;
    uint32_t init(uint32_t ul_baudrate) override;
    uint32_t beginAutoSpeed() override;
    uint32_t set_baudrate(uint32_t ul_baudrate) override;
    void setListenOnlyMode(bool state) override;
    void enable() override;
    void disable() override;
    bool sendFrame(CAN_FRAME& txFrame) override;
    bool rx_avail() override;
    uint16_t available() override;
    uint32_t get_rx_buff(CAN_FRAME &msg) override;

    // Legacy API for backward compatibility
    bool begin(uint32_t baudrate);
    void end();
    bool sendMessage(uint32_t id, uint8_t* data, uint8_t len);
    bool receiveMessage(uint32_t &id, uint8_t* data, uint8_t &len);
    void setTermination(bool enabled);

    // Filter configuration (legacy)
    bool setFilter(uint32_t filter_id, uint32_t filter_mask, uint32_t filter_bank = 0);
    bool setFilterRange(uint32_t id_low, uint32_t id_high, uint32_t filter_bank = 0);
    bool disableFilter(uint32_t filter_bank = 0);

    // Interrupt configuration (legacy)
    bool enableRxInterrupt(CANRxCallback callback);
    void disableRxInterrupt();

    // Static interrupt handlers
    static void handleRxInterrupt(CAN_HandleTypeDef* hcan);

    // Static CAN handles
    static CAN_HandleTypeDef     hcan1;
    static CAN_HandleTypeDef     hcan2;
    static CAN_HandleTypeDef     hcan3;

  private:
    uint32_t rx_pin;
    uint32_t tx_pin;
    int8_t term_pin;
    CAN_TypeDef* can_instance;
    CAN_HandleTypeDef* hcan;

    CAN_HandleTypeDef* getCAN();
    uint8_t getFilterBank();
    bool initGPIO();
};