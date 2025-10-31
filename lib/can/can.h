#pragma once
#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "PeripheralPins.h"
#include "pinmap.h"
#include "Arduino.h"

// Callback function type for RX interrupts
typedef void (*CANRxCallback)(uint32_t id, uint8_t* data, uint8_t len);

class CANBus {
  static CANRxCallback         callback1;
  static CANRxCallback         callback2;
  static CANRxCallback         callback3;

  public:
    // Constructor now takes Arduino pin numbers for RX, TX, and optional termination control
    CANBus(uint32_t rx_pin, uint32_t tx_pin, int8_t term_pin = -1);
    bool begin(uint32_t baudrate);
    void end();
    bool sendMessage(uint32_t id, uint8_t* data, uint8_t len);
    bool receiveMessage(uint32_t &id, uint8_t* data, uint8_t &len);
    void setTermination(bool enabled);

    // Filter configuration
    bool setFilter(uint32_t filter_id, uint32_t filter_mask, uint32_t filter_bank = 0);
    bool setFilterRange(uint32_t id_low, uint32_t id_high, uint32_t filter_bank = 0);
    bool disableFilter(uint32_t filter_bank = 0);

    // Interrupt configuration
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