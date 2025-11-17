/*
 * Arduino port of libopeninv CanHardware
 * Wraps the existing CANBus class to provide the CanHardware interface
 */
#include "canhardware_arduino.h"

CanHardwareArduino* CanHardwareArduino::instance = nullptr;

CanHardwareArduino::CanHardwareArduino(CANBus* canBus)
    : CanHardware(), can(canBus)
{
    instance = this;
}

void CanHardwareArduino::SetBaudrate(enum baudrates baudrate)
{
    uint32_t baud;

    switch(baudrate)
    {
        case Baud125:  baud = CAN_BPS_125K; break;
        case Baud250:  baud = CAN_BPS_250K; break;
        case Baud500:  baud = CAN_BPS_500K; break;
        case Baud800:  baud = 800000; break;
        case Baud1000: baud = CAN_BPS_1000K; break;
        default:       baud = CAN_BPS_500K; break;
    }

    can->init(baud);
}

void CanHardwareArduino::Send(uint32_t canId, uint32_t data[2], uint8_t len)
{
    CAN_FRAME frame;
    convertToCanFrame(canId, data, len, frame);

    // Debug: Print outgoing CAN message
    Serial.printf("CAN TX: 0x%03X [%d] %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                  canId, len,
                  frame.data.uint8[0], frame.data.uint8[1], frame.data.uint8[2], frame.data.uint8[3],
                  frame.data.uint8[4], frame.data.uint8[5], frame.data.uint8[6], frame.data.uint8[7]);

    can->sendFrame(frame);
}

void CanHardwareArduino::ConfigureFilters()
{
    // Configure CAN filters for registered user messages
    for (int i = 0; i < nextUserMessageIndex && i < MAX_USER_MESSAGES; i++)
    {
        if (userMasks[i] == 0)
        {
            // Exact match filter
            can->_setFilter(userIds[i], 0x7FF, false);
        }
        else
        {
            // Masked filter
            can->_setFilter(userIds[i], userMasks[i], false);
        }
    }
}

void CanHardwareArduino::canRxCallback(uint32_t id, uint8_t* data, uint8_t len)
{
    if (instance)
    {
        uint32_t data32[2];
        // Convert byte array to 32-bit array (libopeninv format)
        memcpy(data32, data, 8);

        // Update timestamp
        instance->lastRxTimestamp = millis();

        // Call the CanHardware HandleRx which will dispatch to registered callbacks
        instance->HandleRx(id, data32, len);
    }
}

void CanHardwareArduino::convertToCanFrame(uint32_t canId, uint32_t data[2], uint8_t len, CAN_FRAME& frame)
{
    frame.id = canId;
    frame.extended = (canId > 0x7FF) ? 1 : 0;
    frame.rtr = 0;
    frame.length = len;
    memcpy(frame.data.uint8, data, len);
}
