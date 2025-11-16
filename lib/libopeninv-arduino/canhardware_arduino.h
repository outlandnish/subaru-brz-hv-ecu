/*
 * Arduino port of libopeninv CanHardware
 * Wraps the existing CANBus class to provide the CanHardware interface
 */
#ifndef CANHARDWARE_ARDUINO_H
#define CANHARDWARE_ARDUINO_H

#include <Arduino.h>
#include "can.h"
#include "canhardware.h"

class CanHardwareArduino : public CanHardware
{
public:
    CanHardwareArduino(CANBus* canBus);

    void SetBaudrate(enum baudrates baudrate) override;
    void Send(uint32_t canId, uint32_t data[2], uint8_t len) override;

protected:
    void ConfigureFilters() override;

private:
    CANBus* can;

    // Static callback handler for CAN interrupts
    static void canRxCallback(uint32_t id, uint8_t* data, uint8_t len);
    static CanHardwareArduino* instance; // For interrupt callback

    // Convert between data formats
    void convertToCanFrame(uint32_t canId, uint32_t data[2], uint8_t len, CAN_FRAME& frame);
};

#endif // CANHARDWARE_ARDUINO_H
