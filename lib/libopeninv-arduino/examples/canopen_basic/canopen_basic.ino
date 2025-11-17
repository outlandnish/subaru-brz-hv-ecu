/**
 * @file canopen_basic.ino
 * @brief Basic example of using libopeninv-arduino with CANOpen
 *
 * This example demonstrates:
 * - Initializing the CAN bus
 * - Setting up CANOpen SDO for parameter access
 * - Configuring CAN message mapping
 * - Saving/loading parameters to/from flash
 */

#include <Arduino.h>
#include "can.h"
#include "canhardware_arduino.h"
#include "params.h"
#include "param_save.h"
#include "cansdo.h"
#include "canmap.h"

// CAN pins (adjust for your board)
#define CAN_RX_PIN PB8
#define CAN_TX_PIN PB9
#define CAN_TERM_PIN -1  // No termination resistor control

// Create CAN bus instance
CANBus can(CAN_RX_PIN, CAN_TX_PIN, CAN_TERM_PIN);

// Create libopeninv hardware wrapper
CanHardwareArduino canHardware(&can);

// Create CAN mapping and SDO handlers
CanMap canMap(&canHardware);
CanSdo canSdo(&canHardware, &canMap);

// Parameter change callback - called when a parameter is modified via CANOpen
void Param::Change(Param::PARAM_NUM param)
{
    // You can respond to parameter changes here
    switch(param)
    {
        case Param::canNodeId:
            // Update CAN node ID
            canSdo.SetNodeId(Param::GetInt(param));
            break;

        default:
            break;
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}

    Serial.println("\n=== libopeninv-arduino CANOpen Example ===");

    // Initialize parameters with defaults
    Param::LoadDefaults();

    // Try to load parameters from flash
    if (parm_load() == 0)
    {
        Serial.println("Parameters loaded from flash");
    }
    else
    {
        Serial.println("No saved parameters, using defaults");
        Serial.println("Parameters will be saved on first write");
    }

    // Print current parameters
    Serial.println("\nCurrent Parameters:");
    Serial.print("  CAN Node ID: ");
    Serial.println(Param::GetInt(Param::canNodeId));

    // Initialize CAN bus at 500 kbps
    if (!can.begin(CAN_BPS_500K))
    {
        Serial.println("ERROR: CAN initialization failed!");
        while (1) delay(100);
    }

    Serial.println("CAN bus initialized at 500 kbps");

    // Set up CANOpen node ID
    canSdo.SetNodeId(Param::GetInt(Param::canNodeId));

    // Example: Map a received CAN message to a parameter
    // Receive pack voltage from CAN ID 0x100, byte 0-1 (16 bits, little endian)
    // Multiply by 0.1 to convert from 0.1V units to V
    canMap.AddRecv(Param::packVoltage, 0x100, 0, 16, 0.1f);

    // Example: Send pack current on CAN ID 0x200, byte 0-1 (16 bits, little endian)
    // Multiply by 10 to convert from A to 0.1A units
    canMap.AddSend(Param::packCurrent, 0x200, 0, 16, 10.0f);

    // Save CAN mapping to flash
    canMap.Save();

    Serial.println("\nCANOpen ready!");
    Serial.println("  SDO Request: 0x6");
    Serial.print(Param::GetInt(Param::canNodeId), HEX);
    Serial.println("  SDO Response: 0x5");
    Serial.print(Param::GetInt(Param::canNodeId), HEX);
    Serial.println("\nYou can now:");
    Serial.println("  - Read/write parameters via SDO (index 0x2000)");
    Serial.println("  - Parameters are automatically mapped to CAN messages");
    Serial.println("  - Type 's' to save parameters to flash");
}

void loop()
{
    // Periodically send CAN mapped messages (every 100ms)
    static unsigned long lastSend = 0;
    if (millis() - lastSend >= 100)
    {
        lastSend = millis();

        // Update some example values
        static float voltage = 300.0f;
        static float current = 50.0f;

        voltage += (random(-10, 10) / 10.0f);  // Simulate voltage variation
        current += (random(-5, 5) / 10.0f);     // Simulate current variation

        Param::SetFloat(Param::packVoltage, voltage);
        Param::SetFloat(Param::packCurrent, current);

        // Send all mapped CAN messages
        canMap.SendAll();
    }

    // Handle user input
    if (Serial.available())
    {
        char cmd = Serial.read();

        if (cmd == 's' || cmd == 'S')
        {
            Serial.println("Saving parameters to flash...");
            uint32_t crc = parm_save();
            Serial.print("Saved with CRC: 0x");
            Serial.println(crc, HEX);
        }
        else if (cmd == 'l' || cmd == 'L')
        {
            Serial.println("Loading parameters from flash...");
            if (parm_load() == 0)
            {
                Serial.println("Parameters loaded successfully");
            }
            else
            {
                Serial.println("Failed to load parameters (CRC error)");
            }
        }
        else if (cmd == 'p' || cmd == 'P')
        {
            Serial.println("\nCurrent Parameters:");
            Serial.print("  CAN Node ID: ");
            Serial.println(Param::GetInt(Param::canNodeId));
            Serial.print("  Pack Voltage: ");
            Serial.print(Param::GetFloat(Param::packVoltage));
            Serial.println(" V");
            Serial.print("  Pack Current: ");
            Serial.print(Param::GetFloat(Param::packCurrent));
            Serial.println(" A");
        }
    }
}
