# Subaru BRZ HV ECU

> **SAFETY NOTICE:** This system controls high-voltage contactors on a traction battery pack.
> Incorrect firmware can result in arc flash, fire, or electrocution. Do not flash untested builds
> to hardware connected to a live HV pack.

This is the HV ECU for a Subaru BRZ EV conversion. It manages:
- **Battery management** — cell voltage monitoring, balancing, and protection via NXP MC33772 BCC ICs
- **HV contactor control** — precharge, engage, and safe shutdown via DRV8874 H-bridge drivers
- **CAN communication** — IVT-S current/voltage shunt (HV CAN), CHAdeMO EVSE controller (M3 CAN)
- **CANopen** — parameter access and spot-value reporting via libopeninv

## Hardware

See [`hardware/`](hardware/) for schematics and pin maps.

Key specs:
- MCU: STM32F413VH running FreeRTOS
- 16 Taycan modules (8 per BCC chain, two MC33772 chains)
- DRV8874 H-bridge drivers for HV and AC contactors
- IVT-S 1000 A current shunt on HV CAN bus
- CHAdeMO controller (Foccci) on M3 CAN bus

## Getting Started

Clone with submodules:

```bash
git clone --recurse-submodules <repo-url>
# or after a plain clone:
git submodule update --init --recursive
```

## Building and Flashing

This project uses [PlatformIO](https://platformio.org/).

```bash
# Build main firmware
pio run -e hv-ecu

# Flash via ST-Link
pio run -e hv-ecu --target upload

# Serial monitor (115200 baud)
pio device monitor
```

## First-Boot Procedure

1. Flash firmware with ST-Link connected and HV pack **disconnected**.
2. Open a serial monitor at 115200 baud.
3. Type `status` to verify BCC devices are detected and cell voltages are sensible.
4. Type `params` to review default charge parameters before connecting HV.
5. Connect HV pack. Type `hv connect` to initiate precharge sequence.

## Running Tests

Unit tests run on-target via PlatformIO Unity:

```bash
pio test -e bms-test
```

## License

MIT — see [LICENSE](LICENSE).
