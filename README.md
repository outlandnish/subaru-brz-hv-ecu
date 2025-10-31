# BMS Charging System for 6S2P Battery Packs

An intelligent battery management system for charging 6S2P (6 cells in series, 2 in parallel) lithium battery packs with automatic cell balancing and visual status indicators.

## Features

- **Intelligent Charging**: Automatically manages charging cycle with configurable target voltage
- **High-Speed Monitoring**: 50 Hz voltage measurements with exponential filtering for smooth, responsive readings
- **Cell Balancing**: Monitors individual cell voltages and balances cells when imbalance exceeds threshold
- **Fault Detection**: Monitors overvoltage, undervoltage, temperature, and cell balancing faults
- **Visual Status**: 5 NeoPixel LEDs provide real-time status indication
- **Serial Console**: Interactive command-line interface for monitoring and control
- **FreeRTOS**: Multi-tasking architecture for reliable operation
- **Safety**: Automotive-grade fault monitoring with 50 Hz sampling for rapid fault detection

## Hardware

- **MCU**: STM32 (configured for BRZ HV ECU)
- **BMS IC**: NXP MC33772C Battery Cell Controller
- **Interface**: TPL (Transformer Physical Layer) SPI communication
- **LEDs**: 5x NeoPixel (WS2812B) status indicators
- **Contactor**: Single high-voltage relay controlled via H-bridge driver with fault detection

## NeoPixel Status Indicators

The system uses 5 NeoPixels divided into two groups:
- **LEDs 0-3**: BMS State indicators
- **LED 4**: Contactor status indicator

### BMS State Indicators (LEDs 0-3)

| Color | Pattern | State | Description |
|-------|---------|-------|-------------|
| **Purple** | Solid | Initialization | System starting up, initializing BCC hardware |
| **Blue** | Breathing | Idle | Ready to charge, waiting for user command |
| **Green** | Chase | Charging | Active charging in progress |
| **Green** | Solid | Complete | Target voltage reached, charging complete |
| **Orange** | Pulsing | Balancing | Cell balancing active |
| **Red** | Flashing | Error | Fault detected (contactor/communication fault) |
| **Off** | - | Sleep | System in low-power mode |

### Contactor Status Indicator (LED 4)

| LED | Color | State | Description |
|-----|-------|-------|-------------|
| **LED 4** | Yellow | Enabled | Contactor is energized (IN1=HIGH, IN2=LOW) |
| **LED 4** | Off | Disabled | Contactor is de-energized (both inputs LOW) |

## Serial Console Commands

### Monitoring Commands

| Command | Description |
|---------|-------------|
| `help` | Show available commands and LED status guide |
| `status` | Display current BMS state |
| `voltages` | Show stack voltage and individual cell voltages with statistics |
| `faults` | Display fault status (overvoltage, undervoltage, temperature, etc.) |
| `config` | Display current charging configuration |
| `dump` | Dump all BCC configuration registers and fuse mirror calibration data (for debugging) |

### Control Commands

| Command | Description |
|---------|-------------|
| `start` | Begin charging cycle |
| `stop` | Stop charging and disable contactor |
| `balance` | Force cell balancing operation |
| `sleep` | Put BCC into low-power sleep mode |
| `wakeup` | Wake BCC from sleep mode |
| `reboot` | Software reset (restart application) |
| `dfu` | Reboot into USB DFU bootloader mode |

### Configuration Commands

| Command | Range | Description |
|---------|-------|-------------|
| `set target <voltage>` | 2.5 - 4.2V | Set target cell voltage |
| `set balance_th <mv>` | 1 - 500mV | Set balance threshold (when to start balancing) |
| `set balance_tgt <mv>` | 1 - 100mV | Set balance target (when to resume charging) |
| `set interval <ms>` | 100 - 10000ms | Set measurement interval |

### Example Usage

```
> voltages
=== Voltages ===
Stack: 21.456 V

Cell 1: 3.5760 V
Cell 2: 3.5780 V
Cell 3: 3.5740 V
Cell 4: 3.5820 V
Cell 5: 3.5750 V
Cell 6: 3.5770 V

Min: 3.5740 V, Max: 3.5820 V, Diff: 8.00 mV

> set target 3.8
Target voltage set to 3.80 V

> start
BMS: Starting charging cycle

> status
=== BMS Status ===
State: Charging

> sleep
Putting BCC into sleep mode...
BCC successfully entered sleep mode.
Note: Use 'wakeup' command to resume operation.

> reboot
=== System Reboot ===
Stopping charging...
Putting BCC into sleep mode...
Performing software reset...
[Device safely shuts down and restarts]

> dfu
=== Entering USB DFU Mode ===
Stopping charging...
Putting BCC into sleep mode...
Rebooting into bootloader...
[Device safely shuts down and reboots into DFU mode]
```

### Special Commands

**`sleep`** - Puts the BCC (Battery Cell Controller) into low-power mode. Useful for:
- Long-term storage
- Minimizing power consumption
- Preventing battery drain when not in use
- Note: Use `wakeup` or `reboot` command to resume operation

**`wakeup`** - Wakes the BCC from sleep mode:
- Attempts to wake the BCC from low-power mode
- For full functionality, recommend using `reboot` command instead
- Full re-initialization may be required for proper operation

**`reboot`** - Safely shuts down and performs a clean software reset:
1. Stops charging and disables contactors
2. Stops cell balancing
3. Puts BCC into sleep mode
4. Performs software reset
5. System restarts and re-initializes all hardware

Use this to:
- Resume operation after sleep mode
- Apply configuration changes
- Recover from errors or unexpected states
- Clean restart without power cycling

**`dfu`** - Safely shuts down the BMS and reboots into USB DFU (Device Firmware Update) mode:
1. Stops charging and disables contactors
2. Stops cell balancing
3. Puts BCC into sleep mode
4. Sets magic RAM flag
5. Performs system reset
6. On reboot, jumps to STM32 bootloader (before USB/serial init)

Use this to:
- Update firmware via USB using STM32CubeProgrammer or dfu-util
- Recover from bricked firmware
- Install new software versions

**How it works:**
- Uses a "magic RAM flag" (0xDEADBEEF) that survives system reset
- Flag stored in `.noinit` RAM section (preserved across soft resets)
- On reboot, `setup()` checks flag BEFORE any peripheral initialization
- If flag is set, jumps directly to STM32 bootloader at 0x1FFF0000
- Bootloader enumerates as USB DFU device

**Troubleshooting:**
If device doesn't appear as DFU device:
1. Check USB cable is connected and working
2. Verify device powered on
3. Look for "STM Device in DFU Mode" in device manager (Windows) or `lsusb` (Linux/Mac)
4. If still not working, you may need SWD programmer to recover

Once in DFU mode, flash firmware using:
```bash
# dfu-util (command line)
dfu-util -a 0 -s 0x08000000 -D firmware.bin

# STM32CubeProgrammer (GUI)
# Connect via USB, select "Download" and flash .bin file
```

**Important Notes:**
- After flashing, device will auto-reboot into new firmware
- No physical buttons needed - fully software-controlled DFU entry
- Safe for HV systems: Always shuts down charging/balancing before DFU
- If DFU entry fails, use SWD programming via ST-Link or J-Link as backup

## Charging Algorithm

The system uses an intelligent charging algorithm with automatic cell balancing:

1. **Idle State**: System ready, monitoring cell voltages
2. **Start Charging**: User initiates charging via `start` command or automatically
3. **Charging**:
   - Contactor enabled, charging current flows
   - Continuous monitoring of cell voltages
   - If cell imbalance > `balance_threshold_mv`: Stop charging, enter balancing
4. **Cell Balancing**:
   - Contactor disabled (no charging current)
   - High cells are discharged through balancing resistors
   - Monitor until imbalance < `balance_target_mv`
   - Resume charging
5. **Complete**: All cells reach `target_cell_voltage`
6. **Stop**: Contactor disabled, system returns to idle

## Default Configuration

```cpp
target_cell_voltage      = 3.6V    // Target voltage per cell
balance_threshold_mv     = 50.0mV  // Start balancing when cells differ by this amount
balance_target_mv        = 10.0mV  // Resume charging when cells are balanced to this level
balancing_timer_min      = 5 min   // Balancing timer duration
measurement_interval_ms  = 20ms    // Voltage measurement frequency (50 Hz)
voltage_filter_alpha     = 0.2     // Exponential filter coefficient
```

## Architecture

### FreeRTOS Tasks

1. **Master Task** (Priority 2)
   - Manages charging state machine
   - Controls contactor via H-bridge driver
   - Coordinates balancing operations
   - Updates LED status indicators

2. **BCC0 Monitor Task** (Priority 2)
   - Initializes BCC hardware (after scheduler starts)
   - Reads cell voltages and stack voltage at 50 Hz (20ms interval)
   - Applies exponential filtering to smooth measurements (alpha = 0.2)
   - Checks fault status every 5 seconds
   - Periodic reporting to console (every 1 second)

3. **Console Task** (Priority 1)
   - Handles serial input
   - Processes user commands
   - Provides interactive interface

### BMS States

- `BMS_Initialization`: System starting up
- `BMS_Idle`: Ready to charge
- `BMS_Charging`: Active charging
- `BMS_CellBalancing`: Balancing cells
- `BMS_Cooldown`: Stopped by user
- `BMS_Sleep`: Low-power mode
- `BMS_Error`: Fault detected

## Safety Features

- **Contactor Fault Detection**: Monitors H-bridge fault pin, immediately disables contactor on fault
- **Communication Loss Detection**: Automatically enters error state and disables charging if BCC communication is lost for >5 seconds
- **Cell Voltage Monitoring**: Continuous monitoring prevents overcharge
- **Fault Monitoring**: Periodically checks for:
  - Cell overvoltage/undervoltage faults
  - Temperature faults (overtemperature/undertemperature)
  - Cell balancing circuit faults
  - Communication errors
  - General fault conditions
- **Automatic Balancing**: Prevents cell imbalance and extends battery life
- **Error State**: System locks in error state until reset on critical faults
- **Visual Indicators**:
  - Red flashing state LEDs immediately indicate fault conditions
  - Contactor LEDs (3-4) show real-time H-bridge and contactor status for safety verification

## Pin Configuration

### BMS0 (MC33772C)
- TX SCK: PA5
- TX CS: PA6
- TX DATA: PA7
- RX SCK: PA9
- RX CS: PB9
- RX DATA: PA10
- ENABLE: PE8
- INTB: PE9

### H-Bridge Contactor Driver
- H-Bridge IN1: PC0 (HIGH to enable contactor)
- H-Bridge IN2: PC1 (LOW to enable contactor)
- H-Bridge Enable: PC2 (enable signal)
- H-Bridge Fault: PC3 (fault input)

### Status LEDs
- NeoPixel Data: PD5 (5 LEDs)

## Building and Flashing

This project uses PlatformIO:

```bash
# Build the project
pio run

# Upload to board
pio run --target upload

# Open serial monitor
pio device monitor
```

## Serial Console

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1

Connect via USB and use any serial terminal (Arduino IDE Serial Monitor, PlatformIO Serial Monitor, screen, minicom, etc.)

## License

[Your License Here]

## Contributing

[Your Contributing Guidelines Here]

## Credits

- NXP MC33772C Battery Cell Controller
- Adafruit NeoPixel Library
- STM32 FreeRTOS
