# BMS Charging System for 6S2P Battery Packs

An intelligent battery management system for charging 6S2P (6 cells in series, 2 in parallel) lithium battery packs with automatic cell balancing and visual status indicators.

## Hardware

- **MCU**: STM32 (configured for BRZ HV ECU)
- **BMS IC**: NXP MC33772C Battery Cell Controller
- **Interface**: TPL (Transformer Physical Layer) SPI communication
- **LEDs**: 5x NeoPixel (WS2812B) status indicators

## NeoPixel Status Indicators

The system uses 5 NeoPixels divided into three groups:
- **LED 0-1**: BMS state
- **LED 2**: HV status (disabled, pre-charge, active, fault, shutdown)
- **LED 3**: EVSE (charging connector) status indicator
- **LED 4**: PCS State (charge enabled/disabled, dcdc enabled / disabled, fault)

### BMS State Indicators (LEDs 0-1)

| Color | Pattern | State | Description |
|-------|---------|-------|-------------|
| **Purple** | Solid | Initialization | System starting up, initializing BCC hardware |
| **Blue** | Breathing | Idle | Ready to charge, waiting for user command |
| **Green** | Chase | Charging | Active charging in progress |
| **Green** | Solid | Complete | Target voltage reached, charging complete |
| **Orange** | Pulsing | Balancing | Cell balancing active |
| **Red** | Flashing | Error | Fault detected (contactor/communication fault) |
| **Off** | - | Sleep | System in low-power mode |

### HV Status Indicator (LED 2)

| Color | Pattern | State | Description |
|-------|---------|-------|-------------|
| **Green** | Solid | Active | HV active |
| **Orange** | Solid | Shutdown | HV shutdown |
| **Red** | Flashing | Fault | HV fault |
| **Off** | - | Disabled | HV Disabled |

### EVSE Status Indicator (LED 3)

| Color | Pattern | State | Description |
|-------|---------|-------|-------------|
| **Off** | - | State A | No charging cable connected |
| **Cyan** | Solid | State B | Cable connected, vehicle not ready to charge |
| **Green** | Solid | State C | Vehicle ready to charge / charging |
| **Magenta** | Solid | State D | Vehicle requires ventilation (uncommon) |
| **Red** | Flashing | State E | No power available from EVSE |
| **Red** | Solid | Fault | EVSE fault detected |

### PCS Status Indicator (LED 4)



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
   - Checks fault status every 5 seconds
   - Periodic reporting to console (every 1 second)

3. **Console Task** (Priority 1)
   - Handles serial input
   - Processes user commands
   - Provides interactive interface

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
  - EVSE LED (3) shows charging cable connection and control pilot status
  - Contactor LED (4) shows real-time H-bridge and contactor status for safety verification

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
