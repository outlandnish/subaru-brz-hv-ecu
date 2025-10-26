# BMS Charging System for 6S2P Battery Packs

An intelligent battery management system for charging 6S2P (6 cells in series, 2 in parallel) lithium battery packs with automatic cell balancing and visual status indicators.

## Features

- **Intelligent Charging**: Automatically manages charging cycle with configurable target voltage
- **Cell Balancing**: Monitors individual cell voltages and balances cells when imbalance exceeds threshold
- **Visual Status**: 5 NeoPixel LEDs provide real-time status indication
- **Serial Console**: Interactive command-line interface for monitoring and control
- **FreeRTOS**: Multi-tasking architecture for reliable operation
- **Safety**: Contactor fault detection and automatic shutdown

## Hardware

- **MCU**: STM32 (configured for BRZ HV ECU)
- **BMS IC**: NXP MC33772C Battery Cell Controller
- **Interface**: TPL (Transformer Physical Layer) SPI communication
- **LEDs**: 5x NeoPixel (WS2812B) status indicators
- **Contactors**: High-voltage relay control with fault detection

## NeoPixel Status Indicators

The system uses 5 NeoPixels divided into two groups:
- **LEDs 0-2**: BMS State indicators
- **LEDs 3-4**: Contactor status indicators

### BMS State Indicators (LEDs 0-2)

| Color | Pattern | State | Description |
|-------|---------|-------|-------------|
| **Purple** | Solid | Initialization | System starting up, initializing BCC hardware |
| **Blue** | Breathing | Idle | Ready to charge, waiting for user command |
| **Green** | Chase | Charging | Active charging in progress |
| **Green** | Solid | Complete | Target voltage reached, charging complete |
| **Orange** | Pulsing | Balancing | Cell balancing active |
| **Red** | Flashing | Error | Fault detected (contactor/communication fault) |
| **Off** | - | Sleep | System in low-power mode |

### Contactor Status Indicators (LEDs 3-4)

| LED | Color | State | Description |
|-----|-------|-------|-------------|
| **LED 3** | Yellow | Closed | Negative contactor is energized/closed |
| **LED 3** | Off | Open | Negative contactor is de-energized/open |
| **LED 4** | Cyan | Closed | Positive contactor is energized/closed |
| **LED 4** | Off | Open | Positive contactor is de-energized/open |

## Serial Console Commands

### Monitoring Commands

| Command | Description |
|---------|-------------|
| `help` | Show available commands and LED status guide |
| `status` | Display current BMS state |
| `voltages` | Show stack voltage and individual cell voltages with statistics |
| `config` | Display current charging configuration |
| `dump` | Dump all BCC configuration registers (for debugging) |

### Control Commands

| Command | Description |
|---------|-------------|
| `start` | Begin charging cycle |
| `stop` | Stop charging and disable contactors |
| `balance` | Force cell balancing operation |

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
```

## Charging Algorithm

The system uses an intelligent charging algorithm with automatic cell balancing:

1. **Idle State**: System ready, monitoring cell voltages
2. **Start Charging**: User initiates charging via `start` command or automatically
3. **Charging**:
   - Contactors enabled, charging current flows
   - Continuous monitoring of cell voltages
   - If cell imbalance > `balance_threshold_mv`: Stop charging, enter balancing
4. **Cell Balancing**:
   - Contactors disabled (no charging current)
   - High cells are discharged through balancing resistors
   - Monitor until imbalance < `balance_target_mv`
   - Resume charging
5. **Complete**: All cells reach `target_cell_voltage`
6. **Stop**: Contactors disabled, system returns to idle

## Default Configuration

```cpp
target_cell_voltage      = 3.6V    // Target voltage per cell
balance_threshold_mv     = 50.0mV  // Start balancing when cells differ by this amount
balance_target_mv        = 10.0mV  // Resume charging when cells are balanced to this level
balancing_timer_min      = 5 min   // Balancing timer duration
measurement_interval_ms  = 1000ms  // Voltage measurement frequency
```

## Architecture

### FreeRTOS Tasks

1. **Master Task** (Priority 2)
   - Manages charging state machine
   - Controls contactors
   - Coordinates balancing operations
   - Updates LED status indicators

2. **BCC0 Monitor Task** (Priority 2)
   - Initializes BCC hardware (after scheduler starts)
   - Reads cell voltages and stack voltage
   - Periodic reporting (every 10 seconds)

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

- **Contactor Fault Detection**: Monitors fault pin, immediately disables contactors on fault
- **Communication Loss Detection**: Automatically enters error state and disables charging if BCC communication is lost for >5 seconds
- **Cell Voltage Monitoring**: Continuous monitoring prevents overcharge
- **Automatic Balancing**: Prevents cell imbalance and extends battery life
- **Error State**: System locks in error state until reset on critical faults
- **Visual Indicators**:
  - Red flashing state LEDs immediately indicate fault conditions
  - Contactor LEDs show real-time contactor status for safety verification

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

### Contactors
- Negative Contactor: PC0
- Positive Contactor: PC1
- Contactor Enable: PC2
- Contactor Fault: PC3

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
