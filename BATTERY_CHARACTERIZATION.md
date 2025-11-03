# Battery Characterization Tool

This tool reads GUID, configuration registers, and fuse mirror data from up to 8 batteries on each BCC chain (BCC0 and BCC1) and outputs the data in CSV format.

## Overview

The battery characterization application is a separate PlatformIO environment that:
- Initializes both BCC0 and BCC1 chains
- Detects up to 8 MC33772C devices on each chain (16 batteries total)
- Reads GUID from each battery
- Reads all configuration registers (44 registers for MC33772C)
- Reads all fuse mirror data (32 fuse addresses for MC33772C)
- Outputs all data in CSV format via serial port

## Hardware Setup

1. Connect up to 8 batteries to BCC0 daisy chain
2. Connect up to 8 batteries to BCC1 daisy chain
3. Ensure proper power supply to the board
4. Connect ST-Link debugger or USB cable

## Building and Uploading

### Build the application
```bash
pio run -e battery-char
```

### Upload to board
```bash
pio run -e battery-char -t upload
```

## Capturing Data

### Method 1: Using Python Capture Script (Recommended)

The included Python script automatically captures serial output and saves it to a CSV file:

```bash
# Auto-detect port and use timestamped filename
python3 capture_battery_data.py

# Specify port and output file
python3 capture_battery_data.py --port /dev/ttyACM0 --output my_batteries.csv

# With custom timeout
python3 capture_battery_data.py --timeout 120
```

The script will:
- Auto-detect the serial port (or use specified port)
- Create a CSV file with timestamp
- Capture all data automatically
- Stop when characterization is complete

### Method 2: Manual Serial Monitor

```bash
# Using PlatformIO
pio device monitor

# Or using screen (Linux/Mac)
screen /dev/ttyACM0 115200

# Save output to file
pio device monitor > battery_data.txt
```

Then manually extract the CSV data from the output.

## CSV Output Format

The tool outputs data in the following CSV format:

```csv
BCC,CID,Field,Address,Value,Description
0,1,GUID,N/A,0x01ABCD1234,Device GUID
0,1,INIT,0x0001,0x1234,Initialization register
0,1,CELL_OV_UV,0x0010,0x5678,Configuration register
0,1,FUSE_00,0x00,0xABCD,Fuse mirror data
...
```

### CSV Columns:
- **BCC**: BCC controller number (0 or 1)
- **CID**: Cluster ID (1-8 for each device in the chain)
- **Field**: Register or data field name
- **Address**: Register address in hex
- **Value**: Register value in hex (or ERROR_N if read failed)
- **Description**: Description of the field

## Data Analysis

Once you have the CSV file, you can:

1. **Open in Excel/LibreOffice**: Import as CSV for easy viewing
2. **Filter by BCC**: See all data from one controller
3. **Filter by CID**: See all data from one battery
4. **Compare GUIDs**: Track individual batteries
5. **Analyze registers**: Compare configuration across batteries
6. **Check fuse data**: Verify factory calibration values

## Example Workflow

```bash
# 1. Build and upload the application
pio run -e battery-char -t upload

# 2. Wait a few seconds for the board to boot

# 3. Run the capture script
python3 capture_battery_data.py

# 4. The script will automatically save data to battery_data_YYYYMMDD_HHMMSS.csv

# 5. Open the CSV file in your favorite spreadsheet application
```

## Troubleshooting

### No devices detected
- Check battery connections
- Verify power supply
- Check enable pins are properly configured
- Verify daisy chain wiring

### Communication errors
- Reduce communication speed
- Check for electrical noise
- Verify SPI connections

### Serial port not found
- Check USB cable connection
- Verify ST-Link drivers installed
- Specify port manually: `python3 capture_battery_data.py --port /dev/ttyACM0`

### Incomplete data capture
- Increase timeout: `python3 capture_battery_data.py --timeout 120`
- Check for communication issues with batteries
- Review error messages in output

## Architecture Details

### Source Files
- `src/battery_char.cpp` - Main characterization application
- `src/hv-ecu-v0-pins.h` - Pin definitions
- `capture_battery_data.py` - Data capture helper script

### PlatformIO Environment
The `[env:battery-char]` environment in `platformio.ini`:
- Uses minimal dependencies (no FreeRTOS)
- Filters out main BMS application files
- Optimized for quick builds

### Data Collection Process
1. Initialize BCC0 and BCC1 controllers
2. Detect devices on each chain (scan CIDs 1-8)
3. For each detected device:
   - Read GUID
   - Read INIT register
   - Read all 44 configuration registers (MC33772C)
   - Read all 32 fuse mirror addresses
4. Output all data as CSV via serial

## Notes

- The tool assumes MC33772C (6-cell) devices by default
- Modify `devices_0[i] = BCC_DEVICE_MC33772` if using MC33771C (14-cell)
- Each device takes ~5-10 seconds to characterize
- Total time for 16 batteries: ~2-3 minutes
- Data is output in real-time as it's collected
- Press reset button to run characterization again
