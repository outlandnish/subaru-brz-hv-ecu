#!/usr/bin/env python3
"""
Battery Characterization Data Capture Script

This script captures serial output from the battery characterization tool
and saves it to a CSV file with timestamp.

Usage:
    python3 capture_battery_data.py [--port /dev/ttyACM0] [--output battery_data.csv]
"""

import serial
import argparse
import sys
import time
from datetime import datetime

def find_serial_port():
    """Try to find the device serial port automatically."""
    import serial.tools.list_ports

    # Look for STM32 devices
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'STM32' in port.description or 'USB' in port.description:
            return port.device

    # Fallback to common device names
    common_ports = ['/dev/ttyACM0', '/dev/ttyUSB0', 'COM3', 'COM4']
    for port in common_ports:
        try:
            s = serial.Serial(port)
            s.close()
            return port
        except:
            pass

    return None

def capture_data(port, output_file, baudrate=115200, timeout=60):
    """
    Capture serial data and save to CSV file.

    Args:
        port: Serial port path
        output_file: Output CSV file path
        baudrate: Serial baudrate (default 115200)
        timeout: Timeout in seconds to wait for no more data
    """
    try:
        # Open serial port
        print(f"Opening serial port: {port}")
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for connection to stabilize

        # Open output file
        print(f"Saving data to: {output_file}")
        with open(output_file, 'w') as f:
            # Write file header with timestamp
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            f.write(f"# Battery Characterization Data\n")
            f.write(f"# Captured: {timestamp}\n")
            f.write(f"# Port: {port}\n")
            f.write("#\n")

            csv_started = False
            csv_header_written = False
            last_data_time = time.time()
            line_count = 0

            print("\nCapturing data... (press Ctrl+C to stop)")
            print("-" * 60)

            while True:
                # Check for timeout
                if time.time() - last_data_time > timeout:
                    print(f"\nNo data received for {timeout} seconds. Stopping.")
                    break

                # Read line from serial
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()

                        if not line:
                            continue

                        last_data_time = time.time()

                        # Print to console
                        print(line)

                        # Check if this is the BCC config dump CSV marker
                        if '=== CSV Format: BCC Configuration Dump ===' in line:
                            csv_started = True
                            csv_header_written = False
                            continue

                        # If CSV started but header not written, next line is the header
                        if csv_started and not csv_header_written:
                            if ',' in line and not line.startswith('#'):
                                f.write(line + '\n')
                                csv_header_written = True
                                f.flush()
                            continue

                        # Write CSV data to file (after header is written)
                        if csv_started and csv_header_written and ',' in line and not line.startswith('#'):
                            # Skip lines that look like separators or markers
                            if not line.startswith('===') and not line.startswith('---'):
                                f.write(line + '\n')
                                line_count += 1
                                f.flush()  # Ensure data is written immediately

                        # Check for completion
                        if 'Characterization Complete!' in line or 'Config dump complete' in line:
                            print(f"\n\nCapture complete! {line_count} data lines saved.")
                            break

                    except UnicodeDecodeError:
                        continue

                time.sleep(0.01)  # Small delay to prevent CPU spinning

        ser.close()
        print(f"\nData saved to: {output_file}")
        return True

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}")
        print(f"Details: {e}")
        return False
    except KeyboardInterrupt:
        print("\n\nCapture interrupted by user.")
        ser.close()
        return True
    except Exception as e:
        print(f"Error: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Capture battery characterization data to CSV file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-detect port and use default filename
  python3 capture_battery_data.py

  # Specify port and output file
  python3 capture_battery_data.py --port /dev/ttyACM0 --output batteries_2024.csv

  # Specify custom timeout
  python3 capture_battery_data.py --timeout 120
        """
    )

    parser.add_argument(
        '--port', '-p',
        help='Serial port (auto-detect if not specified)',
        default=None
    )

    parser.add_argument(
        '--output', '-o',
        help='Output CSV file path',
        default=None
    )

    parser.add_argument(
        '--baudrate', '-b',
        help='Serial baudrate (default: 115200)',
        type=int,
        default=115200
    )

    parser.add_argument(
        '--timeout', '-t',
        help='Timeout in seconds (default: 60)',
        type=int,
        default=60
    )

    args = parser.parse_args()

    # Find port if not specified
    if args.port is None:
        print("Auto-detecting serial port...")
        args.port = find_serial_port()
        if args.port is None:
            print("Error: Could not find serial port automatically.")
            print("Please specify the port using --port option.")
            sys.exit(1)
        print(f"Found port: {args.port}")

    # Generate output filename with timestamp if not specified
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"battery_data_{timestamp}.csv"

    print("\n" + "=" * 60)
    print("Battery Characterization Data Capture")
    print("=" * 60)
    print(f"Port:    {args.port}")
    print(f"Output:  {args.output}")
    print(f"Timeout: {args.timeout}s")
    print("=" * 60 + "\n")

    # Capture data
    success = capture_data(args.port, args.output, args.baudrate, args.timeout)

    if success:
        print("\n✓ Capture completed successfully!")
        sys.exit(0)
    else:
        print("\n✗ Capture failed!")
        sys.exit(1)

if __name__ == '__main__':
    main()
