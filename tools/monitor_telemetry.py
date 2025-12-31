#!/usr/bin/env python3
"""
Thermal Telemetry Monitor
Connects to the thermal management system via Serial and logs telemetry data.

Usage:
    python monitor_telemetry.py COM3 115200
    python monitor_telemetry.py /dev/ttyUSB0 115200
"""

import serial
import json
import sys
import time
from datetime import datetime

def monitor_serial(port, baudrate=115200):
    """
    Monitor and parse JSON telemetry from thermal management system
    
    Args:
        port: Serial port (e.g., 'COM3' or '/dev/ttyUSB0')
        baudrate: Communication speed (default 115200)
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baud")
        print("Waiting for telemetry data...\n")
        print(f"{'Time':<12} {'Temp (°C)':<12} {'Fan %':<10} {'Power State':<15} {'Thermal State':<15}")
        print("-" * 80)
        
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            
            if line.startswith('{'):
                try:
                    data = json.loads(line)
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    
                    temp = data.get('temp_c', 0.0)
                    fan_pct = data.get('fan_percent', 0.0)
                    pwr_state = data.get('pwr_state', 'UNKNOWN')
                    thermal_state = data.get('thermal_state', 'UNKNOWN')
                    
                    print(f"{timestamp:<12} {temp:<12.2f} {fan_pct:<10.1f} {pwr_state:<15} {thermal_state:<15}")
                    
                except json.JSONDecodeError:
                    print(f"[WARNING] Invalid JSON: {line}")
            elif line:
                # Print non-JSON debug messages
                print(f"[DEBUG] {line}")
                
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user")
        ser.close()
        sys.exit(0)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python monitor_telemetry.py <PORT> [BAUDRATE]")
        print("Example: python monitor_telemetry.py COM3 115200")
        sys.exit(1)
    
    port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    monitor_serial(port, baudrate)
