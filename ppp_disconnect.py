#!/usr/bin/env python3
"""
PPP Disconnect Utility

Stops pppd and sends escape sequence to ESP32 to exit passthrough mode.
Run from SSH when you need to disconnect PPP.

Usage:
    sudo python3 ppp_disconnect.py
    
Or make executable:
    chmod +x ppp_disconnect.py
    sudo ./ppp_disconnect.py
"""

import subprocess
import serial
import time
import sys

SERIAL_PORT = '/dev/ttyAMA0'  # Adjust if different
BAUD_RATE = 115200
PPP_STOP_SEQUENCE = b'+++STOPPPP\n'

def disconnect_ppp():
    print("=== PPP Disconnect Utility ===")
    
    # Step 1: Kill pppd if running
    print("1. Stopping pppd...")
    try:
        # Try graceful stop first
        subprocess.run(['sudo', 'pkill', '-TERM', 'pppd'], capture_output=True, timeout=5)
        time.sleep(2)
        
        # Force kill if still running
        result = subprocess.run(['pgrep', 'pppd'], capture_output=True)
        if result.returncode == 0:
            print("   Force killing pppd...")
            subprocess.run(['sudo', 'pkill', '-KILL', 'pppd'], capture_output=True, timeout=5)
            time.sleep(1)
        
        print("   pppd stopped")
    except Exception as e:
        print(f"   Note: {e}")
    
    # Step 2: Wait for serial port to be released
    print("2. Waiting for serial port...")
    time.sleep(3)
    
    # Step 3: Send escape sequence to ESP32
    print(f"3. Sending stop signal to ESP32 via {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(0.5)  # Let port stabilize
        
        # Send escape sequence multiple times to ensure receipt
        for i in range(3):
            ser.write(PPP_STOP_SEQUENCE)
            ser.flush()
            time.sleep(0.5)
        
        ser.close()
        print("   Stop signal sent!")
        print("\n4. ESP32 will restart to normal mode in ~3 seconds")
        print("   modem.py should automatically reconnect after restart")
        
    except serial.SerialException as e:
        print(f"   Could not open serial port: {e}")
        print("   The serial port may still be in use. Try again in a few seconds.")
        return False
    except Exception as e:
        print(f"   Error: {e}")
        return False
    
    print("\n=== PPP Disconnect Complete ===")
    return True

if __name__ == '__main__':
    success = disconnect_ppp()
    sys.exit(0 if success else 1)
