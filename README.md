# Walter IO Board Firmware

**Version:** Rev 9.3a  
**Date:** February 5, 2026  
**Authors:** Todd Adams & Doug Harty

ESP32-based IO board firmware for the Walter cellular modem platform. Provides relay control, sensor monitoring, web configuration, and PPP passthrough for remote access via cellular.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Configuration](#hardware-configuration)
3. [MCP23017 I2C Emulation](#mcp23017-i2c-emulation)
4. [Relay Control & Pin Mapping](#relay-control--pin-mapping)
5. [False Alarm Prevention](#false-alarm-prevention)
6. [Serial Communication Protocol](#serial-communication-protocol)
7. [PPP Passthrough Mode](#ppp-passthrough-mode)
8. [Web Configuration Portal](#web-configuration-portal)
9. [BlueCherry Cloud Integration](#bluecherry-cloud-integration)
10. [Python Module (modem.py)](#python-module-modempy)
11. [Fault Codes](#fault-codes)
12. [Quality Check Test Mode](#quality-check-test-mode)

---

## System Overview

The Walter IO Board bridges a Linux device (Raspberry Pi) with a Walter cellular modem. The ESP32 acts as:

1. **I2C Slave** - Emulates an MCP23017 I/O expander for the Linux master
2. **Relay Controller** - Drives 5 relay outputs for motor and valve control
3. **Sensor Monitor** - Reads overfill sensor with false alarm prevention
4. **Serial Bridge** - Communicates with Linux device via RS-232 (Serial1)
5. **Web Server** - Provides configuration portal via WiFi AP
6. **Cloud Gateway** - Connects to BlueCherry platform for OTA updates and messaging

```
┌─────────────────┐     I2C (0x20)     ┌─────────────────┐     LTE      ┌─────────────┐
│  Linux Device   │◄──────────────────►│   ESP32 IO      │◄────────────►│  BlueCherry │
│  (Raspberry Pi) │     RS-232         │   Board         │              │   Cloud     │
│                 │◄──────────────────►│                 │              └─────────────┘
│  modem.py       │    GPIO44/43       │  Walter Modem   │
└─────────────────┘                    └─────────────────┘
                                              │
                                              ▼
                                       ┌─────────────┐
                                       │   Relays    │
                                       │  (5 outputs)│
                                       └─────────────┘
```

---

## Hardware Configuration

### ESP32 Pin Assignments

| Function | GPIO | Description |
|----------|------|-------------|
| I2C SDA | 4 | I2C data line to Linux master |
| I2C SCL | 5 | I2C clock line |
| CR0_MOTOR | 11 | Relay 0 - Vacuum pump motor |
| CR1 | 18 | Relay 1 |
| CR2 | 17 | Relay 2 |
| CR5 | 16 | Relay 3 |
| DISP_SHUTDN | 13 | Relay 4 - Site shutdown (V6) - **CRITICAL** |
| ESP_OVERFILL | 38 | Overfill sensor input (active LOW) |
| ESP_WATCHDOG | 39 | Serial watchdog pulse output |
| Serial1 RX | 44 | RS-232 receive from Linux |
| Serial1 TX | 43 | RS-232 transmit to Linux |
| 3.3V Power EN | GPIO varies | Enables 3.3V rail |

### Serial Configuration

| Port | Baud | Use |
|------|------|-----|
| Serial | 115200 | Debug output (USB) |
| Serial1 | 115200 | Linux communication (RS-232) |
| ModemSerial | 115200 | Walter cellular modem |

---

## MCP23017 I2C Emulation

The ESP32 emulates a Microchip MCP23017 16-bit I/O expander at I2C address **0x20**. This allows the Linux device to control relays and read sensors using standard I2C libraries (smbus2, Adafruit_MCP23017, etc.).

### Supported Registers (BANK=0 Mode)

| Address | Register | Description |
|---------|----------|-------------|
| 0x00 | IODIRA | I/O Direction Port A (0=output, 1=input) |
| 0x01 | IODIRB | I/O Direction Port B |
| 0x02-0x03 | IPOLA/B | Input polarity |
| 0x04-0x05 | GPINTENA/B | Interrupt-on-change enable |
| 0x06-0x09 | DEFVAL/INTCON | Interrupt configuration |
| 0x0A-0x0B | IOCON | Configuration register |
| 0x0C-0x0D | GPPUA/B | Pull-up resistor enable |
| 0x0E-0x0F | INTFA/B | Interrupt flags |
| 0x10-0x11 | INTCAPA/B | Interrupt capture |
| **0x12** | **GPIOA** | **Port A - Relay outputs** |
| **0x13** | **GPIOB** | **Port B - Overfill input** |
| 0x14-0x15 | OLATA/B | Output latch |

### Port A - Relay Outputs (GPIOA, 0x12)

| Bit | MCP Pin | ESP32 GPIO | Relay | Function |
|-----|---------|------------|-------|----------|
| 0 | GPA0 | 11 | CR0_MOTOR | Vacuum pump |
| 1 | GPA1 | 18 | CR1 | Valve 1 |
| 2 | GPA2 | 17 | CR2 | Valve 2 |
| 3 | GPA3 | 16 | CR5 | Valve 5 |
| 4 | GPA4 | 13 | DISP_SHUTDN | Site shutdown (V6) |
| 5 | GPA5 | - | (reserved) | PPP early exit trigger |
| 6-7 | GPA6-7 | - | (unused) | - |

### Port B - Inputs (GPIOB, 0x13)

| Bit | MCP Pin | ESP32 GPIO | Function |
|-----|---------|------------|----------|
| 0 | GPB0 | 38 | Overfill sensor (active LOW) |
| 1-7 | GPB1-7 | - | (unused) |

### Linux I2C Example (Python)

```python
import smbus2

bus = smbus2.SMBus(1)
MCP_ADDR = 0x20
GPIOA = 0x12
GPIOB = 0x13

# Set all relays OFF
bus.write_byte_data(MCP_ADDR, GPIOA, 0x00)

# Turn on Motor (bit 0) and CR1 (bit 1)
bus.write_byte_data(MCP_ADDR, GPIOA, 0x03)

# Read overfill sensor
overfill = bus.read_byte_data(MCP_ADDR, GPIOB)
if overfill & 0x01:
    print("OVERFILL ALARM!")
```

### Thread Safety

The MCP emulator runs on **Core 0** as a FreeRTOS task, while the main application runs on **Core 1**. A mutex protects shared relay state:

- I2C callbacks use 1ms timeout (must respond quickly)
- Web/serial requests use 10ms timeout
- If mutex unavailable, operation proceeds with atomic volatile access

---

## Relay Control & Pin Mapping

### Operating Modes

The firmware supports predefined relay patterns for different operating modes:

| Mode | Value | Relays Active | Bitmask |
|------|-------|---------------|---------|
| STANDBY | 0 | None | 0x00 |
| RUN | 1 | Motor + CR1 + CR5 | 0x0B |
| PURGE | 2 | Motor + CR2 | 0x05 |
| BURP | 3 | CR5 only | 0x08 |

### DISP_SHUTDN (V6) - Critical Safety Relay

The DISP_SHUTDN relay (GPA4, GPIO13) controls site shutdown and requires special protection:

**Problem:** During power cycles and firmware flashes, the GPIO can momentarily go LOW, causing unintended shutdown.

**Solution:** 20-second startup lockout:
- Relay forced HIGH (ON) for first 20 seconds after boot
- All MCP and web commands to turn it OFF are ignored during lockout
- After lockout, normal control resumes
- Status logged to Serial Monitor for diagnostics

```
[DISP_SHUTDN] ⚡ Protection ACTIVE - relay FORCED ON for 20 seconds
[DISP_SHUTDN] ✓ Protection expired - MCP/web control enabled
```

---

## False Alarm Prevention

### Overfill Sensor Validation

The overfill sensor (GPIO38) is active-LOW with internal pull-up. Without proper validation, transient signals during startup can cause false alarms.

**Multi-Layer Protection:**

1. **Startup Lockout (15 seconds)**
   - Ignores all readings for first 15 seconds after boot
   - Allows GPIO and pull-up to fully stabilize
   - Returns "Normal" status during lockout

2. **High Trigger Threshold**
   - Requires **8 consecutive LOW readings** (1 per second) to trigger alarm
   - Prevents transient noise from causing false alarms

3. **Hysteresis for Clearing**
   - Requires **5 consecutive HIGH readings** to clear alarm
   - Prevents rapid alarm toggling on noisy signals

4. **Counter Capping**
   - Counters capped at 100 to prevent overflow
   - State machine is deterministic

### Timing Diagram

```
Boot ──────────────────────────────────────────────────────────────►
     │◄─── 15s lockout ───►│◄─── Monitoring Active ────────────────►
     
Sensor:  ???  ???  HIGH HIGH HIGH LOW LOW LOW LOW LOW LOW LOW LOW HIGH HIGH
Status:  [----LOCKOUT----]  NORMAL NORMAL NORMAL ──counting──  ALARM  ──clear──
                                              (8x LOW = ALARM)     (5x HIGH = CLEAR)
```

### Serial Monitor Output

```
[Overfill] Validation system starting - 15 second lockout
[Overfill] ✓ Validation complete - monitoring enabled
[Overfill] Current GPIO38 state: HIGH (normal)

╔═══════════════════════════════════════════════════════╗
║  🚨 OVERFILL ALARM ACTIVATED                          ║
║  8 consecutive LOW readings on GPIO38                 ║
╚═══════════════════════════════════════════════════════╝

╔═══════════════════════════════════════════════════════╗
║  ✅ OVERFILL ALARM CLEARED                            ║
║  5 consecutive HIGH readings on GPIO38                ║
╚═══════════════════════════════════════════════════════╝
```

---

## Serial Communication Protocol

### ESP32 → Linux (every 15 seconds)

```json
{
  "datetime": "2026-02-05 14:30:00",
  "sdcard": "OK",
  "passthrough": 0,
  "lte": 1,
  "rsrp": "-85.5",
  "rsrq": "-10.2"
}
```

| Field | Type | Description |
|-------|------|-------------|
| datetime | string | UTC timestamp from modem |
| sdcard | string | "OK" or "FAULT" |
| passthrough | int | 0=normal, 1=passthrough active |
| lte | int | 1=LTE connected, 0=not connected |
| rsrp | string | Signal strength in dBm |
| rsrq | string | Signal quality in dB |

### Linux → ESP32 (sensor data)

```json
{
  "type": "data",
  "gmid": "CSX-1234",
  "press": -14.22,
  "mode": 0,
  "current": 0.07,
  "fault": 0,
  "cycles": 484
}
```

### Signal Quality Interpretation

| RSRP (dBm) | Quality |
|------------|---------|
| >= -80 | Excellent |
| -80 to -90 | Good |
| -90 to -100 | Fair |
| < -100 | Poor |

---

## PPP Passthrough Mode

Passthrough mode creates a direct serial bridge between the Linux device and the cellular modem, enabling PPP connections for remote access.

### Architecture (Preference-Based Boot)

When passthrough is requested, the ESP32:
1. Stores passthrough flag and timeout in non-volatile preferences
2. Restarts immediately
3. On boot, checks preference **BEFORE** loading WalterModem library
4. If set, runs minimal passthrough mode (no WiFi, no web server)
5. Clears preference immediately (prevents getting stuck)

### Minimal Passthrough Mode

Only these components run during passthrough:
- 3.3V power enabled
- MCP23017 emulator (I2C still works)
- Serial bridge: Serial1 ↔ ModemSerial
- Safe relay states (DISP_SHUTDN = ON)

**NOT running:** WiFi AP, web server, WalterModem library, BlueCherry

### Triggering Passthrough

**From BlueCherry (remote):**
```
Message containing "remote 60" → 60 minute passthrough
Message containing "remote" → 60 minute default
```

**From Linux (modem.py):**
```python
manager.connect_ppp(30)  # 30 minute session
```

**From Web Interface:**
Click "Enter Passthrough Mode" button

### Exit Methods

1. **Timeout** - Auto-restarts after specified minutes (default 60, max 1440)
2. **Serial Escape** - Linux sends `+++STOPPPP` to trigger restart
3. **I2C Command** - Set MCP GPA5 (bit 5) HIGH via I2C
4. **Power cycle** - Always boots to normal mode

### Disconnecting PPP from SSH

When logged in via SSH during an active PPP session, you can disconnect PPP using:

**Method 1: Signal (Recommended)**
```bash
# modem.py writes its PID to /tmp/modem_manager.pid
kill -USR1 $(cat /tmp/modem_manager.pid)
```

**Method 2: Standalone Script**
```bash
sudo python3 ppp_disconnect.py
```

**Method 3: Manual Commands**
```bash
# Kill pppd
sudo pkill pppd

# Send stop signal to ESP32
echo "+++STOPPPP" | sudo tee /dev/ttyAMA0
```

The disconnect process:
1. Terminates pppd (SIGTERM, then SIGKILL if needed)
2. Waits for serial port to be released
3. Sends `+++STOPPPP` escape sequence to ESP32
4. ESP32 restarts to normal mode
5. modem.py automatically reconnects

### Linux PPP Setup

```bash
# /etc/ppp/peers/walter
/dev/serial0
115200
noauth
defaultroute
usepeerdns
persist
nodetach
debug
```

```bash
# /etc/chatscripts/walter
ABORT "NO CARRIER"
ABORT "ERROR"
TIMEOUT 30
"" AT
OK AT+CGDCONT=1,"IP","soracom.io"
OK ATD*99#
CONNECT ""
```

---

## Web Configuration Portal

The ESP32 creates a WiFi Access Point for configuration:

**SSID:** `GM IO Board XXXXXX` (random hex on first boot, then device ID)  
**Password:** None (open network)  
**IP:** 192.168.4.1

### Features

- **Real-time status** via AJAX (no page refresh)
- **Relay control** - Individual or mode-based
- **Passthrough toggle** - Enter/exit PPP mode
- **Watchdog control** - Enable/disable serial timeout
- **Device naming** - Set custom device ID
- **Quality Check mode** - Password-protected (1793) testing features

### Captive Portal

The firmware responds to captive portal detection from iOS/Android/Windows, automatically redirecting to the configuration page.

---

## BlueCherry Cloud Integration

The firmware connects to the BlueCherry IoT platform via the Walter modem for:

- **OTA firmware updates**
- **Remote commands** ("remote", "restart")
- **Status reporting** via CBOR

### Non-Fatal Operation

BlueCherry connection failures don't crash the system:
- Retries hourly
- Weekly restart if never connected
- Fault code 4096 indicates offline status
- Local operation continues normally

### Remote Commands

| Message Contains | Action |
|-----------------|--------|
| "remote" or "remote XX" | Enter passthrough for XX minutes |
| "restart" | Perform ESP.restart() |

---

## Python Module (modem.py)

The `modem.py` module runs on the Linux device and handles serial communication with the ESP32.

### Installation

```python
# Requires pyserial
pip install pyserial
```

### Key Features

```python
from modem import SerialManager

# Initialize
manager = SerialManager(data_handler)
manager.start()

# Check ESP32 status
status = manager.get_esp32_status()
# Returns: {'datetime': '...', 'sdcard': 'OK', 'lte_connected': True, 
#           'rsrp': -85.5, 'rsrq': -10.2, 'signal_quality': 'Good', ...}

# Check signal quality
signal = manager.get_signal_quality()
# Returns: {'rsrp': -85.5, 'rsrq': -10.2, 'quality': 'Good', 'lte_connected': True}

# Start PPP passthrough (30 minute session)
manager.connect_ppp(30)

# Stop PPP and return to normal
manager.disconnect_ppp()

# Check if LTE connected
if manager.is_lte_connected():
    print("Cellular connection active")
```

### PPP Connection Flow

1. `connect_ppp(timeout)` sends `{"command":"passthrough","timeout":XX}` to ESP32
2. ESP32 stores preference and restarts
3. modem.py waits, then starts `sudo pppd call walter nodetach debug`
4. Background thread monitors timeout
5. On timeout or `disconnect_ppp()`, stops pppd and sends `+++STOPPPP`
6. ESP32 detects escape sequence and restarts to normal mode

### External Control via Signals

modem.py supports Unix signals for external control from SSH sessions:

| Signal | Action |
|--------|--------|
| SIGUSR1 | Disconnect PPP and return to normal mode |

```bash
# PID file location
/tmp/modem_manager.pid

# Disconnect PPP from another terminal
kill -USR1 $(cat /tmp/modem_manager.pid)
```

This is useful when you're logged in via SSH during a PPP session and need to disconnect programmatically.

---

## Fault Codes

Fault codes are additive and reported in the status:

| Code | Description |
|------|-------------|
| 1024 | Serial watchdog triggered (no data for 30+ minutes) |
| 4096 | BlueCherry platform offline |
| 5120 | Both watchdog AND BlueCherry faults |

These codes are **added** to any existing fault codes from the Linux device.

### Serial Watchdog

- Monitors Serial1 for incoming JSON data
- If no valid data for 30 minutes, triggers fault
- Pulses GPIO39 HIGH for 1 second
- Configurable via web interface

---

## Quality Check Test Mode

Password-protected mode for factory testing when I2C master not connected.

**Password:** 1793

### Features

- Quick mode switching (STANDBY/RUN/PURGE/BURP)
- Individual relay control
- ADC readings from ADS1015 (when installed)
- Real-time sensor display

### Accessing

1. Open web portal (192.168.4.1)
2. Click "Quality Check Mode" button
3. Enter password: 1793
4. Hidden QC controls become visible

---

## Files

| File | Description |
|------|-------------|
| `IO_BOARD_FIRMWARE9_1-29-26.ino` | Main ESP32 firmware |
| `modem.py` | Linux Python module for serial communication |
| `ppp_disconnect.py` | Standalone script to disconnect PPP from SSH |
| `BlueCherryZTP.cpp/h` | BlueCherry cloud integration |
| `BlueCherryZTP_CBOR.cpp/h` | CBOR encoding for BlueCherry |
| `ESP_AsyncDNSServer/` | Captive portal DNS library |
| `walter-as-linux-modem.txt` | Reference passthrough implementation |

---

## License

Proprietary - Contact authors for licensing information.
