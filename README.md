# Walter IO Board Firmware

**Version:** Rev 10.36
**Date:** February 19, 2026
**Authors:** Todd Adams & Doug Harty

ESP32-based IO board firmware for the Walter cellular modem platform. Controls relays, monitors sensors, provides web configuration portal, and handles CBOR cellular data transmission.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Configuration](#hardware-configuration)
3. [Relay Control & Operating Modes](#relay-control--operating-modes)
4. [Sensor Monitoring](#sensor-monitoring)
5. [Serial Communication Protocol](#serial-communication-protocol)
6. [Web Configuration Portal](#web-configuration-portal)
7. [Wireless Display Panel Support](#wireless-display-panel-support)
8. [Cellular Data Transmission](#cellular-data-transmission)
9. [Safety Systems](#safety-systems)
10. [Revision History](#revision-history)

---

## System Overview

The Walter IO Board connects a Linux controller (Raspberry Pi) to a Walter cellular modem. The system has a clear division of responsibility:

- **Linux (Python)** = The brain. Makes all operating decisions — when to start cycles, when to stop, alarm detection, scheduling, profile management.
- **ESP32** = The body. Reads sensors, controls relays, sends data to the cloud, and reports status to Linux at 5Hz (every 200ms).

The ESP32 only takes autonomous control in **failsafe mode** — when the Linux device has been unresponsive for an extended period.

```
┌──────────────────┐         Serial (JSON)        ┌──────────────────┐
│   Linux Device   │ ◄──────────────────────────► │   ESP32 IO Board │
│  (Raspberry Pi)  │                               │                  │
│                  │  ← Pressure, Current, Overfill │  ADS1015 ADC     │
│  Python Control  │     (5Hz / every 200ms)       │  (60Hz sampling) │
│  Panel (Kivy)    │                               │                  │
│                  │  Mode commands, cycle counts → │  5 Relay Outputs │
│  io_manager.py   │     (every 15 seconds)        │  (Motor, Valves) │
│  modem.py        │                               │                  │
│  alarm_manager.py│  ← Web portal commands        │  Web Portal (SPA)│
│                  │     (on demand)                │  WiFi AP         │
│                  │                               │                  │
│                  │                      LTE ────►│  Walter Modem    │
│                  │                               │  CBOR → Cloud    │
└──────────────────┘                               └──────────────────┘
```

### What Each Serial Direction Carries

| Direction | Frequency | Purpose | Data |
|-----------|-----------|---------|------|
| ESP32 → Linux | **5Hz** (every 200ms) | **CRITICAL** — Real-time sensor data for controller decisions | Pressure (IWC), Current (A), Overfill, SD card, Relay Mode, Failsafe, Shutdown |
| ESP32 → Linux | Fresh data only (~60s) | Supplementary status (only when modem returns new data) | Datetime, LTE, cell signal, profile, failsafe |
| Linux → ESP32 | Every 15 seconds | CBOR payload data for cellular transmission | Device ID, pressure, current, mode, faults, cycles |
| ESP32 → Linux | On demand | Web portal commands forwarded to Python | start_cycle, stop_cycle, start_test, set_profile |

**Important:** The CBOR builder uses the ESP32's own ADC readings for pressure and current — not the values from the Linux message. This ensures cellular data is always real-time regardless of the 15-second Linux send rate.

---

## Sensor Monitoring (ADS1015 ADC)

The ESP32 reads an ADS1015 12-bit ADC at I2C address 0x48. A dedicated FreeRTOS task on Core 0 reads the ADC at 60Hz.

| Channel | Measurement | Rolling Average | Update Rate |
|---------|-------------|-----------------|-------------|
| Channel 0 | Pressure (single-ended) | 60 samples = 1.0 second | Every 16ms |
| Channels 2 & 3 | Current (differential) | 20 samples = 0.33 second | Every 16ms |

The rolling averages smooth out electrical noise while still reflecting real sensor changes within 1 second. These values are sent to the Linux device at 5Hz (every 200ms) in the fast sensor packet.

---

## Relay Control & Operating Modes

The ESP32 controls 5 relay outputs based on mode commands received from the Linux device via serial JSON. Each mode sets a specific combination of relays atomically (all at once, not one-by-one).

| Mode | Number | CR0_MOTOR | CR1 | CR2 | CR5 | Purpose |
|------|--------|-----------|-----|-----|-----|---------|
| Idle | 0 | OFF | OFF | OFF | OFF | Standby |
| Run | 1 | ON | ON | OFF | ON | Normal operation |
| Purge | 2 | ON | OFF | ON | OFF | Exhaust cycle |
| Burp | 3 | OFF | OFF | OFF | ON | Quick exhaust |
| Fresh Air | 8 | OFF | OFF | ON | ON | Bleed / fresh air intake |
| Leak Test | 9 | OFF | ON | ON | ON | Leak test |

**DISP_SHUTDN (GPIO13)** is managed separately and is NOT affected by mode changes. It stays HIGH (ON) unless explicitly commanded LOW via `{"mode":"shutdown"}` serial command. A 20-second startup lockout prevents accidental shutdown during power cycles.

---

## Hardware Configuration

### ESP32 Pin Assignments

| Function | GPIO | Description |
|----------|------|-------------|
| CR0_MOTOR | 11 | Relay 0 — Motor control |
| CR1 | 18 | Relay 1 — Valve 1 |
| CR2 | 17 | Relay 2 — Valve 2 |
| CR5 | 16 | Relay 3 — Valve 5 |
| DISP_SHUTDN | 13 | Relay 4 — Shutdown control |
| ESP_OVERFILL | 38 | Overfill sensor input (active LOW, internal pull-up) |
| ESP_WATCHDOG | 39 | Serial watchdog pulse output |
| I2C SDA | 4 | I2C master to ADS1015 ADC |
| I2C SCL | 5 | I2C master clock |
| Serial1 RX | 44 | RS-232 receive from Linux |
| Serial1 TX | 43 | RS-232 transmit to Linux |
| WS2812B LED | 15 | Status LED |

### Serial Configuration

| Port | Baud | Use |
|------|------|-----|
| Serial | 115200 | Debug output (USB) |
| Serial1 | 115200 | Linux communication (RS-232) |
| ModemSerial | 115200 | Walter cellular modem |

### I2C Configuration (Master Mode)

| Parameter | Value |
|-----------|-------|
| Address | 0x48 (ADS1015 ADC) |
| SDA/SCL | GPIO 4 / GPIO 5 |
| Bus Speed | 100 kHz |

> **Note:** The ESP32 is the I2C master for ADS1015 ADC communication. Serial communication handles all control commands from the Linux device.

---

## Serial Communication Protocol

### Fast Sensor Packet (ESP32 → Linux, 5 times per second)

```json
{"pressure":-14.22,"current":0.07,"overfill":0,"sdcard":"OK","relayMode":1,"failsafe":0,"shutdown":0}
```

Sent at 5Hz (every 200ms). This is the **critical** data path. The Linux controller uses these values to make real-time decisions about cycle starts, stops, alarms, and safety shutdowns. Rev 10.8 added `failsafe` and `shutdown` fields for system state visibility.

### Cellular Status Packet (ESP32 → Linux, every 10 seconds)

```json
{
  "passthrough": 0,
  "lte": 1,
  "rsrp": "-85.5",
  "rsrq": "-10.2",
  "operator": "T-Mobile",
  "band": "12",
  "mcc": 310, "mnc": 260, "cellId": 12345, "tac": 678,
  "profile": "CS2",
  "imei": "351234567890123",
  "imsi": "310410123456789",
  "iccid": "8901260882310000000",
  "technology": "LTE-M",
  "espnow": 0,
  "temperature": 85.2
}
```

**Rev 10.36: Added ESP32 temperature monitoring.** Sent on a 10-second timer using cached values from the last successful modem query. Rev 10.9 added IMEI/IMSI/ICCID/technology fields. `datetime` and `failsafe` removed — datetime sent separately only when fresh; failsafe in 5Hz fast sensor packet.

### Datetime Packet (ESP32 → Linux, only when fresh from modem)

```json
{"datetime": "2026-02-19 13:23:51"}
```

**Only sent when `modemTimeFresh` flag is set** — i.e., when the modem clock returns a genuinely new timestamp. This prevents stale/frozen datetime values. Python parser handles this standalone packet via `if "datetime" in data`.

### CBOR Data Payload (Linux → ESP32, every 15 seconds)

```json
{"type":"data","gmid":"RND-0003","press":-14.22,"mode":1,"current":0.07,"fault":0,"cycles":484}
```

This data is used **only** for building CBOR payloads for cellular transmission. The ESP32's CBOR builder uses its own ADC values for pressure and current.

### Web Portal Command Forwarding (ESP32 → Linux, on demand)

```json
{"command":"start_cycle","type":"run"}
{"command":"stop_cycle"}
{"command":"start_test","type":"leak"}
```

---

## Web Configuration Portal

The ESP32 creates a WiFi Access Point for a full-featured Single Page Application (SPA):

**SSID:** `GM IO Board XXXXXX` (uses device ID after setup)  
**Password:** None (open network)  
**IP:** 192.168.4.1

### Screens and Features

| Screen | Features |
|--------|----------|
| **Main** | Live pressure, current, mode, cycle count. Start/Stop cycle buttons. |
| **Alarms** | Active alarm indicators with alarm descriptions. |
| **Maintenance** | Password-protected (PW: 878). Clear alarms, run tests, passthrough, diagnostics. |
| **Tests** | Leak Test, Functionality Test, Efficiency Test — each with Start/Stop and elapsed timer. |
| **Clean Canister** | 15-minute motor clean cycle with countdown. |
| **Settings** | Manual Mode, Profiles, Overfill Override, About, Configuration, Reboot. |
| **Manual Mode** | Manual Purge with live sensor readout. |
| **Profiles** | Select and confirm profile change (PW: 1793, validated server-side). |
| **Configuration** | Device name (PW: gm2026), serial watchdog toggle. |
| **Diagnostics** | Serial data stream, cellular info, SD card, uptime, MAC address. |

### Security

| Area | Password | Validation |
|------|----------|------------|
| Maintenance screen | 878 | Client-side (session-locked) |
| Profile changes | 1793 | Server-side (ESP32 validates) |
| Configuration changes | gm2026 | Server-side (ESP32 validates) |

### Command Flow

All web portal buttons that control the system (Start Cycle, tests, etc.) are forwarded to the Linux device via serial when connected. The Linux Python program handles the full cycle with proper alarm monitoring, database logging, and step timing. Only in failsafe mode does the ESP32 run cycles autonomously.

---

## Remote Access & Troubleshooting

The system includes several ways to access and troubleshoot the Green Machine remotely, ensuring it can be monitored and maintained even when you're not at the gas station.

### Cellular Data Transmission

Every Green Machine sends performance data to the cloud automatically:

- **Real-time monitoring**: Pressure, current, and operating status
- **Performance tracking**: Cycle counts and system uptime
- **Fault reporting**: Automatic alerts for problems or maintenance needs
- **Data frequency**: Updates sent every 15 seconds during operation

### Remote Troubleshooting Access

Technicians can connect remotely to diagnose issues:

- **PPP passthrough mode**: Creates a secure remote connection for troubleshooting
- **Web portal access**: Full system control and diagnostics through cellular modem
- **Command execution**: Remote commands for testing and configuration
- **Log access**: System event logs and diagnostic information

### Failsafe Protection

The system includes automatic protection if the main control computer fails:

- **Watchdog monitoring**: Constantly checks if the main computer is responding
- **Automatic alerts**: Notifies when communication is lost
- **Limited autonomous operation**: Can continue basic vapor recovery if enabled
- **Safe shutdown**: Prevents unsafe operation during communication failures

### Wireless Display Panels

As mentioned earlier, wireless display panels provide local access without needing to connect to the main control computer. This is especially useful for:

- **Quick status checks** during fuel deliveries
- **Emergency shutdown** if needed
- **Visual confirmation** that the system is operating properly
- **Temporary control** if the main computer interface is unavailable

### Cloud-Based Updates

The system can receive firmware updates and configuration changes remotely:

- **Automatic updates**: Scheduled maintenance windows for software updates
- **Remote configuration**: System settings can be adjusted without site visits
- **Performance optimization**: Updates to improve vapor recovery efficiency
- **Security patches**: Protection against potential vulnerabilities

---

## Cellular Data Transmission

The ESP32 transmits operational data to cloud services via the Walter cellular modem using CBOR encoding.

### CBOR Data Format

Readings are batched in groups of 12 and sent via CoAP/CBOR. Each reading contains:

| Index | Field | Source | Notes |
|-------|-------|--------|-------|
| 0 | Device ID | Extracted from `gmid` | e.g., "CSX-1234" → 1234 |
| 1 | Sequence | ESP32 local counter | Incremented per reading |
| 2 | Pressure x100 | ESP32 ADS1015 ADC | Real-time vacuum measurement |
| 3 | Cycles | Linux `cycles` field | Round-tripped from Linux |
| 4 | Faults | Linux `fault` + ESP32 faults | Additive fault codes |
| 5 | Mode | ESP32 `currentRelayMode` | Current relay state |
| 6 | Temperature x100 | ESP32 internal temp sensor | Board temperature in °F |
| 7 | Current x100 | ESP32 ADS1015 ADC | Real-time motor current |

**Key point:** Pressure, current, and mode in CBOR data come from ESP32's own sensors and state — not from Linux round-trip values. This ensures cellular data is always real-time.

### Transmission Triggers

- **Time-based**: Every 15 seconds during operation
- **Manual**: Via web portal "Send Status" button
- **Failsafe**: Continues during autonomous operation
- **Backfill**: After PPP passthrough sessions

### Cloud Integration

- **BlueCherry Platform**: IoT cloud service for remote monitoring
- **OTA Updates**: Firmware updates scheduled during maintenance windows
- **Remote Commands**: Configuration and control via cellular commands
- **Fast Polling**: 15-second update intervals for urgent monitoring (60 min max)

---


### Safety Systems

#### Overfill Protection
The system includes multiple layers of protection against liquid gasoline entering the vapor recovery system:

- **Multi-point verification**: Requires multiple sensor readings to trigger
- **Startup delay**: Prevents false alarms during system startup
- **Automatic shutdown**: Safely stops operation if overfill detected

#### Fault Detection and Reporting

The system monitors for various problems and reports them with specific codes:

| Code | Problem Type | Description |
|------|-------------|-------------|
| 1024 | Storage | IO Board SD card memory failure |
| 2048 | Communication | Comfile not responding |
| 4096 | Failsafe Mode | Failsafe mode enabled |
| 8192 | Network | Bluecherry Cloud connection lost |
| 16384 | Passthrough Mode | PPP Modem Connection

Fault codes help technicians quickly identify and resolve issues. The system continues operating safely even when some faults are present, while alerting operators to problems requiring attention.

### Remote Maintenance

Many maintenance tasks can be performed remotely:

- **Software updates**: Firmware updates delivered over cellular
- **Configuration changes**: System settings adjusted remotely
- **Diagnostic data**: Performance logs reviewed by technicians
- **Remote troubleshooting**: Direct system access when needed

---

## Recent Updates

### Version 10.36 - February 19, 2026

**Major New Features:**
- **Wireless Display Panel Support**: Added ESPNow wireless communication for remote monitoring and control of vapor recovery systems
- **Automatic Discovery**: Display panels automatically find and connect to Green Machine systems within 100 feet
- **Temperature Monitoring**: Added ESP32 board temperature to cellular status reports for thermal monitoring
- **Enhanced Communication**: Support for both wireless display panels and traditional serial connections

**Technical Improvements:**
- **Fragmentation Support**: Large data packets automatically split for reliable wireless transmission
- **Priority Communication**: Wireless displays take priority when connected, serial when not
- **Heartbeat Monitoring**: Automatic detection and reconnection for wireless display panels
- **Thermal Diagnostics**: Continuous monitoring of system temperature for preventive maintenance

**User Benefits:**
- **Convenient Monitoring**: Check vapor recovery status from anywhere on the gas station property
- **Easy Setup**: Display panels connect automatically - no complex configuration required
- **Backup Control**: Wireless displays provide alternative control if main systems need service
- **Visual Confirmation**: Clear indicators show when vapor capture is working properly

### Previous Versions

| Version | Date | Key Improvements |
|---------|------|------------------|
| **10.35** | **2/20/2026** | **Non-blocking firmware overhaul with enhanced fault code corrections** |
| **10.34** | **2/17/2026** | **Web portal command isolation and ICCID status integration** |
| **10.32** | **2/17/2026** | **Serial command reliability improvements and fast-path diagnostics** |
| **10.29** | **2/16/2026** | **Profile-specific fault code mapping system** |
| **10.28** | **2/16/2026** | **Profile-aware alarm system and enhanced web diagnostics** |
| **10.13** | **2/11/2026** | **Non-blocking cellular boot sequence** - System becomes operational in ~3 seconds |

### System Architecture Evolution

The Walter IO Board has evolved from basic relay control to a comprehensive IoT platform:

- **Rev 9.x**: MCP23017 I2C slave emulation for relay control
- **Rev 10.0-10.28**: ESP32 I2C master, serial JSON control, web portal, failsafe mode
- **Rev 10.29-10.35**: Enhanced diagnostics, safety systems, and cloud integration
- **Rev 10.36+**: Wireless display panel support and thermal monitoring

Each version maintains backward compatibility while adding new capabilities to improve vapor recovery effectiveness and system reliability.

---

## Documentation and Resources

### User Guides
- **COMMUNICATION_PROTOCOL_GUIDE.md**: Complete communication protocol reference for serial, wireless, and web interfaces
- **IO_Board_Firmware_Feature_Summary.md**: Detailed feature overview and capabilities
- **Linux_Device_Setup_Guide.md**: Installation and configuration guide

### Technical Documentation
- **IO_BOARD_FIRMWARE_SERIAL.ino**: Main ESP32 firmware source code
- **BlueCherryZTP.cpp/h**: Cloud integration and CBOR encoding
- **ESP_AsyncDNSServer/**: Web portal captive portal libraries

### Legacy Documentation
- **MCP23017_COMPATIBILITY.md**: Historical reference for Rev 9.x systems
- **WATCHDOG_FEATURE_SUMMARY.md**: Serial watchdog implementation details
- **THREAD_SAFETY_ISSUE_REPORT.md**: Historical bug analysis and fixes

### Support and Contact

For technical support, installation assistance, or questions about the Green Machine Emissions Control System:

- **Documentation**: All guides are included in the firmware package
- **Web Interface**: Built-in diagnostics and configuration tools
- **Cloud Monitoring**: Remote system health and performance tracking
- **Wireless Support**: Display panels provide local troubleshooting capabilities

The system is designed for reliable, long-term operation with minimal maintenance requirements and comprehensive remote monitoring capabilities.

---

## Files

| File | Description |
|------|-------------|
| `IO_BOARD_FIRMWARE_SERIAL.ino` | Main ESP32 firmware |
| `BlueCherryZTP.cpp/h` | BlueCherry cloud integration |
| `BlueCherryZTP_CBOR.cpp/h` | CBOR encoding for BlueCherry |
| `ESP_AsyncDNSServer/` | Captive portal DNS library |
| `README.md` | This file |
| `COMMUNICATION_PROTOCOL_GUIDE.md` | Complete serial JSON protocol reference (Rev 10+) |
| `MCP23017_COMPATIBILITY.md` | Historical reference (Rev 9.x emulation — deprecated) |
| `Serial Communication Messages structure outline.md` | Serial JSON message format reference |
| `WATCHDOG_FEATURE_SUMMARY.md` | Serial watchdog implementation details |
| `THREAD_SAFETY_ISSUE_REPORT.md` | Historical: Rev 9.0/9.1 relay control bug analysis |

---

## License

Proprietary — Contact authors for licensing information.
