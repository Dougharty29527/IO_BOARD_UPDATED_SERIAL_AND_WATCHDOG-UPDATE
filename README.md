# Walter IO Board Firmware

**Version:** Rev 10.19
**Date:** February 13, 2026  
**Authors:** Todd Adams & Doug Harty

ESP32-based IO board firmware for the Walter cellular modem platform. Provides relay control, sensor monitoring, web configuration portal, CBOR cellular data transmission, and PPP passthrough for remote access.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Hardware Configuration](#hardware-configuration)
3. [Relay Control & Operating Modes](#relay-control--operating-modes)
4. [Sensor Monitoring (ADS1015 ADC)](#sensor-monitoring-ads1015-adc)
5. [Serial Communication Protocol](#serial-communication-protocol)
6. [Web Configuration Portal](#web-configuration-portal)
7. [Failsafe Mode](#failsafe-mode)
8. [PPP Passthrough Mode](#ppp-passthrough-mode)
9. [BlueCherry Cloud Integration](#bluecherry-cloud-integration)
10. [Overfill Safety System](#overfill-safety-system)
11. [Fault Codes](#fault-codes)
12. [Revision History](#revision-history)

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

## Hardware Configuration

### ESP32 Pin Assignments

| Function | GPIO | Description |
|----------|------|-------------|
| CR0_MOTOR | 11 | Relay 0 — Vacuum pump motor |
| CR1 | 18 | Relay 1 — Valve 1 |
| CR2 | 17 | Relay 2 — Valve 2 |
| CR5 | 16 | Relay 3 — Valve 5 |
| DISP_SHUTDN | 13 | Relay 4 — Site shutdown (V6) **CRITICAL** |
| ESP_OVERFILL | 38 | Overfill sensor input (active LOW, internal pull-up) |
| ESP_WATCHDOG | 39 | Serial watchdog pulse output |
| I2C SDA | 4 | I2C master to ADS1015 ADC |
| I2C SCL | 5 | I2C master clock |
| Serial1 RX | 44 | RS-232 receive from Linux |
| Serial1 TX | 43 | RS-232 transmit to Linux |
| WS2812B LED | 15 | Status LED (Blue=Idle, Green=Run, Orange=Purge) |

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

> **Note:** The ESP32 is the I2C **master** as of Rev 10.0. The previous MCP23017 I2C slave emulation (Rev 9.x) has been completely removed due to intermittent errors that caused the Python Adafruit library to crash. The Linux device no longer communicates with the ESP32 via I2C — all communication is via serial JSON.

---

## Relay Control & Operating Modes

The ESP32 controls 5 relay outputs based on mode commands received from the Linux device via serial JSON. Each mode sets a specific combination of relays atomically (all at once, not one-by-one).

| Mode | Number | Motor | CR1 | CR2 | CR5 | Purpose |
|------|--------|-------|-----|-----|-----|---------|
| Idle | 0 | OFF | OFF | OFF | OFF | Standby — no operation |
| Run | 1 | ON | ON | OFF | ON | Normal vacuum cycle |
| Purge | 2 | ON | OFF | ON | OFF | Exhaust cycle |
| Burp | 3 | OFF | OFF | OFF | ON | Quick exhaust |
| Fresh Air | 8 | OFF | OFF | ON | ON | Bleed / fresh air intake |
| Leak Test | 9 | OFF | ON | ON | ON | Leak test (no motor) |

**DISP_SHUTDN (V6)** is managed separately and is NOT affected by mode changes. It stays HIGH (ON) unless explicitly commanded LOW via `{"mode":"shutdown"}` serial command. A 20-second startup lockout prevents accidental shutdown during power cycles.

---

## Sensor Monitoring (ADS1015 ADC)

The ESP32 reads an ADS1015 12-bit ADC at I2C address 0x48. A dedicated FreeRTOS task on Core 0 reads the ADC at 60Hz.

| Channel | Measurement | Rolling Average | Update Rate |
|---------|-------------|-----------------|-------------|
| Channel 0 | Pressure (single-ended) | 60 samples = 1.0 second | Every 16ms |
| Channels 2 & 3 | Current (abs differential) | 20 samples = 0.33 second | Every 16ms |

The rolling averages smooth out electrical noise while still reflecting real sensor changes within 1 second. These values (`adcPressure`, `adcCurrent`) are sent to the Linux device at 5Hz (every 200ms) in the fast sensor packet and used directly for CBOR cellular data.

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
  "technology": "LTE-M"
}
```

**Rev 10.9: Sent on a 10-second timer** using cached values from the last successful modem query. Previously this was freshness-gated (only sent when modem returned new data), which meant the Linux device received nothing if modem queries failed silently. `datetime` and `failsafe` removed — datetime is sent separately only when fresh; failsafe is already in the 5Hz fast sensor packet. Rev 10.9 also added `imei`, `imsi`, `iccid`, and `technology` fields (static values queried once at boot).

### Datetime Packet (ESP32 → Linux, only when fresh from modem)

```json
{"datetime": "2026-02-10 13:23:51"}
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

## Failsafe Mode

Failsafe mode is an optional autonomous operation mode that can be enabled/disabled via the web portal or BlueCherry commands.

### Configuration
- **Default:** Failsafe is **disabled** for safety
- **Enable via:** Web Portal Settings → Failsafe Mode toggle (password required)
- **Enable via:** BlueCherry `enable_failsafe` command
- **Disable via:** BlueCherry `exit_failsafe` command

### Watchdog Behavior
When failsafe is **disabled**:
- Watchdog pulses GPIO39 every 10 minutes if no serial data
- Never enters autonomous failsafe operation
- Safe for installations where autonomous operation is not desired

When failsafe is **enabled**:
- Watchdog pulses GPIO39 every 30 minutes if no serial data
- After 2 failed attempts (60 minutes), enters failsafe mode
- ESP32 takes autonomous control using ADC sensors

### Autonomous Operation
When in failsafe mode:
- Runs vacuum cycles based on pressure thresholds
- Uses active profile timing settings
- CBOR data continues sending to cloud
- Web portal remains functional for manual control
- Fault code 16384 indicates active failsafe operation

Normal operation resumes automatically when serial communication is restored.

---

## PPP Passthrough Mode

Passthrough mode creates a direct serial bridge between the Linux device and the cellular modem, enabling PPP connections for remote SSH access.

### How It Works

1. Command received (web portal, BlueCherry, or serial)
2. ESP32 stores flag in non-volatile preferences and restarts
3. On boot, runs minimal passthrough (no WiFi, no web server)
4. Serial1 ↔ ModemSerial bridge enables Linux PPP

### Exit Methods

1. **Timeout** — Auto-restarts after specified minutes (default 60)
2. **Serial Escape** — Linux sends `+++STOPPPP`
3. **Power cycle** — Always boots to normal mode

---

## Data Backfill After Passthrough

When the ESP32 exits passthrough mode, it automatically sends any sensor data collected during the passthrough session to the Linux device for cloud synchronization.

### Why Backfill is Needed

During passthrough mode:
- The ESP32 continues collecting sensor data and logging to SD card at 15-second intervals
**Data Storage:** Sensor data is logged to SD card in CSV format for local storage and retrieval. Cellular transmission uses CBOR format for efficient cloud communication.
- The cellular modem is busy with PPP connections and cannot send CBOR data to the cloud
- This creates a gap in cloud data during remote access sessions

### How Backfill Works

1. **Entry**: When entering passthrough, ESP32 saves current datetime and sets backfill flag in EEPROM
2. **During**: Sensor data continues to be collected and stored on SD card normally
3. **Exit**: After passthrough ends and ESP32 reboots, it checks the backfill flag
4. **Send**: Reads SD card data collected after the passthrough start time and sends as JSON array to Linux
5. **Clear**: Backfill flag is cleared to prevent re-sending on subsequent boots

### Backfill Message Format

```json
{"backfill":[{"pressure":-14.22,"current":0.07,"mode":0,"fault":0,"cycles":484},{"pressure":-14.25,"current":0.06,"mode":1,"fault":0,"cycles":485}]}
```

The Linux device receives this data and forwards it to the cloud with appropriate timestamps added server-side.

---

## BlueCherry Cloud Integration

Connects to the BlueCherry IoT platform via Walter modem for:

- **OTA firmware updates** (scheduled: 7AM–1PM EST, every 15 minutes, or every 15 seconds in fast mode)
- **Remote commands** ("remote XX" for passthrough, "restart", "fast_poll", "exit_fast_poll")
- **CBOR data transmission** (sensor readings, fault codes, cycle counts)
- **Fast polling mode** (15-second intervals for 60 minutes max) for urgent message retrieval

BlueCherry failures are non-fatal — local operation continues normally. Fault code 4096 indicates BlueCherry offline status.

---

## Overfill Safety System

The overfill sensor (GPIO38, active LOW with internal pull-up) uses multi-layer protection to prevent false alarms:

1. **Startup Lockout** — 15 seconds of ignored readings after boot
2. **High Trigger Threshold** — 8 consecutive LOW readings to trigger alarm
3. **Hysteresis for Clearing** — 5 consecutive HIGH readings to clear
4. **Counter Capping** — Capped at 100 to prevent overflow

---

## Fault Codes

| Code | Description |
|------|-------------|
| 1024 | SD card failure |
| 2048 | Serial watchdog triggered (no Linux data for 30+ minutes) |
| 4096 | Failsafe enabled (setting - can enter failsafe if watchdog triggers) |
| 8192 | BlueCherry platform offline |
| 16384 | Failsafe mode active (ESP32 autonomously controlling relays) |

**Examples:**
- Watchdog triggered: `2048`
- Failsafe enabled but not active: `4096`
- Failsafe actively running: `20480` (4096 + 16384)
- All faults active: `31744` (1024 + 2048 + 4096 + 8192 + 16384)

These codes are **added** to any existing fault codes from the Linux device.

---

## Revision History

| Version | Date | Description |
|---------|------|-------------|
| **Rev 10.13** | **2/11/2026** | **Non-blocking cellular boot: setup() completes in ~3 seconds. LTE registration, BlueCherry, modem identity, and modem time run as background state machine in loop(). No more ESP.restart() on LTE failure during boot. Web dashboard shows "Connecting..." during cellular init. All IO functions (serial, relays, web portal, watchdog) operational immediately.** |
| Rev 10.12 | 2/11/2026 | Functionality test reworked from incorrect mode 9 to proper 10x (Run 60s → Purge 60s) multi-step cycle. Watchdog pin (GPIO39) internal pull-down. Device name fix: web portal name no longer overwritten by Linux gmid. |
| Rev 10.11 | 2/11/2026 | ADC stale data detection: sends pressure=-99.9 sentinel after 60s of failed reads. DISP_SHUTDN GPIO hold survives reboots/OTA. Web portal tests: ESP32 takes sole relay control, blocks Linux mode commands. |
| Rev 10.10 | 2/10/2026 | ADC I2C error diagnostics: loud ###-bordered Serial Monitor alerts for pressure/current read failures. Three-tier current validation (NACK, full-scale rail, abnormal). Invalid reads skipped to protect rolling average. Reinit and reconnect attempts now logged. Always-on (not gated by debug mode). |
| Rev 10.9 | 2/10/2026 | Serial debug mode (toggle via web portal, EEPROM-persisted). IMEI/IMSI/ICCID/Technology added to cellular JSON. Cellular packet now 10-second timer. Datetime split into own packet. Calibration safety: middle 20% ADC range. EEPROM validates on boot. ADC outlier rejection. |
| Rev 10.8 | 2/10/2026 | Mode confirmation: 15-second relay state refresh re-applies currentRelayMode to GPIO pins. Fast sensor packet extended with failsafe and shutdown fields. Three-layer mode delivery redundancy (immediate + 15s payload + relay refresh). Zero Python changes required. |
| Rev 10.7 | 2/9/2026 | Pressure sensor calibration via serial or web portal. Instant non-blocking zero point adjustment using existing 60-sample rolling average. EEPROM persistence. ESP32 sends ps_cal result to Linux for database save. New "type":"cmd" message type. |
| Rev 10.6 | 2/9/2026 | Fixed pressure sign (vacuum now negative). Fixed current stuck at 0A (hardware differential mode). Added `{"mode":"normal"}` to clear 72-hour shutdown without reboot. |
| Rev 10.5 | 2/9/2026 | Sensor data now sent at 5Hz (was 1Hz). Cellular/datetime sent only on fresh modem data (was every 15s). |
| Rev 10.4 | 2/9/2026 | Fixed web portal tests/cycles not starting (Python screen guard bypass). Full button audit. |
| Rev 10.3 | 2/9/2026 | Faster pressure updates (200→60 sample rolling average = 1 second window). |
| Rev 10.2 | 2/8/2026 | Fixed stale CBOR pressure. ESP32 CBOR now uses own ADC, not round-tripped Python values. Web test handlers added. |
| Rev 10.1 | 2/8/2026 | Performance: serial read 5s→100ms. Password-protected web portal (Maintenance 878, Profile 1793). |
| Rev 10.0 | 2/8/2026 | **MAJOR REWRITE:** MCP23017 emulation removed. ESP32 is now I2C master for ADS1015 ADC. Serial JSON relay control. Failsafe mode. Full SPA web portal. |
| Rev 9.4f | 2/6/2026 | MCP23017 emulator rewrite, RGB LED, web portal enhancements (final Rev 9 release). |

### What Changed from Rev 9 to Rev 10

| Feature | Rev 9.x | Rev 10.x |
|---------|---------|----------|
| ESP32 I2C role | Slave (emulating MCP23017) | Master (reading ADS1015 ADC) |
| Relay control | Linux writes I2C registers | Linux sends serial JSON mode commands |
| Sensor reading | Linux reads I2C ADC directly | ESP32 reads ADC, sends values via serial |
| Web portal | Read-only diagnostic dashboard | Full SPA with cycle/test/profile control |
| Failsafe | None | Autonomous cycles when Linux is down |
| Python I2C libraries | Required (adafruit_mcp230xx, ads1x15) | Not needed — removed |

---

## Files

| File | Description |
|------|-------------|
| `IO_BOARD_FIRMWARE9_1-29-26.ino` | Main ESP32 firmware |
| `BlueCherryZTP.cpp/h` | BlueCherry cloud integration |
| `BlueCherryZTP_CBOR.cpp/h` | CBOR encoding for BlueCherry |
| `ESP_AsyncDNSServer/` | Captive portal DNS library |
| `README.md` | This file |
| `SERIAL_PROTOCOL_GUIDE.md` | Complete serial JSON protocol reference (Rev 10+) |
| `MCP23017_COMPATIBILITY.md` | Historical reference (Rev 9.x emulation — deprecated) |
| `Serial Communication Messages structure outline.md` | Serial JSON message format reference |
| `WATCHDOG_FEATURE_SUMMARY.md` | Serial watchdog implementation details |
| `THREAD_SAFETY_ISSUE_REPORT.md` | Historical: Rev 9.0/9.1 relay control bug analysis |

---

## License

Proprietary — Contact authors for licensing information.
