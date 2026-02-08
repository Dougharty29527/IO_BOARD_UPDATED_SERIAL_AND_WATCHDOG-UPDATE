# Serial Communication Messages Structure Outline

**System:** Walter IO Board (ESP32) <-> Linux Comfile (Python)
**Firmware Version:** Rev 10.0
**Transport:** RS-232 via UART (Serial1) at 115,200 baud, 8N1
**Format:** All messages are single-line JSON terminated with `\n`

---

## Architecture Overview

The Walter IO Board (ESP32-S3) and a Linux single-board computer (Comfile) communicate over a dedicated RS-232 serial link. The ESP32 handles:
- Cellular modem (LTE Cat-M1) for cloud telemetry
- ADS1015 ADC for pressure and current sensing
- Relay control (Motor, CR1, CR2, CR5, DISP_SHUTDN)
- Overfill sensor (GPIO38)
- SD card logging
- WiFi configuration portal (web UI)
- Failsafe autonomous operation when Linux is down

The Linux device handles:
- Main control logic (compressor cycle management)
- PySimpleGUI operator interface
- Sending relay mode commands and receiving sensor/status data

---

## Message Direction: Linux -> ESP32

All messages from Linux are JSON objects sent on Serial1. The ESP32 parses them in real-time. There are two message categories: **data packets** (periodic) and **command messages** (on-demand).

### 1. Data Packet (Periodic — sent by Linux control program)

The primary message type. Sent every few seconds by the Linux `control.py` program. The ESP32 uses the `mode` field to set physical relay outputs.

```json
{
  "type": "data",
  "gmid": "CSX-0042",
  "press": 1.25,
  "mode": 1,
  "current": 12.5,
  "fault": 0,
  "cycles": 1847
}
```

| Field     | Type   | Required | Description |
|-----------|--------|----------|-------------|
| `type`    | string | **Yes**  | Must be `"data"` for this message type. All other `type` values are ignored. |
| `gmid`    | string | No       | Device name / site ID (e.g. `"CSX-0042"`, `"RND-0003"`). If present and different from stored name, updates EEPROM and WiFi AP SSID. |
| `press`   | float  | No       | Pressure reading in inches of water column (IWC). Stored as `pressure`. |
| `mode`    | int    | No       | **Relay mode number (0-9).** Controls physical relay outputs immediately. See [Relay Modes](#relay-modes) below. |
| `current` | float  | No       | Motor current in amps. Stored as `current`. |
| `fault`   | int    | No       | Fault/error bitmask from Linux. ESP32 adds its own fault bits before transmitting to cloud. |
| `cycles`  | int    | No       | Cumulative compressor cycle count. |

**ESP32 behavior on receipt:**
- Resets the serial watchdog timer (prevents failsafe activation)
- If currently in failsafe mode, **exits failsafe** and returns to serial-controlled operation
- Sets relay outputs via `setRelaysForMode(mode)` (unless in failsafe mode)
- Updates device name in EEPROM if `gmid` differs from stored value
- Saves data to SD card (if serial link was recently active)

### 2. Shutdown Command

Special `mode` value that triggers site shutdown. This is the **only** way to set `DISP_SHUTDN` (GPIO13) LOW in Rev 10.

```json
{
  "type": "data",
  "mode": "shutdown"
}
```

| Field  | Type   | Description |
|--------|--------|-------------|
| `type` | string | `"data"` |
| `mode` | string | Literal string `"shutdown"` (not an integer). Sets GPIO13 LOW. |

**Note:** When `mode` is the string `"shutdown"` instead of an integer, the ESP32 activates the dispenser shutdown pin and returns immediately without processing other fields. `DISP_SHUTDN` starts HIGH on every boot and stays HIGH unless this command is received.

### 3. Passthrough Command (Enter PPP Mode)

Requests the ESP32 to bridge its modem serial port to the Linux device for PPP/AT command access.

```json
{
  "command": "passthrough",
  "timeout": 60
}
```

| Field     | Type   | Required | Description |
|-----------|--------|----------|-------------|
| `command` | string | **Yes**  | `"passthrough"` |
| `timeout` | int    | No       | PPP session duration in minutes. Default: `60`. Range: 1-1440 (24 hours). |

**ESP32 behavior:**
1. Saves any active relay cycle state (relay mode + remaining seconds) to EEPROM
2. Sends `{"passthrough":"remote XX"}` back to Linux (notification)
3. Immediately restarts into passthrough boot mode (100ms delay for TX flush)
4. On reboot: sets up raw serial bridge (Serial1 <-> ModemSerial), restores saved relay state, and quietly finishes any interrupted purge step in the background

**CRITICAL:** Linux begins its PPP chat script the instant this command is sent, so the ESP32 restarts with no delay. Do NOT wait for a response.

### 4. Set Profile Command

Changes the active compressor profile on the ESP32.

```json
{
  "command": "set_profile",
  "profile": "CS8"
}
```

| Field     | Type   | Required | Description |
|-----------|--------|----------|-------------|
| `command` | string | **Yes**  | `"set_profile"` |
| `profile` | string | **Yes**  | Profile name. Valid values: `"CS2"`, `"CS3"`, `"CS8"`, `"CS9"`, `"CS12"`, `"CS13"` |

**ESP32 behavior:** Saves the active profile to EEPROM. Profile determines alarm thresholds used in failsafe mode and on the web dashboard.

### 5. Start Cycle Command

Tells the ESP32 to begin an autonomous relay cycle (failsafe mode).

```json
{
  "command": "start_cycle",
  "type": "run"
}
```

| Field     | Type   | Required | Description |
|-----------|--------|----------|-------------|
| `command` | string | **Yes**  | `"start_cycle"` |
| `type`    | string | **Yes**  | Cycle type: `"run"` (normal), `"purge"` (manual purge), `"clean"` (clean canister) |

### 6. Stop Cycle Command

Stops any running autonomous relay cycle and idles all relays.

```json
{
  "command": "stop_cycle"
}
```

| Field     | Type   | Required | Description |
|-----------|--------|----------|-------------|
| `command` | string | **Yes**  | `"stop_cycle"` |

---

## Message Direction: ESP32 -> Linux

### 1. Status Packet (Periodic — every 15 seconds)

The ESP32 sends a comprehensive status JSON to Linux every 15 seconds. This is the primary telemetry channel.

```json
{
  "datetime": "2026-02-08 14:30:00",
  "sdcard": "OK",
  "passthrough": 0,
  "lte": 1,
  "rsrp": "-85.5",
  "rsrq": "-10.2",
  "operator": "T-Mobile",
  "band": "B12",
  "mcc": 310,
  "mnc": 260,
  "cellId": 12345678,
  "tac": 5678,
  "pressure": 1.25,
  "current": 12.50,
  "overfill": 0,
  "profile": "CS8",
  "failsafe": 0
}
```

| Field         | Type   | Description |
|---------------|--------|-------------|
| `datetime`    | string | Current UTC timestamp from cellular modem, format `"YYYY-MM-DD HH:MM:SS"` |
| `sdcard`      | string | SD card status: `"OK"` or `"FAULT"` |
| `passthrough` | int    | `0` = normal mode, `1` = passthrough/PPP mode active |
| `lte`         | int    | `1` = connected to LTE network, `0` = not connected |
| `rsrp`        | string | Reference Signal Received Power in dBm (e.g. `"-85.5"`). `"--"` if unavailable. Typical: -80 excellent, -100 poor. |
| `rsrq`        | string | Reference Signal Received Quality in dB (e.g. `"-10.2"`). `"--"` if unavailable. Typical: -10 good, -20 poor. |
| `operator`    | string | Network operator name (e.g. `"T-Mobile"`). `"--"` if unavailable. |
| `band`        | string | LTE band (e.g. `"B12"`). `"--"` if unavailable. |
| `mcc`         | uint   | Mobile Country Code (e.g. `310` for USA) |
| `mnc`         | uint   | Mobile Network Code (e.g. `260` for T-Mobile) |
| `cellId`      | uint   | Cell tower ID |
| `tac`         | uint   | Tracking Area Code |
| `pressure`    | float  | Pressure in IWC from ADS1015 ADC pin 0 (rolling average, 60Hz sampling) |
| `current`     | float  | Motor current in amps from ADS1015 ADC pins 2-3 differential (`abs(Ch2 - Ch3)`, rolling average) |
| `overfill`    | int    | `0` = normal, `1` = overfill alarm active (GPIO38 LOW with hysteresis validation) |
| `profile`     | string | Active compressor profile name (e.g. `"CS8"`, `"CS12"`) |
| `failsafe`    | int    | `0` = normal (Linux controls relays), `1` = failsafe mode (ESP32 controls relays autonomously) |

### 2. Passthrough Notification

Sent to Linux immediately before the ESP32 restarts into passthrough boot mode. This is the signal for Linux to begin its PPP chat script.

```json
{
  "passthrough": "remote 60"
}
```

| Field         | Type   | Description |
|---------------|--------|-------------|
| `passthrough` | string | `"remote XX"` where XX is the timeout in minutes. |

**CRITICAL TIMING:** The ESP32 restarts ~100ms after sending this message. Linux must begin its chat script immediately upon receipt.

---

## Relay Modes

The `mode` field in the data packet maps to physical relay states:

| Mode | Name           | CR0_MOTOR | CR1   | CR2   | CR5   | Description |
|------|----------------|-----------|-------|-------|-------|-------------|
| 0    | IDLE           | LOW (off) | LOW   | LOW   | LOW   | All relays off |
| 1    | RUN            | HIGH (on) | HIGH  | LOW   | HIGH  | Normal compressor operation |
| 2    | PURGE          | HIGH (on) | LOW   | HIGH  | LOW   | Canister purge |
| 3    | BURP           | LOW (off) | LOW   | LOW   | HIGH  | Brief pressure equalization |
| 8    | FRESH AIR      | LOW (off) | LOW   | HIGH  | HIGH  | Special burp / fresh air mode |
| 9    | LEAK TEST      | LOW (off) | HIGH  | HIGH  | HIGH  | Leak test (no motor) |

**`DISP_SHUTDN` (GPIO13):** NOT affected by mode changes. Starts HIGH on boot. Only set LOW by the `{"mode":"shutdown"}` command. This is a separate safety pin, not part of the relay mode table.

---

## Fault Code Bitmask

Fault codes are cumulative (OR'd together). The ESP32 adds its own fault bits to whatever the Linux device reports in the `fault` field.

| Bit Value | Source  | Meaning |
|-----------|---------|---------|
| (varies)  | Linux   | Application-specific faults from `control.py` (passed through in `fault` field) |
| 1024      | ESP32   | **Watchdog fault** — no serial data from Linux for 30+ minutes |
| 4096      | ESP32   | **BlueCherry offline** — cloud messaging service disconnected |
| 8192      | ESP32   | **Panel down / Failsafe** — ESP32 is controlling relays autonomously (Linux unresponsive after 2 restart attempts) |

The combined fault code sent to the cloud is: `linux_faults + watchdog(1024) + bluecherry(4096) + failsafe(8192)`

---

## Compressor Profiles

Each profile defines alarm thresholds and monitoring behavior. Stored in ESP32 EEPROM.

| Profile | Description               | Pressure Setpoint | Zero Press | Low Press | High Press | Var Press | 72hr Shutdown | Motor Current | Overfill |
|---------|---------------------------|-------------------|------------|-----------|------------|-----------|---------------|---------------|----------|
| CS2     | Tokheim                   | -2.0              | Yes        | Yes (-6.0)| Yes (2.0)  | Yes (0.20)| No            | Yes (3.0-25.0)| Yes      |
| CS3     | Non-GVR                   | -2.0              | Yes        | Yes (-6.0)| Yes (2.0)  | No        | No            | Yes (3.0-25.0)| Yes      |
| CS8     | Mexican Non-GVR           | -2.0              | Yes        | Yes (-6.0)| Yes (2.0)  | No        | No            | Yes (3.0-25.0)| Yes      |
| CS9     | Dresser Wayne             | -2.0              | Yes        | Yes (-6.0)| Yes (2.0)  | Yes (0.20)| No            | Yes (3.0-25.0)| Yes      |
| CS12    | Gilbarco Encore           | -2.0              | Yes        | Yes (-6.0)| Yes (2.0)  | Yes (0.20)| No            | Yes (3.0-25.0)| Yes      |
| CS13    | Gilbarco Encore S (Stage) | -2.0              | Yes        | Yes (-6.0)| Yes (2.0)  | Yes (0.20)| Yes           | Yes (3.0-25.0)| Yes      |

---

## Autonomous Cycle Sequences (Failsafe Mode)

When the ESP32 enters failsafe mode, it runs relay cycles autonomously. Each cycle is a sequence of `{mode, duration_seconds}` steps.

### Normal Run Cycle
```
Run(120s) → Idle(3s) → [Purge(50s) → Burp(5s)] × 6 → Idle(15s)
```
Total: 15 steps, ~468 seconds per cycle.

### Manual Purge Cycle
```
[Purge(50s) → Burp(5s)] × 6
```
Total: 12 steps, 330 seconds.

### Clean Canister Cycle
```
Run(900s)
```
Total: 1 step, 15 minutes.

---

## Serial Timing and Watchdog

| Parameter                  | Value      | Description |
|----------------------------|------------|-------------|
| **Read interval**          | 5 seconds  | ESP32 checks for incoming serial data every 5s |
| **Status send interval**   | 15 seconds | ESP32 sends status JSON to Linux every 15s (configurable via web/BlueCherry) |
| **Watchdog timeout**       | 30 minutes | If no serial data for 30 min, watchdog triggers |
| **Watchdog first pulse**   | 1 second   | First restart attempt: 1s pulse on GPIO39 |
| **Watchdog long pulse**    | 30 seconds | Subsequent restart attempts: 30s pulse on GPIO39 |
| **Failsafe trigger**       | 2 attempts | After 2 failed restart attempts, ESP32 enters failsafe mode |
| **Failsafe safety timeout**| 30 minutes | If no commands during failsafe, relay operations stop |

---

## Passthrough Mode (PPP Bridge)

During passthrough mode, the ESP32 acts as a transparent serial bridge between Linux (Serial1) and the cellular modem (ModemSerial). This allows Linux to establish PPP connections for firmware updates or remote access.

**Entry sequence:**
1. Linux sends `{"command":"passthrough","timeout":60}`
2. ESP32 saves relay state to EEPROM, sends `{"passthrough":"remote 60"}` to Linux
3. ESP32 restarts immediately (~100ms)
4. On boot: ESP32 detects passthrough flag, skips WalterModem library, sets up raw serial bridge
5. Any active purge step finishes quietly in background; relays idle when step timer expires

**Exit conditions:**
- Linux sends escape sequence `+++STOPPPP` through the bridge
- Timeout expires (default 60 minutes)
- Both cause ESP32 to restart into normal mode

**During passthrough:**
- ESP32 buffers sensor data (mode, pressure, current, fault, cycles) every 15 seconds as CBOR samples
- Buffer capacity: 480 samples (~120 minutes at 15s intervals, ~8.2KB)
- Buffered data is flushed to the cloud after passthrough ends

---

## CBOR Buffer Sample Structure (Internal)

Each buffered sample during passthrough contains:

| Field       | Type     | Size    | Description |
|-------------|----------|---------|-------------|
| `timestamp` | uint32_t | 4 bytes | Unix epoch seconds |
| `mode`      | uint8_t  | 1 byte  | Relay mode (0-9) |
| `pressure`  | float    | 4 bytes | Pressure in IWC |
| `current`   | float    | 4 bytes | Motor current in amps |
| `fault`     | uint16_t | 2 bytes | Combined fault code |
| `cycles`    | uint16_t | 2 bytes | Cycle count |

Total: 17 bytes per sample. Max 480 samples = 8,160 bytes.

---

## Quick Reference: Message Routing Summary

```
Linux → ESP32 (Serial1 RX):
  ├── {"type":"data", "mode":1, ...}        → Set relays, store values, reset watchdog
  ├── {"type":"data", "mode":"shutdown"}     → Set DISP_SHUTDN LOW (GPIO13)
  ├── {"command":"passthrough", "timeout":N} → Enter PPP bridge mode
  ├── {"command":"set_profile", "profile":X} → Change active profile
  ├── {"command":"start_cycle", "type":X}    → Start autonomous relay cycle
  └── {"command":"stop_cycle"}               → Stop relay cycle, idle all relays

ESP32 → Linux (Serial1 TX):
  ├── {"datetime":..., "pressure":..., ...}  → Status packet (every 15s)
  └── {"passthrough":"remote XX"}            → PPP mode notification (before restart)
```
