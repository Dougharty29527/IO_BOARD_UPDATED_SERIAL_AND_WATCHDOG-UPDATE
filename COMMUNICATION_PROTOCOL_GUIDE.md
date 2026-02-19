# ESP32 IO Board Communication Protocol Guide

**Firmware:** IO_BOARD_FIRMWARE Rev 10.36
**Date:** February 19, 2026
**Audience:** Management stakeholders interested in system capabilities, and technical staff implementing communication with the ESP32 IO Board

---

## Executive Summary

The ESP32 IO Board serves as the central control system for vacuum pump operations, providing reliable communication through multiple channels. While the core functionality remains serial-based RS-232 communication with Linux devices, the system now includes advanced ESPNow wireless capabilities for remote display panels.

**Key Communication Methods:**
- **RS-232 Serial** (Primary): Wired communication with Linux control systems
- **ESPNow Wireless** (Secondary): Wireless communication with display panels
- **WiFi Web Portal** (Configuration): Browser-based system management and diagnostics

This guide explains how data flows between the ESP32 IO Board and connected devices, ensuring reliable operation and system monitoring.

---

## Purpose of This Document

The ESP32 IO Board serves as the central hardware controller for vacuum pump operations, communicating through multiple channels depending on the connected device type.

**For Management Stakeholders:**
This document provides an overview of how the ESP32 IO Board communicates with different system components, ensuring you understand the reliability and monitoring capabilities of the vacuum control system.

**For Technical Implementers:**
This document provides the complete specification for all communication protocols. Whether you're implementing Linux control software, developing a wireless display panel, or integrating with the web portal, you'll find the exact message formats, timing requirements, and data flows needed for reliable system operation.

**Communication Channels:**
- **Linux Control Systems**: Use RS-232 serial communication for primary control and monitoring
- **Display Panels**: Use ESPNow wireless communication for remote monitoring and control
- **Human Operators**: Use WiFi web portal for configuration and diagnostics

---

## Physical Layer

### RS-232 Serial Communication (Primary Channel)

| Parameter | Value |
|-----------|-------|
| Protocol | RS-232 serial (TTL logic levels via MAX3232 or equivalent) |
| Baud rate | **115200** |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None (no RTS/CTS, no XON/XOFF) |
| ESP32 RX pin | **GPIO44** |
| ESP32 TX pin | **GPIO43** |
| Linux port | `/dev/serial0` (Raspberry Pi UART) |

### ESPNow Wireless Communication (Display Panel Channel)

| Parameter | Value |
|-----------|-------|
| Protocol | ESPNow (ESP32 proprietary wireless protocol) |
| Frequency | 2.4 GHz (WiFi channel 1) |
| Range | Typically 50-100 meters line-of-sight |
| Security | No encryption (local network only) |
| Packet size | Maximum 250 bytes (248 bytes payload + 2 bytes header) |
| Discovery | Automatic beacon broadcasting |
| Connection | Auto-pairing with display panels |

### Message Format

All communication channels use **JSON objects** as the message format, one per transmission, terminated by `\n` (newline).

- **RS-232 Serial**: ESP32 uses `Serial1.println()` which appends `\r\n`. Connected devices should accept both `\n` and `\r\n` terminators.
- **ESPNow Wireless**: JSON payloads are transmitted as raw strings within ESPNow packets, maintaining the same format as serial communication.

### ESP32 Communication Ports

The ESP32 manages multiple communication channels simultaneously:
- **Serial** (USB): Debug output only. Not part of operational protocols.
- **Serial1** (GPIO44/43): RS-232 communication with Linux control systems.
- **Serial2** (GPIO14/48): Cellular modem communication (internal use only).
- **ESPNow**: Wireless communication with display panels (shares WiFi radio).
- **WiFi AP**: Web portal for human operators (192.168.4.1).

---

## Message Flow Overview

### Primary Communication: Linux Control System (RS-232 Serial)

```
Linux Device                    ESP32 IO Board
     │                               │
     │  ◄─── Fast Sensor Packet ───  │  Every 200ms (5Hz)
     │       (pressure, current,     │  Real-time operational data
     │        overfill, sdcard)      │
     │                               │
     │  ◄─── Cellular Status ─────  │  Every 10 seconds
     │       (lte, rsrp, rsrq,      │  Network connectivity status
     │        operator, band, etc.)  │
     │                               │
     │  ◄─── Datetime Packet ─────  │  Only when fresh from modem
     │       (datetime only)         │  Accurate time synchronization
     │                               │
     │  ── Data Payload ──────────►  │  Every 15 seconds
     │     (gmid, mode, fault,       │  Cloud data transmission
     │      cycles, press, current)  │
     │                               │
     │  ── Immediate Mode ────────►  │  On mode change (<100ms)
     │     (mode number only)        │  Relay control commands
     │                               │
     │  ── Web Portal Commands ──►  │  User-initiated actions
     │       (relay control,         │  Manual control interface
     │        calibration, tests)    │
     │                               │
     │  ── System Commands ───────►  │  Configuration and maintenance
     │     (passthrough, calibration,│  System management
     │      profile changes, etc.)   │
```

### Secondary Communication: Display Panel (ESPNow Wireless)

```
Display Panel                  ESP32 IO Board
     │                               │
     │  ◄─── Fast Sensor Packet ───  │  Every 200ms (5Hz)
     │       (pressure, current,     │  Same real-time data as serial
     │        overfill, sdcard)      │
     │                               │
     │  ◄─── Cellular Status ─────  │  Every 10 seconds
     │       (lte, rsrp, rsrq,      │  Network status for monitoring
     │        operator, band, etc.)  │
     │                               │
     │  ◄─── Datetime Packet ─────  │  Only when fresh from modem
     │       (datetime only)         │  Time display
     │                               │
     │  ── Mode Commands ─────────►  │  Touchscreen control
     │     (mode 0-9, emergency)     │  Remote relay control
     │                               │
     │  ── System Commands ───────►  │  Configuration access
     │     (calibration, diagnostics)│  Panel-initiated actions
     │                               │
     │  ── Heartbeat ─────────────►  │  Every 5 seconds
     │     (keepalive signals)       │  Connection monitoring
```

### Auto-Discovery and Connection Management

```
Display Panel                  ESP32 IO Board
     │                               │
     │  ◄─── Beacon Broadcast ────  │  Every 2 seconds (unpaired)
     │     {"type":"beacon",         │  IO Board identification
     │      "gmid":"CSX-1234",       │
     │      "fw":"10.36"}            │
     │                               │
     │  ── Pair Request ──────────►  │  After beacon received
     │     {"type":"pair_request",   │  Connection establishment
     │      "name":"Display-1"}      │
     │                               │
     │  ◄─── Pair Confirm ────────  │  Immediate response
     │     {"type":"pair_confirm",   │  Connection confirmed
     │      "gmid":"CSX-1234"}       │
     │                               │
     │  ◄─── Data Stream ─────────  │  After pairing complete
     │     (sensor + cellular data)  │  Continuous monitoring
```

---

## ESPNow Wireless Communication Protocol

### Overview

ESPNow provides wireless communication between the ESP32 IO Board and remote display panels. This allows operators to monitor and control vacuum systems from a distance without running cables.

**Key Features:**
- **Automatic Discovery**: Display panels automatically find and connect to IO Boards
- **Seamless Integration**: Same JSON message format as RS-232 serial
- **Priority System**: When a display panel is connected, it receives all sensor data instead of the Linux system
- **Reliable Connection**: Built-in heartbeat monitoring and automatic reconnection
- **Fragmentation Support**: Large messages (like cellular status) are automatically split across multiple packets

### ESPNow Packet Structure

Each ESPNow transmission consists of a 2-byte header followed by JSON payload:

```
Byte 0: Packet Type
  0x01 = JSON data (sensor, cellular, commands)
  0x10 = Discovery beacon (IO Board broadcasting)
  0x11 = Pair request (Display → IO Board)
  0x12 = Pair confirmation (IO Board → Display)

Byte 1: Fragment Information
  High nibble: Fragment index (0-based)
  Low nibble: Total fragments
  Example: 0x01 = single packet, 0x12 = fragment 1 of 2

Bytes 2-249: JSON Payload (up to 248 bytes)
```

### Connection Process

1. **Discovery Phase**: IO Board broadcasts beacons every 2 seconds containing its identity
2. **Pairing Phase**: Display panel responds with pair request, IO Board confirms connection
3. **Active Phase**: Continuous data exchange with heartbeat monitoring
4. **Reconnection**: Automatic recovery if connection is lost

### Communication Priority

- **ESPNow Active**: When a display panel is connected, all outgoing sensor and cellular data goes to the display panel via ESPNow
- **ESPNow Inactive**: When no display panel is connected, all data goes to the Linux system via RS-232 serial
- **Bidirectional**: Commands can come from either the Linux system (RS-232) or display panel (ESPNow)

---

## ESP32 to Connected Device Messages

### 1. Fast Sensor Packet (5Hz / every 200ms)

This is the critical real-time data path. The Linux device uses these values for all operational decisions.

```json
{"pressure":-14.22,"current":0.07,"overfill":0,"sdcard":"OK","relayMode":1,"failsafe":0,"shutdown":0}
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `pressure` | float | Typically +/-36, or **-99.9** | Vacuum pressure in inches of water column (IWC). The pressure sensor measures vacuum levels with 1-second averaging and validates/discards invalid ADC readings (I2C bus errors, out-of-range values). Negative values indicate vacuum (normal operation), 0.0 is atmospheric pressure, positive values are above atmospheric. A value of -99.9 indicates a sensor fault. |
| `current` | float | 0.0 to ~15.0 | Motor current in amps. Read from ADS1015 ADC channels 2-3 using **hardware differential** (`readADC_Differential_2_3()`) at GAIN_TWO (1mV/count) for full 12-bit precision on the difference. Validates and discards invalid readings (I2C bus errors, rail-pinned values, and abnormally high outliers). Reports **windowed peak** (max after removing highest outlier, 60-sample window) to match Python's approach. 0.0 when motor is off. Pin 1 (AIN1) is unused. **Rev 10.11: Sent as 0.0 during sensor fault (when pressure is -99.9).** |
| `overfill` | int | 0 or 1 | Overfill alarm state. 0 = normal, 1 = alarm active. Read from GPIO38 with 8-of-5 hysteresis (must read LOW 8 out of last 5 checks to trigger). Has internal pull-up. |
| `sdcard` | string | `"OK"` or `"FAULT"` | SD card status. Reads cached card type, no disk I/O. |
| `relayMode` | int | 0-9 | Current relay mode the ESP32 has applied. Useful for confirming the relay state matches what Linux requested. |
| `failsafe` | int | 0 or 1 | **Rev 10.8.** 0 = normal serial-controlled operation, 1 = ESP32 in autonomous failsafe mode (Comfile panel unresponsive). |
| `shutdown` | int | 0 or 1 | **Rev 10.8.** 0 = DISP_SHUTDN HIGH (site normal), 1 = DISP_SHUTDN LOW (72-hour shutdown active). |

**Why 5Hz:** The Linux controller makes real-time decisions based on pressure (start/stop cycles, detect alarms). 200ms updates give the controller sub-second response to pressure changes. The ADC itself samples at 60Hz internally; the 5Hz serial rate is a practical balance between freshness and serial bandwidth.

**Why the ESP32 reads its own ADC:** In the previous hardware (MCP23017 + ADS1115), the Linux device read the ADC directly over I2C. This caused bus contention, timing conflicts, and stale data when the buffer backed up. Moving ADC reading to the ESP32 eliminates all of that.

---

### 2. Cellular Status Packet (timer-driven, every 10 seconds)

Sent every 10 seconds using cached values from the last successful modem query. **Rev 10.9 change:** Previously this was freshness-gated (only sent when modem returned new data). If modem queries failed silently, the Linux device received nothing. Now the Linux device always receives the last-known cellular state on a timer.

`datetime` and `failsafe` have been **removed** from this packet:
- `datetime` is now sent in its own standalone packet only when fresh (see section 2b below)
- `failsafe` is already included in the 5Hz fast sensor packet (Rev 10.8)

```json
{"passthrough":0,"lte":1,"rsrp":"-85.5","rsrq":"-10.2","operator":"T-Mobile","band":"B2","mcc":310,"mnc":260,"cellId":12345678,"tac":9876,"profile":"CS8","imei":"351234567890123","imsi":"310410123456789","iccid":"8901260882310000000","technology":"LTE-M","espnow":0,"temperature":85.2}
```

| Field | Type | Description |
|-------|------|-------------|
| `passthrough` | int | 0 = normal operation, 1 = PPP passthrough mode active |
| `lte` | int | 1 = connected to LTE network, 0 = not connected |
| `rsrp` | string | Reference Signal Received Power in dBm (e.g., `"-85.5"`). `"--"` if unavailable. Typical: -80 excellent, -100 poor. |
| `rsrq` | string | Reference Signal Received Quality in dB (e.g., `"-10.2"`). `"--"` if unavailable. Typical: -10 good, -20 poor. |
| `operator` | string | Network operator name (e.g., `"T-Mobile"`). `"--"` if unavailable. |
| `band` | string | LTE band (e.g., `"B2"`, `"B4"`). `"--"` if unavailable. |
| `mcc` | int | Mobile Country Code (e.g., 310 for USA) |
| `mnc` | int | Mobile Network Code (e.g., 260 for T-Mobile) |
| `cellId` | int | Cell Tower ID |
| `tac` | int | Tracking Area Code |
| `profile` | string | Active equipment profile on ESP32 (e.g., `"CS8"`, `"CS9"`, `"CS12"`) |
| `imei` | string | **Rev 10.9.** Modem IMEI (15-digit hardware identity). Queried once at boot. |
| `imsi` | string | **Rev 10.9.** SIM IMSI (International Mobile Subscriber Identity). `"--"` if unavailable. |
| `iccid` | string | **Rev 10.9.** SIM ICCID (Integrated Circuit Card ID). `"--"` if unavailable. |
| `technology` | string | **Rev 10.9.** Radio Access Technology: `"LTE-M"`, `"NB-IoT"`, `"Auto"`, or `"Unknown"`. |
| `espnow` | int | **Rev 10.36.** ESPNow connection status: `1` = display panel connected, `0` = no display panel |
| `temperature` | float | **Rev 10.36.** ESP32 board temperature in degrees Fahrenheit. Typical range: 70-120°F. Used for thermal monitoring and diagnostics. |

**Why 10-second timer:** Guarantees connected devices always have cellular info, even during modem communication failures. Uses cached values from the last successful query — stale but useful beats missing entirely.

**ESPNow fragmentation:** When this packet exceeds 248 bytes, it's automatically split across multiple ESPNow packets and reassembled by the display panel.

---

### 2b. Datetime Packet (fresh-only, when modem clock updates)

Sent **only** when the modem clock returns a genuinely new timestamp (`modemTimeFresh` flag). This is a standalone single-field packet. Never repeated with stale/frozen values.

```json
{"datetime":"2026-02-10 13:23:51"}
```

| Field | Type | Description |
|-------|------|-------------|
| `datetime` | string | UTC timestamp from the cellular modem clock. Format: `"YYYY-MM-DD HH:MM:SS"` |

**Why separate from cellular:** Datetime should only update when the modem provides a fresh clock reading. Sending a stale datetime on a timer would make it appear the modem clock is frozen. The Python parser handles this standalone packet identically — it checks `if "datetime" in data` independent of other fields.

---

### 3. Web Portal Commands (HTTP POST to ESP32)

Web portal commands are sent from the user's browser to the ESP32 via HTTP POST requests to `/api/command`. The ESP32 processes these commands and either handles them directly or forwards them to the Linux system for logging/coordination.

**Commands handled directly by ESP32:**
```json
{"command":"toggle_relay","value":"CR1"}
{"command":"toggle_relay","value":"CR2"}
{"command":"toggle_relay","value":"CR5"}
{"command":"emergency_stop"}
{"command":"calibrate_pressure"}
{"command":"set_manual_adc_zero","value":"964.0"}
{"command":"overfill_override","value":"on"}
{"command":"clear_press_alarm"}
{"command":"clear_motor_alarm"}
{"command":"enable_failsafe"}
{"command":"exit_failsafe"}
{"command":"restart"}
{"command":"send_status"}
```

**Commands that ESP32 handles directly but also notifies Linux:**
```json
{"command":"start_cycle","type":"manual_purge"}
{"command":"start_cycle","type":"clean"}
{"command":"stop_cycle"}
{"command":"start_test","type":"leak"}
{"command":"start_test","type":"func"}
{"command":"start_test","type":"eff"}
{"command":"stop_test"}
```

**Commands forwarded to Linux for primary handling:**
```json
{"command":"start_cycle","type":"run"}
{"command":"set_profile","value":"CS8","password":"1793"}
```

| Field | Description |
|-------|-------------|
| `command` | The action: `"toggle_relay"`, `"emergency_stop"`, `"calibrate_pressure"`, `"start_cycle"`, `"start_test"`, `"set_profile"`, etc. |
| `type` | For start commands: `"run"`, `"manual_purge"`, `"clean"`, `"leak"`, `"func"`, `"eff"` |
| `value` | For toggle_relay: `"CR1"`, `"CR2"`, `"CR5"`; for set_profile: profile name; for ADC zero: numeric value |
| `password` | Required for set_profile command (password: `"1793"`) |

**ESP32 direct control actions:**
- `toggle_relay/CR1` — Toggle CR1 relay ON/OFF (individual valve control)
- `toggle_relay/CR2` — Toggle CR2 relay ON/OFF (individual valve control)
- `toggle_relay/CR5` — Toggle CR5 relay ON/OFF (individual valve control)
- `emergency_stop` — Set ESP32 to idle mode (mode 0), stops all operations
- `calibrate_pressure` — Calibrate pressure sensor zero point (requires ambient air)
- `set_manual_adc_zero` — Set pressure sensor zero point to specific ADC value
- `overfill_override` — Override overfill alarm detection
- `clear_press_alarm` — Clear pressure alarm state
- `clear_motor_alarm` — Clear motor current alarm state
- `enable_failsafe` — Enable autonomous failsafe mode
- `exit_failsafe` — Disable autonomous failsafe mode
- `restart` — Reboot the ESP32 immediately
- `send_status` — Manually trigger CBOR status update to cloud server

**ESP32 test operations (direct control with Linux notification):**
- `start_cycle/manual_purge` — ESP32 controls purge mode relays, notifies Linux
- `start_cycle/clean` — ESP32 controls run mode for canister cleaning, notifies Linux
- `start_test/leak` — ESP32 runs leak test sequence, notifies Linux
- `start_test/func` — ESP32 runs functionality test cycles, notifies Linux
- `start_test/eff` — ESP32 runs efficiency test, notifies Linux

**Linux-handled operations:**
- `start_cycle/run` — Linux manages normal operation cycles with full monitoring

**Web portal security:** The Maintenance screen requires password `"878"` to unlock. Profile changes require password `"1793"`. Device name/watchdog changes require password `"gm2026"`.

---

### 4. Passthrough Request (ESP32 wants Linux to start PPP)

```json
{"passthrough":"remote 60"}
```

The number is the timeout in minutes. Linux should:
1. Send `ready\n` back to the ESP32
2. Close the serial port
3. Start `pppd` (e.g., `sudo pppd call walter nodetach debug`)
4. After timeout or PPP exit, send the escape sequence `+++STOPPPP\n` to signal the ESP32 to restart
5. Reopen the serial port for normal operation

---

### 5. Data Backfill (ESP32 sends collected data after passthrough)

Sent automatically after ESP32 reboots from passthrough mode. Contains SD card data collected during the passthrough session that was not sent to the cloud due to cellular modem being busy with PPP.

```json
{"backfill":[{"pressure":-14.22,"current":0.07,"mode":0,"fault":0,"cycles":484},{"pressure":-14.25,"current":0.06,"mode":1,"fault":0,"cycles":485}]}
```

| Field | Type | Description |
|-------|------|-------------|
| `backfill` | array | Array of data objects collected during passthrough mode |
| `pressure` | float | Vacuum pressure in inches of water column (IWC) |
| `current` | float | Motor current in amps |
| `mode` | int | Relay mode (0-9) |
| `fault` | int | Active alarm bitmask |
| `cycles` | int | Run cycle count |

**Why backfill is needed:** During passthrough mode, the ESP32 continues to collect sensor data and store it on the SD card, but cannot send CBOR data to the cloud because the cellular modem is busy with PPP. After passthrough ends, this data is sent to the Linux device for cloud synchronization. Date/time is added by the Linux device/server, not included in the backfill data.

---

## Linux to ESP32 Messages

### 1. Periodic Data Payload (every 15 seconds)

This is the primary data feed for the ESP32's CBOR encoder and SD card logger.

```json
{"type":"data","gmid":"CSX-1234","press":-14.22,"mode":0,"current":0.07,"fault":0,"cycles":484}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | string | **Yes** | `"data"` for status/relay packets, `"cmd"` for command messages (see Section 4). |
| `gmid` | string | No | Device name/ID (e.g., `"CSX-1234"`). If changed, ESP32 updates EEPROM and WiFi AP name. |
| `press` | float | No | Pressure from Linux (informational). **ESP32 uses its own ADC for CBOR/SD, not this value.** Included for consistency and debugging. |
| `mode` | int, `"shutdown"`, or `"normal"` | No | Relay mode number (0-9), `"shutdown"` (GPIO13 LOW), or `"normal"` (GPIO13 HIGH — clears 72-hour alarm). See below. |
| `current` | float | No | Current from Linux (informational). **ESP32 uses its own ADC for CBOR/SD.** |
| `fault` | int | No | Active alarm bitmask from Linux. Stored and included in CBOR transmissions. |
| `cycles` | int | No | Run cycle count from Linux. Stored and included in CBOR transmissions. |

**Important:** The `press` and `current` values in this payload are NOT used by the ESP32 for its own CBOR pressure/current fields. The ESP32 reads its own ADS1015 ADC for those. The Linux values are stored for display on the web portal diagnostics page and as a sanity check. The `fault`, `cycles`, and `gmid` fields ARE used in CBOR.

**No trailing newline required.** The ESP32 parser looks for `{` and `}` delimiters, not newlines. A trailing `\n` is fine but not needed.

---

### 2. Immediate Mode Command (on relay mode change)

When the Linux device needs to change relay states, it sends a minimal mode command:

```json
{"type":"data","mode":1}
```

| Mode | Name | Motor | CR1 (V1) | CR2 (V2) | CR5 |
|------|------|-------|-----------|-----------|-----|
| 0 | Rest / Idle | OFF | OFF | OFF | OFF |
| 1 | Run | **ON** | **ON** | OFF | **ON** |
| 2 | Purge | **ON** | OFF | **ON** | OFF |
| 3 | Burp | OFF | OFF | OFF | **ON** |
| 8 | Bleed / Fresh Air | OFF | OFF | **ON** | **ON** |
| 9 | Leak Test | OFF | **ON** | **ON** | **ON** |

Modes 4-7 are undefined and map to all-off (safe state).

**Response time:** The ESP32 checks for serial data every **100ms**. Relays are set atomically within the same call. Total latency from Linux write to relay activation: ~100-200ms.

**This should be sent immediately when the mode changes**, not bundled into the 15-second payload. The 15-second payload also contains a `mode` field, but by the time it arrives the relays should already be in the correct state from the immediate command.

---

### 3. Shutdown / Normal Commands

**Activate 72-hour shutdown (GPIO13 LOW):**
```json
{"type":"data","mode":"shutdown"}
```

**Clear 72-hour shutdown (GPIO13 HIGH — restore site):**
```json
{"type":"data","mode":"normal"}
```

`"shutdown"` is the **only** way to pull DISP_SHUTDN (GPIO13) LOW. GPIO13 starts HIGH on every ESP32 boot and stays HIGH until the shutdown command is received. When the 72-hour alarm clears, the Linux device sends `"normal"` to restore GPIO13 HIGH without requiring an ESP32 reboot.

---

### 4. Command Messages (`type: "cmd"`)

Command messages use `"type":"cmd"` to distinguish them from data packets. The ESP32 routes on the `"cmd"` field.

#### Pressure Sensor Calibration

**Calibrate pressure sensor zero point:**
```json
{"type":"cmd","cmd":"cal"}
```

This command tells the ESP32 to recalibrate the pressure sensor zero point. The sensor should be **open to ambient air (no vacuum applied)** when this command is issued. Whatever the sensor is currently reading becomes the new 0.0 IWC reference. For example, if the sensor reads -0.5 IWC due to altitude, after calibration it will read 0.0 IWC.

**Calibration process (instant, non-blocking — Rev 10.7+):**
1. Uses the existing 60-sample rolling average (`adcPressure`) — no delay or sample loop
2. Computes the new zero point: `newZero = oldZero + (currentPressure × slope)`
3. **Rev 10.9:** Validates the result is within the middle 20% of scale (~868-1060 ADC counts, factory default ±10%). If out of range, calibration is **REJECTED** with an error message — this prevents calibrating under vacuum.
4. Saves the new zero point to EEPROM (Preferences key: `"adcZero"` in namespace `"RMS"`)
5. Clears the pressure averaging buffer so readings immediately reflect the new calibration
6. Sends `{"type":"data","ps_cal":NNN.NN}` back to Linux with the new zero point

**IMPORTANT:** The sensor **must be at ambient air pressure** (no vacuum) when calibrating. If the sensor is under vacuum (e.g., reading -6 IWC), the calibration will be rejected because the computed zero point falls outside the allowed range.

**Persistence:** The calibration value is saved to EEPROM and loaded automatically on every ESP32 boot. **Rev 10.9:** On boot, the stored value is validated against the same ±10% range. If a bad value was saved by a prior firmware version, it is rejected and the factory default (964.0) is used instead. The device self-heals on next reboot.

**Factory default:** 964.0 (= 15422/16, the ADS1015 12-bit equivalent of the Python `pressure_sensor.py` default 15422.0 for the ADS1115 16-bit).

**Also available from:** Web portal Maintenance screen → "Calibrate Pressure" button, which sends `POST /api/command {"command":"calibrate_pressure"}`.

**Python helper:** `modem.send_calibration_command()` sends `{"type":"cmd","cmd":"cal"}` to the ESP32.

---

### 4. Command Messages (Linux to ESP32)

**Passthrough request:**
```json
{"command":"passthrough","timeout":60}
```

Tells the ESP32 to enter passthrough mode for `timeout` minutes. The ESP32 will restart into a serial bridge mode where Serial1 is connected directly to the cellular modem (Serial2). Linux can then run PPP over the serial line.

**Set profile:**
```json
{"command":"set_profile","profile":"CS8"}
```

Changes the active equipment profile on the ESP32.

**Start/Stop failsafe cycle:**
```json
{"command":"start_cycle","type":"run"}
{"command":"stop_cycle"}
```

These tell the ESP32 to start/stop its own failsafe cycle engine. In normal operation (Linux is connected), you would NOT send these — the Linux device manages cycles and just sends mode numbers. These are useful for testing or if Linux wants the ESP32 to run autonomously.

**Fast BlueCherry polling:**
```json
{"type":"cmd","cmd":"fast_poll","val":1}   // Enable fast polling (15s intervals, 60min max)
{"type":"cmd","cmd":"fast_poll","val":0}   // Disable fast polling (return to scheduled)
```

Enables fast BlueCherry message retrieval by checking every 15 seconds instead of the normal scheduled window (7AM-1PM EST every 15 minutes). Automatically times out after 60 minutes maximum. Use for urgent remote control scenarios.

**BlueCherry fast polling messages:**
- `fast_poll` - Enable fast polling (15s intervals, 60min timeout)
- `exit_fast_poll` - Disable fast polling (return to scheduled)

---

### Commands from ESPNow Display Panels

Display panels can send the same web portal commands as the browser interface, allowing remote touchscreen control of vacuum operations:

**Direct ESP32 Control Commands:**
```json
{"command":"toggle_relay","value":"CR1"}    // Toggle CR1 valve
{"command":"toggle_relay","value":"CR2"}    // Toggle CR2 valve
{"command":"toggle_relay","value":"CR5"}    // Toggle CR5 valve
{"command":"emergency_stop"}                // Emergency stop all
{"command":"calibrate_pressure"}            // Calibrate pressure sensor
{"command":"restart"}                       // Reboot ESP32
```

**Test and Cycle Commands:**
```json
{"command":"start_cycle","type":"run"}      // Start normal run cycle
{"command":"start_cycle","type":"manual_purge"}  // Manual purge operation
{"command":"start_cycle","type":"clean"}    // Canister cleaning cycle
{"command":"stop_cycle"}                    // Stop current operation
{"command":"start_test","type":"leak"}      // Leak test
{"command":"start_test","type":"func"}      // Functionality test
{"command":"start_test","type":"eff"}       // Efficiency test
{"command":"stop_test"}                     // Stop current test
```

**Legacy Serial Protocol Commands (also supported):**
```json
{"type":"data","mode":1}      // Set to Run mode
{"type":"data","mode":0}      // Emergency stop (idle)
{"type":"data","mode":"shutdown"}  // Activate 72-hour shutdown
{"type":"cmd","cmd":"cal"}    // Calibrate pressure sensor
```

**Connection Maintenance:**
```json
{"type":"keepalive"}          // Connection heartbeat (every 5 seconds)
```

Display panels receive all the same responses as the web portal, including calibration results, relay state confirmations, and command acknowledgments.

---

## Communication Watchdog and Failsafe Mode

The ESP32 monitors both RS-232 serial and ESPNow wireless communication channels for incoming data. When either channel is active (receiving data), the watchdog is reset. Watchdog behavior depends on whether failsafe mode is enabled or disabled.

### Failsafe Enable/Disable Setting

Failsafe mode can be enabled/disabled via:
- **Web Portal**: Settings → Failsafe Mode toggle (password required)
- **BlueCherry**: `enable_failsafe` or `exit_failsafe` commands

**Default state:** Failsafe is **disabled** (safe default to prevent unintended autonomous operation).

### Watchdog Behavior

| Failsafe Setting | Watchdog Action |
|------------------|----------------|
| **Disabled** | Pulses GPIO39 every **10 minutes** if no serial data. Never enters failsafe mode. |
| **Enabled** | Pulses GPIO39 every **30 minutes** if no serial data. Enters failsafe after 2 attempts. |

### Watchdog Timeline (Failsafe Enabled)

| Time Since Last Data | Action |
|---------------------|--------|
| 0 - 30 min | Normal operation. `resetSerialWatchdog()` called on every received `{`. |
| 30 min | **Watchdog pulse #1**: GPIO39 goes HIGH for **1 second** (short power-cycle attempt). Fault code **2048** added. |
| 60 min | **Watchdog pulse #2**: GPIO39 goes HIGH for **30 seconds** (long power-cycle). |
| After 2 failed attempts | **Failsafe mode**: ESP32 takes autonomous control. Fault code **16384** added. |

### Failsafe Mode (When Enabled and Triggered)

When failsafe is both enabled AND watchdog has triggered twice:
- ESP32 runs autonomous cycle engine using ADC pressure/current readings
- Uses active profile settings for run/purge/burp timing
- CBOR data continues sending to cloud every 15 seconds
- Web portal still works for manual control
- Fault code **16384** indicates active failsafe operation

**Exiting failsafe:** When serial data resumes (any valid JSON), ESP32 automatically exits failsafe and returns to normal Linux-controlled operation.

### Fault Code Bitmask

The ESP32 adds these codes to the `fault` value from Linux:

| Code | Meaning |
|------|---------|
| 1024 | SD card failure |
| 2048 | Serial watchdog triggered (no data for 30+ minutes) |
| 4096 | Failsafe enabled (setting - can enter failsafe if watchdog triggers) |
| 8192 | BlueCherry cloud platform offline |
| 16384 | Failsafe mode active (ESP32 autonomously controlling relays) |

**Examples:**
- Watchdog triggered: `fault = 2048`
- Failsafe enabled but not active: `fault = 4096`
- Failsafe actively running: `fault = 4096 + 16384 = 20480`
- All faults: `fault = 1024 + 2048 + 4096 + 8192 + 16384 = 31744`

---

## Linux Alarm Bitmask (fault field)

The Linux device should send an alarm bitmask in the `fault` field of the periodic payload. The ESP32 stores this and includes it in CBOR transmissions.

| Bit | Value | Alarm Name |
|-----|-------|------------|
| 0 | 1 | Vacuum pump fault |
| 1 | 2 | Panel power issue |
| 2 | 4 | Overfill detected |
| 3 | 8 | Digital storage fault |
| 4 | 16 | Under pressure |
| 5 | 32 | Over pressure |
| 6 | 64 | Zero pressure |
| 7 | 128 | Variable pressure |
| 8 | 256 | Pressure sensor fault |
| 9 | 512 | 72-hour shutdown |

---

## CBOR Data Format (what gets sent to the cloud)

The ESP32 batches readings and sends them via CoAP/CBOR to the BlueCherry cloud platform. Each reading contains 8 fields:

| Index | Field | Source | Notes |
|-------|-------|--------|-------|
| 0 | Device ID | Extracted numeric ID from `gmid` | e.g., `"CSX-1234"` → `1234` |
| 1 | Sequence | ESP32 local counter | Incremented per reading |
| 2 | Pressure x100 | **ESP32 ADS1015 ADC** | NOT from Linux `press` field |
| 3 | Cycles | Linux `cycles` field | Round-tripped from Linux |
| 4 | Faults | Linux `fault` + ESP32 faults | Additive (see fault codes above) |
| 5 | Mode | **ESP32 `currentRelayMode`** | NOT from Linux `mode` round-trip |
| 6 | Temperature x100 | ESP32 internal temp sensor (degF) | Chip temperature, not ambient |
| 7 | Current x100 | **ESP32 ADS1015 ADC** | NOT from Linux `current` field |

Readings are batched in groups of **12** then CBOR-encoded and sent. Same data is also written to the SD card at `/logfileYYYY.log`.

**Key point:** Pressure, current, and mode in the CBOR data come from the ESP32's own sensors and state, NOT from the Linux device's serial payload. This means CBOR data is always fresh even if the Linux device falls behind on its 15-second updates.

---

## Web Portal Test Mode Protection

When a test is started from the web portal (e.g., leak test), the ESP32 sets `testRunning = true` and tracks the test type. While a test is running:

- **Non-zero** mode commands from serial are **IGNORED** (protects the test from being overridden by Linux periodic data)
- **Mode 0** (rest) from serial **always passes through** and clears the test state (universal stop signal)
- Tests can also be stopped via the web portal Stop button

This prevents a race condition where the Linux device's periodic 15-second payload could reset the relay mode to 0 while a web portal test is in progress.

---

## Mode Delivery Redundancy (Rev 10.8)

The relay mode has **three layers of delivery** to compensate for missed serial commands. No Python changes are required — the existing Linux 15-second payload already provides the retry mechanism.

### Layer 1: Immediate Mode Command (Linux → ESP32)

When the Linux device changes mode, it sends `{"type":"data","mode":N}` immediately. Latency: ~100-200ms. This is the primary relay control path.

### Layer 2: 15-Second Payload Re-send (Linux → ESP32)

The Linux device's `create_payload()` always includes the current `mode` field in every 15-second status packet. The ESP32 applies the mode **every time** it receives a data packet (not just on change). If an immediate mode command is lost, the next 15-second payload corrects it.

### Layer 3: ESP32 Relay State Refresh (ESP32 internal)

Every 15 seconds, the ESP32 calls `setRelaysForMode(currentRelayMode)` to re-assert the stored mode to physical GPIO pins. This guards against:

- A relay driver glitch that silently dropped a pin
- A GPIO write that failed during the original mode application
- Any drift between the `currentRelayMode` variable and actual pin states

This refresh is **skipped** during failsafe mode (failsafe manages its own relay cycle) and during passthrough mode (relays should be idle).

### Worst-Case Recovery

If an immediate mode command is lost, the next 15-second Linux payload corrects the mode. If that payload is also lost, the one after that (30 seconds max). Meanwhile, the relay refresh ensures the ESP32 keeps driving the correct GPIO states for whatever mode it last received.

```
  Time ─────────────────────────────────────────────────►
  
  Linux:    MODE 1 ──────────── payload(mode=1) ──── payload(mode=1) ───
                │                     │                     │
  ESP32:        ▼ setRelaysForMode(1) ▼ setRelaysForMode(1) ▼ setRelaysForMode(1)
                                      ▲                     ▲
  Refresh:                 refresh(1) ┘          refresh(1) ┘
                           (every 15s)           (every 15s)
```

---

## Timing Summary

### RS-232 Serial Communication (Linux Control System)

| Event | Interval | Direction | Purpose |
|-------|----------|-----------|---------|
| Fast sensor packet | 200ms (5Hz) | ESP32 → Linux | Real-time pressure, current, overfill, SD status, failsafe, shutdown |
| Cellular status | 10s (timer) | ESP32 → Linux | Cached cellular info (lte, rsrp, rsrq, operator, band, cell tower) |
| Datetime packet | Event-driven | ESP32 → Linux | Fresh modem clock only — never repeated with stale values |
| Periodic data payload | 15s | Linux → ESP32 | CBOR/SD logging data (gmid, fault, cycles) + mode re-send |
| Immediate mode command | On change | Linux → ESP32 | Relay control (<200ms response) |
| Relay state refresh | 15s | ESP32 internal | Re-applies currentRelayMode to GPIO pins |
| Mode polling (internal) | 100ms | Linux internal | ModeManager mmap check for child process mode changes |
| ESP32 serial check | 100ms | ESP32 internal | How often ESP32 checks Serial1 for incoming data |
| Serial watchdog timeout | 30 min | ESP32 internal | Triggers power-cycle pulse on GPIO39 |

### Web Portal Commands (Browser to ESP32)

| Event | Interval | Direction | Purpose |
|-------|----------|-----------|---------|
| Web command processing | <50ms | Browser → ESP32 | HTTP POST to /api/command endpoint |
| Calibration operation | <1s | ESP32 direct | Pressure sensor zero point calibration |
| Relay toggle response | <10ms | ESP32 direct | Immediate GPIO state change |
| Test start operation | <100ms | ESP32 direct | Initialize test sequence and take relay control |
| Profile change | <50ms | ESP32 direct | Switch active equipment profile (password protected) |
| ESP32 restart | <500ms delay | ESP32 direct | Reboot with 500ms response delay |

### ESPNow Wireless Communication (Display Panel)

| Event | Interval | Direction | Purpose |
|-------|----------|-----------|---------|
| Fast sensor packet | 200ms (5Hz) | ESP32 → Display | Same real-time data as serial channel |
| Cellular status | 10s (timer) | ESP32 → Display | Network status for remote monitoring |
| Datetime packet | Event-driven | ESP32 → Display | Fresh modem time for display |
| Discovery beacon | 2s (when unpaired) | ESP32 broadcast | Allows display panels to find IO Board |
| Heartbeat timeout | 10s (silence threshold) | ESP32 monitor | Detects display panel disconnection |
| Display heartbeat | 5s | Display → ESP32 | Maintains wireless connection |
| ESPNow message processing | 100ms | ESP32 internal | Checks for incoming wireless data |
| Fragment reassembly | Variable | ESP32 internal | Combines multi-packet messages |

### Communication Channel Priority

- **ESPNow Connected**: All outgoing data (sensors, cellular) goes to display panel via ESPNow
- **ESPNow Disconnected**: All outgoing data goes to Linux via RS-232 serial
- **Incoming Commands**: Accepted from both channels simultaneously
- **Watchdog Reset**: Occurs when data is received from either channel

---

## Design Decisions and Why

### Why serial instead of I2C?
The previous hardware (MCP23017 + ADS1115) used I2C with the Linux device as bus master. This caused frequent field failures: bus lockups from electrical noise, timing conflicts between ADC reads and relay writes, and complex recovery logic. Serial is more reliable over longer wiring runs and eliminates an entire class of failures.

### Why does the ESP32 read its own ADC?
Moving ADC reading to the ESP32 eliminates I2C bus contention, provides consistent 60Hz sampling (Python threads couldn't guarantee this), and means the pressure data in CBOR is always fresh regardless of the Linux device's polling rate.

### Why two separate packet types from ESP32?
Pressure and current change rapidly and need frequent updates for real-time control decisions. Cellular signal data changes slowly (~60 seconds) and is expensive to query from the modem (~2-5 seconds per query). Separating them avoids either over-querying the modem or under-reporting sensor data.

### Why does mode 0 bypass the testRunning guard?
Without this, the `testRunning` flag could become permanently stuck. If a web portal test starts but the browser is closed without pressing Stop, `testRunning` stays true. Since the guard blocks all serial mode commands, there would be no way for Linux to clear it. Mode 0 (rest) is the universal "stop everything" signal and must always work.

### Why does the Linux device send pressure/current in the 15s payload if the ESP32 doesn't use them?
Historical compatibility and debugging. The CBOR format was originally designed for a system where Linux read the sensors. The ESP32 now reads its own sensors for CBOR but keeping the fields in the serial payload allows the web portal diagnostics page to display what Linux thinks the values are, which is useful for troubleshooting sensor disagreements.

### Why ESPNow instead of WiFi TCP/UDP for display panels?
ESPNow provides several advantages for industrial control applications:
- **Lower latency**: Direct radio communication without TCP/IP overhead
- **Better reliability**: No connection drops from IP address changes or network congestion
- **Lower power consumption**: Optimized for short, frequent messages
- **Simpler implementation**: No need for IP addressing, DHCP, or socket programming
- **Deterministic timing**: Messages arrive in order without retransmission delays

### Why automatic ESPNow discovery instead of manual MAC address configuration?
Manual configuration creates maintenance overhead and potential errors:
- MAC addresses must be entered correctly on both devices
- Device replacement requires reconfiguration
- Multiple IO Boards in the same area require careful MAC management
- Automatic discovery eliminates these issues while maintaining security through proximity requirements

### Why ESPNow takes priority over serial when both are connected?
Display panels serve as the primary human-machine interface:
- Operators expect immediate feedback from the panel they're using
- Serial communication continues to work for all control functions
- The system gracefully falls back to serial-only operation when wireless disconnects
- Watchdog monitoring covers both communication channels

### Why does the periodic payload only send every 15 seconds?
This payload feeds the CBOR builder which transmits to the cloud. The cloud ingestion rate is 15-second intervals. Sending more frequently would waste bandwidth and not improve cloud data granularity. The real-time control path (immediate mode commands) is separate and much faster.

### Why use a memory-mapped file for mode state?
The Linux application uses `multiprocessing` (fork) for cycle sequences. After fork, file descriptors for serial ports are unreliable. The mmap file (`/tmp/vst_mode.bin`) is safely shared between parent and child processes. The child writes the mode, the parent's polling loop reads it and sends to the ESP32.

---

## Quick Start for New Implementations

### Linux Control System Implementation

1. **Open** `/dev/serial0` at 115200 baud, 8N1, no flow control
2. **Start receiving** — read lines, parse JSON, store sensor values
3. **Start sending periodic payload** every 15 seconds with device ID, fault code, cycle count
4. **Send mode commands immediately** when you need to change relay states: `{"type":"data","mode":N}`
5. **Handle web portal commands** — parse `{"command":"..."}` messages and act on them
6. **Drain the buffer** — always read ALL available data and use only the latest JSON line. If you read only one line at a time, you'll fall behind and get stale data.
7. **Send mode 0** on shutdown/cleanup to idle all relays

### Display Panel Implementation (ESPNow)

1. **Initialize ESPNow** in STA mode on WiFi channel 1
2. **Listen for beacons** from IO Board (broadcast every 2 seconds)
3. **Send pair request** when beacon received: `{"type":"pair_request","name":"Display-1"}`
4. **Wait for confirmation** and start receiving sensor data
5. **Send keepalive** every 5 seconds: `{"type":"keepalive"}`
6. **Send commands** using same JSON format as serial (modes, calibration, etc.)
7. **Handle fragmentation** for messages larger than 248 bytes
8. **Reconnect automatically** if connection lost (watch for new beacons)

**Minimal "hello world" to verify serial works:**

```python
import serial, json, time

ser = serial.Serial('/dev/serial0', 115200, timeout=1)

# Read a few sensor packets
for _ in range(10):
    line = ser.readline().decode('ascii', errors='ignore').strip()
    if line.startswith('{'):
        data = json.loads(line)
        print(f"Pressure: {data.get('pressure')} IWC, Current: {data.get('current')} A")
    time.sleep(0.2)

# Send a mode command (mode 1 = RUN)
ser.write(b'{"type":"data","mode":1}\n')
ser.flush()
print("Sent mode 1 (RUN) — motor and valves should activate")

time.sleep(5)

# Return to rest
ser.write(b'{"type":"data","mode":0}\n')
ser.flush()
print("Sent mode 0 (REST) — all relays off")

ser.close()
```
