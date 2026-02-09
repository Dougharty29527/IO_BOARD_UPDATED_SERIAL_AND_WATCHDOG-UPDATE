# ESP32 IO Board Serial Protocol Guide

**Firmware:** IO_BOARD_FIRMWARE9 Rev 10.5  
**Date:** February 9, 2026  
**Audience:** Anyone writing software that talks to this ESP32 over serial

---

## Purpose of This Document

The ESP32 IO Board communicates with a Linux device (Raspberry Pi / Comfile) over a serial RS-232 connection. The ESP32 handles all physical hardware: relay switching, ADC sensor reading, overfill detection, SD card logging, cellular modem, and a WiFi web portal. The Linux device handles business logic: deciding when to run cycles, detecting alarm conditions, and sending status to the cloud.

This document describes every byte on the wire between the two devices. If you are rewriting the Linux-side software, this is the complete specification.

---

## Physical Layer

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

All messages are **JSON objects**, one per line, terminated by `\n` (newline). The ESP32 uses `Serial1.println()` which appends `\r\n`. The Linux device should be tolerant of both `\n` and `\r\n`.

The ESP32 has three serial ports:
- **Serial** (USB): Debug output only. Not part of this protocol.
- **Serial1** (GPIO44/43): This protocol. All communication with the Linux device.
- **Serial2** (GPIO14/48): Walter cellular modem. Not accessible to Linux.

---

## Message Flow Overview

```
Linux Device                    ESP32 IO Board
     │                               │
     │  ◄─── Fast Sensor Packet ───  │  Every 200ms (5Hz)
     │       (pressure, current,     │  Sent regardless of Linux state
     │        overfill, sdcard)      │
     │                               │
     │  ◄─── Cellular Status ─────  │  Only when modem returns fresh data (~60s)
     │       (datetime, lte, rsrp,   │  Never repeated with stale values
     │        rsrq, operator, etc.)  │
     │                               │
     │  ── Data Payload ──────────►  │  Every 15 seconds
     │     (gmid, mode, fault,       │  Used for CBOR/cellular/SD logging
     │      cycles, press, current)  │
     │                               │
     │  ── Immediate Mode ────────►  │  On mode change (within 100ms)
     │     (mode number only)        │  This is the relay control path
     │                               │
     │  ◄─── Web Portal Command ──  │  When user presses button on web portal
     │       (start_cycle, stop,     │
     │        start_test, etc.)      │
     │                               │
     │  ── Command ───────────────►  │  Passthrough, set_profile, etc.
     │                               │
```

---

## ESP32 to Linux Messages

### 1. Fast Sensor Packet (5Hz / every 200ms)

This is the critical real-time data path. The Linux device uses these values for all operational decisions.

```json
{"pressure":-14.22,"current":0.07,"overfill":0,"sdcard":"OK","relayMode":1}
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `pressure` | float | Typically -20.0 to +5.0 | Vacuum pressure in inches of water column (IWC). Read from ADS1015 ADC channel 0 at 60Hz with 1-second rolling average. Negative = vacuum. |
| `current` | float | 0.0 to ~15.0 | Motor current in amps. Read from ADS1015 ADC channels 2-3 (differential) at 60Hz with rolling average. 0.0 when motor is off. |
| `overfill` | int | 0 or 1 | Overfill alarm state. 0 = normal, 1 = alarm active. Read from GPIO38 with 8-of-5 hysteresis (must read LOW 8 out of last 5 checks to trigger). Has internal pull-up. |
| `sdcard` | string | `"OK"` or `"FAULT"` | SD card status. Reads cached card type, no disk I/O. |
| `relayMode` | int | 0-9 | Current relay mode the ESP32 has applied. Useful for confirming the relay state matches what Linux requested. |

**Why 5Hz:** The Linux controller makes real-time decisions based on pressure (start/stop cycles, detect alarms). 200ms updates give the controller sub-second response to pressure changes. The ADC itself samples at 60Hz internally; the 5Hz serial rate is a practical balance between freshness and serial bandwidth.

**Why the ESP32 reads its own ADC:** In the previous hardware (MCP23017 + ADS1115), the Linux device read the ADC directly over I2C. This caused bus contention, timing conflicts, and stale data when the buffer backed up. Moving ADC reading to the ESP32 eliminates all of that.

---

### 2. Cellular Status Packet (event-driven, ~60s)

Sent only when the Walter modem library returns genuinely new data. Never repeated with cached values.

```json
{"datetime":"2026-02-09 13:10:49","passthrough":0,"lte":1,"rsrp":"-85.5","rsrq":"-10.2","operator":"T-Mobile","band":"B2","mcc":310,"mnc":260,"cellId":12345678,"tac":9876,"profile":"CS8","failsafe":0}
```

| Field | Type | Description |
|-------|------|-------------|
| `datetime` | string | UTC timestamp from the cellular modem clock. Format: `"YYYY-MM-DD HH:MM:SS"` |
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
| `failsafe` | int | 1 = ESP32 is in autonomous failsafe mode (Linux was unresponsive), 0 = normal |

**Why event-driven:** The cellular modem is slow to query (~2-5 seconds per signal info request). Sending only on fresh data avoids flooding the serial line with repeated stale values and avoids confusing the Linux log. Signal info refreshes every ~60 seconds; time syncs every ~15 minutes.

---

### 3. Web Portal Command (forwarded from ESP32 web UI)

When a user interacts with the ESP32's WiFi web portal (default IP: `192.168.4.1`), certain commands are forwarded to the Linux device over serial. The ESP32 does NOT execute these commands itself (unless in failsafe mode).

**Cycle commands:**
```json
{"command":"start_cycle","type":"run"}
{"command":"start_cycle","type":"manual_purge"}
{"command":"start_cycle","type":"clean"}
{"command":"stop_cycle"}
```

**Test commands:**
```json
{"command":"start_test","type":"leak"}
{"command":"start_test","type":"func"}
{"command":"start_test","type":"eff"}
```

Note: `stop_test` is forwarded as `{"command":"stop_cycle"}` (same as stop_cycle — Python's `stop_cycle()` stops everything).

| Field | Description |
|-------|-------------|
| `command` | The action: `"start_cycle"`, `"stop_cycle"`, or `"start_test"` |
| `type` | For start commands: `"run"`, `"manual_purge"`, `"clean"`, `"leak"`, `"func"`, `"eff"` |

**What the Linux device should do with these:**
- `start_cycle/run` — Start a normal run cycle (same as pressing Start on the touchscreen)
- `start_cycle/manual_purge` — Start a manual purge cycle
- `start_cycle/clean` — Start a canister clean (15 min motor run)
- `stop_cycle` — Stop whatever is currently running
- `start_test/leak` — Start a leak test (30 min, mode 9, all valves open, motor off)
- `start_test/func` — Start a functionality test (10 x run/purge cycles)
- `start_test/eff` — Start an efficiency test (120s fill/run)

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

## Linux to ESP32 Messages

### 1. Periodic Data Payload (every 15 seconds)

This is the primary data feed for the ESP32's CBOR encoder and SD card logger.

```json
{"type":"data","gmid":"CSX-1234","press":-14.22,"mode":0,"current":0.07,"fault":0,"cycles":484}
```

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `type` | string | **Yes** | Must be `"data"`. Any other value is rejected. |
| `gmid` | string | No | Device name/ID (e.g., `"CSX-1234"`). If changed, ESP32 updates EEPROM and WiFi AP name. |
| `press` | float | No | Pressure from Linux (informational). **ESP32 uses its own ADC for CBOR/SD, not this value.** Included for consistency and debugging. |
| `mode` | int or `"shutdown"` | No | Relay mode number (0-9) or the string `"shutdown"`. See Mode Numbers below. |
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

### 3. Shutdown Command

```json
{"type":"data","mode":"shutdown"}
```

This is the **only** way to activate the DISP_SHUTDN output (GPIO13 LOW). The `mode` field is the **string** `"shutdown"` (not a number). GPIO13 starts HIGH on every ESP32 boot and stays HIGH until this command is received. There is no "un-shutdown" command — the ESP32 must be restarted.

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

---

## Serial Watchdog and Failsafe Mode

The ESP32 monitors the serial line for incoming data. If no JSON message is received for **30 minutes**, the watchdog triggers.

### Watchdog Timeline

| Time Since Last Data | Action |
|---------------------|--------|
| 0 - 30 min | Normal operation. `resetSerialWatchdog()` called on every received `{`. |
| 30 min | **Watchdog pulse #1**: GPIO39 goes HIGH for **1 second** (short power-cycle attempt on the Linux device). Fault code **1024** added. |
| 60 min | **Watchdog pulse #2**: GPIO39 goes HIGH for **30 seconds** (long power-cycle). |
| After 2 failed attempts | **Failsafe mode**: ESP32 takes autonomous control of relays using its own pressure readings. Fault code **8192** added. |

### Failsafe Mode

When the ESP32 enters failsafe mode:
- It runs its own cycle engine (auto-start based on pressure thresholds)
- It uses the active profile's settings for run/purge/burp timing
- CBOR data continues to be sent to the cloud (with fault code 8192)
- The web portal still works and can control cycles

**Exiting failsafe:** When serial data resumes (Linux sends any valid JSON), the ESP32 automatically exits failsafe mode and returns to normal operation (Linux controls relays).

### Fault Code Bitmask

The ESP32 adds these codes to the `fault` value from Linux:

| Code | Meaning |
|------|---------|
| 1024 | Serial watchdog triggered (no data for 30+ minutes) |
| 4096 | BlueCherry cloud platform offline |
| 8192 | Failsafe mode active (ESP32 running autonomously) |

These are additive. A device in failsafe with no cloud connection would have fault = Linux_faults + 8192 + 4096.

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

## Timing Summary

| Event | Interval | Direction | Purpose |
|-------|----------|-----------|---------|
| Fast sensor packet | 200ms (5Hz) | ESP32 → Linux | Real-time pressure, current, overfill, SD status |
| Cellular status | ~60s (event-driven) | ESP32 → Linux | Modem data, only when fresh |
| Periodic data payload | 15s | Linux → ESP32 | CBOR/SD logging data (gmid, fault, cycles) |
| Immediate mode command | On change | Linux → ESP32 | Relay control (<200ms response) |
| Mode polling (internal) | 100ms | Linux internal | ModeManager mmap check for child process mode changes |
| ESP32 serial check | 100ms | ESP32 internal | How often ESP32 checks Serial1 for incoming data |
| Serial watchdog timeout | 30 min | ESP32 internal | Triggers power-cycle pulse on GPIO39 |

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

### Why does the periodic payload only send every 15 seconds?
This payload feeds the CBOR builder which transmits to the cloud. The cloud ingestion rate is 15-second intervals. Sending more frequently would waste bandwidth and not improve cloud data granularity. The real-time control path (immediate mode commands) is separate and much faster.

### Why use a memory-mapped file for mode state?
The Linux application uses `multiprocessing` (fork) for cycle sequences. After fork, file descriptors for serial ports are unreliable. The mmap file (`/tmp/vst_mode.bin`) is safely shared between parent and child processes. The child writes the mode, the parent's polling loop reads it and sends to the ESP32.

---

## Quick Start for a New Linux Implementation

1. **Open** `/dev/serial0` at 115200 baud, 8N1, no flow control
2. **Start receiving** — read lines, parse JSON, store sensor values
3. **Start sending periodic payload** every 15 seconds with device ID, fault code, cycle count
4. **Send mode commands immediately** when you need to change relay states: `{"type":"data","mode":N}`
5. **Handle web portal commands** — parse `{"command":"..."}` messages and act on them
6. **Drain the buffer** — always read ALL available data and use only the latest JSON line. If you read only one line at a time, you'll fall behind and get stale data.
7. **Send mode 0** on shutdown/cleanup to idle all relays

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
