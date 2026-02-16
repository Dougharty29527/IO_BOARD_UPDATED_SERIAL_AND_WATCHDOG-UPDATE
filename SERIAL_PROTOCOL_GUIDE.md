# ESP32 IO Board Serial Protocol Guide

**Firmware:** IO_BOARD_FIRMWARE9 Rev 10.13  
**Date:** February 10, 2026  
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
     │  ◄─── Cellular Status ─────  │  Every 10 seconds (cached values)
     │       (lte, rsrp, rsrq,      │  Always sent on timer
     │        operator, band, etc.)  │
     │                               │
     │  ◄─── Datetime Packet ─────  │  Only when fresh from modem clock
     │       (datetime only)         │  Never repeated with stale values
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
{"pressure":-14.22,"current":0.07,"overfill":0,"sdcard":"OK","relayMode":1,"failsafe":0,"shutdown":0}
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `pressure` | float | Typically +/-36, or **-99.9** | Vacuum pressure in inches of water column (IWC). The pressure sensor measures vacuum levels with 1-second averaging. Negative values indicate vacuum (normal operation), 0.0 is atmospheric pressure, positive values are above atmospheric. A value of -99.9 indicates a sensor fault. |
| `current` | float | 0.0 to ~15.0 | Motor current in amps. Read from ADS1015 ADC channels 2-3 using **hardware differential** (`readADC_Differential_2_3()`) at GAIN_TWO (1mV/count) for full 12-bit precision on the difference. Reports **windowed peak** (max after removing highest outlier, 60-sample window) to match Python's approach. 0.0 when motor is off. Pin 1 (AIN1) is unused. **Rev 10.11: Sent as 0.0 during sensor fault (when pressure is -99.9).** |
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
{"passthrough":0,"lte":1,"rsrp":"-85.5","rsrq":"-10.2","operator":"T-Mobile","band":"B2","mcc":310,"mnc":260,"cellId":12345678,"tac":9876,"profile":"CS8","imei":"351234567890123","imsi":"310410123456789","iccid":"8901260882310000000","technology":"LTE-M"}
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

**Why 10-second timer:** Guarantees the Linux device always has cellular info, even during modem communication failures. Uses cached values from the last successful query — stale but useful beats missing entirely.

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

## Serial Watchdog and Failsafe Mode

The ESP32 monitors the serial line for incoming data. Watchdog behavior depends on whether failsafe mode is enabled or disabled.

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

| Event | Interval | Direction | Purpose |
|-------|----------|-----------|---------|
| Fast sensor packet | 200ms (5Hz) | ESP32 → Linux | Real-time pressure, current, overfill, SD status, failsafe, shutdown |
| Cellular status | 10s (timer) | ESP32 → Linux | Cached cellular info (lte, rsrp, rsrq, operator, band, cell tower) |
| Datetime packet | Event-driven | ESP32 → Linux | Fresh modem clock only — never repeated with stale values |
| Periodic data payload | 15s | Linux → ESP32 | CBOR/SD logging data (gmid, fault, cycles) + mode re-send |
| Immediate mode command | On change | Linux → ESP32 | Relay control (<200ms response) |
| **Relay state refresh** | **15s** | **ESP32 internal** | **Re-applies currentRelayMode to GPIO pins (Rev 10.8)** |
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
