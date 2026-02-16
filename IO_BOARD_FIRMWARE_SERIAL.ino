/* ********************************************
 *
 *  Walter IO Board Firmware - Rev 10.20
 *  Date: 2/16/2026
 *  Written By: Todd Adams & Doug Harty
 *  
 *  Based on:
 *  - Rev 9.4f (MCP23017 Emulator + Diagnostic Dashboard)
 *  - RMS_CBOR_1_16 (Walter RMS w/OTA and ZTA)
 *  - Python control.py (Relay cycle logic, profile management, alarm monitoring)
 *  
 *  =====================================================================
 *  REVISION HISTORY (newest first)
 *  =====================================================================
 *
 *  Rev 10.20 (2/16/2026) - Manual ADC Zero Calibration + Serial Debug Logs
 *  - NEW: Manual ADC zero calibration feature (set_manual_adc_zero command)
 *    * Allows manual setting of pressure sensor zero point with validation (500-2000 range)
 *    * Saves calibration to EEPROM and clears pressure averaging buffer
 *    * Web portal integration with user-friendly controls
 *    * Notifies Python program of calibration changes via Serial1
 *  - NEW: Serial data log file for debugging ESP32-Python communication
 *    * Contains log output showing ESP32 packet parsing and sensor data
 *    * Useful for troubleshooting serial buffer backlog and communication issues
 *    * Shows pressure readings, timestamps, and protocol debugging information
 *  - IMPROVED: Enhanced sendFastSensorPacket() buffer size for additional JSON fields
 *  - IMPROVED: startConfigAP() function with additional configuration options
 *
 *  Rev 10.19 (2/13/2026) - Web Portal Enhancements + Direct Relay Control
 *  - NEW: executeRemoteCommand() central dispatch — all remote commands (Web Portal,
 *    BlueCherry, Serial1 from Python) route through one function for identical behavior
 *  - NEW: notifyPythonOfCommand() helper — sends unified JSON to Python on Serial1
 *  - NEW: Manual relay override mode (cr0/cr1/cr2/cr5 commands with value 0/1)
 *    * Enters manualRelayOverride mode on any relay command
 *    * Blocks 15-second relay refresh and Linux serial mode commands
 *    * exit_manual command or starting a new cycle/test returns to normal operation
 *  - NEW: enable_failsafe / exit_failsafe commands from Web Portal and BlueCherry
 *  - NEW: BlueCherry keywords for all unified commands (stop_test, overfill_override,
 *    clear_press_alarm, clear_motor_alarm, enable_failsafe, exit_failsafe,
 *    cr0/cr1/cr2/cr5 <0|1>, exit_manual, calibrate_pressure)
 *  - NEW: parsedSerialData() accepts unified {"command":"<name>","value":"<val>"} format
 *    from Python in addition to legacy {"type":"cmd","cmd":"cal"} format
 *  - NEW: Dynamic captive portal status display with mobile-friendly design
 *    * Shows UST pressure and run cycles with real-time data
 *    * Red alarm card when any ESP32 alarm is active
 *    * Date/time in top right corner (mobile-optimized layout)
 *    * "VST GREEN MACHINE" header with darker blue styling
 *    * "Advanced Controls" button at bottom linking to full dashboard
 *  - FIX: stop_test now sends correct command name to Python (was sending "stop_cycle")
 *  - Removed JSON "source" field from Serial1 messages to minimize serial traffic
 *  - Updated calibration response format: {"command":"calibrate_pressure","ps_cal":...}
 *  - Web Portal /api/command handler refactored to delegate to executeRemoteCommand()
 *  
 *  Rev 10.17 (2/12/2026) - Fault Code Reassignment + SD Card Fault
 *  - BREAKING: Fault codes reassigned (coordinate with Linux/server if parsing these)
 *    * 1024: SD card failure (NEW — previously used for watchdog)
 *    * 2048: Serial watchdog activated (was 1024)
 *    * 4096: Failsafe mode (was 8192)
 *    * 8192: BlueCherry connection failed (was 4096)
 *  - NEW: SD card fault detection via getSDCardFaultCode()
 *    * Returns 1024 when SD card is not mounted or inaccessible
 *    * Uses existing isSDCardOK() check (SD.cardType() != CARD_NONE)
 *  - IMPROVED: getCombinedFaultCode() now includes all 4 fault sources
 *  - IMPROVED: buildSendDataArray() uses getCombinedFaultCode() instead of
 *    only getBlueCherryFaultCode(), so all ESP32 faults are sent to BlueCherry
 *
 *  Rev 10.16 (2/12/2026) - Staggered Relay Pin Switching for Noise Reduction
 *  - NEW: RELAY_DELAY define (default 1ms) — easily adjustable for testing
 *    * Controls delay between each individual relay GPIO pin change
 *    * Simultaneous activation of CR0_MOTOR, CR1, CR2, CR5 caused electrical
 *      noise coupling into ADS1015 analog inputs during mode transitions
 *  - IMPROVED: Relay pin changes now staggered by RELAY_DELAY ms between each pin
 *    * setRelaysForMode() mutex path: all 7 mode cases (0,1,2,3,8,9,default)
 *    * setRelaysForMode() fallback path: all 5 mode cases
 *    * Boot-time relay init in adcReaderTask(): 4 pins set LOW at startup
 *    * Total time per mode change: ~3 × RELAY_DELAY ms (3 gaps between 4 pins)
 *    * Replaces single post-switch delay with per-pin staggering
 *
 *  Rev 10.15 (2/11/2026) - Relay Noise Settling + Skip Redundant Relay Writes
 *  - NEW: 1ms delay after relay GPIO writes in setRelaysForMode()
 *    * Allows solenoid coil back-EMF and relay contact bounce to settle
 *    * Applied in both mutex and fallback (safety) code paths
 *    * Prevents EMI transients from coupling into ADS1015 analog inputs
 *  - IMPROVED: Serial data handler skips redundant setRelaysForMode() calls
 *    * Linux sends mode in every JSON data packet (~every 7 seconds)
 *    * Previously, every packet triggered full GPIO writes even if mode unchanged
 *    * Now only calls setRelaysForMode() when mode actually changes
 *    * 15-second relay refresh still force-writes as a safety net against pin drift
 *    * Eliminates unnecessary relay driver activity during steady-state operation
 *
 *  Rev 10.14 (2/11/2026) - ADC/SD Card SPI Interference Fix
 *  - BUG FIX: Pressure reading showed spurious -0.71 IWC every ~7 seconds
 *    * Root cause: SD card SPI write traffic (~500-800ms) couples ~32mV analog
 *      interference into ADS1015, biasing raw ADC by ~16 counts below true value
 *    * Affected readings entered 60-sample rolling average, contaminating it
 *    * Pattern: P=0.00 → P=-0.71 for ~800ms → P=0.00 (repeats on each SD write)
 *  - FIX: Cross-core SD write gate (sdWriteActive flag)
 *    * SaveToSD() sets sdWriteActive=true before SPI operations, clears after
 *    * ADC reader task (Core 0) checks flag — skips I2C reads while SPI is active
 *    * Rolling average holds last good values during SD writes (no contamination)
 *    * Zero-latency recovery: ADC resumes reading instantly when SPI completes
 *  - NOTE: ~50 ADC samples (~800ms at 60Hz) are skipped per SD write cycle.
 *    The rolling average uses the previous 60 good samples during this window,
 *    so pressure output remains stable and accurate.
 *
 *  Rev 10.13 (2/11/2026) - Non-Blocking Cellular Initialization
 *  - ARCHITECTURE: Cellular connection no longer blocks setup() for 5+ minutes.
 *    * setup() completes in ~3 seconds — GPIO, ADC, serial, web portal, relays all operational
 *    * LTE registration, BlueCherry, modem identity, and modem time run as a background
 *      state machine (runCellularInitStateMachine) in loop()
 *    * 13-state CellularInitState enum drives the connection process
 *    * Each state performs ONE operation and returns immediately to loop()
 *    * Serial to Linux (5Hz pressure/current/overfill) starts immediately on boot
 *    * Web portal works immediately — relay control, tests, diagnostics all available
 *    * Watchdog starts immediately — no 5-minute blind spot
 *  - FIX: No more ESP.restart() on LTE failure during boot
 *    * Previously, failed lteConnect() → 10s delay → restart → infinite loop on poor signal
 *    * Now: state machine retries from CELL_INIT_SET_MINIMUM after 60-second cooldown
 *    * System continues operating (serial, relays, web) while retrying cellular
 *  - FIX: initModemTime() had no timeout (infinite hang possible)
 *    * Now limited to single attempt with non-fatal fallback
 *  - NEW: Web dashboard shows "Connecting..." (orange) during cellular init instead of
 *    "Disconnected" (red). cellularReady field added to /api/status JSON.
 *  - All modem-dependent loop operations (LTE reconnect, firmware check, daily status,
 *    refreshCellSignalInfo) gated on cellularInitComplete flag
 *
 *  Rev 10.12 (2/11/2026) - Functionality Test Rework + Watchdog Pull-Down + Device Name Fix
 *  - FIX: Functionality test was incorrectly using mode 9 (leak test config: all valves open,
 *    motor OFF). Now correctly mirrors Python io_manager.py functionality_test():
 *    * 10 repetitions of Run (60s) → Purge (60s) = 20 steps, 1200 seconds (20 min)
 *    * Motor runs during BOTH Run (mode 1) and Purge (mode 2), only off when test completes
 *    * New FUNCTIONALITY_TEST_CYCLE[] CycleStep array (20 steps)
 *    * Multi-step web test engine in loop() steps through sequence automatically
 *    * Web UI shows Cycle (e.g., "3 of 10") and Phase (Run/Purge) during func test
 *    * /api/status includes testMultiStep, testStep, testStepTotal fields
 *  - NEW: Watchdog pin (GPIO39) internal pull-down resistor enabled via ESP-IDF
 *    * gpio_pulldown_en() keeps pin LOW during reboots/firmware updates
 *    * Prevents floating pin from triggering unwanted watchdog pulse
 *    * Complements the DISP_SHUTDN (GPIO13) pull-up from Rev 10.11
 *  - FIX: Device name set via web portal was overwritten by Linux every 15 seconds
 *    * Root cause: Linux sends gmid in periodic JSON; ESP32 accepted it and overwrote EEPROM
 *    * Fix: ESP32 device name is now exclusively set via web portal /setdevicename endpoint
 *    * Linux gmid is logged for diagnostics but never overwrites the stored device name
 *    * Also removed legacy HTTP response device name overwrite
 *
 *  Rev 10.11 (2/11/2026) - ADC Stale Data Detection & DISP_SHUTDN GPIO Hold
 *  - BUG FIX: FAST-TX packets sent stale pressure/current when ADC failed silently
 *    * Previously: adcPressure and adcCurrent globals retained last good values forever
 *    * If ADC reads failed (I2C errors, bus hang), Linux received unchanged data
 *    * The sensor appeared to be working when it was actually not reading
 *    * Issue manifested as "data retained same stale values" when pressure/current changed
 *    * Often recovered silently after the ADC reinit cycle restored communication
 *  - NEW: Stale data detection with 60-second timeout (ADC_STALE_TIMEOUT_MS)
 *    * lastSuccessfulAdcRead tracks millis() of most recent valid pressure or current read
 *    * adcDataStale flag goes TRUE when no good read for 60 continuous seconds
 *    * When stale: FAST-TX sends pressure=-99.9 (ADC_FAULT_PRESSURE sentinel) and current=0.0
 *    * Python should interpret -99.9 as "sensor fault — data unavailable"
 *    * Loud !!!-bordered Serial Monitor alert printed ONCE when data goes stale
 *    * ###-bordered recovery message printed when good reads resume
 *    * Flag auto-clears on next successful read — no manual intervention needed
 *  - NEW: Web dashboard /api/status includes adcDataStale and adcLastGoodMs fields
 *    * adcDataStale: true/false — whether FAST-TX is sending fault values
 *    * adcLastGoodMs: milliseconds since last successful ADC read (for diagnostics)
 *  - NOTE: 60-second timeout chosen to allow for I2C bus recovery and reinit attempts
 *    (reinit triggers at 10 consecutive errors at 60Hz = ~167ms, so 60s is very generous)
 *  - NEW: DISP_SHUTDN (GPIO13 / RTC_GPIO13 / CR6) internal pull-up + GPIO hold
 *    * Problem: during firmware updates or ESP.restart(), GPIO13 floated LOW for ~100-500ms
 *      between chip reset and setup() execution, causing unwanted site shutdowns
 *    * Solution: Three-layer protection per ESP-IDF GPIO docs for ESP32-S3:
 *      1. gpio_hold_en() — latches pin state through SOFTWARE resets (ESP.restart, OTA)
 *      2. gpio_deep_sleep_hold_en() — holds ALL digital GPIOs through DEEP SLEEP
 *         (ESP-IDF: "For ESP32-S3, gpio_hold_en alone cannot hold digital GPIO
 *          during Deep-sleep — call gpio_deep_sleep_hold_en additionally")
 *      3. gpio_pullup_en() — internal ~45kΩ pull-up as backup for full power-on resets
 *    * initializeDispShutdownPinWithHold() runs as FIRST GPIO init in setup()
 *    * All DISP_SHUTDN writes now use writeDispShutdownPinWithHold() which
 *      temporarily disables hold, writes the pin, then re-enables hold
 *    * Includes driver/gpio.h for ESP-IDF gpio_hold_en/dis, gpio_pullup_en
 *    * Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html
 *  - REWORK: Web portal tests (leak, func, eff, clean) — ESP32 owns relays entirely
 *    * Previously: ESP32 forwarded test commands to Linux and relied on Linux to set
 *      relay modes. The testRunning guard blocked Linux's mode response, causing tests
 *      to not start or have long delays.
 *    * Now: ESP32 takes SOLE control of relays when a web portal test starts.
 *      ALL Linux mode commands are BLOCKED for the duration of the test.
 *      Relay control returns to Linux when the test completes or is stopped.
 *    * ESP32 sets relays IMMEDIATELY when Start button is pressed (no round-trip delay)
 *    * Auto-stop: loop() checks test timer and auto-completes when duration expires
 *      - leak: 30 min (mode 9), func: 20 min (10x run/purge cycle), eff: 2 min (mode 1)
 *      - clean canister: 2 hours (mode 1) — now uses same testRunning mechanism
 *    * Manual stop: Stop button on web portal sets relays to idle, clears testRunning
 *    * Linux still receives start/stop commands for logging/awareness
 *    * Linux stop_cycle command does NOT end a web portal test — only the web portal can
 *    * Helper functions: getTestDurationSeconds(), getTestRelayMode()
 *
 *  Rev 10.10 (2/10/2026) - ADC I2C Error Diagnostics
 *  - NEW: Loud, obnoxious Serial Monitor error logging for ALL ADC read failures
 *    * Pressure CH0: detects negative or >=2048 raw values (invalid for single-ended)
 *    * Current CH2-CH3 differential: three-tier validation
 *      1. rawDiff == -1: I2C NACK/bus failure signature → read SKIPPED
 *      2. rawDiff == ±2048: full-scale rail (clipping or corruption) → read SKIPPED
 *      3. absDiff > 800: abnormally high (>expected 580 max) → WARNING, still processed
 *    * Each error prints a ###-bordered block with raw value, error count, timestamp
 *    * Easy to spot in Serial Monitor when monitoring for I2C line noise issues
 *  - NEW: Invalid current reads are now SKIPPED to protect rolling average
 *    * Previously: all differential reads were processed regardless of validity
 *    * Now: I2C NACK (-1) and full-scale rail (±2048) values are discarded
 *    * Bad reads increment adcErrorCount toward 10-error reinit threshold
 *  - IMPROVED: Reinit (10 consecutive errors) now prints !!!!-bordered block
 *    * Reinit success and failure both print #### blocks for visibility
 *  - IMPROVED: ADS1015 reconnect attempts (when not initialized) now log every try
 *    * 5-second retry loop prints #### blocks for each attempt, success, or failure
 *  - NOTE: All ADC error messages print ALWAYS (not gated by serialDebugMode)
 *    * I2C errors are critical diagnostics — must be visible even in quiet mode
 *  - IMPROVED: Pressure valid range tightened from >=0 to >=0 && <2048
 *    * Catches edge case where I2C returns max 12-bit value (bus corruption)
 *
 *  Rev 10.9 (2/10/2026) - Debug Mode + Cellular Identity + Packet Split + Cal Safety
 *  - NEW: Serial Monitor debug mode (default OFF — quiet output)
 *    * When OFF: only essential messages (boot, errors, mode changes, watchdog, calibration)
 *    * When ON: verbose output (FAST-TX 5Hz, RS232-TX CELL, RS232-TX TIME, RELAY-REFRESH,
 *      cell refresh, SD writes) — useful for serial timing verification
 *    * Toggle via Web Config Portal (/setdebug?enabled=1&pwd=...) with password protection
 *    * Saved to EEPROM — persists across reboots
 *    * Global flag: serialDebugMode (bool)
 *  - NEW: IMEI, IMSI, ICCID, Technology added to cellular status JSON (10s timer)
 *    * imei: modem hardware identity (queried once at boot from getIdentity)
 *    * imsi: SIM subscriber identity (queried once at boot from getSIMCardIMSI)
 *    * iccid: SIM card ID (queried once at boot from getSIMCardID)
 *    * technology: Radio Access Technology (queried once at boot from getRAT)
 *    * Python parser ignores unknown fields — zero Python changes required
 *  - NEW: Boot display shows IMSI, ICCID, RAT alongside IMEI and MAC
 *  - CHANGED: Cellular status packet now sent every 10 seconds on TIMER
 *    * Previously: only sent when modemDataFresh was true (freshness-gated)
 *    * Problem: if modem queries fail silently, Linux device got NO cellular data
 *    * Now: sendCellularStatusToSerial() sends cached values every 10 seconds
 *    * Linux always has last-known cellular state even during modem issues
 *  - CHANGED: Removed "datetime" and "failsafe" from cellular status packet
 *    * "datetime" now sent separately via sendDateTimeToSerial() ONLY when fresh
 *    * "failsafe" already included in 5Hz fast sensor packet (Rev 10.8)
 *    * Prevents stale/frozen datetime from confusing the Linux controller
 *  - NEW: Standalone datetime packet {"datetime":"..."} sent only on modemTimeFresh
 *    * Small packet (~40 bytes) sent independently of cellular info
 *    * Python parser uses 'if "datetime" in data' — fully compatible
 *  - RENAMED: sendStatusToSerial() → sendCellularStatusToSerial() + sendDateTimeToSerial()
 *    * Split into two focused functions for clarity and independent timing
 *  - NEW: CELLULAR_SEND_INTERVAL (10000ms) and lastCellularSendTime timer
 *  - FIXED: Bad calibration under vacuum saved wrong zero point to EEPROM
 *    * Root cause: Calibrating while sensor reads -6 IWC set newZero ≈ 827
 *    * This made all subsequent readings ≈ 0.0 IWC (calibrated vacuum as zero)
 *    * Old validation range 200-1800 was far too wide — accepted nearly anything
 *  - NEW: Calibration range restricted to middle 20% of scale (factory default ±10%)
 *    * New allowed range: ~868 to ~1060 ADC counts (derived from 964.0 ±10%)
 *    * At sea level: newZero ≈ 964 → accepted
 *    * At altitude (-0.5 IWC): newZero ≈ 953 → accepted
 *    * Under vacuum (-6 IWC): newZero ≈ 827 → REJECTED with error message
 *    * Constants: CAL_TOLERANCE_PERCENT, CAL_RANGE_MIN, CAL_RANGE_MAX
 *  - FIXED: EEPROM load had no range validation (accepted any positive value)
 *    * Bad stored values now rejected on boot → falls back to factory default 964.0
 *    * Devices with corrupt EEPROM self-heal on next reboot (no manual intervention)
 *  - NEW: ADC pressure outlier rejection in addPressureSample()
 *    * Discards samples deviating >2.0 IWC from current rolling average
 *    * Eliminates 1.0 IWC oscillation caused by electrical noise / relay switching
 *    * Gas pressure physically cannot change >2 IWC in 16ms (one sample period)
 *    * Only active after buffer is full (60 samples) — initial fill unrestricted
 *    * New diagnostic counter: adcOutlierCount (visible on web portal diagnostics)
 *  - IMPROVED: 100µs PGA settling delay after ADC gain switch (GAIN_TWO → GAIN_ONE)
 *    * Reduces baseline noise floor when alternating pressure/current reads
 *    * 0.6% overhead on 16ms ADC cycle — negligible impact
 *
 *  =====================================================================
 *  Rev 10.8 (2/10/2026) - Mode Confirmation: 15-Second Relay Refresh + Extended Sensor Packet
 *  - NEW: Periodic relay state refresh every 15 seconds in main loop()
 *    * Re-applies currentRelayMode to physical GPIO pins via setRelaysForMode()
 *    * Guards against relay driver glitch, failed GPIO write, or pin drift
 *    * Only runs during normal serial-controlled mode (skipped in failsafe)
 *    * Compensates for missed mode commands: worst-case 15-second recovery
 *    * Uses lastRelayRefreshTime timer + RELAY_REFRESH_INTERVAL constant
 *  - IMPROVED: sendFastSensorPacket() now includes failsafe and shutdown state
 *    * Added "failsafe":0/1 and "shutdown":0/1 fields to 5Hz JSON packet
 *    * Linux program can see system state without needing a new message type
 *    * Python ignores unrecognized JSON fields — zero Python changes required
 *  - ARCHITECTURE: Mode delivery redundancy (no Python changes needed):
 *    * Immediate: send_mode_immediate() on mode change (~1ms)
 *    * Periodic: Linux 15-second payload always includes mode field
 *    * Refresh: ESP32 re-applies stored mode to relay pins every 15 seconds
 *    * Failsafe: After 2 watchdog restarts, ESP32 takes autonomous control
 *
 *  =====================================================================
 *  Rev 10.7 (2/9/2026) - Pressure Sensor Calibration + Instant Zero Point
 *  - NEW: Pressure calibration via serial {"type":"cmd","cmd":"cal"} or web portal button
 *    * Instant non-blocking: uses existing 60-sample rolling average (no delay loop)
 *    * Shifts adcZeroPressure so current reading becomes 0.0 IWC (works at any altitude)
 *    * Saved to EEPROM (Preferences key "adcZero") — persists across reboots
 *    * Loaded from EEPROM at boot before ADC task starts
 *    * Factory default 964.0 used if no EEPROM value exists
 *  - NEW: "type":"cmd" message type for serial commands (separate from "type":"data")
 *    * ESP32 routes on the "cmd" field: "cal" → calibratePressureSensorZeroPoint()
 *    * Extensible for future commands
 *  - NEW: ESP32 sends {"type":"data","ps_cal":964.50} back to Linux after calibration
 *    * Python saves calibration factor to gm_db as 'adc_zero'
 *  - NEW: Web portal "Calibrate Pressure" button on Maintenance screen
 *  - NEW: "ADC Zero Point" field on Diagnostics screen and in /api/status JSON
 *  - NEW: /api/command "calibrate_pressure" endpoint for web portal calibration
 *
 *  =====================================================================
 *  Rev 10.6 (2/9/2026) - ADC Pressure Sign Fix, Current Differential, Normal Command
 *  - FIXED: Pressure sign convention — removed incorrect negation, vacuum now negative
 *  - FIXED: Current reading stuck at 0.0A — switched to hardware differential mode
 *    * readADC_Differential_2_3() at GAIN_TWO (1mV/count) instead of two single-ended reads
 *    * Recalculated calibration constants: (156, 580) counts → (2.1, 8.0) amps
 *  - FIXED: Current uses windowed peak detection (not average) matching Python behavior
 *  - NEW: {"mode":"normal"} serial command clears 72-hour shutdown (GPIO13 HIGH)
 *    * Eliminates need for ESP32 reboot to exit shutdown state
 *
 *  =====================================================================
 *  Rev 10.5 (2/9/2026) - Optimized Serial Update Rates: 5Hz Sensors, Fresh-Only Cellular
 *  - IMPROVED: Fast sensor packet rate increased from 1Hz to 5Hz (200ms interval)
 *    * Pressure, current, overfill, and SD card status now sent 5 times per second
 *    * Gives the Linux controller much faster visibility into real-time sensor changes
 *    * SD card status added to fast packet (cheap SD.cardType() check, no disk I/O)
 *  - CHANGED: Cellular/datetime fields now sent ONLY when modem returns fresh data
 *    * Previously: datetime, lte, rsrp, rsrq, passthrough sent every 15 seconds on timer
 *    * Problem: Values repeated unchanged between modem queries, wasting bandwidth.
 *      datetime appeared to "freeze then jump" because interpolated value was stale.
 *    * Now: modemDataFresh flag set by refreshCellSignalInfo() (~every 60s) and
 *      lteConnect(). modemTimeFresh flag set by initModemTime().
 *    * REV 10.9: sendStatusToSerial() split into sendCellularStatusToSerial() (10s timer)
 *      and sendDateTimeToSerial() (fresh only). Cellular no longer freshness-gated.
 *    * Result: Cellular data always arrives on 10s timer. Datetime only when fresh.
 *  - ARCHITECTURE: Two distinct serial packet types to Linux:
 *    * Fast (5Hz): {"pressure":X,"current":X,"overfill":X,"sdcard":"OK","relayMode":X,"failsafe":0,"shutdown":0}
 *    * Fresh cellular: {"datetime":"...","lte":X,"rsrp":"...","rsrq":"...", ... }
 *    * Python modem.py parser handles both — same JSON field extraction logic
 *    * Rev 10.8: Fast packet extended with failsafe + shutdown state fields
 *  
 *  =====================================================================
 *  Rev 10.4 (2/9/2026) - Web Portal Test/Cycle Fix + Full Button Audit
 *  - FIXED: Web portal Leak Test, Functionality Test, Efficiency Test, Clean Canister,
 *    Start Cycle, and Manual Purge buttons did nothing when pressed.
 *    * Root cause: Python IOManager functions had "screen guards" that only allowed
 *      execution when the Kivy touchscreen was on the matching screen (e.g., leak_test()
 *      only ran if touchscreen was on "LeakTest" screen). When commands came from the
 *      web portal, the touchscreen was typically on "Main", so all commands were silently
 *      blocked.
 *    * Fix: Added from_web parameter to all affected Python functions. Web portal
 *      commands now bypass the Kivy screen guard (web portal has its own PW protection).
 *  - AUDITED: All 25+ web portal buttons verified against their ESP32 and Python handlers
 *    * Every button confirmed to reach the correct handler with correct parameters
 *    * All navigation links verified to target existing screen IDs
 *    * All password-protected actions verified (Maintenance PW, Profile PW, Config PW)
 *  - NOTE: This fix is in the Python code, not the ESP32 firmware. The ESP32 command
 *    forwarding was already correct — it properly sent the commands to Linux via Serial1.
 *    The Python side was discarding them due to the screen guard.
 *  
 *  =====================================================================
 *  Rev 10.3 (2/9/2026) - Faster Pressure Updates for Real-Time Controller Response
 *  - IMPROVED: Pressure rolling average reduced from 200 samples to 60 samples
 *    * Was: 200 samples at 60Hz = 3.3-second averaging window
 *    * Now: 60 samples at 60Hz = 1.0-second averaging window
 *    * Pressure values now reflect sensor changes within 1 second
 *    * Still provides noise filtering — just responds faster to real changes
 *  - WHY: The Linux controller relies on real-time pressure from the ESP32 to make
 *    critical operating decisions (start/stop cycles, detect alarms, trigger safety
 *    shutdowns). A 3.3-second lag meant the controller was always reacting to
 *    conditions that were already 3+ seconds old. This was especially noticeable
 *    when rapidly applying or releasing vacuum during maintenance and testing.
 *  - ARCHITECTURE NOTE: The ESP32 sends two types of serial messages to Linux:
 *    * Fast sensor packet (every 1 second): Pressure, current, overfill, relay mode
 *      → This is CRITICAL — the Linux controller uses these for all real-time decisions
 *    * Full status packet (every 15 seconds): Adds datetime, SD, LTE, cell info
 *      → Supplementary information, not time-sensitive
 *  - The Linux program sends data BACK to the ESP32 every 15 seconds. That data is
 *    ONLY used for building CBOR payloads for cellular transmission. The CBOR builder
 *    uses the ESP32's own ADC readings for pressure and current (not the Linux values),
 *    so cellular data is always real-time regardless of the 15-second Linux send rate.
 *  
 *  =====================================================================
 *  Rev 10.2 (2/8/2026) - CBOR Pressure Fix + Web Test Handlers
 *  - FIXED: Stale CBOR pressure — ESP32 CBOR builder now uses own ADC readings
 *    * Previously round-tripped Python values were used, causing stale/delayed data
 *    * ESP32 ADC (60Hz rolling average) provides real-time pressure and current
 *  - NEW: Web portal test handlers added for remote test initiation
 *
 *  =====================================================================
 *  Rev 10.1 (2/8/2026) - Performance & Security: Faster Relay Response, Portal Passwords
 *  - PERFORMANCE: Serial read interval reduced from 5s to 100ms
 *    * Relay commands from Linux now apply within ~100ms (was up to 5 seconds)
 *    * Eliminates user-visible delay when starting cycles or changing modes
 *  - SECURITY: Profile selection screen now requires password "1793" to confirm
 *    * Password validated server-side (ESP32) — cannot be bypassed by the browser
 *    * Users select a profile, then must enter password and click Confirm to apply
 *  - SECURITY: Maintenance screen now requires password "878" to access
 *    * Password gate shown on every visit; stays unlocked for the browser session
 *    * Prevents unauthorized access to alarm clearing, tests, and passthrough
 *  - IMPROVED: Serial "no data" warning frequency adjusted for faster read interval
 *
 *  =====================================================================
 *  Rev 10.0 (2/8/2026) - MAJOR REWRITE: MCP Emulation Removed, Serial Relay Control
 *  - REMOVED: MCP23017 I2C slave emulation (all 22 registers, handlers, task)
 *  - NEW: ESP32 is now I2C MASTER reading ADS1015 ADC at 0x48
 *    * Channel 0: Pressure sensor (single-ended, 60-sample rolling average)
 *    * Channels 2&3: Current monitoring (abs differential, 20-sample rolling avg)
 *    * 60Hz polling on Core 0 FreeRTOS task with error recovery
 *  - NEW: Relay control via serial JSON mode field (replaces I2C register writes)
 *    * Linux sends {"type":"data",...,"mode":0,...} → ESP32 sets relays for mode
 *    * setRelaysForMode(): Mode 0=Idle, 1=Run, 2=Purge, 3=Burp, 8=FreshAir
 *  - NEW: Failsafe relay control when Comfile is down (after 2 restart attempts)
 *    * Full autonomous cycle logic: pressure auto-start, run/purge/burp sequences
 *    * Current monitoring: low (<3A for 9 readings), high (>25A for 2s) protection
 *    * Adds 4096 to fault code when in failsafe mode (REV 10.17: was 8192)
 *  - NEW: ProfileManager class with 6 profiles (CS2, CS3, CS8, CS9, CS12, CS13)
 *    * Per-profile alarm thresholds, fault code maps, cycle sequences
 *    * Stored in EEPROM, selectable via serial command or web portal
 *  - NEW: Full web interface SPA replicating all PySimpleGUI screens
 *    * 20+ screens: Main, Alarms, Maintenance, Manual, Tests, Profiles, About, etc.
 *    * Profile-dependent alarm layouts and fault code display
 *    * Command POST endpoint for button actions
 *  - NEW: CBOR ring buffer during passthrough mode (~10KB, 120 min capacity)
 *    * Buffers sensor data every 15s during PPP session
 *    * Flushes to modem after passthrough ends
 *  - SIMPLIFIED: DISP_SHUTDN stays HIGH on boot, only goes LOW on {"mode":"shutdown"}
 *  - SIMPLIFIED: Overfill is direct GPIO38 read (no MCP emulation needed)
 *  - CHANGED: PPP request finishes current purge step before entering passthrough
 *  - ESP32 sends pressure, current, overfill status to Linux via serial JSON
 *  
 *  =====================================================================
 *  Rev 9.4f (2/6/2026) - MCP23017 Emulator Rewrite + RGB LED + Web Enhancements
 *  - Final Rev 9 release before Rev 10.0 major rewrite
 *  - MCP23017 emulator rewrite with improved register handling
 *  - RGB LED (WS2812B on GPIO9): blue breathing idle, red flash passthrough,
 *    green mode 1, orange mode 2, yellow mode 3
 *  - Web portal diagnostic dashboard enhancements
 *  
 *  Rev 9.4e (2/6/2026) - CRITICAL: Fix Overfill Polarity Inversion
 *  - FIXED: gpioB_value polarity was inverted vs real MCP23017 behavior
 *    * Real MCP23017: pin HIGH (1) = normal, pin LOW (0) = overfill alarm
 *    * Emulation had it backwards: 0x00 for "safe" which Linux reads as LOW = alarm!
 *    * This caused FALSE OVERFILL ALARMS on every ESP32 reboot/flash
 *    * Also caused real overfill alarms to be INVISIBLE to Linux master
 *  - All gpioB_value assignments now match real MCP23017 pin behavior:
 *    * Boot/lockout safe value: 0x01 (HIGH = normal, no alarm)
 *    * Alarm active: 0x00 (LOW = sensor triggered)
 *    * No alarm: 0x01 (HIGH = pull-up, normal)
 *  - Updated I2C handler lockout return from 0x00 to 0x01
 *  - Affects: global init, mcpEmulatorTask, mcpReadInputsFromPhysicalPins,
 *    handleI2CRead (GPIOA sequential + GPIOB), passthrough boot, setup() pre-init
 *  
 *  Rev 9.4d (2/6/2026) - Cell Tower Info + Scheduled Firmware Checks
 *  - NEW: Cell tower info on web dashboard (MCC/MNC, Cell ID)
 *  - NEW: Cell tower info in serial JSON to modem.py (operator, band, mcc, mnc, cellId)
 *  - Uses WalterModem getCellInformation() for all cellular fields
 *  - Changed firmware/OTA check from frequency-based (every 60s) to schedule-based
 *  - Schedule: 7:00 AM - 1:00 PM EST, every 15 minutes (25 checks/day)
 *  - First boot still runs immediately to sync modem time
 *  - Uses currentTimestamp (UTC) converted to EST for schedule evaluation
 *  - Prevents duplicate checks within same 15-minute slot
 *  - Reduces unnecessary modem traffic outside business hours
 *  - FIXED: RSRP/RSRQ/Operator/Band showing "0"/"Unknown"/"--" on web dashboard
 *    * lteConnect() now stores cell info in globals when first retrieved
 *    * Added refreshCellSignalInfo() called every 60s in main loop
 *    * Renamed buildStatusUpdate() -> refreshCellSignalInfo() for clarity
 *    * Fixed signal bar showing "Excellent" for RSRP=0 (0 means "no data")
 *    * Dashboard shows "--" with gray bar until real signal data arrives
 *  
 *  Rev 9.4b (2/6/2026) - Captive Portal Fix (CRITICAL)
 *  - FIXED: Blank web page caused by AsyncWebServer template processor
 *    * All literal % in HTML/CSS/JS must be escaped as %% when using webProcessor
 *    * Template engine was treating CSS "width:0%" and JS "+'%'" as template vars
 *    * This deleted large chunks of HTML between consecutive % characters
 *  - Rebuilt captive portal for rock-solid detection on all platforms:
 *    * iOS/macOS: /hotspot-detect.html handler
 *    * Android: /generate_204 handler (returns 200 instead of expected 204)
 *    * Windows: /connecttest.txt, /ncsi.txt, /fwlink handlers
 *    * Firefox: /canonical.html handler
 *    * Catch-all: 302 redirect for any unrecognized URL
 *  - Lightweight captive portal landing page with dashboard link
 *  - Switched AJAX from fetch() to XMLHttpRequest for wider captive portal compat
 *  - Uses absolute URLs (http://192.168.4.1/...) in all AJAX calls
 *  - WiFi mode explicitly set to WIFI_AP with stabilization delay
 *  
 *  Rev 9.4a (2/6/2026) - Code Cleanup
 *  - REMOVED: 10 unused global variables (webOverrideActive, lastWebActivity,
 *    WEB_OVERRIDE_TIMEOUT, mode_start_time, previousMillis, statusInterval,
 *    passthroughStartTime, passthroughTimeoutMs, DEFAULT_PASSTHROUGH_TIMEOUT_MS,
 *    profile, global id, imeisv, svn, global lastStatusTime)
 *  - REMOVED: Dead set_relay_mode() function (never called after web relay removal)
 *  - REMOVED: Web override timeout check from MCP emulator loop
 *  - REMOVED: Web override priority check from mcpWriteOutputsToPhysicalPins()
 *  - Relay control exclusively via I2C from Linux master (cleaner code path)
 *  
 *  Rev 9.4 (2/5/2026) - Simplified Diagnostic Dashboard
 *  - REMOVED: Web relay control (mode buttons, individual relay buttons)
 *  - REMOVED: Quality Check Test Mode (password section, ADS1015 ADC)
 *  - KEPT: Web passthrough toggle with confirmation dialog
 *  - REMOVED: Web I2C reset endpoint
 *  - NEW: Read-only Service Diagnostic Dashboard for field contractors
 *  - Dashboard shows: Serial data, Cellular modem status, IO board health
 *  - Displays: RSRP/RSRQ signal quality with visual bar, LTE/BlueCherry status
 *  - Displays: SD card status, I2C transaction counts, overfill sensor, uptime
 *  - Relay control now exclusively via I2C from Linux master (no web conflicts)
 *  - Kept: Device name configuration, watchdog toggle
 *  
 *  Rev 9.3a (2/5/2026) - Critical Safety Fixes
 *  - FIXED: DISP_SHUTDN relay randomly turning off
 *    * gpioA_value now initialized to 0x10 (V6 ON) to match physical state
 *    * I2C handler forces bit 4 ON during 20-second protection period
 *    * set_relay_mode() now preserves V6 state instead of overwriting
 *    * Web /setrelays endpoint always preserves V6 state (can only change via I2C)
 *  - FIXED: False overfill alarms on ESP32 reboot
 *    * Multi-layer protection: variables reset in setup(), task, and passthrough boot
 *    * I2C handler returns 0x00 for GPIOB during 15-second startup lockout
 *    * GPIO38 pull-up configured before I2C slave initialization
 *  - FIXED: Serial output corruption from MCP emulator thread
 *    * Removed all Serial.print calls from Core 0 functions
 *    * Affects: I2C handlers, bus recovery, health check, overfill/DISP protection
 *  
 *  Rev 9.3 (2/5/2026) - Preference-Based Passthrough Boot Mode
 *  - Passthrough now uses preference-based boot strategy for clean modem access
 *  - When "remote XX" command received, stores flag in preferences and restarts
 *  - On boot, checks preference BEFORE loading WalterModem library
 *  - If passthrough requested: runs minimal mode (MCP emulator + serial bridge only)
 *  - No WiFi, web server, or WalterModem during passthrough (matches walter-as-linux-modem.txt)
 *  - Early exit: Linux device can set MCP GPA5 (bit 5) via I2C to trigger restart
 *  - Auto-restarts to normal operation after timeout (default 60 min, max 24 hours)
 *  - Preference is cleared immediately on boot to prevent getting stuck in passthrough
 *  
 *  Rev 9.2f (2/5/2026) - On-Demand Passthrough - SUPERSEDED by 9.3
 *  - Attempted runtime passthrough by reinitializing ModemSerial
 *  - Replaced by preference-based boot for cleaner modem access
 *  
 *  Rev 9.2e (2/4/2026) - Passthrough-Only Boot Mode - SUPERSEDED
 *  - Used preferences flag - replaced by on-demand approach in 9.2f
 *  
 *  Rev 9.2c (2/4/2026) - BlueCherry Remote Commands
 *  - BlueCherry message containing "remote" (case-insensitive) enters passthrough mode
 *  - BlueCherry message containing "restart" (case-insensitive) performs ESP.restart()
 *  - Moved web config HTML back to .ino file (fixes iOS captive portal blank page)
 *  - Compressed HTML/CSS/JS for smaller size
 *  
 *  Rev 9.2b (2/4/2026) - Passthrough Mode Simplification
 *  - WalterModem library configures modem at 115200 (same as host serial)
 *  - Passthrough is now a simple bridge with no baud rate changes
 *  - Simplified enterPassthroughMode() and exitPassthroughMode()
 *  
 *  Rev 9.2a (2/4/2026) - Critical Relay Protection
 *  - Added DISP_SHUTDN startup protection (20 second lockout)
 *  - Relay forced ON during power cycles/firmware flashes
 *  - Prevents accidental site shutdown during startup
 *  - Enhanced overfill validation (15 sec lockout, 8 readings to trigger)
 *  - Both systems log status to Serial Monitor for diagnostics
 *  
 *  Rev 9.2 (2/4/2026) - BlueCherry Message Receiving
 *  - Added support for receiving non-firmware MQTT messages via BlueCherry
 *  - Messages with topic != 0 are printed to Serial Monitor
 *  - Uses blueCherrySync() built-in mechanism (no separate MQTT connection)
 *  - Topic 0 = firmware update, Topic != 0 = custom messages
 *  
 *  Rev 9.1g (1/29/2026) - Passthrough UI Fix for iOS Safari
 *  - Replaced native confirm() dialog with custom HTML modal
 *  - Custom modal works consistently on iOS Safari, Mac Safari, Chrome, etc.
 *  - Added passthroughPending flag to prevent AJAX refresh race condition
 *  - Improved button event handling (preventDefault, stopPropagation)
 *  
 *  Rev 9.1f (1/30/2026) - Dynamic WiFi AP Naming, Serial Status & Passthrough Mode
 *  - WiFi AP name includes random hex suffix on first boot (e.g., "GM IO Board 5c3dfc")
 *  - AP name auto-updates when device ID received from Linux serial
 *  - Manual device name entry via web portal configuration
 *  - Device name persists across reboots in EPROM
 *  - Sends JSON status to Python device every 15 seconds via RS-232 (Serial1)
 *  - JSON format: {"datetime":"YYYY-MM-DD HH:MM:SS","sdcard":"OK|FAULT","passthrough":0|1}
 *  - Passthrough Mode - bridges RS-232 to modem for AT commands/PPP connections
 *  - Web button to enter/exit passthrough mode (NOT persisted - reboot returns to normal)
 *  - I2C emulator and web server continue running in passthrough mode
 *  
 *  Rev 9.1e (1/30/2026) - MCP23017 Compatibility & Debug Improvements
 *  - Added INTCAP register support for Adafruit library compatibility
 *  - Added periodic I2C status logging (every 10 seconds when debug enabled)
 *  - Fixed serial output corruption from multi-core access
 *  
 *  Rev 9.1d (1/30/2026) - BlueCherry Stability Fix
 *  - CRITICAL: Removed modem.reset() on BlueCherry sync failure
 *  - This was causing LTE disconnect -> ESP.restart() loop
 *  - Added fault code for BlueCherry offline (originally 4096, now 8192 per REV 10.17)
 *  - See REV 10.17 for current fault code assignments
 *  - System now stays running during BlueCherry platform outages
 *  - Weekly restart still occurs if BlueCherry never connects
 *  
 *  Rev 9.1c (1/30/2026) - I2C Timeout Fix
 *  - CRITICAL: Reduced mutex timeouts in I2C callbacks from 100ms to 1ms
 *  - I2C slave callbacks must respond in microseconds, not milliseconds
 *  - Prevents OSError [Errno 5] "Input/output error" on Linux master
 *  - Added fallback behavior when mutex unavailable (proceed anyway)
 *  
 *  Rev 9.1b (1/29/2026) - Thread Safety Fix
 *  - Fixed race condition: gpioA_value writes now mutex-protected
 *  - Fixed overfill false alarms: Added INPUT_PULLUP and hysteresis
 *  - Overfill alarm requires 3 consecutive HIGH readings to clear
 *  
 *  Rev 9.1 (1/29/2026) - MCP23017 Enhancements
 *  - Full MCP23017 register support (all 22 registers)
 *  - I2C transaction counting and error tracking
 *  - Non-fatal BlueCherry init (hourly retry, weekly restart)
 *  
 *  Rev 9.0 (1/29/2026) - Initial Release
 *  - AJAX-based web interface (no page refresh flashing)
 *  - Serial watchdog with configurable GPIO pulse
 *  - Thread-safe relay control with mutex protection
 *  - MCP23017 emulation for I2C relay control
 *  
 *  =====================================================================
 *  FEATURES
 *  =====================================================================
 *  - ADS1015 12-bit ADC reading (pressure + current) via I2C master at 0x48
 *  - 5 relay outputs (Motor, CR1, CR2, CR5, DISP_SHUTDN) controlled via serial
 *  - Overfill sensor input (GPIO38, active-low with internal pull-up)
 *  - Failsafe autonomous relay control when Comfile panel is down
 *  - ProfileManager: 6 profiles with per-profile alarms and cycle sequences
 *  - Full SPA web interface replicating PySimpleGUI control screens
 *  - CBOR data buffering during PPP passthrough sessions
 *  - Serial watchdog: 30-min timeout, GPIO39 pulse, failsafe after 2 attempts
 *  - BlueCherry OTA support with graceful fallback
 *  - Thread-safe operation across Core 0 (ADC) and Core 1 (Main/Web)
 *  
 *  =====================================================================
 *  FAULT CODES (REV 10.17 — reassigned)
 *  =====================================================================
 *  - 1024: SD card failure (card not mounted or inaccessible)
 *  - 2048: Serial watchdog activated (no serial data for 30+ minutes)
 *  - 4096: Failsafe mode (ESP32 autonomous relay control, Comfile down)
 *  - 8192: BlueCherry connection failed (OTA updates unavailable)
 *  - Note: These are bit flags ADDED to any existing fault codes from Linux.
 *    Multiple faults stack: e.g., SD fail + watchdog = 1024 + 2048 = 3072
 *  
 ***********************************************/

// Define the software version as a macro
#define VERSION "Rev 10.19"

// REV 10.16: Delay between individual relay GPIO pin changes (milliseconds).
// Simultaneous activation of CR0_MOTOR, CR1, CR2, CR5 causes electrical noise
// that couples into the ADS1015 analog inputs. Staggering each pin change by
// RELAY_DELAY ms reduces the combined inrush/back-EMF transient.
// Adjust this value to tune the trade-off between switching speed and noise.
//
// Usage: Used in setRelaysForMode() and boot-time relay init (adcReaderTask).
//   Example: with RELAY_DELAY=1, a 4-pin mode change takes ~3ms (3 gaps × 1ms).
#define RELAY_DELAY 1
String ver = VERSION;

// Password required to change device name or toggle watchdog via web portal
// Change this to your desired password. Case-sensitive, sent as URL parameter.
#define CONFIG_PASSWORD "1793"

// ### Libraries ###
#include <esp_mac.h>
#include "driver/gpio.h"             // ESP-IDF GPIO driver for gpio_hold_en/dis, gpio_pullup_en
                                      // Used to latch DISP_SHUTDN HIGH through reboots & OTA updates
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESPAsyncDNSServer.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include "WalterModem.h"
#include "BlueCherryZTP.h"
#include "FS.h"
#include <SD.h>
#include <SPI.h>
#include <tinycbor.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>    // WS2812B RGB LED control (install via Library Manager)
#include <Adafruit_ADS1X15.h>     // ADS1015 12-bit ADC (install via Library Manager)

// =====================================================================
// FORWARD DECLARATIONS
// =====================================================================
// The Arduino IDE auto-generates function prototypes, but FAILS when
// functions are called inside lambda callbacks (e.g. AsyncWebServer
// request handlers). These explicit forward declarations fix all
// "not declared in this scope" errors for functions defined later
// in the file that are referenced inside lambdas or before their
// definition. Without these, the compiler sees the call site before
// the function definition and has no prototype to match.
// =====================================================================
size_t buildCborFromReadings(uint8_t* buf, size_t bufSize, int data[][10], size_t rowCount);
String buildCborPayloadHistoryJson();
bool   sendCborArrayViaSocket(uint8_t* buffer, size_t size);
void   cleanupOldLogFiles(int currentYear);
String formatTimestamp(int64_t seconds);
bool   lteConnected();
bool   isSDCardOK();
String getDateTimeString();
String getJsonValue(const char* json, const char* key);
void   parsedSerialData();
void   sendFastSensorPacket();
void   loadPressureCalibrationFromEeprom();
void   savePressureCalibrationToEeprom();
void   calibratePressureSensorZeroPoint();
void   notifyPythonOfCommand(const char* command, const char* value);
void   executeRemoteCommand(String cmd, String value);

// =====================================================================
// ADS1015 ADC CONFIGURATION (Rev 10 - replaces MCP23017 emulation)
// =====================================================================
// ESP32 is now I2C MASTER on SDA=4, SCL=5 reading the ADS1015 ADC.
// Linux no longer drives these I2C lines (communicates via serial only).
#define ADS1015_ADDRESS 0x48    // ADS1015 default I2C address
#define SDA_PIN 4               // ESP32 I2C data pin (now MASTER)
#define SCL_PIN 5               // ESP32 I2C clock pin (now MASTER)
Adafruit_ADS1015 ads;           // 12-bit ADC object

// ESP32 GPIO pins for relay outputs (MCP Port A)
#define CR0_MOTOR 11   // MCP GPA0 - Motor (Vac Pump)
#define CR1       18   // MCP GPA1
#define CR2       17   // MCP GPA2
#define CR5       16   // MCP GPA3
#define DISP_SHUTDN  13   // MCP GPA4

// ESP32 GPIO pin for overfill input (MCP Port B)
#define ESP_OVERFILL  38   // MCP GPB0

// ESP32 GPIO pin for serial watchdog pulse output
#define ESP_WATCHDOG_PIN  39   // Pulses HIGH for 1 second when no serial data for 5 minutes

// WS2812B RGB status LED on GPIO9
// Color meanings: Blue breathing=Idle, Green=Run, Orange=Purge, Yellow=Burp, Red flash=Passthrough
#define STATUS_LED_PIN    9
#define STATUS_LED_COUNT  1
Adafruit_NeoPixel statusLED(STATUS_LED_COUNT, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

// Run modes for relay patterns
enum RunMode {
    IDLE_MODE = 0,
    RUN_MODE = 1,
    PURGE_MODE = 2,
    BURP_MODE = 3
};

String mode_names[] = {"Idle", "Run", "Purge", "Burp"};

// =====================================================================
// ADS1015 ADC GLOBALS (Rev 10 - replaces MCP register globals)
// =====================================================================
// ADC readings updated by Core 0 task at 60Hz, read by Core 1 (main loop/web)
volatile float adcPressure = 0.0;       // Pressure in IWC (60-sample rolling average, ~1 sec window)
volatile float adcCurrent = 0.0;        // Abs current in Amps (20-sample rolling average)
volatile float adcPeakCurrent = 0.0;    // Peak current reading (for high-current detection)
volatile int16_t adcRawPressure = 0;    // Raw ADC value for channel 0 (diagnostics)
volatile int16_t adcRawCurrent = 0;     // Raw abs(ch2-ch3) value (diagnostics)
volatile uint32_t adcErrorCount = 0;    // Consecutive read errors (resets on success)
volatile uint32_t adcTotalErrors = 0;   // Total lifetime read errors (diagnostics)
volatile uint32_t adcOutlierCount = 0;  // REV 10.9: Pressure samples rejected by outlier filter (diagnostics)
volatile bool adcInitialized = false;   // True once ADS1015 responds successfully

// REV 10.10: ADC freshness tracking — detect stale data from continuous read failures.
// lastSuccessfulAdcRead is updated every time a valid pressure or current value is read.
// If no successful read occurs for ADC_STALE_TIMEOUT_MS (60 seconds), sendFastSensorPacket()
// sends -99.9 for pressure and 0.0 for current to signal a sensor fault to the Linux device.
// The Python program should interpret -99.9 as "ADC sensor fault — data unavailable."
volatile unsigned long lastSuccessfulAdcRead = 0;    // millis() of last good ADC read (either channel)
const unsigned long ADC_STALE_TIMEOUT_MS = 60000;    // 60 seconds — data considered stale after this
const float ADC_FAULT_PRESSURE = -99.9;              // Sentinel value: "pressure sensor fault"
volatile bool adcDataStale = false;                   // True when no good read for > 60 seconds

// REV 10.14: SD card SPI write gate for ADC reader
// SD card writes generate sustained SPI bus traffic (clock + data) that causes electrical
// interference on the ADS1015 I2C bus or analog input. This biases the ADC reading by
// ~16 counts (≈ -0.71 IWC) for the full 500-800ms duration of the SPI activity.
// When sdWriteActive is true, the ADC reader task holds its previous good values instead
// of reading the ADC, preventing the biased samples from entering the rolling average.
//
// Set by Core 1 (SaveToSD) before SPI writes, cleared after. Checked by Core 0 (adcReaderTask).
volatile bool sdWriteActive = false;

// ADC calibration constants (matched to Python pressure_sensor.py two-point formula)
// Calibration data: ADC 4080 = -31.35 IWC, ADC 26496 = +30.0 IWC (16-bit ADS1115)
// Slope m = (26496 - 4080) / (30.0 - (-31.35)) = 365.44 ADC(16-bit) per IWC
// 12-bit ADS1015 equivalent: 365.44 / 16 = 22.84 ADC(12-bit) per IWC
// Formula: pressure_IWC = (raw_adc - adcZeroPressure) / pressureSlope12bit
//   - Vacuum (raw < zero): NEGATIVE IWC  (normal operating range)
//   - Atmospheric (raw = zero): 0.0 IWC
//   - Above atmospheric (raw > zero): POSITIVE IWC
// adcZeroPressure: Loaded from EEPROM at boot (loadPressureCalibrationFromEeprom).
// Calibrated via {"cal":1} serial command (calibratePressureSensorZeroPoint).
// Factory default: 964.0 (= 15422/16, atmospheric pressure on ADS1015 12-bit).
float adcZeroPressure = 964.0;        // Overwritten by EEPROM value in setup()
float pressureSlope12bit = 22.84;     // 365.44/16 — ADC counts (12-bit) per IWC
const float ADC_ZERO_FACTORY_DEFAULT = 964.0;  // Factory default if EEPROM has no calibration

// REV 10.9: Calibration range validation — middle 20% of scale centered on factory default.
// Prevents bad calibrations (e.g., sensor under vacuum) from saving a wildly wrong zero point.
// ±10% of 964.0 = ±96.4 ADC counts → allowed range: 867.6 to 1060.4
// A calibration at -6 IWC vacuum would compute newZero ≈ 827 → REJECTED (below 867.6).
// At sea level ambient: newZero ≈ 964 → accepted. High altitude (-0.5 IWC): newZero ≈ 953 → accepted.
const float CAL_TOLERANCE_PERCENT = 10.0;  // ±10% = 20% total band (middle 20% of scale)
const float CAL_RANGE_MIN = ADC_ZERO_FACTORY_DEFAULT * (1.0f - CAL_TOLERANCE_PERCENT / 100.0f);  // ~867.6
const float CAL_RANGE_MAX = ADC_ZERO_FACTORY_DEFAULT * (1.0f + CAL_TOLERANCE_PERCENT / 100.0f);  // ~1060.4

// Current: Python uses mapit(abs(ch2-ch3), 1248, 4640, 2.1, 8.0) on ADS1115 (16-bit)
// Those 16-bit single-ended differences correspond to physical voltages:
//   1248 × 125µV = 156mV differential → 2.1A
//   4640 × 125µV = 580mV differential → 8.0A
//
// On ADS1015 we use readADC_Differential_2_3() at GAIN_TWO (±2.048V, 1mV/count)
// which reads the hardware differential directly with full 12-bit precision.
//   156mV / 1mV = 156 counts → 2.1A
//   580mV / 1mV = 580 counts → 8.0A
float adcZeroCurrent = 156.0;     // Differential counts at 2.1A (GAIN_TWO, 1mV/count)
float adcFullCurrent = 580.0;     // Differential counts at 8.0A (GAIN_TWO, 1mV/count)
float currentRangeLow = 2.1;      // Amps at zero ADC
float currentRangeHigh = 8.0;     // Amps at full ADC

// Rolling average buffer for pressure (60 samples = 1 second at 60Hz)
#define PRESSURE_AVG_SAMPLES 60
float pressureBuffer[PRESSURE_AVG_SAMPLES] = {0};
int pressureBufferIdx = 0;
int pressureSampleCount = 0;       // Tracks how many samples collected (for initial fill)

// Windowed peak buffer for current (60 samples = 1 second at 60Hz)
// Matches Python's approach: collect 60 readings, remove min/max outliers,
// then return the MAXIMUM of the trimmed set (peak detection, not average).
// This correctly captures motor current peaks for alarm threshold comparison.
#define CURRENT_PEAK_SAMPLES 60
float currentPeakBuffer[CURRENT_PEAK_SAMPLES] = {0};
int currentPeakIdx = 0;
int currentPeakCount = 0;

// =====================================================================
// RELAY CONTROL GLOBALS (Rev 10 - controlled via serial, not I2C)
// =====================================================================
volatile uint8_t currentRelayMode = 0;        // Current relay mode (0-9)
volatile bool dispShutdownActive = true;       // DISP_SHUTDN state: true=HIGH(on), false=LOW(shutdown)
// DISP_SHUTDN starts HIGH on boot and ONLY goes LOW on {"mode":"shutdown"} serial command

// =====================================================================
// OVERFILL GLOBALS (simplified - direct GPIO38 read, no MCP emulation)
// =====================================================================
unsigned long overfillCheckTimer = 0;
uint8_t overfillLowCount = 0;
bool overfillAlarmActive = false;

// Overfill validation system - prevents false alarms during power cycles/flashes/reboots
// GPIO38 can have transient states during startup before pull-up stabilizes
// PROTECTION LAYERS:
//   1. overfillValidationComplete=false until 15 seconds after boot
//   2. Require 8 consecutive LOW readings (8 seconds) to trigger alarm
//   3. Require 5 consecutive HIGH readings (5 seconds) to clear alarm (hysteresis)
//   Overfill sensor is ACTIVE LOW: LOW = overfill detected, HIGH = normal
bool overfillValidationComplete = false;    // True after startup validation period
unsigned long overfillStartupTime = 0;      // When overfill monitoring started
const unsigned long OVERFILL_STARTUP_LOCKOUT = 15000;  // 15 second lockout after boot
const uint8_t OVERFILL_TRIGGER_COUNT = 8;   // Consecutive LOW readings to trigger
const uint8_t OVERFILL_CLEAR_COUNT = 5;     // Consecutive HIGH readings to clear

// =====================================================================
// REV 10.18: Manual relay override mode
// =====================================================================
// When true, individual cr0/cr1/cr2/cr5 commands control GPIO pins directly.
// The 15-second relay refresh and Linux serial mode commands are blocked
// while in manual override. Use "exit_manual" to restore normal operation.
// Any start_cycle or start_test command also auto-exits manual mode.
bool manualRelayOverride = false;

// =====================================================================
// FAILSAFE MODE GLOBALS (Rev 10 - autonomous relay control when Comfile is down)
// =====================================================================
bool failsafeMode = false;                  // True when ESP32 controls relays autonomously
const int FAILSAFE_FAULT_CODE = 4096;       // REV 10.17: Added to fault code when in failsafe (was 8192)
const int MAX_WATCHDOG_ATTEMPTS_BEFORE_FAILSAFE = 2;  // Enter failsafe after 2 restart attempts

// Failsafe run cycle state
bool failsafeCycleRunning = false;          // True when a failsafe cycle is active
int failsafeCycleStep = 0;                  // Current step in the cycle sequence
unsigned long failsafeCycleStepTimer = 0;   // Timer for current step
unsigned long failsafeLastCycleComplete = 0;// When last cycle completed
int failsafeHighCurrentCount = 0;           // High current event counter (max 4)
int failsafeLowCurrentCount = 0;            // Consecutive low current readings
const float LOW_CURRENT_THRESHOLD = 3.0;    // Amps - alarm if below for 9 readings
const float HIGH_CURRENT_THRESHOLD = 25.0;  // Amps - alarm if above for 2 seconds
const int LOW_CURRENT_CONSECUTIVE = 9;      // Readings below threshold to trigger
const int MAX_HIGH_CURRENT_EVENTS = 4;      // Max high current events before stop
unsigned long failsafeNoCommandTimeout = 0; // Safety: stop if no commands for this long

// Run cycle sequences (from Python control.py)
// Format: {mode, duration_seconds}
struct CycleStep {
    uint8_t mode;
    uint16_t durationSeconds;
};

// Normal run cycle: Run 120s, Idle 3s, then 6x(Purge 50s, Burp 5s), Idle 15s
const CycleStep NORMAL_RUN_CYCLE[] = {
    {1, 120}, {0, 3},
    {2, 50}, {3, 5}, {2, 50}, {3, 5}, {2, 50}, {3, 5},
    {2, 50}, {3, 5}, {2, 50}, {3, 5}, {2, 50}, {3, 5},
    {0, 15}
};
const int NORMAL_RUN_CYCLE_STEPS = 15;

// Manual purge: 6x(Purge 50s, Burp 5s)
const CycleStep MANUAL_PURGE_CYCLE[] = {
    {2, 50}, {3, 5}, {2, 50}, {3, 5}, {2, 50}, {3, 5},
    {2, 50}, {3, 5}, {2, 50}, {3, 5}, {2, 50}, {3, 5}
};
const int MANUAL_PURGE_CYCLE_STEPS = 12;

// Clean canister: Run for 15 minutes
const CycleStep CLEAN_CANISTER_CYCLE[] = {
    {1, 900}
};
const int CLEAN_CANISTER_CYCLE_STEPS = 1;

// REV 10.11: Functionality test — mirrors Python io_manager.py functionality_test()
// Sequence: 10 repetitions of (Run 60s, Purge 60s) = 20 steps, 1200 seconds total (20 min)
// Run = mode 1 (Motor + CR1 + CR5 ON), Purge = mode 2 (Motor + CR2 ON)
// Motor runs during BOTH run and purge; only off when test completes.
const CycleStep FUNCTIONALITY_TEST_CYCLE[] = {
    {1, 60}, {2, 60},   // Rep 1:  Run 60s, Purge 60s
    {1, 60}, {2, 60},   // Rep 2:  Run 60s, Purge 60s
    {1, 60}, {2, 60},   // Rep 3:  Run 60s, Purge 60s
    {1, 60}, {2, 60},   // Rep 4:  Run 60s, Purge 60s
    {1, 60}, {2, 60},   // Rep 5:  Run 60s, Purge 60s
    {1, 60}, {2, 60},   // Rep 6:  Run 60s, Purge 60s
    {1, 60}, {2, 60},   // Rep 7:  Run 60s, Purge 60s
    {1, 60}, {2, 60},   // Rep 8:  Run 60s, Purge 60s
    {1, 60}, {2, 60},   // Rep 9:  Run 60s, Purge 60s
    {1, 60}, {2, 60}    // Rep 10: Run 60s, Purge 60s
};
const int FUNCTIONALITY_TEST_CYCLE_STEPS = 20;

// Pointer to current active cycle sequence
const CycleStep* activeCycleSequence = NORMAL_RUN_CYCLE;
int activeCycleLength = NORMAL_RUN_CYCLE_STEPS;

// =====================================================================
// CBOR PASSTHROUGH BUFFER (Rev 10 - buffers data during PPP sessions)
// =====================================================================
// ~17 bytes per sample, every 15s, 120 min = 480 samples = ~8.2KB
#define CBOR_BUFFER_MAX_SAMPLES 480    // 120 minutes at 15-second intervals
#define CBOR_SAMPLE_SIZE 17            // timestamp(4) + mode(1) + press(4) + current(4) + fault(2) + cycles(2)

struct CborBufferSample {
    uint32_t timestamp;
    uint8_t mode;
    float pressure;
    float current;
    uint16_t fault;
    uint16_t cycles;
};

CborBufferSample* cborPassthroughBuffer = NULL;  // Allocated in setup()
int cborBufferWriteIdx = 0;           // Next write position
int cborBufferCount = 0;              // Number of valid samples in buffer
bool cborBufferingActive = false;     // True during passthrough mode

// Web diagnostics: Store last 5 CBOR payloads for diagnostics page
#define CBOR_PAYLOAD_HISTORY_SIZE 5
struct CborPayloadHistory {
    unsigned long timestamp;
    uint8_t data[1024];
    size_t size;
};
CborPayloadHistory cborPayloadHistory[CBOR_PAYLOAD_HISTORY_SIZE];
int cborPayloadHistoryIdx = 0;
unsigned long lastCborBufferTime = 0; // Last time a sample was buffered

// Thread safety
SemaphoreHandle_t relayMutex = NULL;

// Current mode tracking (read-only from web dashboard, set by I2C master)
RunMode current_mode = IDLE_MODE;

// REV 10.9: Serial Monitor debug mode
// When OFF (default): only essential messages print to USB Serial Monitor
//   (boot, errors, mode changes, watchdog events, calibration)
// When ON: verbose messages print (FAST-TX 5Hz, SD card writes, cell refresh, relay refresh)
// Toggled via Web Config Portal with password. Saved to EEPROM.
bool serialDebugMode = false;                    // Debug mode for Serial Monitor output (default: OFF)

// Serial watchdog variables
bool watchdogEnabled = true;                     // Watchdog feature enable/disable flag (default: enabled)
bool failsafeEnabled = false;                     // Failsafe mode enable/disable flag (default: disabled)
unsigned long lastSerialDataTime = 0;            // Timestamp of last received serial data
bool watchdogTriggered = false;                  // Flag to track if we're in watchdog state (no serial)
int watchdogRebootAttempts = 0;                  // Counter for reboot attempts (0 = no attempts yet)
unsigned long lastWatchdogPulseTime = 0;         // Timestamp of last watchdog pulse
const unsigned long WATCHDOG_TIMEOUT = 1800000;  // 30 minutes in milliseconds (30 * 60 * 1000)
const unsigned long WATCHDOG_FIRST_PULSE = 1000; // First reboot attempt: 1 second pulse
const unsigned long WATCHDOG_LONG_PULSE = 30000; // Subsequent attempts: 30 second pulse
const int SD_CARD_FAULT_CODE = 1024;             // REV 10.17: Fault code for SD card failure
const int WATCHDOG_FAULT_CODE = 2048;            // REV 10.17: Fault code for serial watchdog (was 1024)
const int BLUECHERRY_FAULT_CODE = 8192;          // REV 10.17: Fault code for BlueCherry disconnected (was 4096)
const int FAILSAFE_ACTIVE_FAULT_CODE = 16384;   // NEW: Added when failsafe mode is actively controlling relays

// REV 10.10: LTE connection check interval
// Previously: lteConnected() was checked EVERY loop iteration. If the cellular modem
// temporarily lost registration (normal tower handoff), lteConnect() was called immediately,
// blocking the main loop for up to 5+ minutes (7s of delays + 300s registration timeout).
// During this entire block, ZERO serial data was sent to Linux — no pressure, current, or
// overfill updates. This caused the "serial stall after ~1 hour" bug.
// Now: LTE is checked every 60 seconds. If it drops, serial data continues flowing while
// reconnection is attempted on the next check cycle. No more ESP.restart() on failure.
unsigned long lastLteCheckTime = 0;
const unsigned long LTE_CHECK_INTERVAL = 60000;  // 60 seconds between LTE status checks
int lteReconnectFailures = 0;                     // Track consecutive reconnect failures

// BlueCherry connection tracking variables
// Allows system to run without BlueCherry, but schedules weekly restart for OTA retry
bool blueCherryConnected = false;                // True if BlueCherry initialized successfully
bool fastPollingActive = false;                   // True when fast BlueCherry polling is enabled
unsigned long fastPollingStartTime = 0;          // When fast polling was activated (for timeout)
const unsigned long FAST_POLLING_TIMEOUT = 60UL * 60UL * 1000UL; // 60 minutes in milliseconds
unsigned long blueCherryLastAttempt = 0;         // Timestamp of last BlueCherry connection attempt
unsigned long systemStartTime = 0;               // Timestamp when system started (for weekly restart)
const unsigned long WEEKLY_RESTART_MS = 604800000UL;  // 7 days in milliseconds (7 * 24 * 60 * 60 * 1000)
const unsigned long BC_RETRY_INTERVAL = 3600000; // Retry BlueCherry every hour (60 * 60 * 1000)

// =====================================================================
// REV 10.13: NON-BLOCKING CELLULAR INITIALIZATION STATE MACHINE
// =====================================================================
// Previously, setup() blocked for up to 5+ minutes waiting for LTE registration,
// BlueCherry init, and modem time. If lteConnect() failed, the ESP32 restarted
// entirely — losing relay control, sensor data, and serial to Linux.
//
// Now: setup() completes in ~3 seconds (GPIO, ADC, serial, web portal, relays).
// Cellular connection runs as a background state machine in loop().
// All critical IO functions operate immediately while cellular connects.
//
// State flow:
//   SET_MINIMUM → WAIT_MINIMUM → DEFINE_PDP → SET_AUTH → SET_FULL →
//   WAIT_FULL → SET_AUTO_SELECT → WAIT_REGISTRATION → GET_CELL_INFO →
//   BLUECHERRY → GET_IDENTITY → GET_TIME → COMPLETE
//
// On failure: retries from SET_MINIMUM after a 60-second cooldown.
// No more ESP.restart() on LTE failure during boot.
enum CellularInitState {
    CELL_INIT_SET_MINIMUM,       // Set modem to minimum op state
    CELL_INIT_WAIT_MINIMUM,      // Wait 2 seconds for modem state change
    CELL_INIT_DEFINE_PDP,        // Define APN/PDP context
    CELL_INIT_SET_AUTH,          // Set PDP authentication parameters
    CELL_INIT_SET_FULL,          // Set modem to full op state
    CELL_INIT_WAIT_FULL,         // Wait 2 seconds for modem state change
    CELL_INIT_SET_AUTO_SELECT,   // Set automatic network selection
    CELL_INIT_WAIT_REGISTRATION, // Poll lteConnected() every 1s (up to 300s)
    CELL_INIT_GET_CELL_INFO,     // Retrieve cell info (RSRP, operator, band)
    CELL_INIT_BLUECHERRY,        // Attempt BlueCherry init (up to 3 tries)
    CELL_INIT_GET_IDENTITY,      // Query IMEI, ICCID, IMSI, RAT
    CELL_INIT_GET_TIME,          // Get clock from modem (with timeout)
    CELL_INIT_COMPLETE,          // Done — cellular fully operational
    CELL_INIT_RETRY_COOLDOWN     // Wait 60s before retrying after failure
};

CellularInitState cellularInitState = CELL_INIT_SET_MINIMUM;
bool    cellularInitComplete = false;  // True when state machine reaches COMPLETE
unsigned long cellularInitTimer = 0;   // millis()-based timer for wait states
int     cellularInitRegAttempts = 0;   // Registration poll counter (up to 300)
int     cellularInitBcAttempts = 0;    // BlueCherry retry counter (up to 3)
int     cellularInitRetryCount = 0;    // Full-cycle retry counter
const int CELL_INIT_MAX_REG_WAIT = 300;   // Max seconds to wait for network registration
const int CELL_INIT_MAX_BC_ATTEMPTS = 3;  // Max BlueCherry init attempts
const unsigned long CELL_INIT_RETRY_COOLDOWN_MS = 60000; // 60 seconds between full retries

// =====================================================================
// RMS CONFIGURATION (Updated SD pins)
// =====================================================================
#define BC_DEVICE_TYPE "vaporcf1"
#define BC_TLS_PROFILE 1
#define MISO 7             // SD card MISO
#define MOSI 6             // SD card MOSI
#define SCLK 15            // SD card SCK (clock)
#define CS 40              // SD card CS (chip select)
#define HTTP_PROFILE 0
#define TLS_PROFILE 0
#define COAP_PROFILE 1
#define MAX_BUFFER_SIZE 2000
#define RX_PIN 44
#define TX_PIN 43
#define ModemSerial Serial2

// Walter Modem hardware pins (for direct passthrough without library)
#define WALTER_MODEM_PIN_RX 14
#define WALTER_MODEM_PIN_TX 48
#define WALTER_MODEM_PIN_RTS 21
#define WALTER_MODEM_PIN_CTS 47
#define WALTER_MODEM_PIN_RESET 45
#define ESP_POWER_EN  0

// Modem baud rate - WalterModem library configures modem to this speed
// Both normal operation and passthrough use the same 115200 baud rate
// No baud rate changes needed for passthrough mode

// Cellular network configuration
#define CELLULAR_APN "soracom.io"
#define CELLULAR_USERNAME "sora"
#define CELLULAR_PASSWORD "sora"
#define CELLULAR_AUTH_PROTOCOL WALTER_MODEM_PDP_AUTH_PROTO_PAP

// ### Global Objects and Variables ###
Preferences preferences;
byte otaBuffer[SPI_FLASH_BLOCK_SIZE] = { 0 };
WalterModem modem;
WalterModemRsp rsp = {};
const char *bc_ca_cert = "-----BEGIN CERTIFICATE-----\r\n\
MIIBlTCCATqgAwIBAgICEAAwCgYIKoZIzj0EAwMwGjELMAkGA1UEBhMCQkUxCzAJ\r\n\
BgNVBAMMAmNhMB4XDTI0MDMyNDEzMzM1NFoXDTQ0MDQwODEzMzM1NFowJDELMAkG\r\n\
A1UEBhMCQkUxFTATBgNVBAMMDGludGVybWVkaWF0ZTBZMBMGByqGSM49AgEGCCqG\r\n\
SM49AwEHA0IABJGFt28UrHlbPZEjzf4CbkvRaIjxDRGoeHIy5ynfbOHJ5xgBl4XX\r\n\
hp/r8zOBLqSbu6iXGwgjp+wZJe1GCDi6D1KjZjBkMB0GA1UdDgQWBBR/rtuEomoy\r\n\
49ovMAnj5Hpmk2gTGjAfBgNVHSMEGDAWgBR3Vw0Y1sUvMhkX7xySsX55tvsu8TAS\r\n\
BgNVHRMBAf8ECDAGAQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAKBggqhkjOPQQDAwNJ\r\n\
ADBGAiEApN7DmuufC/aqyt6g2Y8qOWg6AXFUyTcub8/Y28XY3KgCIQCs2VUXCPwn\r\n\
k8jR22wsqNvZfbndpHthtnPqI5+yFXrY4A==\r\n\-----END CERTIFICATE-----\r\n\
-----BEGIN CERTIFICATE-----\r\n\
MIIBmDCCAT+gAwIBAgIUDjfXeosg0fphnshZoXgQez0vO5UwCgYIKoZIzj0EAwMw\r\n\
GjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMB4XDTI0MDMyMzE3MzU1MloXDTQ0\r\n\
MDQwNzE3MzU1MlowGjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMFkwEwYHKoZI\r\n\
zj0CAQYIKoZIzj0DAQcDQgAEB00rHNthOOYyKj80cd/DHQRBGSbJmIRW7rZBNA6g\r\n\
fbEUrY9NbuhGS6zKo3K59zYc5R1U4oBM3bj6Q7LJfTu7JqNjMGEwHQYDVR0OBBYE\r\n\
FHdXDRjWxS8yGRfvHJKxfnm2+y7xMB8GA1UdIwQYMBaAFHdXDRjWxS8yGRfvHJKx\r\n\
fnm2+y7xMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgGGMAoGCCqGSM49\r\n\
BAMDA0cAMEQCID7AcgACnXWzZDLYEainxVDxEJTUJFBhcItO77gcHPZUAiAu/ZMO\r\n\
VYg4UI2D74WfVxn+NyVd2/aXTvSBp8VgyV3odA==\r\n\-----END CERTIFICATE-----\r\n";

String diagData = "NO SERIAL DATA RECEIVED - CHECK CONNECTION";
char dataBuffer[MAX_BUFFER_SIZE];
static char outgoingMsg[MAX_BUFFER_SIZE] = "";
uint8_t cborBuffer[1024];
char deviceName[20];
String Modem_Status = "Unknown";
String modemBand = "";            // LTE band number (e.g. "12", "4")
String modemNetName = "";         // Operator/network name (e.g. "T-Mobile")
String modemRSRP = "";            // Reference Signal Received Power in dBm (e.g. "-89.5")
String modemRSRQ = "";            // Reference Signal Received Quality in dB (e.g. "-11.2")
uint16_t modemCC = 0;             // Country Code (e.g. 310 for US)
uint16_t modemNC = 0;             // Network Code (e.g. 260 for T-Mobile US)
uint32_t modemCID = 0;
uint32_t modemTAC = 0;            // Cell ID - unique identifier of the serving cell tower

// REV 10.5/10.9 -- Freshness tracking for modem/cellular data
// modemDataFresh: set TRUE when refreshCellSignalInfo() or lteConnect() gets new cell data.
//   REV 10.9: No longer gates sending -- cellular packet is sent every 10 seconds on timer.
//   Flag is cleared after each 10-second send for internal tracking only.
// modemTimeFresh: set TRUE when initModemTime() gets a fresh clock from modem.
//   REV 10.9: Gates sending of standalone datetime packet. Only sends when fresh.
//   This prevents the "datetime repeats then updates" problem.
bool modemDataFresh = false;      // Set true when refreshCellSignalInfo() gets new cell data
bool modemTimeFresh = false;      // Set true when modem clock is freshly queried
String imei = "";
String modemIMSI = "";                           // REV 10.9: SIM card IMSI (International Mobile Subscriber Identity)
String modemICCID = "";                          // REV 10.9: SIM card ICCID (Integrated Circuit Card ID)
String modemTechnology = "";                     // REV 10.9: Radio Access Technology ("LTE-M", "NB-IoT", "Auto", "Unknown")
String macStr = "";
int64_t currentTimestamp = 0;
uint32_t lastMillis = 0;
bool firstTime = true;
static unsigned long lastSendTime = 0;
static unsigned long lastFirmwareCheckMillis = 0;   // millis() of last firmware check
static bool firmwareCheckDoneThisSlot = false;       // Prevents re-checking within same 15-min slot
static unsigned long readTime = 0;
// SPEED FIX: Was 5000ms (5 seconds!) — caused massive delay between Python
// sending a mode command and ESP32 actually reading it. readSerialData() is
// lightweight (just checks Serial1.available()), so 100ms is safe and responsive.
static unsigned int readInterval = 100;   // 100ms — relay commands apply within 100ms
static unsigned long int seq = 0;
static unsigned long lastSDCheck = 0;
unsigned long currentTime = millis();
int unsigned noSerialCount = 0;
static unsigned int noComInterval = 86400000;
static unsigned int sendInterval;
static unsigned int parseSDInterval=15000;
#define STATUS_INTERVAL 86400000       // 24 hours in milliseconds - daily unified status report
unsigned long lastDailyStatusTime = 0; // Tracks when last daily status was sent to port 5686
int lastParseTime = 0 ;
String DeviceName = "";  // Device name (e.g., "RND-0007", "CSX-1234")
bool firstLostComm = 0;
int NoSerial = 0;
AsyncWebServer server(80);
AsyncDNSServer dnsServer;
const byte DNS_PORT = 53;

// Dynamic WiFi AP name - includes device ID or random hex suffix
// Format: "GM IO Board XXXXXX" where XXXXXX is device ID or random hex
char apSSID[32] = "GM IO Board";  // Will be updated with suffix
String apSuffix = "";             // Stores the current suffix (random or device ID)
bool apNameFromSerial = false;    // True if AP name was set from serial device ID

// =====================================================================
// WEB PORTAL TEST STATE TRACKING (Rev 10.11: ESP32 owns relays during tests)
// =====================================================================
// When a test is started from the Web Config Portal, the ESP32 takes SOLE
// control of the relays. ALL mode commands from the Linux device are BLOCKED
// until the test completes (timer expires) or is manually stopped from the
// web portal. This ensures tests run reliably regardless of what Linux sends.
//
// The ESP32:
//   1. Sets relays immediately when the Start button is pressed
//   2. Blocks ALL Linux mode commands for the duration of the test
//   3. Auto-stops the test when the timer expires (relays → idle, control → Linux)
//   4. Still forwards the command to Linux for logging/awareness
//
// Supported test types and their configurations:
//   "leak"  — Mode 9 (all valves open, motor OFF) for 30 minutes [SINGLE-MODE]
//   "func"  — 10x (Run 60s → Purge 60s) = 20 min  [MULTI-STEP cycle]
//             Mirrors Python io_manager.py functionality_test()
//             Motor runs during both Run (mode 1) and Purge (mode 2)
//   "eff"   — Mode 1 (run mode) for 2 minutes [SINGLE-MODE]
//   "clean" — Mode 1 (run mode) for 120 minutes (2 hours) [SINGLE-MODE]
//
// Usage example (single-mode):
//   testRunning = true; activeTestType = "leak"; testStartTime = millis();
//   setRelaysForMode(9);  // ESP32 owns the relays now
//   // ... 30 minutes later, auto-stop fires in loop()
//
// Usage example (multi-step):
//   testRunning = true; activeTestType = "func"; webTestMultiStep = true;
//   webTestCycleSequence = FUNCTIONALITY_TEST_CYCLE; webTestCycleLength = 20;
//   // ... runWebPortalTestCycleLogic() in loop() steps through all 20 steps
//   // ... auto-completes when last step finishes
String  activeTestType = "";          // "leak", "func", "eff", "clean", "" (empty = no test)
unsigned long testStartTime = 0;      // millis() when test started
bool    testRunning = false;          // True while a web portal test is in progress

// REV 10.11: Web portal multi-step test cycle state (for functionality test)
// Functionality test is a 20-step sequence (10x run-purge) that requires stepping
// through modes, unlike leak/eff/clean which hold a single mode for the duration.
// These variables track the web portal cycle separately from the failsafe cycle.
bool    webTestMultiStep = false;               // True when running a multi-step test (func)
const CycleStep* webTestCycleSequence = NULL;   // Pointer to step array
int     webTestCycleLength = 0;                 // Number of steps in the sequence
int     webTestCycleStep = 0;                   // Current step index
unsigned long webTestCycleStepTimer = 0;        // millis() when current step started

// REV 10.11: Test duration lookup (in seconds) — matches web UI JavaScript durations
// Used by auto-stop in loop() to complete tests. Func test is multi-step, but
// total duration is still 1200s for the web UI countdown timer.
// Returns 0 for unknown test types (no auto-stop).
unsigned long getTestDurationSeconds(const String& testType) {
    if (testType == "leak")  return 1800;   // 30 minutes
    if (testType == "func")  return 1200;   // 20 minutes (10x run 60s + purge 60s)
    if (testType == "eff")   return 120;    // 2 minutes
    if (testType == "clean") return 7200;   // 120 minutes (2 hours)
    return 0;  // Unknown — no auto-stop
}

// REV 10.11: Get the INITIAL relay mode for a given test type.
// For single-mode tests (leak, eff, clean), this is the only mode used.
// For multi-step tests (func), this returns the FIRST step mode — subsequent
// steps are driven by runWebPortalTestCycleLogic().
// Returns 0 (idle) for unknown test types.
uint8_t getTestRelayMode(const String& testType) {
    if (testType == "leak")  return 9;   // All valves open, motor OFF
    if (testType == "func")  return 1;   // First step is Run (multi-step cycle handles the rest)
    if (testType == "eff")   return 1;   // Run mode
    if (testType == "clean") return 1;   // Run mode (motor on for 2 hours)
    return 0;  // Unknown
}

// REV 10.11: Check if a test type uses a multi-step cycle sequence.
// Multi-step tests are driven by runWebPortalTestCycleLogic() in loop(),
// NOT by the simple timer-based auto-stop.
bool isMultiStepTest(const String& testType) {
    return (testType == "func");
}

// REV 10.5/10.9: Serial JSON output to Python device (via Serial1/RS-232)
// Three sending mechanisms (Rev 10.9):
//   1. FAST sensor packet: pressure, current, overfill, sdcard, relayMode, failsafe, shutdown — every 200ms (5Hz)
//   2. Cellular status: lte, rsrp, rsrq, operator, band, mcc, mnc, cellId, tac, profile — every 10s (timer)
//   3. Datetime: {"datetime":"..."} — ONLY when modemTimeFresh is true (fresh from modem)
unsigned long lastSensorSendTime = 0;
const unsigned long SENSOR_SEND_INTERVAL = 200;    // 200ms = 5Hz - fast sensor updates (pressure, current, overfill, sdcard)

// REV 10.9: Cellular status now sent every 10 seconds on a TIMER (not freshness-gated).
// Previously (Rev 10.5): only sent when modemDataFresh/modemTimeFresh flags were set.
// Problem: if modem queries fail silently, the cellular packet was NEVER sent, leaving
// the Linux device with no signal/LTE info at all. Now we always send cached values
// every 10 seconds so the Linux device always has the latest known state.
// Datetime is sent separately, ONLY when modemTimeFresh is true (fresh from modem).
unsigned long lastCellularSendTime = 0;
const unsigned long CELLULAR_SEND_INTERVAL = 10000; // 10 seconds — always send cached cellular info

// REV 10.8: Periodic relay state refresh timer
// Re-applies currentRelayMode to physical GPIO pins every 15 seconds.
// Guards against: relay driver glitch, failed GPIO write, pin drift, or missed mode command.
// The Linux 15-second payload also includes mode, so this refresh runs at the same cadence.
// Skipped when in failsafe mode (failsafe manages its own relays autonomously).
//
// Usage: Checked in loop(). If 15 seconds elapsed and not in failsafe, calls
//        setRelaysForMode(currentRelayMode) to re-assert the stored mode to pins.
unsigned long lastRelayRefreshTime = 0;
const unsigned long RELAY_REFRESH_INTERVAL = 15000; // 15 seconds — matches Linux payload cadence

// Passthrough mode - allows IO Board to act as a serial bridge to the modem
// When enabled, Serial1 (RS-232) data is forwarded directly to/from the modem
// This allows Linux host to use AT commands and PPP connections via the modem
// NOTE: Passthrough mode is NOT persisted - always returns to normal on reboot
bool passthroughMode = false;      // True when passthrough is active
int passthroughValue = 0;          // 0=normal operation, 1=passthrough active (sent in JSON status)

// Data backfill system - sends SD card data collected during passthrough mode
// to Linux device after passthrough ends for cloud synchronization
bool backfillPending = false;      // True when backfill procedure should run after boot
String passthroughStartTime = "";  // ISO format datetime when passthrough started
bool purgeNeeded = false;          // True when cycle should be stopped before passthrough

// Forward declarations for passthrough and backfill functions (needed for lambda in web server)
bool sendPassthroughRequestToLinux(unsigned long timeoutMinutes);
void enterPassthroughMode(unsigned long timeoutMinutes = 60);
void exitPassthroughMode();
void runPassthroughLoop();
void dataBackfill();               // Send collected data to Linux device

// QC mode removed in Rev 9.4 - web interface is now read-only diagnostic dashboard
// Relay control is exclusively managed by I2C from the Linux master

uint8_t dataBuf[8] = { 0 };
uint8_t incomingBuf[256] = { 0 };
uint16_t counter = 0;

String idStr = "";
int readings[12][10];
int readingCount = 0;
float pressure = 0.0;
int pres_scaled = 0;
int temp_scaled = 0;
int cycles = 0, faults = 0, mode = 0;
float current = -99.8;
File logFile;
unsigned long lastFlush = 0;
const unsigned long FLUSH_INTERVAL = 30 * 1000;

// =====================================================================
// STATUS LED FUNCTIONS (WS2812B on GPIO9)
// =====================================================================
// Controls a single WS2812B RGB LED to indicate current operating mode:
//   - Mode 0 (Idle):       Blue breathing effect (smooth fade in/out)
//   - Mode 1 (Run):        Solid green
//   - Mode 2 (Purge):      Solid orange
//   - Mode 3 (Burp):       Solid yellow
//   - Passthrough:          Flashing red (500ms on/500ms off)
//
// The LED brightness is kept moderate (max ~80) to avoid being blinding
// in dark enclosures. The breathing effect uses a sine-wave approximation
// for a smooth, organic pulse.
//
// Usage:
//   updateStatusLED();              // Call from loop(), uses current_mode
//   updatePassthroughLED();         // Call from passthrough loop only

// LED brightness cap (0-255) - keep moderate for WS2812B-2020 which is very bright
#define LED_MAX_BRIGHTNESS 80

/**
 * Update the status LED based on current operating mode.
 * Call this periodically from the main loop() - non-blocking, uses millis().
 * 
 * Mode 0 (IDLE_MODE):  Blue breathing effect - sine-wave brightness ramp
 * Mode 1 (RUN_MODE):   Solid green
 * Mode 2 (PURGE_MODE): Solid orange (R:255 G:80 B:0 scaled)
 * Mode 3 (BURP_MODE):  Solid yellow (R:255 G:200 B:0 scaled)
 *
 * Usage example:
 *   // In loop():
 *   updateStatusLED();  // Automatically reads current_mode global
 */
void updateStatusLED() {
    static unsigned long lastLEDUpdate = 0;
    unsigned long now = millis();
    
    // Update LED every 20ms (~50 fps) for smooth breathing animation
    if (now - lastLEDUpdate < 20) return;
    lastLEDUpdate = now;
    
    switch (current_mode) {
        case IDLE_MODE: {
            // Blue breathing effect using sine-wave approximation
            // Full breath cycle = ~4 seconds (4000ms period)
            // sin() returns -1 to +1, we map to 0 to LED_MAX_BRIGHTNESS
            float breathPhase = (now % 4000) / 4000.0 * 2.0 * PI;
            uint8_t brightness = (uint8_t)((sin(breathPhase) + 1.0) * 0.5 * LED_MAX_BRIGHTNESS);
            statusLED.setPixelColor(0, statusLED.Color(0, 0, brightness));  // Blue
            break;
        }
        case RUN_MODE:
            // Solid green
            statusLED.setPixelColor(0, statusLED.Color(0, LED_MAX_BRIGHTNESS, 0));
            break;
        case PURGE_MODE:
            // Solid orange (red + ~30% green)
            statusLED.setPixelColor(0, statusLED.Color(LED_MAX_BRIGHTNESS, LED_MAX_BRIGHTNESS / 3, 0));
            break;
        case BURP_MODE:
            // Solid yellow (red + ~80% green)
            statusLED.setPixelColor(0, statusLED.Color(LED_MAX_BRIGHTNESS, (LED_MAX_BRIGHTNESS * 4) / 5, 0));
            break;
        default:
            // Unknown mode - dim white
            statusLED.setPixelColor(0, statusLED.Color(10, 10, 10));
            break;
    }
    statusLED.show();
}

/**
 * Update the status LED for passthrough mode - flashing red.
 * Call this from the passthrough boot loop. Non-blocking, uses millis().
 * Flashes 500ms on / 500ms off (1 Hz blink rate).
 *
 * Usage example:
 *   // In runPassthroughBootMode() while loop:
 *   updatePassthroughLED();
 */
void updatePassthroughLED() {
    static unsigned long lastFlashUpdate = 0;
    unsigned long now = millis();
    
    // Update every 20ms for responsiveness
    if (now - lastFlashUpdate < 20) return;
    lastFlashUpdate = now;
    
    // 500ms on, 500ms off = 1 second period
    bool ledOn = ((now % 1000) < 500);
    if (ledOn) {
        statusLED.setPixelColor(0, statusLED.Color(LED_MAX_BRIGHTNESS, 0, 0));  // Red
    } else {
        statusLED.setPixelColor(0, statusLED.Color(0, 0, 0));  // Off
    }
    statusLED.show();
}

/**
 * Initialize the WS2812B status LED.
 * Call once during setup() or passthrough init. Sets LED to off initially.
 *
 * Usage example:
 *   initializeStatusLED();  // Call in setup() before main loop
 */
void initializeStatusLED() {
    statusLED.begin();
    statusLED.setBrightness(255);  // Brightness controlled per-pixel via color values
    statusLED.setPixelColor(0, statusLED.Color(0, 0, 0));  // Start OFF
    statusLED.show();
    Serial.println("✓ WS2812B status LED initialized on GPIO9");
}

// =====================================================================
// SERIAL WATCHDOG FUNCTION
// =====================================================================

/**
 * Print an obnoxious banner message for watchdog events
 * Makes it very easy to spot in log files
 * 
 * Usage example:
 *   printWatchdogBanner("REBOOT ATTEMPT #1");
 */
void printWatchdogBanner(const char* message) {
    Serial.printf("[WATCHDOG] %s (Device: %s, Attempts: %d)\r\n", 
                  message, deviceName, watchdogRebootAttempts);
}

/**
 * Pulse GPIO39 HIGH for specified duration, then LOW
 * This function is called when no serial data has been received for 30 minutes
 * The pulse triggers an external relay to power cycle the device sending serial data
 * 
 * First attempt: 1 second pulse (quick reboot)
 * Subsequent attempts: 30 second pulse (longer power cycle)
 * 
 * Usage example:
 *   pulseWatchdogPin(1000);   // 1 second pulse
 *   pulseWatchdogPin(30000);  // 30 second pulse
 */
void pulseWatchdogPin(unsigned long pulseDuration) {
    Serial.printf("[WATCHDOG] GPIO39 HIGH - %lu sec pulse (Device: %s, Attempt #%d)\r\n", 
                  pulseDuration / 1000, deviceName, watchdogRebootAttempts);
    
    digitalWrite(ESP_WATCHDOG_PIN, HIGH);
    
    // For long pulses, print status every 10 seconds
    if (pulseDuration > 5000) {
        unsigned long startTime = millis();
        while (millis() - startTime < pulseDuration) {
            unsigned long remaining = (pulseDuration - (millis() - startTime)) / 1000;
            Serial.printf("[WATCHDOG] Power cycle... %lu sec remaining\r\n", remaining);
            delay(10000);
        }
    } else {
        delay(pulseDuration);
    }
    
    digitalWrite(ESP_WATCHDOG_PIN, LOW);
    Serial.println("[WATCHDOG] GPIO39 LOW - pulse complete, waiting for serial data");
}

/**
 * Check serial watchdog timer and trigger pulse if timeout exceeded
 * Monitors the time since last serial data reception. If 30 minutes pass
 * without any serial data and the watchdog is enabled, triggers a power cycle
 * pulse on GPIO39 to attempt to restore communications.
 * 
 * First attempt: 1 second pulse
 * All subsequent attempts: 30 second pulse (every 30 minutes)
 * 
 * During no-serial state, data is sent with zeroed values and fault code 2048
 * 
 * The watchdog automatically resets when serial data resumes.
 * 
 * Usage example:
 *   checkSerialWatchdog();  // Call periodically in main loop
 */
void checkSerialWatchdog() {
    // Only check if watchdog feature is enabled
    if (!watchdogEnabled) {
        return;
    }
    
    // Calculate time since last serial data
    unsigned long timeSinceLastData = millis() - lastSerialDataTime;
    
    // Check if we've exceeded the initial 30-minute timeout
    if (timeSinceLastData >= WATCHDOG_TIMEOUT) {
        
        // Mark that we're in watchdog state (no serial data)
        if (!watchdogTriggered) {
            watchdogTriggered = true;
            lastWatchdogPulseTime = 0;  // Force immediate first pulse
            Serial.printf("[WATCHDOG] ACTIVATED - no data for %lu min, fault code 2048\r\n", 
                          timeSinceLastData / 60000);
        }
        
        // Check if it's time for another reboot attempt
        // Interval depends on failsafe enabled state:
        // - Failsafe disabled: reboot every 10 minutes
        // - Failsafe enabled: reboot every 30 minutes (normal behavior)
        unsigned long rebootInterval = failsafeEnabled ? WATCHDOG_TIMEOUT : (10UL * 60UL * 1000UL); // 10 min vs 30 min
        unsigned long timeSinceLastPulse = millis() - lastWatchdogPulseTime;

        if (lastWatchdogPulseTime == 0 || timeSinceLastPulse >= rebootInterval) {
            watchdogRebootAttempts++;

            // Determine pulse duration: 1 second for first attempt, 30 seconds for subsequent
            unsigned long pulseDuration;
            if (watchdogRebootAttempts == 1) {
                pulseDuration = WATCHDOG_FIRST_PULSE;  // 1 second
            } else {
                pulseDuration = WATCHDOG_LONG_PULSE;   // 30 seconds
            }

            // Trigger the watchdog pulse
            pulseWatchdogPin(pulseDuration);

            // Record when we pulsed
            lastWatchdogPulseTime = millis();

            unsigned long nextIntervalMinutes = failsafeEnabled ? 30 : 10;
            Serial.printf("[WATCHDOG] Next attempt in %lu min if no serial data\r\n", nextIntervalMinutes);
        }
    }
}

/**
 * Check if we're currently in watchdog state (no serial data)
 * Used by data sending functions to determine if we should send zeroed data
 * 
 * Returns: true if watchdog has been triggered (no serial for 30+ minutes)
 * 
 * Usage example:
 *   if (isInWatchdogState()) {
 *       // Send zeroed data with fault code 2048
 *   }
 */
bool isInWatchdogState() {
    return watchdogTriggered && watchdogEnabled;
}

/**
 * Get the SD card fault code (1024) for use in data packets.
 * Returns 1024 if SD card is not mounted/accessible, 0 if OK.
 * 
 * Usage example:
 *   int fault = getSDCardFaultCode();  // Returns 1024 if SD card failed
 */
int getSDCardFaultCode() {
    return isSDCardOK() ? 0 : SD_CARD_FAULT_CODE;
}

/**
 * Get the watchdog fault code (2048) for use in data packets.
 * Returns 0 if not in watchdog state.
 * 
 * Usage example:
 *   int fault = getWatchdogFaultCode();  // Returns 2048 if in watchdog state
 */
int getWatchdogFaultCode() {
    return isInWatchdogState() ? WATCHDOG_FAULT_CODE : 0;
}

/**
 * Get the BlueCherry fault code (8192) for use in data packets.
 * Returns 8192 if BlueCherry is not connected, 0 if connected.
 * This alerts the user that OTA updates are unavailable.
 * 
 * Usage example:
 *   int fault = getBlueCherryFaultCode();  // Returns 8192 if BC disconnected
 */
int getBlueCherryFaultCode() {
    return blueCherryConnected ? 0 : BLUECHERRY_FAULT_CODE;
}

// getCombinedFaultCode() defined later with failsafe support (Rev 10)

/**
 * Reset watchdog timer when serial data is received
 * Call this function whenever valid serial data is received to reset
 * the 30-minute timeout counter and clear all watchdog state.
 * 
 * Usage example:
 *   resetSerialWatchdog();  // Call when serial data arrives
 */
void resetSerialWatchdog() {
    // Only print recovery message if we were in watchdog state
    if (watchdogTriggered && watchdogEnabled) {
        Serial.println("\n");
        Serial.println("╔══════════════════════════════════════════════════════════════════════════════╗");
        Serial.println("║  ✅✅✅ SERIAL DATA RESTORED - WATCHDOG RESET ✅✅✅                         ║");
        Serial.println("╠══════════════════════════════════════════════════════════════════════════════╣");
        Serial.print("║  Total reboot attempts before recovery: ");
        Serial.print(watchdogRebootAttempts);
        Serial.println("                                   ║");
        Serial.println("║  Resuming normal operation with live data                                   ║");
        Serial.println("╚══════════════════════════════════════════════════════════════════════════════╝");
        Serial.println("\n");
    }
    
    lastSerialDataTime = millis();
    watchdogTriggered = false;
    watchdogRebootAttempts = 0;
    lastWatchdogPulseTime = 0;
}

// =====================================================================
// REV 10.18: UNIFIED COMMAND SYSTEM
// =====================================================================
// notifyPythonOfCommand() - sends a JSON command to the Python program
// on Serial1 so the Linux host can act on or relay the command.
//
// Usage example:
//   notifyPythonOfCommand("overfill_override", "1");   // sends {"command":"overfill_override","value":"1"}
//   notifyPythonOfCommand("stop_test", "");             // sends {"command":"stop_test"}
// =====================================================================

/**
 * Send a JSON command notification to the Python program on Serial1.
 * Omits "value" key when value is empty. No "source" field is sent
 * (removed in Rev 10.18 to minimize serial traffic).
 *
 * @param command  The command name (e.g. "overfill_override")
 * @param value    Optional value string; pass "" to omit from JSON
 */
void notifyPythonOfCommand(const char* command, const char* value) {
    if (strlen(value) > 0) {
        Serial1.printf("{\"command\":\"%s\",\"value\":\"%s\"}\n", command, value);
    } else {
        Serial1.printf("{\"command\":\"%s\"}\n", command);
    }
    Serial.printf("[CMD->PY] %s %s\n", command, value);
}

/**
 * Central command dispatcher - all remote commands (Web Portal, BlueCherry,
 * Serial1 from Python) route through here. This ensures identical behavior
 * regardless of source.
 *
 * Supported commands:
 *   stop_test           - stop the currently running test
 *   overfill_override   - toggle overfill override (value "1" to enable)
 *   clear_press_alarm   - clear pressure alarm fault
 *   clear_motor_alarm   - clear motor alarm fault
 *   enable_failsafe     - enter failsafe mode
 *   exit_failsafe       - exit failsafe mode
 *   start_leak_test     - begin leak test
 *   start_func_test     - begin functionality test
 *   start_eff_test      - begin efficiency test
 *   cr0, cr1, cr2, cr5  - direct relay control (value "0" or "1"), enters manual mode
 *   exit_manual          - exit manual relay override mode
 *   calibrate_pressure  - calibrate the pressure sensor zero point
 *
 * Usage example:
 *   executeRemoteCommand("overfill_override", "1");
 *   executeRemoteCommand("cr1", "1");        // turns on CR1, enters manual mode
 *   executeRemoteCommand("exit_manual", "");  // resumes normal operation
 *
 * @param cmd    The command name string
 * @param value  An optional parameter value (empty string if none)
 */
void executeRemoteCommand(String cmd, String value) {
    Serial.printf("[CMD] Executing: %s value=%s\n", cmd.c_str(), value.c_str());

    // --- Stop Test ---
    if (cmd == "stop_test") {
        testRunning = false;
        notifyPythonOfCommand("stop_test", "");
        Serial.println("[CMD] Test stopped.");
    }
    // --- Overfill Override ---
    else if (cmd == "overfill_override") {
        overfillAlarmActive = false;  // Clear the overfill alarm
        overfillLowCount = 0;        // Reset the low-reading counter
        notifyPythonOfCommand("overfill_override", "1");
        Serial.println("[CMD] Overfill override activated (alarm cleared).");
    }
    // --- Clear Pressure Alarm ---
    else if (cmd == "clear_press_alarm") {
        failsafeLowCurrentCount = 0;
        failsafeHighCurrentCount = 0;
        notifyPythonOfCommand("clear_press_alarm", "");
        Serial.println("[CMD] Pressure alarm cleared.");
    }
    // --- Clear Motor Alarm ---
    else if (cmd == "clear_motor_alarm") {
        failsafeLowCurrentCount = 0;
        failsafeHighCurrentCount = 0;
        notifyPythonOfCommand("clear_motor_alarm", "");
        Serial.println("[CMD] Motor alarm cleared.");
    }
    // --- Enable Failsafe Setting ---
    else if (cmd == "enable_failsafe") {
        failsafeEnabled = true;
        preferences.begin("RMS", false);
        preferences.putBool("failsafe", failsafeEnabled);
        preferences.end();
        notifyPythonOfCommand("enable_failsafe", "");
        Serial.println("[CMD] Failsafe setting ENABLED.");
    }
    // --- Disable Failsafe Setting ---
    else if (cmd == "exit_failsafe") {
        failsafeEnabled = false;
        preferences.begin("RMS", false);
        preferences.putBool("failsafe", failsafeEnabled);
        preferences.end();
        // If disabling failsafe while in failsafe mode, exit failsafe
        if (failsafeMode) {
            exitFailsafeMode();
            Serial.println("[FAILSAFE] Exited failsafe mode due to failsafe being disabled");
        }
        notifyPythonOfCommand("exit_failsafe", "");
        Serial.println("[CMD] Failsafe setting DISABLED.");
    }
    // --- Start Leak Test ---
    else if (cmd == "start_leak_test") {
        testRunning = true;
        manualRelayOverride = false;  // auto-exit manual mode
        notifyPythonOfCommand("start_leak_test", "");
        Serial.println("[CMD] Leak test started.");
    }
    // --- Start Functionality Test ---
    else if (cmd == "start_func_test") {
        testRunning = true;
        manualRelayOverride = false;  // auto-exit manual mode
        notifyPythonOfCommand("start_func_test", "");
        Serial.println("[CMD] Functionality test started.");
    }
    // --- Start Efficiency Test ---
    else if (cmd == "start_eff_test") {
        testRunning = true;
        manualRelayOverride = false;  // auto-exit manual mode
        notifyPythonOfCommand("start_eff_test", "");
        Serial.println("[CMD] Efficiency test started.");
    }
    // --- Direct Relay Control: CR0 (Motor) ---
    else if (cmd == "cr0") {
        manualRelayOverride = true;
        int state = value.toInt();
        digitalWrite(CR0_MOTOR, state ? HIGH : LOW);
        notifyPythonOfCommand("cr0", value.c_str());
        Serial.printf("[CMD] CR0_MOTOR set to %d (manual override ON)\n", state);
    }
    // --- Direct Relay Control: CR1 ---
    else if (cmd == "cr1") {
        manualRelayOverride = true;
        int state = value.toInt();
        digitalWrite(CR1, state ? HIGH : LOW);
        notifyPythonOfCommand("cr1", value.c_str());
        Serial.printf("[CMD] CR1 set to %d (manual override ON)\n", state);
    }
    // --- Direct Relay Control: CR2 ---
    else if (cmd == "cr2") {
        manualRelayOverride = true;
        int state = value.toInt();
        digitalWrite(CR2, state ? HIGH : LOW);
        notifyPythonOfCommand("cr2", value.c_str());
        Serial.printf("[CMD] CR2 set to %d (manual override ON)\n", state);
    }
    // --- Direct Relay Control: CR5 ---
    else if (cmd == "cr5") {
        manualRelayOverride = true;
        int state = value.toInt();
        digitalWrite(CR5, state ? HIGH : LOW);
        notifyPythonOfCommand("cr5", value.c_str());
        Serial.printf("[CMD] CR5 set to %d (manual override ON)\n", state);
    }
    // --- Exit Manual Override ---
    else if (cmd == "exit_manual") {
        manualRelayOverride = false;
        notifyPythonOfCommand("exit_manual", "");
        Serial.println("[CMD] Manual relay override DISABLED, returning to normal mode.");
    }
    // --- Calibrate Pressure Sensor ---
    else if (cmd == "calibrate_pressure") {
        calibratePressureSensorZeroPoint();
        // calibratePressureSensorZeroPoint() sends its own response to Serial1
        Serial.println("[CMD] Pressure calibration executed.");
    }
    // --- Set Manual ADC Zero ---
    // Allows user to manually set the ADC zero point for pressure sensor calibration.
    // Unlike calibrate_pressure (which reads the live ADC), this accepts a user-supplied value.
    // The value is validated against MANUAL_ADC_MIN/MAX (broader range than CAL_RANGE), then saved to EEPROM.
    // HTTP response is sent by the /api/command handler (not here, since request is not in scope).
    // NOTIFIES Python program about calibration change via Serial1 (same as calibrate_pressure).
    //
    // Usage example:
    //   executeRemoteCommand("set_manual_adc_zero", "1200.00");
    else if (cmd == "set_manual_adc_zero") {
        float newAdcZero = value.toFloat();
        const float MANUAL_ADC_MIN = 500.0;   // Allow broader range for manual entry
        const float MANUAL_ADC_MAX = 2000.0;  // ADS1015 12-bit: 0-4095, pressure ~500-2000
        if (newAdcZero >= MANUAL_ADC_MIN && newAdcZero <= MANUAL_ADC_MAX) {
            adcZeroPressure = newAdcZero;
            savePressureCalibrationToEeprom();

            // Clear the pressure averaging buffer so old readings don't linger
            // Fill buffer with current pressure value to avoid temporary incorrect averages
            float currentPressure = adcPressure;
            pressureSampleCount = PRESSURE_AVG_SAMPLES;  // Mark buffer as full
            pressureBufferIdx = 0;
            for (int i = 0; i < PRESSURE_AVG_SAMPLES; i++) pressureBuffer[i] = currentPressure;

            Serial.printf("[CMD] Manual ADC zero set to: %.2f\r\n", adcZeroPressure);

            // Notify Python program about the calibration change (same format as calibrate_pressure)
            char calMsg[128];
            snprintf(calMsg, sizeof(calMsg),
                     "{\"type\":\"data\",\"ps_cal\":%.2f}",
                     adcZeroPressure);
            Serial1.println(calMsg);
            Serial.printf("[CMD] Sent manual ADC zero result to Linux: %s\r\n", calMsg);
        } else {
            Serial.printf("[CMD] ERROR: Manual ADC zero %.2f outside valid range (%.0f-%.0f)\r\n",
                         newAdcZero, MANUAL_ADC_MIN, MANUAL_ADC_MAX);
        }
    }
    // --- Unknown Command ---
    else {
        Serial.printf("[CMD] Unknown command: %s\n", cmd.c_str());
    }
}

// =====================================================================
// RELAY CONTROL FUNCTIONS (Rev 10 - controlled via serial mode field)
// =====================================================================

/**
 * Set relay outputs based on operating mode number.
 * Ported from Python control.py set_relays() function.
 * 
 * Mode 0 = IDLE:      All relays OFF
 * Mode 1 = RUN:       Motor + CR1 + CR5 ON, CR2 OFF
 * Mode 2 = PURGE:     Motor + CR2 ON, CR1 + CR5 OFF
 * Mode 3 = BURP:      CR5 only ON, all others OFF
 * Mode 8 = FRESH AIR: CR2 + CR5 ON, Motor + CR1 OFF
 * Mode 9 = LEAK TEST: CR1 + CR2 + CR5 ON, Motor OFF
 *
 * DISP_SHUTDN is managed separately - NOT affected by mode changes.
 * It stays HIGH unless explicitly set LOW via {"mode":"shutdown"} serial command.
 *
 * Usage example:
 *   setRelaysForMode(1);  // Set relays for RUN mode
 *   setRelaysForMode(0);  // Set relays for IDLE (all off)
 */
void setRelaysForMode(uint8_t modeNum) {
    if (relayMutex != NULL && xSemaphoreTake(relayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        // REV 10.16: Stagger each relay pin change by RELAY_DELAY ms.
        // Simultaneous switching of multiple solenoids causes combined inrush
        // current and back-EMF transients that couple noise into the ADS1015.
        // Staggering each pin change reduces the peak transient amplitude.
        // Total switching time per mode change: ~3 × RELAY_DELAY ms (3 gaps between 4 pins).
        switch (modeNum) {
            case 0:  // IDLE - All relay outputs OFF
                digitalWrite(CR0_MOTOR, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR2, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR5, LOW);
                break;
            case 1:  // RUN - Motor + CR1 + CR5 ON, CR2 OFF
                digitalWrite(CR0_MOTOR, HIGH);
                delay(RELAY_DELAY);
                digitalWrite(CR1, HIGH);
                delay(RELAY_DELAY);
                digitalWrite(CR2, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR5, HIGH);
                break;
            case 2:  // PURGE - Motor + CR2 ON, CR1 + CR5 OFF
                digitalWrite(CR0_MOTOR, HIGH);
                delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR2, HIGH);
                delay(RELAY_DELAY);
                digitalWrite(CR5, LOW);
                break;
            case 3:  // BURP - Only CR5 ON, everything else OFF
                digitalWrite(CR0_MOTOR, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR2, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR5, HIGH);
                break;
            case 8:  // FRESH AIR / SPECIAL BURP - CR2 + CR5 ON
                digitalWrite(CR0_MOTOR, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR2, HIGH);
                delay(RELAY_DELAY);
                digitalWrite(CR5, HIGH);
                break;
            case 9:  // LEAK TEST - CR1 + CR2 + CR5 ON (no motor)
                digitalWrite(CR0_MOTOR, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR1, HIGH);
                delay(RELAY_DELAY);
                digitalWrite(CR2, HIGH);
                delay(RELAY_DELAY);
                digitalWrite(CR5, HIGH);
                break;
            default:
                // Unknown mode - safe state (all off)
                digitalWrite(CR0_MOTOR, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR2, LOW);
                delay(RELAY_DELAY);
                digitalWrite(CR5, LOW);
                break;
        }
        
        currentRelayMode = modeNum;
        // REV 10.11: Test state is now managed by the auto-stop timer in loop()
        // and by the stop_test/stop_cycle web handlers. No auto-clear here.
        // Update current_mode enum for LED color display
        if (modeNum <= 3) {
            current_mode = (RunMode)modeNum;
        }
        xSemaphoreGive(relayMutex);
    } else {
        // Mutex unavailable - write directly (safety fallback)
        // REV 10.16: Stagger pin changes by RELAY_DELAY ms (same as mutex path)
        switch (modeNum) {
            case 0:  // IDLE
                digitalWrite(CR0_MOTOR, LOW);  delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);        delay(RELAY_DELAY);
                digitalWrite(CR2, LOW);        delay(RELAY_DELAY);
                digitalWrite(CR5, LOW);
                break;
            case 1:  // RUN
                digitalWrite(CR0_MOTOR, HIGH); delay(RELAY_DELAY);
                digitalWrite(CR1, HIGH);       delay(RELAY_DELAY);
                digitalWrite(CR2, LOW);        delay(RELAY_DELAY);
                digitalWrite(CR5, HIGH);
                break;
            case 2:  // PURGE
                digitalWrite(CR0_MOTOR, HIGH); delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);        delay(RELAY_DELAY);
                digitalWrite(CR2, HIGH);       delay(RELAY_DELAY);
                digitalWrite(CR5, LOW);
                break;
            case 3:  // BURP
                digitalWrite(CR0_MOTOR, LOW);  delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);        delay(RELAY_DELAY);
                digitalWrite(CR2, LOW);        delay(RELAY_DELAY);
                digitalWrite(CR5, HIGH);
                break;
            default:  // Safe state (all off)
                digitalWrite(CR0_MOTOR, LOW);  delay(RELAY_DELAY);
                digitalWrite(CR1, LOW);        delay(RELAY_DELAY);
                digitalWrite(CR2, LOW);        delay(RELAY_DELAY);
                digitalWrite(CR5, LOW);
                break;
        }
        currentRelayMode = modeNum;
        if (modeNum <= 3) current_mode = (RunMode)modeNum;
    }
    // DISP_SHUTDN is NOT changed here - managed separately via serial shutdown command
    // REV 10.11: Use hold-aware write to maintain latch through reboots/OTA
    writeDispShutdownPinWithHold(dispShutdownActive ? HIGH : LOW);
}

/**
 * Write DISP_SHUTDN pin with GPIO hold — latches state through reboots & OTA.
 * 
 * REV 10.11: Uses ESP-IDF gpio_hold_en/dis to latch DISP_SHUTDN (GPIO13) state
 * so it persists through software resets (ESP.restart(), OTA firmware updates).
 * Without hold, GPIO13 floats during the ~100-500ms between chip reset and setup(),
 * which can cause a momentary LOW and trigger an unwanted site shutdown.
 * 
 * Sequence: disable hold → write pin → re-enable hold
 * The internal pull-up (~45kΩ) provides additional protection as a backup.
 * 
 * Usage example:
 *   writeDispShutdownPinWithHold(HIGH);  // Set CR6 ON and latch it
 *   writeDispShutdownPinWithHold(LOW);   // Set CR6 OFF (shutdown) and latch it
 * 
 * @param state HIGH (site normal) or LOW (site shutdown / 72-hour alarm)
 */
void writeDispShutdownPinWithHold(uint8_t state) {
    gpio_hold_dis((gpio_num_t)DISP_SHUTDN);    // Release hold to allow pin change
    digitalWrite(DISP_SHUTDN, state);            // Write the new state
    gpio_hold_en((gpio_num_t)DISP_SHUTDN);      // Re-latch — persists through reboot/OTA
}

/**
 * Initialize DISP_SHUTDN pin with internal pull-up and GPIO hold.
 * 
 * REV 10.11: Called FIRST in setup, before any other GPIO configuration.
 * Ensures GPIO13 is driven HIGH and latched before anything else can glitch it.
 * 
 * The sequence matters:
 *   1. Release any hold from previous boot (so we CAN reconfigure)
 *   2. Set as OUTPUT and drive HIGH
 *   3. Enable internal pull-up (backup for the brief window during next reset)
 *   4. Disable pull-down (just in case)
 *   5. Enable GPIO hold — latches the HIGH state through software resets
 *   6. Enable deep sleep hold — latches ALL digital GPIOs through deep sleep
 * 
 * Three layers of protection (per ESP-IDF docs for ESP32-S3):
 *   - gpio_hold_en():          holds pin state through SOFTWARE resets (ESP.restart, OTA)
 *   - gpio_deep_sleep_hold_en(): holds ALL digital GPIOs through DEEP SLEEP
 *     (ESP-IDF docs: "For ESP32/S2/C3/S3/C2, gpio_hold_en alone cannot hold
 *      digital GPIO during Deep-sleep. Call gpio_deep_sleep_hold_en additionally.")
 *   - gpio_pullup_en():        internal ~45kΩ pull-up as backup for full power-on resets
 * 
 * GPIO13 = RTC_GPIO13 on ESP32-S3, so it supports both digital and RTC hold domains.
 * 
 * Ref: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html
 * 
 * Usage example:
 *   initializeDispShutdownPinWithHold();  // Call once at top of setup()
 */
void initializeDispShutdownPinWithHold() {
    // Step 1: Release hold from previous boot so we can reconfigure the pin
    // Per ESP-IDF: "configure gpio to a known state BEFORE calling gpio_hold_dis"
    // After software reset, the pin is still latched HIGH from previous hold.
    // Releasing the hold lets us reconfigure, but won't glitch since we immediately re-drive.
    gpio_hold_dis((gpio_num_t)DISP_SHUTDN);
    
    // Step 2: Configure as output and drive HIGH immediately
    pinMode(DISP_SHUTDN, OUTPUT);
    digitalWrite(DISP_SHUTDN, HIGH);
    
    // Step 3: Enable internal pull-up (~45kΩ) — backup for full power-on resets
    // The pull-up keeps the pin HIGH during the brief window before setup() runs
    // after a cold power-on (where hold state is lost)
    gpio_pullup_en((gpio_num_t)DISP_SHUTDN);
    gpio_pulldown_dis((gpio_num_t)DISP_SHUTDN);
    
    // Step 4: Enable GPIO hold — latches the current HIGH state
    // Per ESP-IDF: "state is latched at that moment and will not change when the
    // internal signal or the IO MUX/GPIO configuration is modified"
    // This survives software resets (ESP.restart, OTA firmware updates)
    gpio_hold_en((gpio_num_t)DISP_SHUTDN);
    
    // Step 5: Enable deep sleep hold for ALL digital GPIOs
    // Per ESP-IDF: "For ESP32/S2/C3/S3/C2, gpio_hold_en cannot hold digital GPIO
    // during Deep-sleep. Call gpio_deep_sleep_hold_en additionally."
    // This is a global enable — it won't affect pins that don't have gpio_hold_en set.
    // Belt-and-suspenders: covers deep sleep, light sleep, AND software resets.
    gpio_deep_sleep_hold_en();
    
    Serial.println("[GPIO-HOLD] DISP_SHUTDN (GPIO13/RTC_GPIO13) initialized HIGH");
    Serial.println("[GPIO-HOLD]   Internal pull-up: ENABLED (~45kΩ)");
    Serial.println("[GPIO-HOLD]   gpio_hold_en: ENABLED (survives software resets & OTA)");
    Serial.println("[GPIO-HOLD]   gpio_deep_sleep_hold_en: ENABLED (survives deep sleep)");
}

/**
 * Activate DISP_SHUTDN (site shutdown) - sets GPIO13 LOW.
 * Called ONLY when {"mode":"shutdown"} is received via serial.
 * This is the ONLY way to shut down the site in Rev 10.
 * REV 10.11: Uses GPIO hold to latch the LOW state through reboots.
 */
void activateDispShutdown() {
    dispShutdownActive = false;
    writeDispShutdownPinWithHold(LOW);
    Serial.println("[SHUTDOWN] DISP_SHUTDN set LOW (with hold) - site shutdown activated");
}

/**
 * Deactivate DISP_SHUTDN (restore site) - sets GPIO13 HIGH.
 * Called when a non-shutdown mode is received after a shutdown.
 * REV 10.11: Uses GPIO hold to latch the HIGH state through reboots.
 */
void deactivateDispShutdown() {
    dispShutdownActive = true;
    writeDispShutdownPinWithHold(HIGH);
    Serial.println("[SHUTDOWN] DISP_SHUTDN set HIGH (with hold) - site restored");
}

// =====================================================================
// OVERFILL INPUT VALIDATION (Rev 10 - direct GPIO38 read, no MCP emulation)
// =====================================================================

/**
 * Overfill Input Validation System
 * 
 * PROBLEM: GPIO38 can have transient false LOW readings during:
 *   - Power cycles (before pull-up stabilizes)
 *   - Firmware flashes (reset transitions)
 *   - Electrical noise on the line
 * 
 * SOLUTION: Multi-layer validation
 *   1. Startup lockout: Ignore readings for first 15 seconds after boot
 *   2. High threshold: Require 8 consecutive LOW readings (8 seconds) to trigger
 *   3. Hysteresis: Require 5 consecutive HIGH readings (5 seconds) to clear
 *   4. Validation flag: Track when monitoring is considered reliable
 * 
 * Overfill sensor is ACTIVE LOW: LOW = overfill detected, HIGH = normal
 * GPIO38 has internal pull-up enabled via INPUT_PULLUP.
 * 
 * Usage example:
 *   readOverfillSensor();  // Call every ~1 second from ADC task or main loop
 *   if (overfillAlarmActive) { ... stop relays ... }
 */
static uint8_t overfillHighCount = 0;  // Counter for HIGH readings to clear alarm

/**
 * Read overfill sensor from GPIO38 with validation and hysteresis.
 * Direct GPIO read - no MCP23017 emulation needed in Rev 10.
 * Called every ~1 second from the ADS1015 task on Core 0.
 * 
 * Sets overfillAlarmActive = true when overfill detected (8 consecutive LOW)
 * Sets overfillAlarmActive = false when cleared (5 consecutive HIGH)
 * 
 * Usage example:
 *   readOverfillSensor();  // Call periodically (~1 second)
 *   if (overfillAlarmActive) { setRelaysForMode(0); } // Emergency stop
 */
void readOverfillSensor() {
    unsigned long currentMillis = millis();
    
    // Initialize startup time on first call
    if (overfillStartupTime == 0) {
        overfillStartupTime = currentMillis;
    }
    
    // STARTUP LOCKOUT: Don't process readings for first 15 seconds
    // This allows GPIO38 and internal pull-up resistor to fully stabilize
    if (!overfillValidationComplete) {
        if (currentMillis - overfillStartupTime >= OVERFILL_STARTUP_LOCKOUT) {
            overfillValidationComplete = true;
            overfillLowCount = 0;
            overfillHighCount = 0;
            overfillAlarmActive = false;  // Start with alarm OFF after validation
        }
        return;  // Skip processing during lockout
    }
    
    // Normal operation: Check GPIO38 state every second for overfill alarm detection
    if (currentMillis - overfillCheckTimer >= 1000) {
        overfillCheckTimer = currentMillis;
        
        uint8_t gpio38State = digitalRead(ESP_OVERFILL);
        
        if (gpio38State == LOW) {
            // Sensor detecting overfill condition
            overfillLowCount++;
            overfillHighCount = 0;  // Reset HIGH counter
            
            // Require OVERFILL_TRIGGER_COUNT consecutive LOW readings to trigger alarm
            if (overfillLowCount >= OVERFILL_TRIGGER_COUNT && !overfillAlarmActive) {
                overfillAlarmActive = true;
            }
        } else {
            // Normal state (HIGH due to pull-up)
            overfillHighCount++;
            overfillLowCount = 0;  // Reset LOW counter
            
            // Require OVERFILL_CLEAR_COUNT consecutive HIGH readings to clear alarm (hysteresis)
            if (overfillHighCount >= OVERFILL_CLEAR_COUNT && overfillAlarmActive) {
                overfillAlarmActive = false;
            }
        }
        
        // Cap counters to prevent overflow
        if (overfillLowCount > 100) overfillLowCount = 100;
        if (overfillHighCount > 100) overfillHighCount = 100;
    }
}

// =====================================================================
// PROFILE MANAGER CLASS (Rev 10)
// =====================================================================
// Manages compressor profiles with per-profile alarm thresholds,
// fault code maps, and cycle parameters. Stored in flash as const arrays,
// active profile selection saved to EEPROM (Preferences).
//
// Profiles from Python control.py:
//   CS2  - North American
//   CS3  - International
//   CS8  - Mexican Non-GVR (full alarm set, 72-hour shutdown)
//   CS9  - Mexican GVR
//   CS12 - Franklin Mini-Jet (72-hour shutdown)
//   CS13 - Simplified
//
// Usage example:
//   profileManager.loadActiveProfile();           // Load from EEPROM in setup()
//   profileManager.setActiveProfile("CS8");       // Change via serial or web
//   ProfileConfig* p = profileManager.getActiveProfile();
//   if (p->hasLowPressAlarm && pressure < p->lowPressThreshold) { /* alarm */ }

struct ProfileConfig {
    const char* name;            // "CS8", "CS12", etc.
    const char* description;     // "Mexican Non-GVR"
    float pressureSetPoint;      // Auto-start threshold (IWC) for failsafe
    bool hasZeroPressAlarm;      // Zero-pressure alarm enabled
    bool hasLowPressAlarm;       // Low-pressure alarm enabled
    bool hasHighPressAlarm;      // High-pressure alarm enabled
    bool hasVarPressAlarm;       // Variable-pressure alarm enabled
    bool has72HourShutdown;      // 72-hour shutdown timer enabled
    bool hasPanelPowerAlarm;     // Panel power alarm enabled
    bool hasMotorCurrentAlarm;   // Motor current alarm enabled
    bool hasOverfillAlarm;       // Overfill alarm enabled
    // Alarm thresholds
    float zeroPressLow;          // Zero-press alarm lower bound (e.g., -0.15)
    float zeroPressHigh;         // Zero-press alarm upper bound (e.g., +0.15)
    float lowPressThreshold;     // Low-press alarm threshold (e.g., -6.0)
    float highPressThreshold;    // High-press alarm threshold (e.g., 2.0)
    float varPressRange;         // Variable-press alarm range (e.g., 0.20)
    float lowCurrentThreshold;   // Low motor current alarm (e.g., 3.0A)
    float highCurrentThreshold;  // High motor current alarm (e.g., 25.0A)
};

// Profile definitions (const, stored in flash)
const ProfileConfig PROFILES[] = {
    // CS2 - North American
    {"CS2", "North American", -3.0,
     false, false, false, false, false, true, true, true,
     -0.15, 0.15, -6.0, 2.0, 0.20, 3.0, 25.0},
    // CS3 - International
    {"CS3", "International", -3.0,
     false, false, false, false, false, true, true, true,
     -0.15, 0.15, -6.0, 2.0, 0.20, 3.0, 25.0},
    // CS8 - Mexican Non-GVR (full alarm set)
    {"CS8", "Mexican Non-GVR", -3.0,
     true, true, true, true, true, true, true, true,
     -0.15, 0.15, -6.0, 2.0, 0.20, 3.0, 25.0},
    // CS9 - Mexican GVR
    {"CS9", "Mexican GVR", -3.0,
     false, false, true, false, false, true, true, true,
     -0.15, 0.15, -6.0, 2.0, 0.20, 3.0, 25.0},
    // CS12 - Franklin Mini-Jet
    {"CS12", "Franklin Mini-Jet", -3.0,
     false, false, false, false, true, false, true, true,
     -0.15, 0.15, -6.0, 2.0, 0.20, 3.0, 25.0},
    // CS13 - Simplified
    {"CS13", "Simplified", -3.0,
     false, false, false, false, false, false, true, true,
     -0.15, 0.15, -6.0, 2.0, 0.20, 3.0, 25.0}
};
const int PROFILE_COUNT = 6;

class ProfileManager {
public:
    int activeProfileIndex;   // Index into PROFILES array (default 0 = CS2)
    
    ProfileManager() : activeProfileIndex(0) {}
    
    /**
     * Load active profile selection from EEPROM (Preferences).
     * Call once in setup() after preferences are available.
     */
    void loadActiveProfile() {
        Preferences prefs;
        prefs.begin("RMS", true);  // Read-only
        String savedProfile = prefs.getString("profile", "CS2");
        prefs.end();
        
        for (int i = 0; i < PROFILE_COUNT; i++) {
            if (String(PROFILES[i].name) == savedProfile) {
                activeProfileIndex = i;
                Serial.printf("[PROFILE] Loaded active profile from EEPROM: %s (%s)\r\n",
                              PROFILES[i].name, PROFILES[i].description);
                return;
            }
        }
        activeProfileIndex = 0;  // Default to CS2
        Serial.printf("[PROFILE] Unknown saved profile '%s', defaulting to CS2\r\n", savedProfile.c_str());
    }
    
    /**
     * Set the active profile by name and save to EEPROM.
     * Returns true if profile was found and set, false if name is unknown.
     * 
     * Usage example:
     *   profileManager.setActiveProfile("CS8");  // Switch to CS8
     */
    bool setActiveProfile(const char* profileName) {
        for (int i = 0; i < PROFILE_COUNT; i++) {
            if (strcmp(PROFILES[i].name, profileName) == 0) {
                activeProfileIndex = i;
                // Save to EEPROM
                Preferences prefs;
                prefs.begin("RMS", false);
                prefs.putString("profile", profileName);
                prefs.end();
                Serial.printf("[PROFILE] Active profile changed to: %s (%s)\r\n",
                              PROFILES[i].name, PROFILES[i].description);
                return true;
            }
        }
        Serial.printf("[PROFILE] Unknown profile name: %s\r\n", profileName);
        return false;
    }
    
    /**
     * Get pointer to the currently active profile configuration.
     */
    const ProfileConfig* getActiveProfile() const {
        return &PROFILES[activeProfileIndex];
    }
    
    /**
     * Get the active profile name as a String.
     */
    String getActiveProfileName() const {
        return String(PROFILES[activeProfileIndex].name);
    }
    
    /**
     * Get the active profile description as a String.
     */
    String getActiveProfileDescription() const {
        return String(PROFILES[activeProfileIndex].description);
    }
};

// Global profile manager instance
ProfileManager profileManager;

// =====================================================================
// ADS1015 ADC HELPER FUNCTIONS
// =====================================================================

/**
 * Map a value from one range to another (floating point version).
 * Ported from Python control.py map_value() function.
 * 
 * Note: Pressure now uses a direct linear formula (see adcReaderTask).
 * This function is still used for current ADC-to-amps conversion.
 * 
 * Usage example:
 *   float current = mapFloat(adcRms, 78.0, 290.0, 2.1, 8.0);
 */
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    if (in_max == in_min) return out_min;  // Avoid divide by zero
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Add a sample to the pressure rolling average buffer.
 * Returns the current rolling average.
 * 
 * Usage: Called from ADS1015 task at 60Hz.
 */
float addPressureSample(float sample) {
    // Simple 60-sample rolling average — no outlier rejection.
    //
    // REV 10.10: Removed all outlier rejection logic (soft/hard thresholds, buffer
    // resets). The outlier filter was unstable — it rejected legitimate ADC values
    // at boot and during normal operation on a fixed-pressure sensor with no
    // physical movement between samples. I2C communication failures are already
    // caught and discarded BEFORE this function is called (in adcReaderTask),
    // so bad reads never enter the rolling average.
    //
    // The rolling average itself provides sufficient noise smoothing:
    //   60 samples at 60Hz = 1 second window → smooths electrical noise naturally.
    
    // Add sample to circular buffer
    pressureBuffer[pressureBufferIdx] = sample;
    pressureBufferIdx = (pressureBufferIdx + 1) % PRESSURE_AVG_SAMPLES;
    if (pressureSampleCount < PRESSURE_AVG_SAMPLES) pressureSampleCount++;
    
    // Calculate average over available samples
    float sum = 0;
    int count = min(pressureSampleCount, PRESSURE_AVG_SAMPLES);
    for (int i = 0; i < count; i++) {
        sum += pressureBuffer[i];
    }
    return sum / count;
}

/**
 * Add a sample to the current windowed peak buffer.
 * Returns the peak current (maximum after removing single highest outlier).
 * 
 * Matches Python's approach: collect 60 samples (~1 second at 60Hz),
 * remove min and max outliers, return the maximum of the trimmed set.
 * This captures the true peak motor current for accurate alarm comparison.
 * 
 * Usage: Called from ADS1015 task at 60Hz.
 * 
 * Example:
 *   float peakCurrent = addCurrentSample(3.5);  // Returns windowed peak amps
 */
float addCurrentSample(float sample) {
    currentPeakBuffer[currentPeakIdx] = sample;
    currentPeakIdx = (currentPeakIdx + 1) % CURRENT_PEAK_SAMPLES;
    if (currentPeakCount < CURRENT_PEAK_SAMPLES) currentPeakCount++;
    
    int count = min(currentPeakCount, (int)CURRENT_PEAK_SAMPLES);
    if (count <= 2) {
        // Not enough samples for outlier trimming — return simple max
        float mx = 0;
        for (int i = 0; i < count; i++) {
            if (currentPeakBuffer[i] > mx) mx = currentPeakBuffer[i];
        }
        return mx;
    }
    
    // Find the highest and second-highest values in the buffer.
    // We discard the single highest (outlier) and return the second-highest (trimmed peak).
    // This matches Python's sorted→trimmed→max approach without a full sort.
    float maxVal = -1e9;
    float secondMax = -1e9;
    for (int i = 0; i < count; i++) {
        float v = currentPeakBuffer[i];
        if (v > maxVal) {
            secondMax = maxVal;
            maxVal = v;
        } else if (v > secondMax) {
            secondMax = v;
        }
    }
    return secondMax;  // Peak after removing single highest outlier
}

// =====================================================================
// PRESSURE SENSOR CALIBRATION (EEPROM-backed zero point)
// =====================================================================
// Mirrors Python pressure_sensor.py calibrate() and get_adc_zero().
// The zero point is the raw ADC reading that should correspond to 0.0 IWC.
// Stored in EEPROM (Preferences) under key "adcZero" in namespace "RMS".
// Loaded at boot via loadPressureCalibrationFromEeprom().
// Triggered via {"cal":1} serial command from Linux or web portal.

/**
 * Load the pressure sensor zero-point calibration from EEPROM.
 * Call once in setup() BEFORE starting the ADS1015 reader task.
 * If no calibration has been saved, uses the factory default (964.0).
 *
 * Usage example:
 *   loadPressureCalibrationFromEeprom();  // Call in setup()
 */
void loadPressureCalibrationFromEeprom() {
    Preferences prefs;
    prefs.begin("RMS", true);  // Read-only
    float savedZero = prefs.getFloat("adcZero", -1.0);  // -1 = not set
    prefs.end();
    
    // REV 10.9: Validate EEPROM value against the same middle-20% range used for calibration.
    // If a bad calibration was saved previously (e.g., sensor under vacuum), the stored value
    // will be out of range and we fall back to the factory default. This fixes devices that
    // already have a corrupt EEPROM value — they self-heal on the next reboot.
    if (savedZero >= CAL_RANGE_MIN && savedZero <= CAL_RANGE_MAX) {
        // Stored value is within the valid calibration range — use it
        adcZeroPressure = savedZero;
        Serial.printf("[CAL] Loaded pressure zero from EEPROM: %.2f (valid range: %.0f-%.0f)\r\n",
                      adcZeroPressure, CAL_RANGE_MIN, CAL_RANGE_MAX);
    } else if (savedZero > 0) {
        // Value exists but is out of range — bad calibration from a prior firmware version
        adcZeroPressure = ADC_ZERO_FACTORY_DEFAULT;
        Serial.printf("[CAL] WARNING: EEPROM value %.2f OUT OF RANGE (%.0f-%.0f) — using factory default %.2f\r\n",
                      savedZero, CAL_RANGE_MIN, CAL_RANGE_MAX, ADC_ZERO_FACTORY_DEFAULT);
        Serial.printf("[CAL] Previous calibration was likely done under pressure — re-calibrate at ambient air\r\n");
    } else {
        // No calibration saved — first boot or EEPROM cleared
        adcZeroPressure = ADC_ZERO_FACTORY_DEFAULT;
        Serial.printf("[CAL] No EEPROM calibration — using factory default: %.2f\r\n", adcZeroPressure);
    }
}

/**
 * Save the current pressure zero-point calibration to EEPROM.
 * Called after a successful calibration.
 *
 * Usage example:
 *   savePressureCalibrationToEeprom();  // After calibration completes
 */
void savePressureCalibrationToEeprom() {
    Preferences prefs;
    prefs.begin("RMS", false);  // Read-write
    prefs.putFloat("adcZero", adcZeroPressure);
    prefs.end();
    Serial.printf("[CAL] Saved pressure zero to EEPROM: %.2f\r\n", adcZeroPressure);
}

/**
 * Calibrate the pressure sensor zero point — INSTANT, NON-BLOCKING.
 * 
 * One-shot command: adjusts the ADC zero point so the current reading
 * becomes 0.0 IWC. Takes effect immediately — the very next sensor
 * packet will reflect the new calibration. No delay, no sample loop.
 * 
 * How it works:
 *   The ADC reader task (Core 0, 60Hz) already maintains a 60-sample
 *   rolling average in adcPressure. This function uses that averaged
 *   reading to compute the offset and shifts adcZeroPressure accordingly:
 *
 *     newZero = oldZero + (currentPressure_IWC × slope)
 *
 *   Example: sensor reads -0.5 IWC at altitude
 *     newZero = 964.0 + (-0.5 × 22.84) = 952.58
 *     Next reading: (rawADC - 952.58) / 22.84 = 0.0 IWC  ✓
 *
 * After adjusting:
 *   1. Clears the rolling average buffer (immediate fresh start)
 *   2. Saves the new zero to EEPROM (persists across reboots)
 *   3. Sends {"type":"data","ps_cal":952.58} to Linux via Serial1
 *      so the Python program can save it to its own database
 *   4. Regular sensor data packets immediately reflect the new zero
 *
 * Triggered by:
 *   - Serial: {"type":"cmd","cmd":"cal"}
 *   - Web portal: POST /api/command {"command":"calibrate_pressure"}
 *
 * Usage example:
 *   calibratePressureSensorZeroPoint();  // Instant — returns in <1ms
 */
void calibratePressureSensorZeroPoint() {
    if (!adcInitialized) {
        Serial.println("[CAL] ERROR: ADS1015 not initialized — cannot calibrate");
        return;
    }
    
    // Capture current state before adjusting
    float oldZero = adcZeroPressure;
    float oldPressure = adcPressure;  // 60-sample rolling average (already stable)
    
    // Shift the zero point so the current pressure reading becomes 0.0 IWC
    // Formula: pressure = (rawADC - zero) / slope
    // To make pressure = 0: zero must equal rawADC at current reading
    // Since rawADC = oldZero + (oldPressure × slope):
    float newZero = oldZero + (oldPressure * pressureSlope12bit);
    
    // REV 10.9: Strict range validation — middle 20% of scale (factory default ±10%).
    // Prevents calibrating under pressure: e.g., at -6 IWC → newZero ≈ 827 → REJECTED.
    // Old range (200-1800) was too wide and accepted nearly any value including bad calibrations.
    // New range (~868-1060) ensures the sensor must be near atmospheric (0.0 IWC) to calibrate.
    if (newZero < CAL_RANGE_MIN || newZero > CAL_RANGE_MAX) {
        Serial.printf("[CAL] REJECTED: new zero %.2f outside allowed range (%.0f-%.0f)\r\n",
                      newZero, CAL_RANGE_MIN, CAL_RANGE_MAX);
        Serial.printf("[CAL] Sensor may be under pressure — calibrate ONLY at ambient air\r\n");
        Serial.printf("[CAL] Current reading: %.2f IWC (should be near 0.0 for calibration)\r\n", oldPressure);
        return;
    }
    
    adcZeroPressure = newZero;
    
    Serial.printf("[CAL] Calibration complete (instant)\r\n");
    Serial.printf("[CAL]   Old zero: %.2f → New zero: %.2f (delta: %+.2f)\r\n",
                  oldZero, newZero, newZero - oldZero);
    Serial.printf("[CAL]   Was reading %.2f IWC — now calibrated to 0.0 IWC\r\n", oldPressure);
    
    // Clear the pressure averaging buffer so old readings don't linger
    pressureSampleCount = 0;
    pressureBufferIdx = 0;
    for (int i = 0; i < PRESSURE_AVG_SAMPLES; i++) pressureBuffer[i] = 0;
    
    // Save to EEPROM so calibration persists across reboots
    savePressureCalibrationToEeprom();
    
    // REV 10.18: Send unified calibration response to Linux via Serial1.
    // Python modem.py now handles both the new "command" format and the
    // legacy "ps_cal" format. The new format is preferred.
    char calMsg[128];
    snprintf(calMsg, sizeof(calMsg),
             "{\"command\":\"calibrate_pressure\",\"ps_cal\":%.2f}",
             adcZeroPressure);
    Serial1.println(calMsg);
    Serial.printf("[CAL] Sent calibration result to Linux: %s\r\n", calMsg);
}

// =====================================================================
// ADS1015 ADC + OVERFILL FREERTOS TASK (Core 0)
// =====================================================================
// Replaces the MCP23017 emulator task. Runs on Core 0 at 60Hz.
// Reads pressure (ch0 single-ended) and current (abs(ch2-ch3) differential).
// Also monitors the overfill sensor (GPIO38) every ~1 second.
//
// Error handling: If ADS1015 read returns -1 or times out, skip that sample
// and increment error counter. After 10 consecutive errors, attempt reinit.
// =====================================================================

/**
 * Initialize ADS1015 as I2C master with bus recovery.
 * Returns true if ADS1015 responded successfully.
 * 
 * Usage example:
 *   if (!initializeADS1015()) { Serial.println("ADS1015 not found!"); }
 */
bool initializeADS1015() {
    // Perform bus recovery first (clears stuck SDA)
    i2cBusRecovery();
    
    // Initialize I2C as MASTER (not slave like Rev 9)
    Wire.begin(SDA_PIN, SCL_PIN);
    // Wire.setClock(100000);  // 100kHz I2C clock
    Wire.setClock(400000);  // 400kHz I2C clock

    // Initialize ADS1015
    if (!ads.begin(ADS1015_ADDRESS, &Wire)) {
        return false;
    }
    
    // Set gain to +/-4.096V range (matches Python GAIN=1)
    ads.setGain(GAIN_ONE);
    
    // Set data rate to 3300 SPS (fastest for ADS1015) for 60Hz polling
    ads.setDataRate(RATE_ADS1015_3300SPS);
    
    adcInitialized = true;
    adcErrorCount = 0;
    lastSuccessfulAdcRead = millis();  // REV 10.10: Start freshness timer on init
    adcDataStale = false;
    return true;
}

/**
 * I2C Bus Recovery for ADS1015 master mode.
 * Clears stuck SDA line by toggling SCL up to 9 times.
 * Called before Wire.begin() or when bus errors detected.
 */
void i2cBusRecovery() {
    // Temporarily configure pins as GPIO
    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, OUTPUT);
    
    // Check if SDA is stuck LOW
    if (digitalRead(SDA_PIN) == LOW) {
        // Toggle SCL up to 9 times to free the bus
        for (int i = 0; i < 9; i++) {
            digitalWrite(SCL_PIN, LOW);
            delayMicroseconds(5);
            digitalWrite(SCL_PIN, HIGH);
            delayMicroseconds(5);
            if (digitalRead(SDA_PIN) == HIGH) break;
            }
        // Generate STOP condition
        pinMode(SDA_PIN, OUTPUT);
        digitalWrite(SDA_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(5);
        digitalWrite(SDA_PIN, HIGH);
        delayMicroseconds(5);
    }
    delayMicroseconds(100);
}

/**
 * ADS1015 ADC Reader + Overfill Monitor - FreeRTOS task running on Core 0.
 * 
 * Polls ADS1015 at ~60Hz (16ms interval):
 *   - Channel 0: Pressure sensor (single-ended)
 *     Formula: pressure_IWC = (raw - adcZeroPressure) / pressureSlope12bit
 *     Matches Python pressure_sensor.py two-point calibration.
 *     Vacuum = NEGATIVE IWC, atmospheric = 0, above atmospheric = POSITIVE.
 *     Output is 60-sample rolling average (~1 second window).
 *   - Channels 2 & 3: Current monitoring (hardware differential via readADC_Differential_2_3)
 *     Uses GAIN_TWO (±2.048V, 1mV/count) for 2x resolution on the current signal.
 *     Formula: current_A = mapFloat(abs(diff), 156, 580, 2.1, 8.0)
 *     Output is windowed peak (max after removing highest outlier, 60-sample window).
 *     Matches Python's max-after-trim approach for accurate alarm comparison.
 * 
 * Also reads GPIO38 overfill sensor every ~1 second.
 * 
 * Error recovery: After 10 consecutive read errors, attempts ADS1015 reinit.
 * 
 * Data is written to volatile globals readable by Core 1 (main loop, web).
 */
void adcReaderTask(void *parameter) {
    // STEP 1: Configure overfill input pin with internal pull-up
    pinMode(ESP_OVERFILL, INPUT_PULLUP);
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Let pull-up stabilize
    
    // STEP 2: Configure relay output pins to safe states
    pinMode(CR0_MOTOR, OUTPUT);
    pinMode(CR1, OUTPUT);
    pinMode(CR2, OUTPUT);
    pinMode(CR5, OUTPUT);
    
    // REV 10.16: Stagger boot-time relay init by RELAY_DELAY ms between each pin.
    // Same noise reduction strategy as setRelaysForMode() — prevents combined
    // inrush transients when all relay drivers initialize simultaneously.
    digitalWrite(CR0_MOTOR, LOW);     // Motor OFF
    delay(RELAY_DELAY);
    digitalWrite(CR1, LOW);           // Relay 1 OFF
    delay(RELAY_DELAY);
    digitalWrite(CR2, LOW);           // Relay 2 OFF
    delay(RELAY_DELAY);
    digitalWrite(CR5, LOW);           // Relay 5 OFF
    // REV 10.11: DISP_SHUTDN is initialized with hold in setup() via initializeDispShutdownPinWithHold()
    // Do NOT reconfigure it here — the hold is already active and the pin is HIGH
    
    // STEP 3: Initialize ADS1015 I2C master
    bool adsOk = initializeADS1015();
    if (!adsOk) {
        // ADS1015 not responding - keep trying in the loop
        adcInitialized = false;
        adcErrorCount = 10;  // Start at threshold to trigger reinit
    }
    
    // Initialize overfill state
    overfillAlarmActive = false;
    overfillValidationComplete = false;
    overfillStartupTime = 0;
    overfillLowCount = 0;
    overfillHighCount = 0;
    overfillCheckTimer = 0;
    
    unsigned long lastAdcRead = 0;
    const unsigned long ADC_POLL_INTERVAL = 16;  // ~60Hz (1000ms / 60 ≈ 16ms)
    
    // Main ADC reader loop
    while (true) {
        unsigned long now = millis();
        
        // =====================================================================
        // ADC READING: Poll ADS1015 at ~60Hz
        // =====================================================================
        if (now - lastAdcRead >= ADC_POLL_INTERVAL) {
            lastAdcRead = now;
            
            // REV 10.14: Skip ADC reads during SD card writes.
            // SD card SPI traffic causes ~32mV analog interference on the ADS1015,
            // biasing pressure readings by -0.71 IWC for the full write duration
            // (~500-800ms). While sdWriteActive is true, we hold the previous good
            // values in the rolling average instead of feeding in biased samples.
            // The flag is set/cleared by SaveToSD() on Core 1.
            //
            // Note: Relay switching noise is handled by the 1ms delay() inside
            // setRelaysForMode() itself, which lets the transient settle before
            // returning. No ADC gating needed — the delay provides sufficient margin.
            if (sdWriteActive) {
                vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield briefly, check again next cycle
                continue;
            }
            
            if (adcInitialized) {
                // Read Channel 0: Pressure sensor (single-ended)
                int16_t rawPressure = ads.readADC_SingleEnded(0);
                
                if (rawPressure >= 0 && rawPressure < 2048) {
                    // Success - convert to pressure IWC using Python-compatible linear formula
                    // pressure = (raw - adc_zero) / slope
                    // Vacuum (raw < adc_zero) → negative IWC (normal operating range)
                    // Above atmospheric (raw > adc_zero) → positive IWC
                    adcRawPressure = rawPressure;
                    float pressureIWC = ((float)rawPressure - adcZeroPressure) / pressureSlope12bit;
                    adcPressure = addPressureSample(pressureIWC);
                    adcErrorCount = 0;  // Reset consecutive error count
                    lastSuccessfulAdcRead = millis();  // REV 10.10: Mark data as fresh
                    if (adcDataStale) {
                        adcDataStale = false;
                        if (serialDebugMode) {
                            Serial.println("####################################################################");
                            Serial.println("## ADC DATA RECOVERED — fresh pressure data after stale period   ##");
                            Serial.println("####################################################################");
                        }
                    }
                } else {
                    adcErrorCount++;
                    adcTotalErrors++;
                    // Silent error handling - only log if debug mode enabled
                    if (serialDebugMode) {
                        Serial.println("####################################################################");
                        Serial.printf("## ADC PRESSURE READ ERROR  |  raw=%d  |  errors=%lu  |  @%lums\r\n",
                                      rawPressure, adcTotalErrors, millis());
                        Serial.println("## Single-ended CH0 should be 0-2047. Negative = I2C NACK/bus error");
                        Serial.printf("## Consecutive errors: %lu (reinit at 10)\r\n", adcErrorCount);
                        Serial.println("####################################################################");
                    }
                }
                
                // Read Channels 2 & 3: Current monitoring (hardware differential)
                // FIX: Previous code read ch2 and ch3 as two separate single-ended values
                // then subtracted them in software. On the 12-bit ADS1015 (2mV/count),
                // both channels returned nearly identical absolute values, making the
                // computed difference ≈ 0 and current always read 0.0A.
                //
                // Now: use the ADS1015's built-in differential mode at GAIN_TWO (1mV/count)
                // which measures the actual voltage difference (AIN2 - AIN3) in a single
                // conversion with full 12-bit precision on the difference itself.
                ads.setGain(GAIN_TWO);   // ±2.048V range, 1mV/count — max expected ~580mV
                int16_t rawDiff = ads.readADC_Differential_2_3();
                ads.setGain(GAIN_ONE);   // Restore to ±4.096V for next pressure read
                delayMicroseconds(100);  // REV 10.9: Allow PGA to settle after gain change (0.6% of 16ms cycle)
                
                int16_t absDiff = abs(rawDiff);

                // ---------------------------------------------------------------
                // CURRENT READ VALIDATION: Detect I2C bus errors or line noise
                // ADS1015 12-bit range: -2048 to +2047
                // Expected motor current range: ~0 to 580 counts differential
                // Values at ±2048 (full-scale rail) or >>800 suggest I2C corruption
                // rawDiff of exactly -1 is a common I2C NACK/failure return
                // ---------------------------------------------------------------
                bool currentReadValid = true;
                if (rawDiff == -1) {
                    // Common I2C failure signature — bus NACK or read timeout
                    currentReadValid = false;
                    adcTotalErrors++;
                    if (serialDebugMode) {
                        Serial.println("####################################################################");
                        Serial.println("## ADC CURRENT READ FAILURE — I2C NACK  (rawDiff == -1)          ##");
                        Serial.printf("## Differential CH2-CH3  |  raw=%d  |  total errors=%lu  |  @%lums\r\n",
                                      rawDiff, adcTotalErrors, millis());
                        Serial.println("## This usually means: I2C bus noise, SDA/SCL glitch, or ADS1015 ##");
                        Serial.println("## not responding. Check wiring, pull-ups, cable shielding.       ##");
                        Serial.println("####################################################################");
                    }
                } else if (rawDiff == -2048 || rawDiff == 2047) {
                    // Value pinned at absolute full-scale rail — likely clipping or bus error
                    currentReadValid = false;
                    adcTotalErrors++;
                    if (serialDebugMode) {
                        Serial.println("####################################################################");
                        Serial.println("## ADC CURRENT FULL-SCALE RAIL — POSSIBLE I2C BUS CORRUPTION     ##");
                        Serial.printf("## Differential CH2-CH3  |  raw=%d  |  total errors=%lu  |  @%lums\r\n",
                                      rawDiff, adcTotalErrors, millis());
                        Serial.println("## Value is pinned at 12-bit limit. Check for shorted/open input ##");
                        Serial.println("## or I2C line noise causing corrupted register reads.            ##");
                        Serial.println("####################################################################");
                    }
                } else if (absDiff > 800) {
                    // Abnormally high — well beyond expected max of ~580 counts at 8.0A
                    // Still use the value but WARN — could be transient noise spike
                    if (serialDebugMode) {
                        Serial.println("################################################################");
                        Serial.printf("## ADC CURRENT ABNORMAL  |  raw=%d  |  abs=%d  (expected <600)\r\n",
                                      rawDiff, absDiff);
                        Serial.printf("## Differential CH2-CH3  |  total errors=%lu  |  @%lums\r\n",
                                      adcTotalErrors, millis());
                        Serial.println("## Value is unusually high — possible noise spike on I2C bus    ##");
                        Serial.println("################################################################");
                    }
                    // NOTE: We still process this value but it will be visible in logs
                }

                if (currentReadValid) {
                    adcRawCurrent = absDiff;
                    float currentAmps = mapFloat((float)absDiff, adcZeroCurrent, adcFullCurrent, currentRangeLow, currentRangeHigh);
                    if (currentAmps < 0) currentAmps = 0;  // Clamp below-threshold to 0
                    adcCurrent = addCurrentSample(currentAmps);
                    lastSuccessfulAdcRead = millis();  // REV 10.10: Mark data as fresh
                    
                    // Track peak current for high-current detection
                    if (currentAmps > adcPeakCurrent) {
                        adcPeakCurrent = currentAmps;
                    }
                } else {
                    // Skip updating current value on bad read to avoid corrupting the rolling average
                    adcErrorCount++;  // Count toward reinit threshold
                }
                
                // Error recovery: After 10 consecutive errors, attempt reinit
                if (adcErrorCount >= 10) {
                    if (serialDebugMode) {
                        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                        Serial.println("!! ADC CRITICAL: 10 CONSECUTIVE ERRORS — REINITIALIZING I2C BUS !!");
                        Serial.printf("!!   Total errors since boot: %lu   |  @%lums\r\n", adcTotalErrors, millis());
                        Serial.println("!!   Stopping Wire, waiting 100ms, then re-initializing ADS1015  !!");
                        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    }
                    adcInitialized = false;
                    Wire.end();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    if (initializeADS1015()) {
                        adcErrorCount = 0;
                        if (serialDebugMode) {
                            Serial.println("####################################################################");
                            Serial.println("## ADC REINIT SUCCESS — ADS1015 responding again after I2C reset ##");
                            Serial.println("####################################################################");
                        }
                    } else {
                        if (serialDebugMode) {
                            Serial.println("####################################################################");
                            Serial.println("## ADC REINIT FAILED — ADS1015 STILL NOT RESPONDING             ##");
                            Serial.println("## Will retry in 1 second. Check I2C wiring & pull-ups!          ##");
                            Serial.println("####################################################################");
                        }
                        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait 1 second before retry
                    }
                }
            } else {
                // ADS1015 not initialized - try periodically
                static unsigned long lastReinitAttempt = 0;
                if (now - lastReinitAttempt >= 5000) {  // Try every 5 seconds
                    lastReinitAttempt = now;
                    Serial.println("####################################################################");
                    Serial.println("## ADC NOT INITIALIZED — Attempting ADS1015 reconnection...       ##");
                    Serial.printf("##   Attempt @%lums  |  Total errors: %lu\r\n", millis(), adcTotalErrors);
                    Serial.println("####################################################################");
                    if (initializeADS1015()) {
                        adcErrorCount = 0;
                        Serial.println("####################################################################");
                        Serial.println("## ADC RECONNECTED — ADS1015 now responding on I2C bus!          ##");
                        Serial.println("####################################################################");
                    } else {
                        Serial.println("####################################################################");
                        Serial.println("## ADC RECONNECT FAILED — Will retry in 5 seconds               ##");
                        Serial.println("####################################################################");
                    }
                }
            }
        }
        
        // =====================================================================
        // OVERFILL: Check GPIO38 every ~1 second (handled inside readOverfillSensor)
        // =====================================================================
        readOverfillSensor();
        
        // Minimum task delay (1ms) for FreeRTOS scheduling
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// =====================================================================
// FAILSAFE RELAY CONTROL (Rev 10)
// =====================================================================
// When the Comfile (Linux panel) is down (after 2 watchdog restart attempts),
// the ESP32 takes over autonomous relay control based on ADC pressure readings.
//
// Activation: watchdogRebootAttempts >= 2 AND no serial data
// Adds 8192 to fault code to indicate "panel down"
// Runs normal cycle sequences based on pressure thresholds
// Monitors current and overfill for safety cutoffs

/**
 * Check if the system should enter failsafe mode.
 * Called from the main loop when the watchdog detects extended serial loss.
 * Returns true if failsafe should be activated.
 * 
 * Conditions for failsafe:
 *   1. Watchdog is triggered (no serial data for 30+ min)
 *   2. At least 2 restart attempts have been made
 *   3. Not already in failsafe mode
 */
bool shouldEnterFailsafeMode() {
    return watchdogTriggered &&
           watchdogRebootAttempts >= MAX_WATCHDOG_ATTEMPTS_BEFORE_FAILSAFE &&
           failsafeEnabled &&  // Only enter failsafe if it's enabled
           !failsafeMode;
}

/**
 * Enter failsafe mode - ESP32 takes over relay control.
 * Sets failsafe flag, resets cycle state, and logs activation.
 */
void enterFailsafeMode() {
    failsafeMode = true;
    failsafeCycleRunning = false;
    failsafeCycleStep = 0;
    failsafeHighCurrentCount = 0;
    failsafeLowCurrentCount = 0;
    failsafeNoCommandTimeout = millis() + (30UL * 60UL * 1000UL);  // 30 min safety timeout
    
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║  ⚠️ FAILSAFE MODE ACTIVATED - ESP32 Controls Relays    ║");
    Serial.println("║  Comfile panel down after 2 restart attempts           ║");
    Serial.println("║  Fault code +8192 active                              ║");
    Serial.println("╚════════════════════════════════════════════════════════╝\n");
}

/**
 * Exit failsafe mode - return to serial-controlled operation.
 * Called when serial data resumes from the Comfile.
 */
void exitFailsafeMode() {
    if (!failsafeMode) return;
    
    failsafeMode = false;
    failsafeCycleRunning = false;
    failsafeCycleStep = 0;
    setRelaysForMode(0);  // Idle all relays
    
    Serial.println("[FAILSAFE] Exiting failsafe mode - serial data resumed");
}

/**
 * Start a failsafe run cycle (normal, purge, or clean).
 * Sets up the cycle sequence and starts the first step.
 * 
 * @param cycleType 0=normal, 1=manual purge, 2=clean canister
 */
void startFailsafeCycle(int cycleType) {
    switch (cycleType) {
        case 0:  // Normal run cycle
            activeCycleSequence = NORMAL_RUN_CYCLE;
            activeCycleLength = NORMAL_RUN_CYCLE_STEPS;
            break;
        case 1:  // Manual purge
            activeCycleSequence = MANUAL_PURGE_CYCLE;
            activeCycleLength = MANUAL_PURGE_CYCLE_STEPS;
            break;
        case 2:  // Clean canister
            activeCycleSequence = CLEAN_CANISTER_CYCLE;
            activeCycleLength = CLEAN_CANISTER_CYCLE_STEPS;
            break;
        default:
            return;
    }
    
    failsafeCycleRunning = true;
    failsafeCycleStep = 0;
    failsafeHighCurrentCount = 0;
    failsafeLowCurrentCount = 0;
    failsafeCycleStepTimer = millis();
    
    // Start first step
    setRelaysForMode(activeCycleSequence[0].mode);
    Serial.printf("[FAILSAFE] Starting cycle type %d, step 0: mode %d for %d sec\r\n",
                  cycleType, activeCycleSequence[0].mode, activeCycleSequence[0].durationSeconds);
}

/**
 * Stop the current failsafe cycle and idle all relays.
 * Called on alarm conditions, overfill, or timeout.
 */
void stopFailsafeCycle(const char* reason) {
    if (failsafeCycleRunning) {
        failsafeCycleRunning = false;
        failsafeCycleStep = 0;
        setRelaysForMode(0);
        Serial.printf("[FAILSAFE] Cycle stopped: %s\r\n", reason);
    }
}

/**
 * Check if the current failsafe cycle step is a purge-related mode.
 * Used by PPP to determine if we need to finish the current step.
 */
bool isCurrentStepPurgeMode() {
    if (!failsafeCycleRunning) return false;
    uint8_t currentStepMode = activeCycleSequence[failsafeCycleStep].mode;
    return (currentStepMode == 2 || currentStepMode == 3);  // PURGE or BURP
}

/**
 * Run one iteration of the failsafe cycle state machine.
 * Called from the main loop when failsafeMode is true.
 * 
 * Handles:
 *   - Cycle step progression (timer-based)
 *   - Auto-start based on pressure threshold
 *   - Low current alarm (3.0A for 9 consecutive readings)
 *   - High current alarm (25.0A for 2 seconds, max 4 events)
 *   - Overfill detection (immediate stop)
 *   - Safety timeout (no commands for 30 minutes)
 */
void runFailsafeCycleLogic() {
    if (!failsafeMode) return;
    
    const ProfileConfig* profile = profileManager.getActiveProfile();
    
    // Safety timeout check
    if (millis() > failsafeNoCommandTimeout) {
        stopFailsafeCycle("Safety timeout - no commands received");
        return;
    }
    
    // Overfill check - immediate stop
    if (overfillAlarmActive) {
        stopFailsafeCycle("Overfill alarm detected");
        return;
    }
    
    // Pressure sensor sanity check
    if (adcRawPressure < 1 || adcPressure < -40.0) {
        stopFailsafeCycle("Pressure sensor fault (ADC < 1 or pressure < -40 IWC)");
        return;
    }
    
    // If cycle is running, monitor current and advance steps
    if (failsafeCycleRunning) {
        uint8_t currentStepMode = activeCycleSequence[failsafeCycleStep].mode;
        
        // Current monitoring during motor-on modes (1=RUN, 2=PURGE)
        if (currentStepMode == 1 || currentStepMode == 2) {
            // Low current alarm: below threshold for consecutive readings
            if (adcCurrent <= profile->lowCurrentThreshold && adcCurrent > 0) {
                failsafeLowCurrentCount++;
                if (failsafeLowCurrentCount >= LOW_CURRENT_CONSECUTIVE) {
                    stopFailsafeCycle("Low current alarm - motor fault");
                    return;
                }
            } else {
                failsafeLowCurrentCount = 0;
            }
            
            // High current alarm: above threshold for 2 seconds
            if (adcCurrent >= profile->highCurrentThreshold) {
                failsafeHighCurrentCount++;
                if (failsafeHighCurrentCount >= MAX_HIGH_CURRENT_EVENTS) {
                    stopFailsafeCycle("High current alarm - exceeded max events");
                    return;
                }
                // Pause briefly (2 seconds), then resume
                setRelaysForMode(0);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                setRelaysForMode(currentStepMode);
            }
        }
        
        // Check if current step duration has elapsed
        unsigned long stepDurationMs = (unsigned long)activeCycleSequence[failsafeCycleStep].durationSeconds * 1000UL;
        if (millis() - failsafeCycleStepTimer >= stepDurationMs) {
            // Advance to next step
            failsafeCycleStep++;
            
            if (failsafeCycleStep >= activeCycleLength) {
                // Cycle complete
                failsafeCycleRunning = false;
                failsafeCycleStep = 0;
                failsafeLastCycleComplete = millis();
                setRelaysForMode(0);
                Serial.println("[FAILSAFE] Cycle completed successfully");
            } else {
                // Start next step
                setRelaysForMode(activeCycleSequence[failsafeCycleStep].mode);
                failsafeCycleStepTimer = millis();
                Serial.printf("[FAILSAFE] Step %d: mode %d for %d sec\r\n",
                              failsafeCycleStep,
                              activeCycleSequence[failsafeCycleStep].mode,
                              activeCycleSequence[failsafeCycleStep].durationSeconds);
            }
        }
    } else {
        // No cycle running - check auto-start condition
        // If pressure exceeds set point and no alarms, start a normal run cycle
        if (adcPressure >= profile->pressureSetPoint && !overfillAlarmActive && adcInitialized) {
            // Don't auto-start too frequently (wait 60 seconds between cycles)
            if (millis() - failsafeLastCycleComplete >= 60000) {
                Serial.printf("[FAILSAFE] Auto-start: pressure %.2f >= setpoint %.2f\r\n",
                              adcPressure, profile->pressureSetPoint);
                startFailsafeCycle(0);  // Normal run cycle
            }
        }
    }
}

/**
 * Get combined fault code from all fault sources.
 * Each fault code is a unique power-of-2 bit flag, so they can be OR'd together.
 * 
 * REV 10.17 fault code assignments:
 *   1024  = SD card failure
 *   2048  = Serial watchdog activated (no serial data for 30+ min)
 *   4096  = Failsafe mode (ESP32 autonomous relay control)
 *   8192  = BlueCherry connection failed (OTA unavailable)
 * 
 * Usage example:
 *   int faultCode = getCombinedFaultCode();  // e.g., 1024 + 2048 + 8192 = 11264
 */
int getCombinedFaultCode() {
    int code = 0;
    if (!isSDCardOK())       code += SD_CARD_FAULT_CODE;
    if (isInWatchdogState()) code += WATCHDOG_FAULT_CODE;
    if (failsafeEnabled)     code += FAILSAFE_FAULT_CODE;      // Failsafe is enabled (4096)
    if (failsafeMode)        code += FAILSAFE_ACTIVE_FAULT_CODE; // Failsafe is actively running (16384)
    if (!blueCherryConnected) code += BLUECHERRY_FAULT_CODE;
    return code;
}

// =====================================================================
// CBOR PASSTHROUGH BUFFER FUNCTIONS (Rev 10)
// =====================================================================
// During PPP passthrough, normal CBOR data transmission is suspended.
// These functions buffer sensor data to be transmitted after the session.

/**
 * Allocate the CBOR passthrough ring buffer.
 * Call once in setup(). Uses heap memory (~8.2KB for 120 minutes).
 */
void allocateCborPassthroughBuffer() {
    cborPassthroughBuffer = (CborBufferSample*)malloc(sizeof(CborBufferSample) * CBOR_BUFFER_MAX_SAMPLES);
    if (cborPassthroughBuffer == NULL) {
        Serial.println("[CBOR] WARNING: Failed to allocate passthrough buffer!");
    } else {
        Serial.printf("[CBOR] Passthrough buffer allocated: %d samples (%d bytes, ~%d min)\r\n",
                      CBOR_BUFFER_MAX_SAMPLES,
                      (int)(sizeof(CborBufferSample) * CBOR_BUFFER_MAX_SAMPLES),
                      CBOR_BUFFER_MAX_SAMPLES * 15 / 60);
    }
    cborBufferWriteIdx = 0;
    cborBufferCount = 0;
    cborBufferingActive = false;
}

/**
 * Add a sensor data sample to the passthrough buffer.
 * Called every 15 seconds during passthrough mode.
 * Wraps around if buffer is full (oldest samples overwritten).
 */
void addSampleToCborBuffer() {
    if (cborPassthroughBuffer == NULL) return;
    
    CborBufferSample sample;
    sample.timestamp = (uint32_t)currentTimestamp;
    sample.mode = currentRelayMode;
    sample.pressure = adcPressure;
    sample.current = adcCurrent;
    sample.fault = (uint16_t)(faults + getCombinedFaultCode());
    sample.cycles = (uint16_t)cycles;
    
    cborPassthroughBuffer[cborBufferWriteIdx] = sample;
    cborBufferWriteIdx = (cborBufferWriteIdx + 1) % CBOR_BUFFER_MAX_SAMPLES;
    if (cborBufferCount < CBOR_BUFFER_MAX_SAMPLES) cborBufferCount++;
}

/**
 * Start buffering CBOR data (called when entering passthrough).
 */
void startCborBuffering() {
    cborBufferingActive = true;
    cborBufferWriteIdx = 0;
    cborBufferCount = 0;
    lastCborBufferTime = millis();
    Serial.println("[CBOR] Passthrough buffering started");
}

/**
 * Stop buffering and flush all buffered samples via modem.
 * Called after PPP session ends and modem reconnects.
 * Encodes all buffered samples as a CBOR array-of-arrays and sends.
 */
void flushCborPassthroughBuffer() {
    cborBufferingActive = false;
    
    if (cborPassthroughBuffer == NULL || cborBufferCount == 0) {
        Serial.println("[CBOR] No buffered data to flush");
        return;
    }
    
    Serial.printf("[CBOR] Flushing %d buffered samples...\r\n", cborBufferCount);
    
    // Encode buffered data as CBOR array-of-arrays
    // Each sample: [timestamp, mode, pressure*100, current*100, fault, cycles]
    // Process in batches of 12 (same as normal CBOR transmission)
    int startIdx = (cborBufferWriteIdx - cborBufferCount + CBOR_BUFFER_MAX_SAMPLES) % CBOR_BUFFER_MAX_SAMPLES;
    
    int batchReadings[12][10];
    int batchCount = 0;
    
    for (int i = 0; i < cborBufferCount; i++) {
        int idx = (startIdx + i) % CBOR_BUFFER_MAX_SAMPLES;
        CborBufferSample* s = &cborPassthroughBuffer[idx];
        
        // Extract device ID from deviceName
        String idStr_local = String(deviceName);
        if (idStr_local.startsWith("CSX-")) idStr_local = idStr_local.substring(4);
        else if (idStr_local.startsWith("RND-")) idStr_local = idStr_local.substring(4);
        int id = idStr_local.toInt();
        
        batchReadings[batchCount][0] = id;
        batchReadings[batchCount][1] = (int)seq++;
        batchReadings[batchCount][2] = (int)(s->pressure * 100);
        batchReadings[batchCount][3] = s->cycles;
        batchReadings[batchCount][4] = s->fault;
        batchReadings[batchCount][5] = s->mode;
        batchReadings[batchCount][6] = 0;  // temp placeholder
        batchReadings[batchCount][7] = (int)(s->current * 100);
        batchCount++;
        
        // Send in batches of 12
        if (batchCount >= 12) {
            size_t cborSize = buildCborFromReadings(cborBuffer, sizeof(cborBuffer), batchReadings, batchCount);
            if (cborSize > 0) {
                sendCborArrayViaSocket(cborBuffer, cborSize);
            }
            batchCount = 0;
        }
    }
    
    // Send remaining samples
    if (batchCount > 0) {
        size_t cborSize = buildCborFromReadings(cborBuffer, sizeof(cborBuffer), batchReadings, batchCount);
        if (cborSize > 0) {
            sendCborArrayViaSocket(cborBuffer, cborSize);
        }
    }
    
    cborBufferCount = 0;
    cborBufferWriteIdx = 0;
    Serial.println("[CBOR] Passthrough buffer flushed successfully");
}

/**
 * Data Backfill Function - sends SD card data collected during passthrough mode
 * to Linux device after passthrough ends for cloud synchronization.
 *
 * Reads the current year's logfile, filters entries by timestamp to only include
 * data collected during the passthrough session, and sends as JSON array to Linux.
 *
 * Expected JSON format sent to Linux:
 * {"backfill":[{"timestamp":"2026-02-13 10:30:15","pressure":-14.22,"current":0.07,"mode":0,"fault":0,"cycles":484}]}
 */
void dataBackfill() {
    if (passthroughStartTime.length() == 0) {
        Serial.println("[BACKFILL] ERROR: No passthrough start time available");
        return;
    }

    Serial.printf("[BACKFILL] Starting backfill procedure for data after: %s\r\n", passthroughStartTime.c_str());

    // Get current year for logfile name
    time_t rawTime = (time_t)currentTimestamp;
    struct tm *timeInfo = gmtime(&rawTime);
    int currentYear = timeInfo->tm_year + 1900;
    String logFileName = "/logfile" + String(currentYear) + ".log";

    // Open the logfile for reading
    File logFile = SD.open(logFileName.c_str(), FILE_READ);
    if (!logFile) {
        Serial.printf("[BACKFILL] ERROR: Could not open logfile: %s\r\n", logFileName.c_str());
        return;
    }

    Serial.printf("[BACKFILL] Opened logfile: %s for backfill\r\n", logFileName.c_str());

    // Convert passthrough start time to timestamp for comparison
    unsigned long startTimestamp = 0;
    struct tm parseTime = {0};
    if (sscanf(passthroughStartTime.c_str(), "%d-%d-%d %d:%d:%d",
               &parseTime.tm_year, &parseTime.tm_mon, &parseTime.tm_mday,
               &parseTime.tm_hour, &parseTime.tm_min, &parseTime.tm_sec) == 6) {
        parseTime.tm_year -= 1900;  // tm_year is years since 1900
        parseTime.tm_mon -= 1;      // tm_mon is 0-based
        startTimestamp = mktime(&parseTime);
    }

    if (startTimestamp == 0) {
        Serial.println("[BACKFILL] ERROR: Could not parse passthrough start time");
        logFile.close();
        return;
    }

    Serial.printf("[BACKFILL] Filtering data with timestamp >= %lu\r\n", startTimestamp);

    // Read through the file and collect matching entries
    String jsonArray = "[";
    bool firstEntry = true;
    int backfillCount = 0;

    while (logFile.available()) {
        String line = logFile.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) continue;

        // Parse CSV line: timestamp,id,seq,pressure,cycles,fault,mode,temp,current
        int comma1 = line.indexOf(',');
        if (comma1 == -1) continue;

        String timestampStr = line.substring(0, comma1);
        unsigned long entryTimestamp = strtoul(timestampStr.c_str(), NULL, 10);

        // Only include entries with timestamp >= passthrough start time
        if (entryTimestamp < startTimestamp) continue;

        // Parse remaining fields
        int pos = comma1 + 1;
        int comma2 = line.indexOf(',', pos);
        if (comma2 == -1) continue;
        String idStr = line.substring(pos, comma2);

        pos = comma2 + 1;
        int comma3 = line.indexOf(',', pos);
        if (comma3 == -1) continue;
        String seqStr = line.substring(pos, comma3);

        pos = comma3 + 1;
        int comma4 = line.indexOf(',', pos);
        if (comma4 == -1) continue;
        String pressureStr = line.substring(pos, comma4);

        pos = comma4 + 1;
        int comma5 = line.indexOf(',', pos);
        if (comma5 == -1) continue;
        String cyclesStr = line.substring(pos, comma5);

        pos = comma5 + 1;
        int comma6 = line.indexOf(',', pos);
        if (comma6 == -1) continue;
        String faultStr = line.substring(pos, comma6);

        pos = comma6 + 1;
        int comma7 = line.indexOf(',', pos);
        if (comma7 == -1) continue;
        String modeStr = line.substring(pos, comma7);

        pos = comma7 + 1;
        int comma8 = line.indexOf(',', pos);
        if (comma8 == -1) continue;
        String tempStr = line.substring(pos, comma8);

        String currentStr = line.substring(comma8 + 1);

        // Format as JSON object (without timestamp as per requirements)
        if (!firstEntry) jsonArray += ",";
        firstEntry = false;

        jsonArray += "{";
        jsonArray += "\"pressure\":" + pressureStr + ",";
        jsonArray += "\"current\":" + currentStr + ",";
        jsonArray += "\"mode\":" + modeStr + ",";
        jsonArray += "\"fault\":" + faultStr + ",";
        jsonArray += "\"cycles\":" + cyclesStr;
        jsonArray += "}";

        backfillCount++;

        // Prevent JSON array from getting too large (limit to ~100 entries)
        if (backfillCount >= 100) {
            Serial.println("[BACKFILL] WARNING: Limiting backfill to 100 entries to prevent buffer overflow");
            break;
        }
    }

    logFile.close();

    jsonArray += "]";

    if (backfillCount == 0) {
        Serial.println("[BACKFILL] No data found for backfill period");
        return;
    }

    Serial.printf("[BACKFILL] Found %d entries for backfill\r\n", backfillCount);

    // Send the backfill data to Linux device
    String backfillMessage = "{\"backfill\":" + jsonArray + "}";
    Serial1.println(backfillMessage);
    Serial.printf("[BACKFILL] Sent backfill data to Linux: %d bytes\r\n", backfillMessage.length());

    Serial.println("[BACKFILL] Backfill procedure completed successfully");
}

// =====================================================================
// WEB INTERFACE
// =====================================================================
// WEB INTERFACE HTML/CSS/JAVASCRIPT
// =====================================================================
// Embedded directly in .ino file for iOS captive portal compatibility

// CRITICAL: In AsyncWebServer with a template processor, every literal % must be
// escaped as %% otherwise the processor treats text between % signs as template
// variables and DELETES chunks of HTML. This was the root cause of the blank page.
const char* control_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Walter IO Board - Rev 10.19</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        html { -webkit-text-size-adjust: 100%%; }
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; max-width: 600px; margin: 0 auto; padding: 0; background: #1565c0; color: #222; min-height: 100vh; }
        /* SPA Navigation & Layout */
        .topbar { background: #1a1a2e; color: #fff; padding: 8px 14px; display: flex; justify-content: space-between; align-items: center; position: sticky; top: 0; z-index: 200; }
        .topbar .title { font-size: 1em; font-weight: 700; }
        .topbar .info { font-size: 0.7em; color: #aaa; }
        .screen { display: none; padding: 12px; }
        .screen.active { display: block; }
        .nav-footer { position: fixed; bottom: 0; left: 0; right: 0; max-width: 600px; margin: 0 auto; background: #fff; border-top: 1px solid #ddd; display: flex; padding: 6px 10px; gap: 6px; z-index: 200; }
        .nav-footer button { flex: 1; padding: 10px 4px; border: none; border-radius: 8px; background: #e0e0e0; font-size: 0.82em; font-weight: 600; cursor: pointer; }
        .nav-footer button.active { background: #1565c0; color: #fff; }
        .hdr { text-align: center; padding: 10px 0; }
        .hdr h1 { font-size: 1.8em; color: #ffffff; }
        .hdr p { margin: 4px 0 0; color: #666; font-size: 0.82em; }
        .badge { position: fixed; top: 8px; right: 8px; padding: 4px 10px; border-radius: 12px; font-size: 11px; font-weight: bold; z-index: 100; }
        .badge.ok { background: #4CAF50; color: #fff; }
        .badge.err { background: #f44336; color: #fff; animation: pulse 1.5s infinite; }
        @keyframes pulse { 0%%,100%%{opacity:1} 50%%{opacity:0.6} }
        .card { background: #fff; border-radius: 10px; box-shadow: 0 1px 4px rgba(0,0,0,0.08); margin-bottom: 12px; overflow: hidden; }
        .card-title { font-size: 0.88em; font-weight: 700; color: #fff; padding: 8px 14px; }
        .card-body { padding: 10px 14px; }
        .bg-blue { background: #1565c0; }
        .bg-green { background: #2e7d32; }
        .bg-gray { background: #455a64; }
        .bg-orange { background: #e65100; }
        .row { display: flex; justify-content: space-between; align-items: center; padding: 6px 0; border-bottom: 1px solid #f5f5f5; }
        .row:last-child { border-bottom: none; }
        .lbl { color: #777; font-size: 0.84em; flex-shrink: 0; }
        .val { font-weight: 600; font-size: 0.92em; color: #222; text-align: right; }
        .ok-text { color: #2e7d32; }
        .warn-text { color: #e65100; }
        .err-text { color: #c62828; font-weight: bold; }
        /* New simplified main screen styles */
        .status-card { background: #4CAF50; border-radius: 12px; margin: 20px; padding: 20px; box-shadow: 0 4px 8px rgba(0,0,0,0.2); transition: background-color 0.3s; }
        .status-card.alarm { background: #f44336; }
        .status-content { text-align: center; color: #fff; }
        .status-mode { font-size: 1.5em; font-weight: 700; margin-bottom: 8px; }
        .status-alarms { font-size: 1em; opacity: 0.9; }
        .readings-container { text-align: center; margin: 40px 20px; }
        .pressure-display { font-size: 2em; font-weight: 700; color: #fff; margin-bottom: 20px; text-shadow: 0 2px 4px rgba(0,0,0,0.3); }
        .cycles-display { font-size: 1.5em; font-weight: 600; color: #fff; opacity: 0.9; text-shadow: 0 2px 4px rgba(0,0,0,0.3); }
        .signal-wrap { display: inline-flex; align-items: center; gap: 6px; }
        .signal-bar { display: inline-block; width: 60px; height: 10px; background: #e0e0e0; border-radius: 5px; overflow: hidden; }
        .signal-fill { height: 100%%; border-radius: 5px; transition: width 0.5s; }
        .cfg-row { display: flex; align-items: center; gap: 8px; padding: 8px 0; flex-wrap: wrap; }
        .cfg-row input[type=text] { padding: 6px 8px; border: 1px solid #ccc; border-radius: 4px; font-size: 14px; width: 140px; }
        .btn { padding: 6px 14px; border: none; border-radius: 4px; cursor: pointer; font-weight: 600; font-size: 0.85em; color: #fff; }
        .btn-save { background: #4CAF50; }
        .btn-save:active { background: #388E3C; }
        .btn-pt { background: #e65100; }
        .btn-pt:active { background: #bf360c; }
        .hint { font-size: 0.72em; color: #999; padding: 2px 0 6px; }
        .ts { text-align: center; font-size: 0.7em; color: #aaa; padding: 8px; }
        .modal-bg { display: none; position: fixed; top: 0; left: 0; width: 100%%; height: 100%%; background: rgba(0,0,0,0.5); z-index: 200; justify-content: center; align-items: center; }
        .modal { background: #fff; border-radius: 8px; max-width: 90%%; width: 360px; overflow: hidden; }
        .modal-hdr { background: #e65100; color: #fff; padding: 12px 16px; font-weight: 700; }
        .modal-body { padding: 16px; color: #333; font-size: 0.9em; line-height: 1.5; }
        .modal-bg { display: none; position: fixed; top: 0; left: 0; right: 0; bottom: 0; background: rgba(0,0,0,0.5); z-index: 300; align-items: center; justify-content: center; }
        .modal { background: #fff; border-radius: 12px; width: 90%%; max-width: 400px; overflow: hidden; }
        .modal-hdr { background: #c62828; color: #fff; padding: 12px 16px; font-weight: 700; }
        .modal-body { padding: 16px; font-size: 0.9em; }
        .modal-body ul { margin: 4px 0 10px; padding-left: 18px; }
        .modal-foot { padding: 10px 16px; background: #f5f5f5; display: flex; justify-content: flex-end; gap: 8px; }
        .btn-cancel { background: #9e9e9e; }
        .btn-danger { background: #c62828; }
        /* Indicators */
        .ind { display: inline-block; width: 14px; height: 14px; border-radius: 50%%; margin-right: 6px; vertical-align: middle; }
        .ind-r { background: #f44336; } .ind-g { background: #4CAF50; } .ind-y { background: #FF9800; }
        /* Grid layouts for screens */
        .menu-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; padding: 8px 0; }
        .menu-btn { display: flex; flex-direction: column; align-items: center; justify-content: center; padding: 18px 8px; border: none; border-radius: 10px; background: #e8eaf6; font-size: 0.85em; font-weight: 600; color: #1a1a2e; cursor: pointer; min-height: 70px; }
        .menu-btn:active { background: #c5cae9; }
        .profile-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; }
        .profile-btn { padding: 14px 8px; border: 2px solid #ccc; border-radius: 8px; background: #fff; font-size: 0.9em; font-weight: 700; cursor: pointer; text-align: center; }
        .profile-btn.active-profile { border-color: #1565c0; background: #e3f2fd; color: #1565c0; }
        .alarm-row { display: flex; align-items: center; padding: 6px 0; border-bottom: 1px solid #f0f0f0; }
        .alarm-row .name { flex: 1; font-size: 0.85em; }
        .keypad { display: grid; grid-template-columns: repeat(3,1fr); gap: 6px; max-width: 280px; margin: 10px auto; }
        .keypad button { padding: 14px; border: 1px solid #ccc; border-radius: 8px; background: #fff; font-size: 1.1em; font-weight: 600; cursor: pointer; }
        .keypad button:active { background: #e0e0e0; }
        .key-wide { grid-column: span 2; }
        .countdown { font-size: 2em; font-weight: 700; text-align: center; padding: 20px; color: #c62828; }
        .test-status { font-size: 1.1em; text-align: center; padding: 10px; font-weight: 600; }
        /* Maintenance footer button */
        .maint-footer { position: fixed; bottom: 0; left: 0; right: 0; max-width: 600px; margin: 0 auto; background: rgba(255,255,255,0.9); border-top: 2px solid #1565c0; padding: 15px; text-align: center; }
        .maint-btn { padding: 12px 30px; background: #4CAF50; color: #fff; border: none; border-radius: 8px; font-size: 1em; font-weight: 700; cursor: pointer; }
        .maint-btn:active { background: #388E3C; }
        .maint-btn:active { background: #0d47a1; }
        /* Padding at bottom for fixed footer */
        .content-wrap { padding-bottom: 80px; }
    </style>
</head>
<body>
    <!-- Top bar with status -->
    <div class="topbar">
        <div><span class="title">Walter IO Board</span><br><span class="info" id="topVer">Rev 10.19</span></div>
        <div style="text-align:right"><span class="info" id="topTime">--</span><br><span class="badge ok" id="connBadge" style="position:static;font-size:10px">Connected</span></div>
    </div>
    <div class="content-wrap">

    <!-- ============ MAIN SCREEN ============ -->
    <div class="screen active" id="scr-main">
        <!-- Status Card - Red if any alarms, Green if OK -->
        <div class="status-card" id="statusCard">
            <div class="status-content">
                <div class="status-mode" id="statusMode">IDLE</div>
                <div class="status-alarms" id="statusAlarms">No Alarms</div>
            </div>
        </div>

        <!-- Pressure and Cycles Display -->
        <div class="readings-container">
            <div class="pressure-display" id="pressureDisplay">UST PRESSURE: --.--</div>
            <div class="cycles-display" id="cyclesDisplay">RUN CYCLES: ------</div>
        </div>

        <!-- Emergency Stop Button -->
        <div style="text-align:center;margin:20px 0">
            <button class="btn" style="padding:15px 30px;background:#c62828;color:#fff;border:none;border-radius:8px;font-weight:700;font-size:1.1em;text-transform:uppercase;animation:pulse 2s infinite" onclick="sendCmd('emergency_stop')">EMERGENCY STOP</button>
        </div>
    </div><!-- end scr-main -->

    <!-- ============ ALARMS SCREEN ============ -->
    <div class="screen" id="scr-alarms">
        <div class="hdr"><h1>Alarm Status</h1><p id="alarmProfile">--</p></div>
        <div class="card"><div class="card-title bg-blue">Active Alarms</div><div class="card-body" id="alarmGrid">
            <div class="alarm-row"><span class="name">Overfill</span><span class="ind ind-g" id="aOvfl"></span></div>
            <div class="alarm-row"><span class="name">Low Pressure</span><span class="ind ind-g" id="aLowP"></span></div>
            <div class="alarm-row"><span class="name">High Pressure</span><span class="ind ind-g" id="aHighP"></span></div>
            <div class="alarm-row"><span class="name">Zero Pressure</span><span class="ind ind-g" id="aZeroP"></span></div>
            <div class="alarm-row"><span class="name">Var Pressure</span><span class="ind ind-g" id="aVarP"></span></div>
            <div class="alarm-row"><span class="name">Low Current</span><span class="ind ind-g" id="aLowC"></span></div>
            <div class="alarm-row"><span class="name">High Current</span><span class="ind ind-g" id="aHighC"></span></div>
            <div class="alarm-row"><span class="name">Panel Power</span><span class="ind ind-g" id="aPanelPwr"></span></div>
            <div class="alarm-row"><span class="name">72-Hour Timer</span><span class="ind ind-g" id="a72Hr"></span></div>
            <div class="alarm-row"><span class="name">Watchdog</span><span class="ind ind-g" id="aWatch"></span></div>
            <div class="alarm-row"><span class="name">Failsafe Mode</span><span class="ind ind-g" id="aFailsafe"></span></div>
        </div></div>
        <div class="card"><div class="card-title bg-gray">Fault Code</div><div class="card-body">
            <div class="row"><span class="lbl">Combined Code</span><span class="val" id="alarmFault">0</span></div>
        </div></div>
    </div><!-- end scr-alarms -->

    <!-- ============ MAINTENANCE PASSWORD GATE ============ -->
    <!-- User must enter PW 878 before seeing maintenance options.   -->
    <!-- Once unlocked, maintUnlocked stays true for the session.    -->
    <div class="screen" id="scr-maint">
        <div class="hdr"><h1>Maintenance</h1></div>
        <!-- Password gate (shown when locked) -->
        <div id="maintLock" style="text-align:center;padding:30px 10px;color:#fff">
            <p style="font-size:0.9em;margin-bottom:12px">Enter maintenance password to continue</p>
            <input type="password" id="maintPwd" maxlength="10" placeholder="Password" style="padding:10px 14px;border:1px solid #ccc;border-radius:6px;font-size:1em;width:140px;text-align:center" onkeypress="if(event.key==='Enter')checkMaintPw()">
            <div style="margin-top:12px">
                <button class="btn" style="padding:10px 28px;background:#4CAF50;color:#fff;border:none;border-radius:6px;font-weight:700" onclick="checkMaintPw()">Enter Maintenance</button>
            </div>
            <p id="maintPwMsg" style="color:#fff;font-size:0.85em;margin-top:8px"></p>
        </div>
        <!-- Maintenance menu (shown when unlocked) -->
        <div id="maintMenu" style="display:none">
            <!-- Emergency Stop Button -->
            <div style="text-align:center;margin:15px 0">
                <button class="btn" style="padding:15px 30px;background:#c62828;color:#fff;border:none;border-radius:8px;font-weight:700;font-size:1.1em;text-transform:uppercase;animation:pulse 2s infinite" onclick="sendCmd('emergency_stop')">EMERGENCY STOP</button>
            </div>
            <div class="menu-grid">
                <button class="menu-btn" onclick="sendCmd('clear_press_alarm')">Clear Press Alarm</button>
                <button class="menu-btn" onclick="sendCmd('clear_motor_alarm')">Clear Motor Alarm</button>
                <button class="menu-btn" onclick="nav('tests')">Run Tests</button>
                <button class="menu-btn" onclick="nav('diagnostics')">Diagnostics</button>
                <button class="menu-btn" onclick="nav('clean')">Clean Canister</button>
                <button class="menu-btn" onclick="showPtModal()">Passthrough</button>
                <button class="menu-btn" onclick="nav('config')" style="background:#1565c0;color:#fff">Configuration</button>
                <button class="menu-btn" onclick="calibratePressure()" style="background:#00695c;color:#fff">Calibrate Pressure</button>
                <button class="menu-btn" onclick="nav('manual-adc-zero')" style="background:#c62828;color:#fff">Manual ADC Zero</button>
            </div>
            <p id="calResult" style="text-align:center;font-size:0.85em;color:#00695c;margin-top:8px"></p>
        </div>
    </div><!-- end scr-maint -->

    <!-- ============ MANUAL ADC ZERO SCREEN ============ -->
    <div class="screen" id="scr-manual-adc-zero">
        <div class="hdr"><h1>Manual ADC Zero</h1></div>
        <div class="card"><div class="card-body">
            <p style="color:#e65100;font-weight:700;text-align:center;padding:8px">Enter a custom ADC zero value to manually set the pressure sensor calibration.</p>
            <div style="text-align:center;padding:20px 10px">
                <input type="number" id="adcZeroInput" step="0.01" placeholder="Enter ADC zero value" style="padding:12px 16px;border:2px solid #ddd;border-radius:8px;font-size:1.1em;width:200px;text-align:center" />
                <br><br>
                <button class="btn" style="padding:12px 32px;background:#2e7d32;color:#fff;border:none;border-radius:8px;font-weight:700;font-size:1.1em" onclick="submitManualAdcZero()">Submit</button>
                <button class="btn" style="padding:12px 32px;background:#666;color:#fff;border:none;border-radius:8px;font-weight:700;font-size:1.1em;margin-left:10px" onclick="nav('maint')">Cancel</button>
            </div>
            <p id="adcZeroResult" style="text-align:center;font-size:0.9em;color:#00695c;margin-top:8px"></p>
        </div></div>
    </div><!-- end scr-manual-adc-zero -->

    <!-- ============ TESTS SCREEN ============ -->
    <div class="screen" id="scr-tests">
        <div class="hdr"><h1>Tests</h1></div>
        <div class="card"><div class="card-body">
            <div class="row"><span class="lbl">Pressure</span><span class="val" id="testPressure">--</span></div>
        </div></div>
        <div class="menu-grid">
            <button class="menu-btn" onclick="nav('leak')">Leak Test</button>
            <button class="menu-btn" onclick="nav('func')">Functionality Test</button>
            <button class="menu-btn" onclick="nav('eff')">Efficiency Test</button>
        </div>
        <!-- Individual Relay Control -->
        <div class="card"><div class="card-title bg-orange">Relay Control</div><div class="card-body">
            <div style="display:flex;gap:8px;margin:10px 0">
                <button class="menu-btn" id="relayCR1" onclick="toggleRelay('CR1')" style="flex:1;background:#e8eaf6">CR1 OFF</button>
                <button class="menu-btn" id="relayCR2" onclick="toggleRelay('CR2')" style="flex:1;background:#e8eaf6">CR2 OFF</button>
                <button class="menu-btn" id="relayCR5" onclick="toggleRelay('CR5')" style="flex:1;background:#e8eaf6">CR5 OFF</button>
            </div>
        </div></div>
    </div>

    <!-- ============ LEAK TEST SCREEN ============ -->
    <div class="screen" id="scr-leak">
        <div class="hdr"><h1>Leak Test</h1></div>
        <div class="card"><div class="card-body">
            <div class="countdown" id="leakTimer">--:--</div>
            <div class="test-status" id="leakStatus">Ready</div>
        </div></div>
        <div style="display:flex;gap:8px;margin:10px 0">
            <button class="btn" style="flex:1;padding:12px;background:#2e7d32;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('start_test','leak')">Start</button>
            <button class="btn" style="flex:1;padding:12px;background:#c62828;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('stop_test')">Stop</button>
        </div>
    </div>

    <!-- ============ FUNCTIONALITY TEST SCREEN ============ -->
    <!-- 10x (Run 60s → Purge 60s) = 20 steps, 20 minutes total -->
    <div class="screen" id="scr-func">
        <div class="hdr"><h1>Functionality Test</h1></div>
        <div class="card"><div class="card-body">
            <div class="countdown" id="funcTimer">--</div>
            <div class="test-status" id="funcStatus">Ready</div>
            <div class="row" style="margin-top:8px"><span class="lbl">Cycle</span><span class="val" id="funcCycle">--</span></div>
            <div class="row"><span class="lbl">Phase</span><span class="val" id="funcPhase">--</span></div>
        </div></div>
        <div style="display:flex;gap:8px;margin:10px 0">
            <button class="btn" style="flex:1;padding:12px;background:#2e7d32;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('start_test','func')">Start</button>
            <button class="btn" style="flex:1;padding:12px;background:#c62828;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('stop_test')">Stop</button>
            </div>
            </div>

    <!-- ============ EFFICIENCY TEST SCREEN ============ -->
    <div class="screen" id="scr-eff">
        <div class="hdr"><h1>Efficiency Test</h1></div>
        <div class="card"><div class="card-body">
            <div class="row"><span class="lbl">Step</span><span class="val" id="effStep">--</span></div>
            <div class="row"><span class="lbl">Mode</span><span class="val" id="effMode">--</span></div>
            <div class="row"><span class="lbl">Time</span><span class="val" id="effTime">--</span></div>
        </div></div>
        <div style="display:flex;gap:8px;margin:10px 0">
            <button class="btn" style="flex:1;padding:12px;background:#2e7d32;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('start_test','eff')">Start</button>
            <button class="btn" style="flex:1;padding:12px;background:#c62828;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('stop_test')">Stop</button>
                </div>
            </div>

    <!-- ============ CLEAN CANISTER SCREEN ============ -->
    <div class="screen" id="scr-clean">
        <div class="hdr"><h1>Clean Canister</h1></div>
        <div class="card"><div class="card-body">
            <p style="color:#c62828;font-weight:700;text-align:center;padding:8px">WARNING: This runs the motor for 15 minutes.<br>Asegurese de que el recipiente este vacio.</p>
            <div class="countdown" id="cleanTimer">15:00</div>
            <div class="test-status" id="cleanStatus">Ready</div>
        </div></div>
        <div style="display:flex;gap:8px;margin:10px 0">
            <button class="btn" style="flex:1;padding:12px;background:#e65100;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('start_cycle','clean')">Start Clean</button>
            <button class="btn" style="flex:1;padding:12px;background:#c62828;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('stop_cycle')">Stop</button>
        </div>
    </div>

    <!-- ============ STARTUP/SETTINGS SCREEN ============ -->
    <div class="screen" id="scr-settings">
        <div class="hdr"><h1>Settings</h1></div>
        <div class="menu-grid">
            <button class="menu-btn" onclick="nav('manual')">Manual Mode</button>
            <button class="menu-btn" onclick="nav('profiles')">Profiles</button>
            <button class="menu-btn" onclick="nav('overfill')">Overfill Override</button>
            <button class="menu-btn" onclick="nav('about')">About</button>
            <button class="menu-btn" onclick="nav('config')">Configuration</button>
            <button class="menu-btn" onclick="sendCmd('restart')">Reboot ESP32</button>
        </div>
    </div>

    <!-- ============ MANUAL MODE SCREEN ============ -->
    <div class="screen" id="scr-manual">
        <div class="hdr"><h1>Manual Mode</h1></div>
        <div class="card"><div class="card-body">
            <div class="row"><span class="lbl">Pressure</span><span class="val" id="manPressure">--</span></div>
            <div class="row"><span class="lbl">Current</span><span class="val" id="manCurrent">--</span></div>
            <div class="row"><span class="lbl">Run Cycles</span><span class="val" id="manCycles">0</span></div>
            <div class="row"><span class="lbl">Mode</span><span class="val" id="manMode">Idle</span></div>
        </div></div>
        <div style="display:flex;gap:8px;margin:10px 0">
            <button class="btn" style="flex:1;padding:12px;background:#2e7d32;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('start_cycle','manual_purge')">Manual Purge</button>
            <button class="btn" style="flex:1;padding:12px;background:#c62828;color:#fff;border:none;border-radius:8px;font-weight:700" onclick="sendCmd('stop_cycle')">Stop</button>
        </div>
    </div>

    <!-- ============ PROFILES SCREEN ============ -->
    <!-- Select a profile, then confirm with password 1793 before applying. -->
    <!-- Buttons highlight on select; "Confirm" sends the change.           -->
    <div class="screen" id="scr-profiles">
        <div class="hdr"><h1>Select Profile</h1><p>Current: <span id="profCurrent">--</span></p></div>
        <div class="profile-grid" id="profGrid">
            <button class="profile-btn" onclick="selectProfile('CS2',this)">CS2<br><small>N. American</small></button>
            <button class="profile-btn" onclick="selectProfile('CS3',this)">CS3<br><small>International</small></button>
            <button class="profile-btn" onclick="selectProfile('CS8',this)">CS8<br><small>Mex Non-GVR</small></button>
            <button class="profile-btn" onclick="selectProfile('CS9',this)">CS9<br><small>Mex GVR</small></button>
            <button class="profile-btn" onclick="selectProfile('CS12',this)">CS12<br><small>Mini-Jet</small></button>
            <button class="profile-btn" onclick="selectProfile('CS13',this)">CS13<br><small>Simplified</small></button>
        </div>
        <!-- Confirm section: password + confirm button (hidden until profile selected) -->
        <div id="profConfirm" style="display:none;text-align:center;padding:14px 10px;margin-top:10px;background:#f5f5f5;border-radius:8px">
            <p style="margin:0 0 8px;font-weight:700;color:#333">Change to: <span id="profSelected" style="color:#1565c0">--</span></p>
            <input type="password" id="profPwd" maxlength="10" placeholder="Password" style="padding:8px 12px;border:1px solid #ccc;border-radius:6px;font-size:1em;width:120px;text-align:center">
            <button class="btn" style="margin-left:8px;padding:8px 22px;background:#2e7d32;color:#fff;border:none;border-radius:6px;font-weight:700" onclick="confirmProfile()">Confirm</button>
            <p id="profPwMsg" style="color:#c62828;font-size:0.85em;margin-top:6px"></p>
        </div>
    </div>

    <!-- ============ OVERFILL OVERRIDE SCREEN ============ -->
    <div class="screen" id="scr-overfill">
        <div class="hdr"><h1>Overfill Override</h1></div>
        <div class="card"><div class="card-body">
            <p style="color:#c62828;font-weight:700;text-align:center;padding:12px">WARNING: Overriding the overfill sensor bypasses a critical safety protection. Only use during maintenance.</p>
            <div style="text-align:center;padding:10px">
                <button class="btn" style="padding:14px 28px;background:#e65100;color:#fff;border:none;border-radius:8px;font-weight:700;font-size:1em" onclick="sendCmd('overfill_override')">Confirm Override</button>
            </div>
        </div></div>
    </div>

    <!-- ============ ABOUT SCREEN ============ -->
    <div class="screen" id="scr-about">
        <div class="hdr"><h1>About</h1></div>
        <div class="card"><div class="card-body">
            <div class="row"><span class="lbl">Profile</span><span class="val" id="aboutProfile">--</span></div>
            <div class="row"><span class="lbl">Firmware</span><span class="val" id="aboutVersion">--</span></div>
            <div class="row"><span class="lbl">Device Name</span><span class="val" id="aboutDevice">--</span></div>
            <div class="row"><span class="lbl">IMEI</span><span class="val" id="aboutImei">--</span></div>
            <div class="row"><span class="lbl">MAC Address</span><span class="val" id="aboutMac">--</span></div>
            <div class="row"><span class="lbl">Modem Status</span><span class="val" id="aboutModem">--</span></div>
            <div class="row"><span class="lbl">IP Address</span><span class="val">192.168.4.1</span></div>
            <div class="row"><span class="lbl">Uptime</span><span class="val" id="aboutUptime">--</span></div>
            <div class="row"><span class="lbl">Board Temp</span><span class="val" id="aboutTemp">--</span></div>
            <div class="row"><span class="lbl">ADC Errors</span><span class="val" id="aboutAdcErr">0</span></div>
        </div></div>
    </div>

    <!-- ============ CONFIGURATION SCREEN ============ -->
    <div class="screen" id="scr-config">
        <div class="hdr"><h1>Configuration</h1></div>
        <div class="card"><div class="card-title bg-orange">Device Settings</div><div class="card-body">
            <div class="cfg-row" style="padding:6px 0">
                <span class="lbl">Password:</span>
                <input type="password" id="cfgPwd" maxlength="20" placeholder="Required" style="padding:6px 8px;border:1px solid #ccc;border-radius:4px;font-size:14px;width:120px;">
            </div>
            <div style="font-size:0.75em;color:#999;padding:2px 0 8px">Password required for changes</div>
            <div class="cfg-row" style="padding:6px 0;display:flex;gap:6px;align-items:center">
                <span class="lbl">Device Name:</span>
                <input type="text" id="nameInput" maxlength="20" value="%DEVICENAME%" style="padding:6px 8px;border:1px solid #ccc;border-radius:4px;font-size:14px;flex:1">
                <button style="padding:6px 14px;background:#1565c0;color:#fff;border:none;border-radius:4px;font-weight:600;cursor:pointer" onclick="saveName()">Save</button>
            </div>
            <span id="nameMsg" style="font-size:11px"></span>
            <div class="cfg-row" style="padding:6px 0;display:flex;gap:6px;align-items:center">
                <span class="lbl">Serial Watchdog:</span>
                <label style="display:flex;align-items:center;cursor:pointer">
                    <input type="checkbox" id="wdToggle" onchange="setWatchdog(this.checked)" style="width:20px;height:20px;margin-right:6px">
                    <span id="wdLabel" style="font-weight:600;font-size:0.85em">--</span>
                </label>
            </div>
            <div style="font-size:0.75em;color:#999;padding:2px 0 8px">Pulses GPIO39 if no serial data for 30 min</div>
            <div class="cfg-row" style="padding:6px 0;display:flex;gap:6px;align-items:center">
                <span class="lbl">Failsafe Mode:</span>
                <label style="display:flex;align-items:center;cursor:pointer">
                    <input type="checkbox" id="fsToggle" onchange="setFailsafe(this.checked)" style="width:20px;height:20px;margin-right:6px">
                    <span id="fsLabel" style="font-weight:600;font-size:0.85em">--</span>
                </label>
            </div>
            <div style="font-size:0.75em;color:#999;padding:2px 0 8px">Enables autonomous relay control after watchdog failures</div>
            <div class="cfg-row" style="padding:6px 0;display:flex;gap:6px;align-items:center">
                <span class="lbl">Serial Debug:</span>
                <label style="display:flex;align-items:center;cursor:pointer">
                    <input type="checkbox" id="dbgToggle" onchange="setDebugMode(this.checked)" style="width:20px;height:20px;margin-right:6px">
                    <span id="dbgLabel" style="font-weight:600;font-size:0.85em">--</span>
                </label>
            </div>
            <div style="font-size:0.75em;color:#999;padding:2px 0 8px">Verbose Serial Monitor output (FAST-TX, cell refresh, relay refresh)</div>
        </div></div>
    </div><!-- end scr-config -->

    <!-- ============ DIAGNOSTICS SCREEN ============ -->
    <div class="screen" id="scr-diagnostics">
        <div class="hdr"><h1>Diagnostics</h1></div>

        <!-- Live Data Section - Values sent to Python program -->
        <div class="card"><div class="card-title bg-green">Live Data</div><div class="card-body">
            <div class="row"><span class="lbl">Pressure</span><span class="val" id="diagPressure">--.--</span></div>
            <div class="row"><span class="lbl">Mode</span><span class="val" id="diagMode">--</span></div>
            <div class="row"><span class="lbl">Current</span><span class="val" id="diagCurrent">--.--</span></div>
            <div class="row"><span class="lbl">Fault</span><span class="val" id="diagFault">0</span></div>
        </div></div>

        <div class="card"><div class="card-title bg-blue">Serial Data</div><div class="card-body">
            <div class="row"><span class="lbl">Serial Status</span><span class="val" id="serialStatus">--</span></div>
            <div class="row"><span class="lbl">Device ID</span><span class="val" id="deviceId">--</span></div>
            <div class="row"><span class="lbl">Fault Code</span><span class="val" id="fault">0</span></div>
        </div></div>
        <div class="card"><div class="card-title bg-green">ADC Readings</div><div class="card-body">
            <div class="row"><span class="lbl">ADC Pressure (raw)</span><span class="val" id="adcRawP">--</span></div>
            <div class="row"><span class="lbl">ADC Current (raw)</span><span class="val" id="adcRawC">--</span></div>
            <div class="row"><span class="lbl">ADC Zero Point</span><span class="val" id="adcZeroP">--</span></div>
            <div class="row"><span class="lbl">ADC Initialized</span><span class="val" id="adcInit">--</span></div>
            <div class="row"><span class="lbl">ADC Errors</span><span class="val" id="adcErrors">0</span></div>
            <div class="row"><span class="lbl">ADC Outliers Rejected</span><span class="val" id="adcOutliers">0</span></div>
        </div></div>
        <div class="card"><div class="card-title bg-green">Cellular Modem</div><div class="card-body">
            <div class="row"><span class="lbl">LTE Status</span><span class="val" id="lteStatus">--</span></div>
            <div class="row"><span class="lbl">RSRP</span><span class="val" id="rsrp">--</span></div>
            <div class="row"><span class="lbl">RSRQ</span><span class="val" id="rsrq">--</span></div>
            <div class="row"><span class="lbl">Operator</span><span class="val" id="diagOperator">--</span></div>
            <div class="row"><span class="lbl">Band</span><span class="val" id="band">--</span></div>
            <div class="row"><span class="lbl">MCC/MNC</span><span class="val" id="mccmnc">--</span></div>
            <div class="row"><span class="lbl">Cell ID</span><span class="val" id="cellId">--</span></div>
            <div class="row"><span class="lbl">BlueCherry</span><span class="val" id="blueCherry">--</span></div>
            <div class="row"><span class="lbl">MAC</span><span class="val" id="mac">--</span></div>
        </div></div>
        <div class="card"><div class="card-title bg-gray">IO Board</div><div class="card-body">
            <div class="row"><span class="lbl">Firmware</span><span class="val" id="version">--</span></div>
            <div class="row"><span class="lbl">Board Temp</span><span class="val" id="temperature">--</span></div>
            <div class="row"><span class="lbl">SD Card</span><span class="val" id="sdCard">--</span></div>
            <div class="row"><span class="lbl">Overfill Sensor</span><span class="val" id="overfill">--</span></div>
            <div class="row"><span class="lbl">GPIO38 Raw</span><span class="val" id="overfillDebug" style="font-size:0.78em;color:#888">--</span></div>
            <div class="row"><span class="lbl">Watchdog</span><span class="val" id="watchdog">--</span></div>
            <div class="row"><span class="lbl">Uptime</span><span class="val" id="uptime">--</span>            </div>
        </div></div>

        <!-- CBOR Payload History -->
        <div class="card"><div class="card-title bg-gray">CBOR Payload History</div><div class="card-body">
            <div id="cborPayloadHistory" style="font-family:monospace;font-size:0.75em;max-height:200px;overflow-y:auto;">
                <div style="color:#999;text-align:center;padding:20px">Waiting for CBOR transmissions...</div>
            </div>
        </div></div>
    </div><!-- end scr-diagnostics -->

    </div><!-- end content-wrap -->

    <!-- Passthrough Modal -->
    <div class="modal-bg" id="ptModal">
        <div class="modal">
            <div class="modal-hdr">Enter Passthrough Mode?</div>
            <div class="modal-body">
                <p style="margin-bottom:8px"><b>This will:</b></p>
                <ul><li>Finish current purge step (if running)</li><li>Idle all relays</li><li>Restart ESP32 into passthrough mode</li><li>Bridge serial directly to modem</li></ul>
                <p style="color:#c62828"><b>Auto-returns to normal after 60 min.</b></p>
            </div>
            <div class="modal-foot">
                <button class="btn btn-cancel" onclick="closePtModal()">Cancel</button>
                <button class="btn btn-danger" onclick="confirmPassthrough()">Enable</button>
            </div>
        </div>
    </div>

    <!-- Maintenance/Back Footer Button -->
    <div class="maint-footer">
        <button class="maint-btn" id="footerBtn" onclick="nav('maint')">Maintenance</button>
    </div>

    <script>
    (function(){
        // Enable zooming for desktop browsers only
        var isDesktop = window.innerWidth >= 1024 || (!/Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent) && window.innerWidth >= 768);
        if (isDesktop) {
            var viewport = document.querySelector('meta[name=viewport]');
            if (viewport) {
                viewport.setAttribute('content', 'width=device-width, initial-scale=1.0, maximum-scale=5.0, user-scalable=yes');
            }
        }

        var errs=0,IP='192.168.4.1',curScr='main';
        var navStack=['main'];
        function $(id){return document.getElementById(id);}
        function upd(id,v){var e=$(id);if(e&&e.textContent!==String(v))e.textContent=v;}
        function conn(ok){var b=$('connBadge');if(b){b.textContent=ok?'OK':'ERR';b.className='badge '+(ok?'ok':'err');}if(ok)errs=0;}
        function setInd(id,alarm){var e=$(id);if(e)e.className='ind '+(alarm?'ind-r':'ind-g');}

        // SPA navigation: show one screen, hide others
        window.nav=function(scr){
            // Don't navigate to the same screen
            if(scr === curScr) return;

            var screens=document.querySelectorAll('.screen');
            for(var i=0;i<screens.length;i++)screens[i].classList.remove('active');
            var target=$('scr-'+scr);if(target)target.classList.add('active');

            // Push current screen to navigation stack before changing
            if(curScr && curScr !== scr) {
                navStack.push(curScr);
            }
            curScr=scr;

            // Update footer button based on current screen
            var btn=$('footerBtn');
            if(scr==='main'){
                // Clear navigation stack when returning to main
                navStack=['main'];
                btn.textContent='Maintenance';
                btn.onclick=function(){nav('maint');};
            } else {
                btn.textContent='Back';
                btn.onclick=function(){
                    // Go back to previous screen in stack, or main if stack is empty
                    var prevScr = navStack.length > 0 ? navStack.pop() : 'main';
                    nav(prevScr);
                };
            }
        };

        // Send command to ESP32 via POST /api/command
        window.sendCmd=function(cmd,val){
            var x=new XMLHttpRequest();
            x.open('POST','http://'+IP+'/api/command',true);
            x.setRequestHeader('Content-Type','application/json');
            var body=JSON.stringify({command:cmd,value:val||''});
            x.send(body);
        };

        // Toggle individual relays
        window.toggleRelay=function(relay){
            var cmd = 'toggle_relay';
            var val = relay;
            var x=new XMLHttpRequest();
            x.open('POST','http://'+IP+'/api/command',true);
            x.setRequestHeader('Content-Type','application/json');
            x.send(JSON.stringify({command:cmd,value:val}));
        };

        // ---- CALIBRATE PRESSURE SENSOR ZERO POINT ----
        // Sends calibrate_pressure command to ESP32. Takes ~1 second (60 samples).
        // Shows result (new zero point) on the Maintenance screen.
        window.calibratePressure=function(){
            if(!confirm('Calibrate pressure sensor?\\nCurrent reading will become 0.0 IWC.\\nEnsure sensor is open to ambient air (no vacuum).\\nThis takes about 1 second.')) return;
            var el=document.getElementById('calResult');
            if(el) el.textContent='Calibrating...';
            var x=new XMLHttpRequest();
            x.open('POST','http://'+IP+'/api/command',true);
            x.setRequestHeader('Content-Type','application/json');
            x.onload=function(){
                if(x.status===200){
                    try{
                        var r=JSON.parse(x.responseText);
                        if(el) el.textContent='Calibrated! ADC Zero: '+r.adcZero.toFixed(2);
                    }catch(e){if(el) el.textContent='Calibration complete';}
                }else{if(el) el.textContent='Calibration failed ('+x.status+')';}
            };
            x.onerror=function(){if(el) el.textContent='Calibration request failed';};
            x.send(JSON.stringify({command:'calibrate_pressure'}));
        };

        // ---- MANUAL ADC ZERO SETTING ----
        // Gets ADC zero value from input field and sends to ESP32 via /api/command.
        // ESP32 validates the value, updates adcZeroPressure, saves to EEPROM,
        // and returns the new value. User is then navigated back to Maintenance screen.
        //
        // Usage: User clicks "Manual ADC Zero" button on Maintenance screen,
        //        enters a numeric value, clicks Submit. The value is saved to EEPROM.
        window.submitManualAdcZero=function(){
            var inputEl=document.getElementById('adcZeroInput');
            var resultEl=document.getElementById('adcZeroResult');
            if(!inputEl || !resultEl) return;

            var adcValue=parseFloat(inputEl.value);
            if(isNaN(adcValue) || adcValue < 0){
                resultEl.style.color='#ff6b6b';
                resultEl.textContent='Invalid ADC value - must be a positive number';
                return;
            }

            if(!confirm('Set manual ADC zero to '+adcValue.toFixed(2)+'?\\nThis will override the current calibration.')) return;

            resultEl.style.color='#00695c';
            resultEl.textContent='Setting ADC zero...';

            var x=new XMLHttpRequest();
            x.open('POST','http://'+IP+'/api/command',true);
            x.setRequestHeader('Content-Type','application/json');
            x.onload=function(){
                if(x.status===200){
                    try{
                        var r=JSON.parse(x.responseText);
                        resultEl.textContent='ADC Zero set to: '+r.adcZero.toFixed(2)+' - Returning to Maintenance...';
                        setTimeout(function(){nav('maint');}, 2000);
                    }catch(e){
                        resultEl.style.color='#ff6b6b';
                        resultEl.textContent='Error: Invalid response from device';
                    }
                }else{
                    resultEl.style.color='#ff6b6b';
                    resultEl.textContent='Failed to set ADC zero ('+x.status+')';
                }
            };
            x.onerror=function(){
                resultEl.style.color='#ff6b6b';
                resultEl.textContent='Connection error - please try again';
            };
            x.send(JSON.stringify({command:'set_manual_adc_zero',value:adcValue.toString()}));
        };

        // ---- PROFILE SELECT + PASSWORD CONFIRM (PW: validated server-side) ----
        // Step 1: User clicks a profile button -> highlights it, shows confirm area
        var pendingProfile='';   // Tracks which profile the user selected
        window.selectProfile=function(name,btn){
            pendingProfile=name;
            // Highlight selected button, remove highlight from others
            var btns=document.querySelectorAll('.profile-btn');
            for(var i=0;i<btns.length;i++) btns[i].classList.remove('active-profile');
            btn.classList.add('active-profile');
            // Show the confirm section with selected profile name
            upd('profSelected',name);
            $('profPwMsg').textContent='';
            $('profPwd').value='';
            $('profConfirm').style.display='block';
        };

        // Step 2: User enters password and clicks Confirm -> sends to server
        // Server validates password "1793" before applying the profile change.
        window.confirmProfile=function(){
            if(!pendingProfile){return;}
            var pw=$('profPwd').value;
            if(!pw){$('profPwMsg').textContent='Password required';return;}
            var x=new XMLHttpRequest();
            x.open('POST','http://'+IP+'/api/command',true);
            x.setRequestHeader('Content-Type','application/json');
            x.onload=function(){
                if(x.status===200){
                    // Success - update display
                    upd('profCurrent',pendingProfile);
                    $('profPwMsg').style.color='#2e7d32';
                    $('profPwMsg').textContent='Profile changed to '+pendingProfile;
                    $('profPwd').value='';
                    // Auto-hide confirm area after 2s
                    setTimeout(function(){$('profConfirm').style.display='none';$('profPwMsg').textContent='';$('profPwMsg').style.color='#fff';},2000);
                } else if(x.status===403){
                    $('profPwMsg').style.color='#fff';
                    $('profPwMsg').textContent='Wrong password';
                } else {
                    $('profPwMsg').style.color='#fff';
                    $('profPwMsg').textContent='Error: '+x.status;
                }
            };
            x.onerror=function(){$('profPwMsg').style.color='#fff';$('profPwMsg').textContent='Connection error';};
            // Send profile change with password for server-side validation
            x.send(JSON.stringify({command:'set_profile',value:pendingProfile,password:pw}));
        };

        // ---- MAINTENANCE PASSWORD GATE (PW: 878, checked client-side) ----
        // Keeps maintenance locked until correct password is entered.
        // Password stays unlocked for the rest of the browser session.
        var maintUnlocked=false;
        window.checkMaintPw=function(){
            var pw=$('maintPwd').value;
            if(pw==='878'){
                maintUnlocked=true;
                $('maintLock').style.display='none';
                $('maintMenu').style.display='block';
                $('maintPwMsg').textContent='';
            } else {
                $('maintPwMsg').textContent='Incorrect password';
            }
        };
        // Intercept nav to maint: always show password gate first
        var origNav=window.nav;
        window.nav=function(scr){
            origNav(scr);
            if(scr==='maint'){
                // Always start with password gate - user must enter 878
                $('maintLock').style.display='block';
                $('maintMenu').style.display='none';
                $('maintPwd').value='';
                $('maintPwMsg').textContent='';
                maintUnlocked = false; // Reset unlock status
            }
        };

        // Config functions
        window.saveName=function(){
            var n=$('nameInput').value.replace(/^\s+|\s+$/g,''),m=$('nameMsg'),p=$('cfgPwd').value;
            if(!p){m.textContent='Password required';m.style.color='#c62828';return;}
            if(n.length<3||n.length>20){m.textContent='3-20 chars';m.style.color='#c62828';return;}
            m.textContent='Saving...';m.style.color='#666';
            var x=new XMLHttpRequest();
            x.open('GET','http://'+IP+'/setdevicename?name='+encodeURIComponent(n)+'&pwd='+encodeURIComponent(p),true);
            x.onload=function(){
                if(x.status===403){m.textContent='Wrong password';m.style.color='#c62828';}
                else{m.textContent='Saved!';m.style.color='#2e7d32';setTimeout(function(){m.textContent='';},3000);}
            };
            x.onerror=function(){m.textContent='Error';m.style.color='#c62828';};
            x.send();
        };
        window.setWatchdog=function(on){
            var p=$('cfgPwd').value;
            if(!p){alert('Password required');$('wdToggle').checked=!on;return;}
            var x=new XMLHttpRequest();
            x.open('GET','http://'+IP+'/setwatchdog?enabled='+(on?'1':'0')+'&pwd='+encodeURIComponent(p),true);
            x.onload=function(){
                if(x.status===403){alert('Wrong password');$('wdToggle').checked=!on;}
                else{$('wdLabel').textContent=on?'ENABLED':'DISABLED';$('wdLabel').style.color=on?'#2e7d32':'#999';}
            };
            x.send();
        };
        window.setFailsafe=function(on){
            var p=$('cfgPwd').value;
            if(!p){alert('Password required');$('fsToggle').checked=!on;return;}
            var x=new XMLHttpRequest();
            x.open('GET','http://'+IP+'/setfailsafe?enabled='+(on?'1':'0')+'&pwd='+encodeURIComponent(p),true);
            x.onload=function(){
                if(x.status===403){alert('Wrong password');$('fsToggle').checked=!on;}
                else{$('fsLabel').textContent=on?'ENABLED':'DISABLED';$('fsLabel').style.color=on?'#2e7d32':'#999';}
            };
            x.send();
        };
        window.setDebugMode=function(on){
            var p=$('cfgPwd').value;
            if(!p){alert('Password required');$('dbgToggle').checked=!on;return;}
            var x=new XMLHttpRequest();
            x.open('GET','http://'+IP+'/setdebug?enabled='+(on?'1':'0')+'&pwd='+encodeURIComponent(p),true);
            x.onload=function(){
                if(x.status===403){alert('Wrong password');$('dbgToggle').checked=!on;}
                else{$('dbgLabel').textContent=on?'ON':'OFF';$('dbgLabel').style.color=on?'#e65100':'#999';}
            };
            x.send();
        };
        window.showPtModal=function(){$('ptModal').style.display='flex';};
        window.closePtModal=function(){$('ptModal').style.display='none';};
        window.confirmPassthrough=function(){
            closePtModal();
            var x=new XMLHttpRequest();
            x.open('GET','http://'+IP+'/setpassthrough?enabled=1',true);
            x.send();
        };

        // Mode name lookup
        var modeNames={0:'Idle',1:'Run',2:'Purge',3:'Burp',8:'Fresh Air',9:'Leak Test'};

        // Update relay button states
        function updateRelayButtons(d){
            var relays = ['CR1', 'CR2', 'CR5'];
            relays.forEach(function(relay){
                var btn = $('relay' + relay);
                if(btn){
                    var isOn = d['relay' + relay] || false;
                    btn.textContent = relay + ' ' + (isOn ? 'ON' : 'OFF');
                    btn.style.background = isOn ? '#4CAF50' : '#e8eaf6';
                    btn.style.color = isOn ? '#fff' : '#1a1a2e';
                }
            });
        }

        // Update CBOR payload history display
        function updateCborPayloadHistory(d){
            var historyDiv = $('cborPayloadHistory');
            if(!historyDiv || !d.cborPayloadHistory) return;

            var history = d.cborPayloadHistory;
            if(history.length === 0){
                historyDiv.innerHTML = '<div style="color:#999;text-align:center;padding:20px">Waiting for CBOR transmissions...</div>';
                return;
            }

            var html = '';
            history.forEach(function(payload, idx){
                if(payload.timestamp && payload.data){
                    var date = new Date(payload.timestamp);
                    var timeStr = date.toLocaleTimeString();
                    html += '<div style="border-bottom:1px solid #eee;padding:8px 0">';
                    html += '<div style="color:#666;font-weight:bold">#' + (history.length - idx) + ' - ' + timeStr + '</div>';
                    html += '<div style="margin-top:4px;color:#333">CBOR Data: [' + payload.data.join(', ') + ']</div>';

                    // Try to decode the CBOR array elements for readability
                    if(payload.data.length >= 15){
                        html += '<div style="margin-top:4px;color:#666;font-size:0.9em">';
                        html += 'Device ID: ' + payload.data[0] + ', ';
                        html += 'Type: IO_Board, ';
                        html += 'LTE Band: ' + payload.data[3] + ', ';
                        html += 'RSRP: ' + payload.data[5] + ' dBm';
                        html += '</div>';
                    }
                    html += '</div>';
                }
            });
            historyDiv.innerHTML = html;
        }

        function poll(){
            var x=new XMLHttpRequest();
            x.timeout=4000;
            x.onreadystatechange=function(){
                if(x.readyState!==4)return;
                if(x.status===200){
                    try{
                        var d=JSON.parse(x.responseText);
                        conn(true);
                        // Top bar
                        upd('topTime',d.datetime||'--');
                        // Main screen - simplified status
                        var modeText = modeNames[d.mode] || d.mode || 'UNKNOWN';
                        var hasAlarms = d.overfillAlarm || d.watchdogTriggered || d.failsafe ||
                                       d.alarmLowPress || d.alarmHighPress || d.alarmZeroPress ||
                                       d.alarmLowCurrent || d.alarmHighCurrent;

                        // Update status card
                        var card = $('statusCard');
                        var modeEl = $('statusMode');
                        var alarmsEl = $('statusAlarms');

                        if (card && modeEl && alarmsEl) {
                            modeEl.textContent = modeText;
                            if (hasAlarms) {
                                card.classList.add('alarm');
                                var alarmList = [];
                                if (d.overfillAlarm) alarmList.push('Overfill');
                                if (d.watchdogTriggered) alarmList.push('Watchdog');
                                if (d.failsafe) alarmList.push('Failsafe');
                                if (d.alarmLowPress) alarmList.push('Low Pressure');
                                if (d.alarmHighPress) alarmList.push('High Pressure');
                                if (d.alarmZeroPress) alarmList.push('Zero Pressure');
                                if (d.alarmLowCurrent) alarmList.push('Low Current');
                                if (d.alarmHighCurrent) alarmList.push('High Current');
                                alarmsEl.textContent = alarmList.length > 0 ? alarmList.join(', ') : 'System Alarm';
                            } else {
                                card.classList.remove('alarm');
                                alarmsEl.textContent = 'No Alarms';
                            }
                        }

                        // Update pressure and cycles displays
                        var pressureVal = d.pressure !== undefined ? parseFloat(d.pressure).toFixed(2) : '--.--';
                        upd('pressureDisplay', 'UST PRESSURE: ' + pressureVal);
                        var cyclesVal = d.cycles !== undefined ? d.cycles.toString() : '--';
                        upd('cyclesDisplay', 'RUN CYCLES: ' + cyclesVal);
                        // Alarms
                        upd('alarmProfile',d.profile);upd('alarmFault',d.fault||'0');
                        setInd('aOvfl',d.overfillAlarm);setInd('aWatch',d.watchdogTriggered);setInd('aFailsafe',d.failsafe);
                        setInd('aLowP',d.alarmLowPress);setInd('aHighP',d.alarmHighPress);
                        setInd('aZeroP',d.alarmZeroPress);setInd('aVarP',d.alarmVarPress);
                        setInd('aLowC',d.alarmLowCurrent);setInd('aHighC',d.alarmHighCurrent);
                        setInd('aPanelPwr',d.failsafe);setInd('a72Hr',false);
                        // Manual mode
                        upd('manPressure',d.pressure);upd('manCurrent',d.current);
                        upd('manCycles',d.cycles);upd('manMode',modeNames[d.mode]||d.mode);
                        // Tests — update timer displays from server test state
                        upd('testPressure',d.pressure);
                        // Known test durations in seconds (matches Python IOManager)
                        var testDurations={leak:1800,func:1200,eff:120,clean:7200};
                        if(d.testRunning){
                            var el=d.testElapsed||0;
                            var dur=testDurations[d.testType]||0;
                            var remain=Math.max(0,dur-el);
                            var mm=Math.floor(remain/60);
                            var ss=remain%%60;
                            var tmr=(mm<10?'0':'')+mm+':'+(ss<10?'0':'')+ss;
                            var stat='Running ('+d.testType+')';
                            // Update the correct screen's timer based on test type
                            if(d.testType==='leak'){upd('leakTimer',tmr);upd('leakStatus',stat);}
                            else if(d.testType==='func'){upd('funcTimer',tmr);upd('funcStatus',stat);if(d.testMultiStep){var rep=Math.floor(d.testStep/2)+1;var phase=(d.testStep%%2===0)?'Run':'Purge';upd('funcCycle',rep+' of '+(d.testStepTotal/2));upd('funcPhase',phase);}}
                            else if(d.testType==='eff'){upd('effTime',tmr);upd('effStep','Running');upd('effMode',modeNames[d.mode]||d.mode);}
                            else if(d.testType==='clean'){upd('cleanTimer',tmr);upd('cleanStatus',stat);}
                        } else {
                            // No test running — only reset to defaults if they were showing active
                            var lt=$('leakStatus');if(lt&&lt.textContent.indexOf('Running')>=0){upd('leakTimer','--:--');upd('leakStatus','Ready');}
                            var ft=$('funcStatus');if(ft&&ft.textContent.indexOf('Running')>=0){upd('funcTimer','--');upd('funcStatus','Ready');upd('funcCycle','--');upd('funcPhase','--');}
                            var es=$('effStep');if(es&&es.textContent.indexOf('Running')>=0){upd('effTime','--');upd('effStep','--');upd('effMode','--');}
                            var cs=$('cleanStatus');if(cs&&cs.textContent.indexOf('Running')>=0){upd('cleanTimer','120:00');upd('cleanStatus','Ready');}
                        }
                        // Profiles
                        upd('profCurrent',d.profile);
                        var btns=document.querySelectorAll('.profile-btn');
                        for(var i=0;i<btns.length;i++){btns[i].classList.remove('active-profile');if(btns[i].textContent.indexOf(d.profile)>=0)btns[i].classList.add('active-profile');}
                        // About
                        upd('aboutProfile',d.profile);upd('aboutVersion',d.version);
                        upd('aboutDevice',d.deviceName);upd('aboutImei',d.imei||'--');
                        upd('aboutMac',d.macAddress);upd('aboutModem',d.modemStatus);
                        upd('aboutUptime',d.uptime);upd('aboutTemp',d.temperature+' F');
                        upd('aboutAdcErr',d.adcErrors||'0');
                        // Diagnostics - Current Values Section
                        upd('diagMode', modeNames[d.mode] || d.mode || '--');
                        upd('diagPressure', d.pressure !== undefined ? parseFloat(d.pressure).toFixed(2) : '--.--');
                        upd('diagCurrent', d.current !== undefined ? parseFloat(d.current).toFixed(2) : '--.--');
                        upd('diagFault', d.fault || '0');

                        // Diagnostics
                        var ss=$('serialStatus');if(ss){ss.textContent=d.serialActive?'Receiving':'No Data';ss.style.color=d.serialActive?'#2e7d32':'#c62828';}
                        upd('deviceId',d.deviceName);upd('fault',d.fault||'0');
                        upd('adcRawP',d.adcRawPressure||'--');upd('adcRawC',d.adcRawCurrent||'--');
                        upd('adcZeroP',d.adcZeroPressure||'--');
                        upd('adcInit',d.adcInitialized?'Yes':'No');upd('adcErrors',d.adcErrors||'0');upd('adcOutliers',d.adcOutliers||'0');
                        var le=$('lteStatus');if(le){if(d.lteConnected){le.innerHTML='<span style=color:#2e7d32>Connected</span>';}else if(!d.cellularReady){le.innerHTML='<span style=color:#e65100>Connecting...</span>';}else{le.innerHTML='<span style=color:#c62828>Disconnected</span>';}}
                        upd('rsrp',(d.rsrp||'--')+' dBm');upd('rsrq',(d.rsrq||'--')+' dB');
                        upd('diagOperator',d.operator||'--');upd('band',d.band||'--');
                        upd('mccmnc',d.mcc&&d.mnc?(d.mcc+'/'+d.mnc):'--');upd('cellId',d.cellId||'--');
                        var bc=$('blueCherry');if(bc)bc.innerHTML=d.blueCherryConnected?'<span style=color:#2e7d32>Connected</span>':'<span style=color:#e65100>Offline</span>';
                        upd('version',d.version);upd('temperature',d.temperature+' F');
                        var sd=$('sdCard');if(sd)sd.innerHTML=d.sdCardOK?'<span style=color:#2e7d32>OK</span>':'<span style=color:#c62828>FAULT</span>';
                        var ov=$('overfill');if(ov)ov.innerHTML=d.overfillAlarm?'<span style=color:#c62828>ALARM</span>':'<span style=color:#2e7d32>Normal</span>';
                        var od=$('overfillDebug');if(od)od.innerHTML='Pin='+(d.gpio38Raw?'HIGH':'LOW')+' L:'+d.overfillLowCnt+' H:'+d.overfillHighCnt;
                        upd('watchdog',d.watchdogEnabled?'Enabled':'Disabled');
                        var wt=$('wdToggle'),wl=$('wdLabel');if(wt)wt.checked=d.watchdogEnabled;if(wl){wl.textContent=d.watchdogEnabled?'ENABLED':'DISABLED';wl.style.color=d.watchdogEnabled?'#2e7d32':'#999';}
                        upd('failsafe',d.failsafeEnabled?'Enabled':'Disabled');
                        var ft=$('fsToggle'),fl=$('fsLabel');if(ft)ft.checked=d.failsafeEnabled;if(fl){fl.textContent=d.failsafeEnabled?'ENABLED':'DISABLED';fl.style.color=d.failsafeEnabled?'#2e7d32':'#999';}
                        var dt=$('dbgToggle'),dl=$('dbgLabel');if(dt)dt.checked=d.serialDebugMode;if(dl){dl.textContent=d.serialDebugMode?'ON':'OFF';dl.style.color=d.serialDebugMode?'#e65100':'#999';}
                        upd('uptime',d.uptime);upd('mac',d.macAddress);
                        // Update relay button states
                        updateRelayButtons(d);
                        // Update CBOR payload history
                        updateCborPayloadHistory(d);
                    }catch(e){errs++;if(errs>=3)conn(false);}
                }else{errs++;if(errs>=3)conn(false);}
            };
            x.onerror=function(){errs++;if(errs>=3)conn(false);};
            x.ontimeout=function(){errs++;if(errs>=3)conn(false);};
            x.open('GET','http://'+IP+'/api/status',true);
            x.send();
        }
        poll();setInterval(poll,2000);
    })();
    </script>
</body>
</html>
)rawliteral";

// =====================================================================
// WEB PROCESSOR FUNCTION
// =====================================================================
// Handles %VARIABLE% template substitution in the HTML
// Template variables use single %: %DEVICENAME% -> replaced with actual value
// Literal percent signs in CSS/JS use double %%: 100%% -> renders as 100%
// The processor sees "DEVICENAME" (without percent signs) as the variable name
// IMPORTANT: return String() (not "") to leave unrecognized variables untouched

String webProcessor(const String& var) {
    if (var == "DEVICENAME") return DeviceName;
    return String();  // Return empty String object for unknown variables
}

// =====================================================================
// RMS FUNCTIONS
// =====================================================================

void initSdLogging() {
    SPI.begin(SCLK, MISO, MOSI, CS);
    int attempts = 0;
    const int maxAttempts = 5;
    while (attempts < maxAttempts) {
        if (SD.cardType() == CARD_NONE) {
            Serial.println("No SD card detected");
            return;
        }
        
        if (SD.begin(CS, SPI, 4000000)) {
            Serial.println("**** SD Card Mounted on attempt " + String(attempts + 1));
            break;
        }
        attempts++;
        Serial.println("**** ERROR - SD Card not Mounted, attempt " + String(attempts) + " of " + String(maxAttempts));
        if (attempts < maxAttempts) {
            delay(2000);
        }
    }
    if (attempts >= maxAttempts) {
        Serial.println("**** ERROR - Failed to mount SD Card after " + String(maxAttempts) + " attempts");
        return;
    }

    time_t rawTime = (time_t)currentTimestamp;
    struct tm *timeInfo = gmtime(&rawTime);
    int currentYear = timeInfo->tm_year + 1900;
    String logFileName = "/logfile" + String(currentYear) + ".log";

    logFile = SD.open(logFileName.c_str(), FILE_APPEND);
    if (!logFile) {
        SD.remove(logFileName.c_str());
        File f = SD.open(logFileName.c_str(), FILE_WRITE);
        if (f) {
            f.close();
            logFile = SD.open(logFileName.c_str(), FILE_APPEND);
        }
        if (!logFile) {
            Serial.print(F("Error: could not open "));
            Serial.print(logFileName);
            Serial.println(F(" for appending"));
        }
    }

    cleanupOldLogFiles(currentYear);
}

void cleanupOldLogFiles(int currentYear) {
    File root = SD.open("/");
    if (!root) {
        Serial.println("Failed to open root directory for cleanup");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        String fileName = String(file.name());
        if (fileName.startsWith("logfile") && fileName.endsWith(".log")) {
            String yearStr = fileName.substring(7, 11);
            int fileYear = yearStr.toInt();
            if (fileYear > 0 && fileYear < currentYear - 2) {
                String fullPath = "/" + fileName;
                if (SD.remove(fullPath.c_str())) {
                    Serial.println("Deleted old log file: " + fullPath);
                } else {
                    Serial.println("Failed to delete old log file: " + fullPath);
                }
            }
        }
        file = root.openNextFile();
    }
    root.close();
}

void SaveToSD(String id, unsigned long seq, int pres, int cycles, int fault, int mode, int temp, float current) {
    if (!logFile) {
        Serial.println("Log file not open, attempting to reopen");
        if (!SD.cardType()) {
            Serial.println("SD card not mounted, attempting to remount");
            int attempts = 0;
            const int maxAttempts = 3;
            while (attempts < maxAttempts) {
                if (SD.begin(CS)) {
                    Serial.println("SD card remounted successfully on attempt " + String(attempts + 1));
                    break;
                }
                attempts++;
                Serial.println("Failed to remount SD card, attempt " + String(attempts) + " of " + String(maxAttempts));
                delay(1000);
            }
            if (attempts >= maxAttempts) {
                Serial.println("Failed to remount SD card after " + String(maxAttempts) + " attempts");
                return;
            }
        }
        time_t rawTime = (time_t)currentTimestamp;
        struct tm *timeInfo = gmtime(&rawTime);
        int currentYear = timeInfo->tm_year + 1900;
        String logFileName = "/logfile" + String(currentYear) + ".log";
        logFile = SD.open(logFileName.c_str(), FILE_APPEND);
        if (!logFile) {
            Serial.print(F("Error: could not reopen "));
            Serial.print(logFileName);
            Serial.println(F(" for appending"));
            File f = SD.open(logFileName.c_str(), FILE_WRITE);
            if (f) {
                f.close();
                logFile = SD.open(logFileName.c_str(), FILE_APPEND);
                if (logFile) {
                    Serial.println("Created and opened log file: " + logFileName);
                } else {
                    Serial.println("Failed to open log file after creation: " + logFileName);
                    return;
                }
            } else {
                Serial.println("Failed to create log file: " + logFileName);
                return;
            }
        } else {
            Serial.println("Successfully reopened log file: " + logFileName);
        }
    }
    String line;
    line.reserve(80);
    line = formatTimestamp(currentTimestamp);
    line += ',' + String(id);
    line += ',' + String(seq);
    line += ',' + String(pres / 100.0, 2);
    line += ',' + String(cycles);
    line += ',' + String(fault);
    line += ',' + String(mode);
    line += ',' + String(temp / 100.0, 2);
    line += ',' + String(current);
    
    Serial.print("Writing to SD: ");
    Serial.println(line);
    
    // REV 10.14: Gate ADC reads during SD card SPI activity.
    // SD card writes generate sustained SPI traffic that causes ~32mV analog interference
    // on the ADS1015, biasing pressure readings by -0.71 IWC for the full write duration.
    // The ADC reader task (Core 0) checks this flag and holds previous good values.
    sdWriteActive = true;
    
    if (logFile.println(line) == 0) {
        Serial.println(F("Error: SD write failed"));
        logFile.close();
        time_t rawTime = (time_t)currentTimestamp;
        struct tm *timeInfo = gmtime(&rawTime);
        int currentYear = timeInfo->tm_year + 1900;
        String logFileName = "/logfile" + String(currentYear) + ".log";
        logFile = SD.open(logFileName.c_str(), FILE_APPEND);
        if (!logFile) {
            Serial.print(F("Error: could not reopen file after write failure "));
            Serial.print(logFileName);
            Serial.println(F(" for appending"));
        } else {
            Serial.println("Reopened file after write failure: " + logFileName);
            if (logFile.println(line) == 0) {
                Serial.println(F("Error: SD write failed on retry"));
            } else {
                Serial.println("Successfully wrote to SD on retry");
            }
        }
    } else {
        Serial.println("Successfully wrote line to SD");
    }

    if (millis() - lastFlush > FLUSH_INTERVAL) {
        Serial.println("Flushing SD card data");
        logFile.flush();
        lastFlush = millis();
        Serial.println("Flush complete");
    }

    time_t rawTime = (time_t)currentTimestamp;
    struct tm *timeInfo = gmtime(&rawTime);
    int currentYear = timeInfo->tm_year + 1900;
    static int lastYear = currentYear;
    if (currentYear != lastYear) {
        logFile.close();
        String newLogFileName = "/logfile" + String(currentYear) + ".log";
        logFile = SD.open(newLogFileName.c_str(), FILE_APPEND);
        if (!logFile) {
            Serial.println(F("Error: could not open new logfile for appending"));
        } else {
            Serial.println("Switched to new log file for year " + String(currentYear));
        }
        lastYear = currentYear;
        cleanupOldLogFiles(currentYear);
    }
    
    // REV 10.14: Clear the SD write gate — ADC reader can resume normal reads.
    // This must be the LAST line before return, after all SD/SPI operations complete.
    sdWriteActive = false;
}

/**
 * Generate a random 6-character hex suffix for the WiFi AP name
 * Uses MAC address + millis() for randomness
 * 
 * Usage: Called on first boot when no device name is set
 * Returns: String like "5c3dfc"
 */
String generateRandomHexSuffix() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    
    // Combine MAC bytes with millis for randomness
    uint32_t seed = mac[3] ^ mac[4] ^ mac[5] ^ (millis() & 0xFF);
    randomSeed(seed);
    
    // Generate 6 random hex characters
    char hexBuf[7];
    for (int i = 0; i < 6; i++) {
        uint8_t nibble = random(16);
        hexBuf[i] = (nibble < 10) ? ('0' + nibble) : ('a' + nibble - 10);
    }
    hexBuf[6] = '\0';
    
    return String(hexBuf);
}

/**
 * Update the WiFi AP SSID name
 * Combines "GM IO Board " with the device ID or random suffix
 * 
 * @param suffix The suffix to append (device ID like "RND-0007" or random hex like "5c3dfc")
 * @param restartAP If true, restarts the WiFi AP with new name
 * 
 * Usage examples:
 *   updateAPName("5c3dfc", false);     // Set random suffix, don't restart yet
 *   updateAPName("RND-0007", true);    // Set device ID and restart AP
 */
void updateAPName(String suffix, bool restartAP) {
    apSuffix = suffix;
    snprintf(apSSID, sizeof(apSSID), "GM IO Board %s", suffix.c_str());
    
    Serial.print("[WiFi] AP name updated to: ");
    Serial.println(apSSID);
    
    if (restartAP) {
        Serial.println("[WiFi] Restarting Access Point with new name...");
        WiFi.softAPdisconnect(true);
        delay(100);
        WiFi.softAP(apSSID);
        Serial.print("[WiFi] AP restarted. IP: ");
        Serial.println(WiFi.softAPIP());
    }
}

/**
 * Initialize the WiFi AP name based on saved device name or generate random
 * Called during startup before startConfigAP()
 * 
 * - If DeviceName is set (not default "CSX-9000"), use it as suffix
 * - If DeviceName is default, generate random hex suffix
 * - Saves the suffix to EPROM for persistence
 */
void initializeAPName() {
    preferences.begin("RMS", false);
    String savedSuffix = preferences.getString("APSuffix", "");
    preferences.end();
    
    // Check if we have a real device name from serial
    if (DeviceName.length() > 0 && DeviceName != "CSX-9000") {
        // Use device name as suffix
        updateAPName(DeviceName, false);
        apNameFromSerial = true;
        Serial.println("[WiFi] Using device name for AP: " + DeviceName);
    } else if (savedSuffix.length() > 0) {
        // Use previously saved random suffix
        updateAPName(savedSuffix, false);
        apNameFromSerial = false;
        Serial.println("[WiFi] Using saved random suffix: " + savedSuffix);
    } else {
        // Generate new random suffix
        String newSuffix = generateRandomHexSuffix();
        updateAPName(newSuffix, false);
        apNameFromSerial = false;
        
        // Save to EPROM
        preferences.begin("RMS", false);
        preferences.putString("APSuffix", newSuffix);
        preferences.end();
        Serial.println("[WiFi] Generated new random suffix: " + newSuffix);
    }
}

/**
 * Start the WiFi Access Point and configure the captive portal web server.
 * 
 * CAPTIVE PORTAL STRATEGY (Rock-Solid):
 * ============================================================
 * Different OS detect captive portals by probing specific URLs:
 *   iOS/macOS:  GET /hotspot-detect.html  (expects non-"Success" body)
 *   Android:    GET /generate_204         (expects non-204 response)
 *   Windows:    GET /connecttest.txt      (expects non-"Microsoft" body)
 *               GET /ncsi.txt             (NCSI probe)
 *               GET /fwlink               (redirect target)
 *   Firefox:    GET /canonical.html       (expects non-canonical body)
 * 
 * The DNS server resolves ALL domains to 192.168.4.1, so these
 * probes arrive at our server. We respond with a lightweight
 * redirect page to force the captive portal popup to open.
 * 
 * CRITICAL FIX (Rev 9.4b): All literal % in HTML must be escaped as %%
 * when using send_P() with a processor function. Otherwise the template
 * engine eats chunks of HTML between % signs -> blank page!
 */

/**
 * Generate dynamic HTML for the captive portal landing page (Rev 10.18)
 * Shows current pressure, cycles, alarms, and datetime in a mobile-friendly layout
 */
String generateCaptivePortalHTML() {
    // Get current data
    float currentPressure = pressure;  // From global variable
    int currentCycles = cycles;        // From global variable
    int alarmCode = getCombinedFaultCode();  // Any ESP32 alarm active?
    String datetimeStr = getDateTimeString(); // Format: "YYYY-MM-DD HH:MM:SS"

    // Parse datetime string to separate date and time
    String dateStr = "---- -- --";
    String timeStr = "--:--:--";
    if (datetimeStr.length() >= 19) {  // "2026-02-12 14:30:45" = 19 chars
        dateStr = datetimeStr.substring(0, 10);  // "2026-02-12"
        timeStr = datetimeStr.substring(11, 19); // "14:30:45"
    }

    // Format pressure with 2 decimal places
    char pressureBuf[16];
    snprintf(pressureBuf, sizeof(pressureBuf), "%.2f", currentPressure);

    // Determine if alarm card should be red
    bool hasAlarms = (alarmCode > 0);
    String alarmCardClass = hasAlarms ? "alarm-active" : "alarm-normal";

    // Generate HTML with embedded CSS for mobile-first design
    String html = "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no'>"
        "<title>VST Green Machine</title>"
        "<style>"
        "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;margin:0;padding:0;background:#2563eb;color:#fff;min-height:100vh;display:flex;flex-direction:column;}"
        ".header{background:#1e40af;padding:16px;text-align:center;font-size:1.25rem;font-weight:700;}"
        ".datetime{position:absolute;top:16px;right:16px;text-align:right;color:#fff;}"
        ".datetime .date{font-size:0.875rem;opacity:0.9;margin-bottom:2px;}"
        ".datetime .time{font-size:1.125rem;font-weight:600;}"
        ".content{flex:1;display:flex;flex-direction:column;justify-content:center;align-items:center;padding:20px;text-align:center;}"
        ".alarm-card{background:#dc2626;border-radius:12px;padding:16px;margin-bottom:24px;max-width:280px;width:100%;box-shadow:0 4px 12px rgba(0,0,0,0.3);}"
        ".alarm-normal{background:#16a34a;}"
        ".status-text{font-size:1.5rem;font-weight:700;margin:8px 0;color:#fff;}"
        ".status-value{font-size:2rem;font-weight:900;margin:4px 0;color:#fff;text-shadow:0 2px 4px rgba(0,0,0,0.3);}"
        ".button-container{position:fixed;bottom:0;left:0;right:0;padding:16px;background:#1e40af;text-align:center;}"
        ".btn{background:#3b82f6;color:#fff;border:none;border-radius:8px;padding:14px 28px;font-size:1.1rem;font-weight:600;text-decoration:none;display:inline-block;min-width:200px;}"
        ".btn:active{background:#2563eb;}"
        "@media (max-width:480px){.content{padding:16px;}.alarm-card{max-width:100%;}.status-text{font-size:1.3rem;}.status-value{font-size:1.8rem;}}"
        "</style>"
        "</head><body>"
        "<div class='header'>VST GREEN MACHINE</div>"
        "<div class='datetime'>"
        "<div class='date'>" + dateStr + "</div>"
        "<div class='time'>" + timeStr + "</div>"
        "</div>"
        "<div class='content'>"
        "<div class='alarm-card " + alarmCardClass + "'>"
        "<div class='status-text'>UST PRESSURE:</div>"
        "<div class='status-value'>" + String(pressureBuf) + " IWC</div>"
        "</div>"
        "<div class='status-text'>RUN CYCLES:</div>"
        "<div class='status-value'>" + String(currentCycles) + "</div>"
        "</div>"
        "<div class='button-container'>"
        "<a href='http://192.168.4.1/' class='btn'>Advanced Controls</a>"
        "</div>"
        "</body></html>";

    return html;
}

void startConfigAP() {
    // Configure WiFi AP with explicit settings for reliability
    WiFi.mode(WIFI_AP);
    WiFi.softAP(apSSID);
    delay(100);  // Let AP stabilize before starting DNS
    
    // Start DNS server - resolve ALL domain lookups to our IP
    // This is what makes captive portal detection work
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    
    // ---- MAIN DASHBOARD PAGE ----
    // Uses send_P with template processor for %%DEVICENAME%% substitution
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", control_html, webProcessor);
    });
    
    // Set watchdog enabled/disabled endpoint (password protected)
    // Usage: GET /setwatchdog?enabled=1&pwd=gm2026
    server.on("/setwatchdog", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!request->hasParam("enabled")) {
            request->send(400, "text/plain", "Missing enabled parameter");
            return;
        }
        
        // Validate password before allowing watchdog changes
        if (!request->hasParam("pwd") || request->getParam("pwd")->value() != CONFIG_PASSWORD) {
            Serial.println("[Web] Watchdog change DENIED - wrong or missing password");
            request->send(403, "text/plain", "Invalid password");
            return;
        }
        
            int enabledVal = request->getParam("enabled")->value().toInt();
            watchdogEnabled = (enabledVal == 1);
            
            // Save to preferences/EPROM
            preferences.begin("RMS", false);
            preferences.putBool("watchdog", watchdogEnabled);
            preferences.end();
            
            Serial.print("Watchdog setting changed: ");
            Serial.println(watchdogEnabled ? "ENABLED" : "DISABLED");
        Serial.println("✓ Watchdog setting saved to EPROM (password verified)");
            
            // Reset watchdog state when toggled
            if (watchdogEnabled) {
                resetSerialWatchdog();
            }
            
            request->send(200, "text/plain", watchdogEnabled ? "Watchdog ENABLED" : "Watchdog DISABLED");
        });

    // Failsafe enable/disable endpoint
    // Usage: GET /setfailsafe?enabled=1&pwd=gm2026
    server.on("/setfailsafe", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!request->hasParam("enabled")) {
            request->send(400, "text/plain", "Missing enabled parameter");
            return;
        }

        // Validate password before allowing failsafe changes
        if (!request->hasParam("pwd") || request->getParam("pwd")->value() != CONFIG_PASSWORD) {
            Serial.println("[Web] Failsafe change DENIED - wrong or missing password");
            request->send(403, "text/plain", "Invalid password");
            return;
        }

            int enabledVal = request->getParam("enabled")->value().toInt();
            failsafeEnabled = (enabledVal == 1);

            // Save to preferences/EPROM
            preferences.begin("RMS", false);
            preferences.putBool("failsafe", failsafeEnabled);
            preferences.end();

            Serial.print("Failsafe setting changed: ");
            Serial.println(failsafeEnabled ? "ENABLED" : "DISABLED");
        Serial.println("✓ Failsafe setting saved to EPROM (password verified)");

            // If disabling failsafe while in failsafe mode, exit failsafe
            if (!failsafeEnabled && failsafeMode) {
                exitFailsafeMode();
                Serial.println("[FAILSAFE] Exited failsafe mode due to failsafe being disabled");
            }

            request->send(200, "text/plain", failsafeEnabled ? "Failsafe ENABLED" : "Failsafe DISABLED");
        });
    
    // REV 10.9: Set serial debug mode endpoint
    // Usage: GET /setdebug?enabled=1&pwd=1793 (enable verbose Serial Monitor output)
    //        GET /setdebug?enabled=0&pwd=1793 (disable — quiet mode, default)
    // Requires CONFIG_PASSWORD. Saved to EEPROM.
    // When ON: FAST-TX (5Hz), cellular refresh, relay refresh print to Serial Monitor
    // When OFF: Only essential messages (boot, errors, mode changes, watchdog events)
    server.on("/setdebug", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!request->hasParam("enabled")) {
            request->send(400, "text/plain", "Missing parameter: enabled");
            return;
        }
        
        if (!request->hasParam("pwd") || request->getParam("pwd")->value() != CONFIG_PASSWORD) {
            Serial.println("[Web] Debug mode change DENIED - wrong or missing password");
            request->send(403, "text/plain", "Invalid password");
            return;
        }
        
        int enabledVal = request->getParam("enabled")->value().toInt();
        serialDebugMode = (enabledVal == 1);
        
        // Save to preferences/EPROM
        preferences.begin("RMS", false);
        preferences.putBool("debugMode", serialDebugMode);
        preferences.end();
        
        Serial.print("Serial debug mode changed: ");
        Serial.println(serialDebugMode ? "ON (verbose)" : "OFF (quiet)");
        Serial.println("✓ Debug mode saved to EPROM (password verified)");
        
        request->send(200, "text/plain", serialDebugMode ? "Debug ON" : "Debug OFF");
    });
    
    // Set passthrough mode endpoint
    // Usage: GET /setpassthrough?enabled=1 (enter passthrough mode)
    // Uses preference-based boot: sets flag, restarts, runs minimal passthrough
    // Auto-returns to normal after 60 min timeout or early exit via MCP GPA5
    server.on("/setpassthrough", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("enabled")) {
            int enabledVal = request->getParam("enabled")->value().toInt();
            
            if (enabledVal == 1) {
                request->send(200, "text/plain", 
                    "PASSTHROUGH MODE REQUESTED\n"
                    "ESP32 will restart in minimal passthrough mode.\n"
                    "Auto-returns to normal after 60 min.");
                delay(500);
                enterPassthroughMode();  // Sets preferences and restarts
            } else {
                request->send(200, "text/plain", "Restarting to normal mode...");
                delay(500);
                ESP.restart();
            }
        } else {
            String status = passthroughMode ? "ACTIVE" : "INACTIVE";
            request->send(200, "text/plain", "Passthrough: " + status);
        }
    });
    
    // Set device name endpoint (manual configuration via web portal, password protected)
    // Usage: GET /setdevicename?name=RND-0007&pwd=gm2026
    // This allows manual entry before Linux device is connected
    server.on("/setdevicename", HTTP_GET, [](AsyncWebServerRequest *request){
        if (!request->hasParam("name")) {
            request->send(400, "text/plain", "Missing name parameter");
            return;
        }
        
        // Validate password before allowing device name changes
        if (!request->hasParam("pwd") || request->getParam("pwd")->value() != CONFIG_PASSWORD) {
            Serial.println("[Web] Device name change DENIED - wrong or missing password");
            request->send(403, "text/plain", "Invalid password");
            return;
        }
        
            String newName = request->getParam("name")->value();
            
            // Validate name (alphanumeric, dashes, 3-20 chars)
            if (newName.length() < 3 || newName.length() > 20) {
                request->send(400, "text/plain", "Name must be 3-20 characters");
                return;
            }
            
            // Update device name
            DeviceName = newName;
            strncpy(deviceName, newName.c_str(), sizeof(deviceName) - 1);
            deviceName[sizeof(deviceName) - 1] = '\0';
            
            // Save to preferences/EPROM
            preferences.begin("RMS", false);
            preferences.putString("DeviceName", DeviceName);
            preferences.putString("APSuffix", DeviceName);  // Also update AP suffix
            preferences.end();
            
            // Update WiFi AP name
            updateAPName(DeviceName, true);  // Restart AP with new name
            apNameFromSerial = false;  // Mark as manually set
            
            Serial.println("[Web] Device name manually set to: " + DeviceName);
        Serial.println("✓ Device name saved to EPROM (password verified)");
            
            request->send(200, "text/plain", "Device name set to: " + DeviceName + "\nWiFi AP updated to: " + String(apSSID));
    });
    
    // API endpoint for AJAX status updates (returns JSON for diagnostic dashboard)
    // READ-ONLY: No relay control, no mode changes - diagnostic data only
    // Usage: GET /api/status returns JSON with all diagnostic fields
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
        float celsius = temperatureRead();
        float fahrenheit = (celsius * 9.0 / 5.0) + 32;
        
        // Determine if serial data is actively being received
        // Consider "active" if data received within last 60 seconds
        bool serialActive = (millis() - lastSerialDataTime < 60000) && (lastSerialDataTime > 0);
        
        // Calculate uptime as human-readable string
        unsigned long uptimeMs = millis() - systemStartTime;
        unsigned long days = uptimeMs / 86400000;
        unsigned long hours = (uptimeMs % 86400000) / 3600000;
        unsigned long minutes = (uptimeMs % 3600000) / 60000;
        char uptimeStr[32];
        if (days > 0) {
            snprintf(uptimeStr, sizeof(uptimeStr), "%lud %luh %lum", days, hours, minutes);
        } else {
            snprintf(uptimeStr, sizeof(uptimeStr), "%luh %lum", hours, minutes);
        }
        
        // REV 10.10: Build JSON with snprintf into stack buffer — ZERO heap allocation.
        // Previously used ~60+ String += operations, each causing heap reallocation.
        // Over hours of web dashboard polling, this fragmented the heap and could
        // contribute to memory exhaustion and stalls.
        
        // Pre-compute alarm states (avoid complex expressions inside snprintf)
        const ProfileConfig* activeProfile = profileManager.getActiveProfile();
        bool alarmLowPress = (adcPressure < activeProfile->lowPressThreshold && currentRelayMode > 0);
        bool alarmHighPress = (adcPressure > activeProfile->highPressThreshold && currentRelayMode > 0);
        bool alarmZeroPress = (activeProfile->hasZeroPressAlarm && fabs(adcPressure) < 0.15 && currentRelayMode > 0);
        bool alarmLowCurrent = (adcCurrent > 0 && adcCurrent < LOW_CURRENT_THRESHOLD && currentRelayMode > 0);
        bool alarmHighCurrent = (adcCurrent > HIGH_CURRENT_THRESHOLD);
        int combinedFault = faults + getCombinedFaultCode();
        unsigned long testElapsedSec = testRunning ? (millis() - testStartTime) / 1000 : 0;
        // REV 10.13: Don't call lteConnected() during cellular init — it sends an AT
        // command that can collide with the state machine on the same modem UART,
        // causing the web server async handler to block and making the portal unresponsive.
        bool lteOk = cellularInitComplete ? lteConnected() : false;
        String dtStr = getDateTimeString();
        String profName = profileManager.getActiveProfileName();
        
        // Safe string accessors (avoid sending null pointers to snprintf)
        const char* rsrpC = modemRSRP.length() > 0 ? modemRSRP.c_str() : "--";
        const char* rsrqC = modemRSRQ.length() > 0 ? modemRSRQ.c_str() : "--";
        const char* operC = modemNetName.length() > 0 ? modemNetName.c_str() : "--";
        const char* bandC = modemBand.length() > 0 ? modemBand.c_str() : "--";
        
        // Build JSON into stack-allocated buffer (no heap allocation)
        static char json[2048];
        int pos = 0;
        
        // Serial & device
        pos += snprintf(json + pos, sizeof(json) - pos,
            "{\"serialActive\":%s,\"deviceName\":\"%s\","
            "\"pressure\":\"%.2f\",\"current\":\"%.2f\","
            "\"mode\":%d,\"cycles\":\"%d\",\"fault\":\"%d\","
            "\"profile\":\"%s\",\"failsafe\":%s,",
            serialActive ? "true" : "false", DeviceName.c_str(),
            adcPressure, adcCurrent,
            currentRelayMode, cycles, combinedFault,
            profName.c_str(), failsafeMode ? "true" : "false");
        
        // Cellular modem (REV 10.13: added cellularReady for web UI connecting state)
        pos += snprintf(json + pos, sizeof(json) - pos,
            "\"lteConnected\":%s,\"cellularReady\":%s,"
            "\"rsrp\":\"%s\",\"rsrq\":\"%s\","
            "\"operator\":\"%s\",\"band\":\"%s\","
            "\"mcc\":%u,\"mnc\":%u,\"cellId\":%u,\"tac\":%u,"
            "\"blueCherryConnected\":%s,\"modemStatus\":\"%s\","
            "\"imei\":\"%s\",",
            lteOk ? "true" : "false",
            cellularInitComplete ? "true" : "false",
            rsrpC, rsrqC,
            operC, bandC,
            modemCC, modemNC, modemCID, modemTAC,
            blueCherryConnected ? "true" : "false", Modem_Status.c_str(),
            imei.c_str());
        
        // IO Board status
        pos += snprintf(json + pos, sizeof(json) - pos,
            "\"version\":\"%s\",\"temperature\":\"%.1f\","
            "\"sdCardOK\":%s,"
            "\"overfillAlarm\":%s,\"gpio38Raw\":%d,"
            "\"overfillLowCnt\":%d,\"overfillHighCnt\":%d,"
            "\"overfillValidated\":%s,"
            "\"relayCR1\":%s,\"relayCR2\":%s,\"relayCR5\":%s,",
            ver.c_str(), fahrenheit,
            isSDCardOK() ? "true" : "false",
            overfillAlarmActive ? "true" : "false", digitalRead(ESP_OVERFILL),
            (int)overfillLowCount, (int)overfillHighCount,
            overfillValidationComplete ? "true" : "false",
            digitalRead(CR1) ? "true" : "false",
            digitalRead(CR2) ? "true" : "false",
            digitalRead(CR5) ? "true" : "false");
        
        // ADC debug — REV 10.10: Added adcDataStale and adcLastGoodMs fields
        // adcDataStale: true when no successful read for > 60 seconds (FAST-TX sends -99.9)
        // adcLastGoodMs: milliseconds since last successful ADC read (0 if never read)
        unsigned long adcLastGoodAgo = (lastSuccessfulAdcRead > 0) ? (millis() - lastSuccessfulAdcRead) : 0;
        pos += snprintf(json + pos, sizeof(json) - pos,
            "\"adcRawPressure\":%d,\"adcRawCurrent\":%d,"
            "\"adcZeroPressure\":%.2f,\"adcInitialized\":%s,"
            "\"adcErrors\":%lu,\"adcOutliers\":%lu,"
            "\"adcDataStale\":%s,\"adcLastGoodMs\":%lu,",
            (int)adcRawPressure, (int)adcRawCurrent,
            adcZeroPressure, adcInitialized ? "true" : "false",
            adcTotalErrors, adcOutlierCount,
            adcDataStale ? "true" : "false", adcLastGoodAgo);
        
        // Alarms & watchdog & debug mode
        pos += snprintf(json + pos, sizeof(json) - pos,
            "\"alarmLowPress\":%s,\"alarmHighPress\":%s,"
            "\"alarmZeroPress\":%s,\"alarmVarPress\":false,"
            "\"alarmLowCurrent\":%s,\"alarmHighCurrent\":%s,"
            "\"watchdogTriggered\":%s,\"watchdogEnabled\":%s,"
            "\"failsafeEnabled\":%s,\"serialDebugMode\":%s,",
            alarmLowPress ? "true" : "false", alarmHighPress ? "true" : "false",
            alarmZeroPress ? "true" : "false",
            alarmLowCurrent ? "true" : "false", alarmHighCurrent ? "true" : "false",
            watchdogTriggered ? "true" : "false", watchdogEnabled ? "true" : "false",
            failsafeEnabled ? "true" : "false", serialDebugMode ? "true" : "false");
        
        // Test state, datetime, uptime, mac
        // Include multi-step test progress (step/total) when running functionality test
        pos += snprintf(json + pos, sizeof(json) - pos,
            "\"testRunning\":%s,\"testType\":\"%s\","
            "\"testElapsed\":%lu,"
            "\"testMultiStep\":%s,\"testStep\":%d,\"testStepTotal\":%d,"
            "\"datetime\":\"%s\",\"uptime\":\"%s\","
            "\"macAddress\":\"%s\","
            "\"cborPayloadHistory\":%s}",
            testRunning ? "true" : "false", activeTestType.c_str(),
            testElapsedSec,
            webTestMultiStep ? "true" : "false", webTestCycleStep, webTestCycleLength,
            dtStr.c_str(), uptimeStr,
            macStr.c_str(),
            buildCborPayloadHistoryJson().c_str());
        
        request->send(200, "application/json", json);
    });
    
    // ---- COMMAND ENDPOINT (Rev 10) ----
    // POST /api/command - handles button actions from web interface
    // Body: {"command":"start_cycle","value":"run"}
    server.on("/api/command", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
        // Parse the POST body as JSON
        String body = "";
        for (size_t i = 0; i < len; i++) body += (char)data[i];
        
        String cmd = getJsonValue(body.c_str(), "command");
        String val = getJsonValue(body.c_str(), "value");
        
        Serial.printf("[WEB CMD] command=%s value=%s\r\n", cmd.c_str(), val.c_str());
        
        if (cmd == "start_cycle") {
            if (failsafeMode) {
                // =========================================================
                // FAILSAFE MODE: ESP32 runs cycles autonomously (Linux down)
                // Use the local failsafe cycle engine directly.
                // =========================================================
                if (val == "run") startFailsafeCycle(0);
                else if (val == "manual_purge") startFailsafeCycle(1);
                else if (val == "clean") startFailsafeCycle(2);
            } else if (val == "manual_purge") {
                // =========================================================
                // REV 10.13: MANUAL PURGE — ESP32 takes sole relay control
                // Manual purge is a maintenance operation started from the
                // web portal. Sets relays to purge mode (mode 2: motor on +
                // purge valve open). No auto-timeout — user presses Stop.
                // ESP32 blocks Linux mode commands while active.
                // =========================================================
                setRelaysForMode(2);  // Purge mode (motor on + purge valve)
                activeTestType = "manual_purge";
                testStartTime = millis();
                testRunning = true;
                
                Serial.printf("[WEB CMD] ===== WEB PORTAL MANUAL PURGE STARTED =====\r\n");
                Serial.printf("[WEB CMD]   Relay mode: 2 (Purge)  |  Duration: manual (user stops)\r\n");
                Serial.printf("[WEB CMD]   ESP32 owns relays — ALL Linux mode commands BLOCKED\r\n");
                
                // Forward to Linux for logging/awareness
                Serial1.println("{\"command\":\"start_cycle\",\"type\":\"manual_purge\"}");
            } else if (val == "clean") {
                // =========================================================
                // REV 10.11: CLEAN CANISTER — ESP32 takes sole relay control
                // Clean canister is a maintenance operation started from the
                // web portal. Like leak/func/eff tests, the ESP32 owns the
                // relays for the duration (2 hours) and blocks Linux mode
                // commands. Uses the same testRunning mechanism as start_test.
                // =========================================================
                setRelaysForMode(1);  // Run mode (motor on for cleaning)
                activeTestType = "clean";
                testStartTime = millis();
                testRunning = true;
                
                Serial.printf("[WEB CMD] ===== WEB PORTAL CLEAN CANISTER STARTED =====\r\n");
                Serial.printf("[WEB CMD]   Relay mode: 1 (Run)  |  Duration: 7200 sec (2 hours)\r\n");
                Serial.printf("[WEB CMD]   ESP32 owns relays — ALL Linux mode commands BLOCKED\r\n");
                
                // Forward to Linux for logging/awareness
                Serial1.println("{\"command\":\"start_cycle\",\"type\":\"clean\"}");
            } else {
                // =========================================================
                // NORMAL MODE: run — Linux controls relays.
                // Forward the start_cycle command to Linux via Serial1.
                // Linux's IOManager runs the cycle with proper alarm monitoring,
                // DB logging, pause/resume, etc. The ESP32 does NOT take over
                // relay control for normal run cycles.
                // =========================================================
                char cmdBuf[128];
                snprintf(cmdBuf, sizeof(cmdBuf),
                         "{\"command\":\"start_cycle\",\"type\":\"%s\"}", val.c_str());
                Serial1.println(cmdBuf);
                Serial.printf("[WEB CMD] Forwarded to Linux: %s\r\n", cmdBuf);
            }
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "stop_cycle") {
            if (failsafeMode) {
                stopFailsafeCycle("Web command");
            } else {
                // Forward stop to Linux — Linux manages its own cycle processes
                Serial1.println("{\"command\":\"stop_cycle\"}");
                Serial.println("[WEB CMD] Forwarded stop_cycle to Linux");
            }
            // REV 10.11: If a web portal test was running, release relay control
            // Set relays to idle and clear test state — Linux resumes relay control
            if (testRunning) {
                Serial.printf("[WEB CMD] ===== WEB PORTAL TEST STOPPED (manual) =====\r\n");
                Serial.printf("[WEB CMD]   Was: %s  |  Elapsed: %lu sec\r\n",
                              activeTestType.c_str(), (millis() - testStartTime) / 1000);
                Serial.printf("[WEB CMD]   Relays → idle (mode 0)  |  Linux resumes relay control\r\n");
                setRelaysForMode(0);  // Idle — all relays OFF
                testRunning = false;
                activeTestType = "";
                webTestMultiStep = false;  // Clear multi-step state
            }
            request->send(200, "application/json", "{\"ok\":true}");
        }
        // =========================================================
        // START TEST — ESP32 takes SOLE control of relays (Rev 10.11)
        //
        // Single-mode tests (leak, eff): Set one relay mode for the duration.
        // Multi-step tests (func): Step through a CycleStep sequence
        //   driven by runWebPortalTestCycleLogic() in loop().
        //
        // The ESP32 blocks ALL Linux mode commands until the test
        // auto-completes or user presses Stop.
        // =========================================================
        else if (cmd == "start_test") {
            uint8_t testMode = getTestRelayMode(val);
            unsigned long testDur = getTestDurationSeconds(val);
            
            if (testMode == 0 || testDur == 0) {
                Serial.printf("[WEB CMD] Unknown test type: %s\r\n", val.c_str());
                request->send(400, "application/json", "{\"ok\":false,\"error\":\"Unknown test type\"}");
            } else {
                // Track test state for timer, auto-stop, and Linux mode blocking
                activeTestType = val;
                testStartTime = millis();
                testRunning = true;
                
                // Check if this is a multi-step test (functionality test)
                if (isMultiStepTest(val)) {
                    // MULTI-STEP: Functionality test = 10x (Run 60s → Purge 60s)
                    // Set up the cycle sequence — runWebPortalTestCycleLogic() in loop()
                    // will advance through the steps automatically.
                    webTestMultiStep = true;
                    webTestCycleSequence = FUNCTIONALITY_TEST_CYCLE;
                    webTestCycleLength = FUNCTIONALITY_TEST_CYCLE_STEPS;
                    webTestCycleStep = 0;
                    webTestCycleStepTimer = millis();
                    
                    // Start the first step
                    setRelaysForMode(webTestCycleSequence[0].mode);
                    
                    Serial.printf("[WEB CMD] ===== WEB PORTAL MULTI-STEP TEST STARTED =====\r\n");
                    Serial.printf("[WEB CMD]   Type: %s  |  Steps: %d  |  Total duration: %lu sec\r\n",
                                  val.c_str(), webTestCycleLength, testDur);
                    Serial.printf("[WEB CMD]   Step 0: mode %d for %d sec (Run)\r\n",
                                  webTestCycleSequence[0].mode,
                                  webTestCycleSequence[0].durationSeconds);
                    Serial.printf("[WEB CMD]   Sequence: 10x (Run 60s → Purge 60s)\r\n");
                } else {
                    // SINGLE-MODE: Leak, Eff — hold one relay mode for the duration
                    webTestMultiStep = false;
                    setRelaysForMode(testMode);
                    
                    Serial.printf("[WEB CMD] ===== WEB PORTAL TEST STARTED =====\r\n");
                    Serial.printf("[WEB CMD]   Type: %s  |  Relay mode: %d  |  Duration: %lu sec\r\n",
                                  val.c_str(), testMode, testDur);
                }
                
                Serial.printf("[WEB CMD]   ESP32 owns relays — ALL Linux mode commands BLOCKED\r\n");
                
                // Forward to Linux for logging/awareness (Linux does NOT control relays)
                char cmdBuf[128];
                snprintf(cmdBuf, sizeof(cmdBuf),
                         "{\"command\":\"start_test\",\"type\":\"%s\"}", val.c_str());
                Serial1.println(cmdBuf);
                
                request->send(200, "application/json", "{\"ok\":true}");
            }
        }
        // STOP TEST — ESP32 releases relay control, returns to idle
        // REV 10.18: Uses executeRemoteCommand() for notification to Python,
        // but keeps web-specific cleanup (relay idle, test state clear) here.
        else if (cmd == "stop_test") {
            if (testRunning) {
                Serial.printf("[WEB CMD] ===== WEB PORTAL TEST STOPPED (manual) =====\r\n");
                Serial.printf("[WEB CMD]   Was: %s  |  Elapsed: %lu sec\r\n",
                              activeTestType.c_str(), (millis() - testStartTime) / 1000);
                if (webTestMultiStep) {
                    Serial.printf("[WEB CMD]   Was on step %d of %d\r\n",
                                  webTestCycleStep, webTestCycleLength);
                }
                Serial.printf("[WEB CMD]   Relays → idle (mode 0)  |  Linux resumes relay control\r\n");
                setRelaysForMode(0);  // Idle — all relays OFF
                testRunning = false;
                activeTestType = "";
                webTestMultiStep = false;  // Clear multi-step state
            }
            // REV 10.18: Send correct command name "stop_test" (was sending "stop_cycle")
            notifyPythonOfCommand("stop_test", "");
            request->send(200, "application/json", "{\"ok\":true}");
        }
        // =========================================================
        // SET PROFILE — Password-protected (PW: 1793)
        // The web UI sends {command:"set_profile", value:"CS8", password:"1793"}
        // We validate the password server-side before applying the change.
        // =========================================================
        else if (cmd == "set_profile") {
            String pw = getJsonValue(body.c_str(), "password");
            if (pw != "1793") {
                // Wrong or missing password — reject with 403 Forbidden
                Serial.printf("[WEB CMD] set_profile REJECTED - wrong password\r\n");
                request->send(403, "application/json", "{\"ok\":false,\"error\":\"Wrong password\"}");
            }
            else if (profileManager.setActiveProfile(val.c_str())) {
                Serial.printf("[WEB CMD] Profile changed to %s (password OK)\r\n", val.c_str());
                request->send(200, "application/json", "{\"ok\":true}");
            } else {
                request->send(400, "application/json", "{\"ok\":false,\"error\":\"Unknown profile\"}");
            }
        }
        // =========================================================
        // REV 10.18: Unified command dispatch for overfill, alarms,
        // failsafe, relay control, and calibration.
        // All of these route through executeRemoteCommand() so the
        // same logic runs regardless of source (Web, BlueCherry, Serial).
        // =========================================================
        else if (cmd == "overfill_override") {
            executeRemoteCommand(cmd, val);
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "clear_press_alarm" || cmd == "clear_motor_alarm") {
            executeRemoteCommand(cmd, val);
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "enable_failsafe") {
            executeRemoteCommand(cmd, val);
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "exit_failsafe") {
            executeRemoteCommand(cmd, val);
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "cr0" || cmd == "cr1" || cmd == "cr2" || cmd == "cr5") {
            executeRemoteCommand(cmd, val);
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "exit_manual") {
            executeRemoteCommand(cmd, val);
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "calibrate_pressure") {
            executeRemoteCommand(cmd, val);
            char resp[128];
            snprintf(resp, sizeof(resp),
                     "{\"ok\":true,\"adcZero\":%.2f}", adcZeroPressure);
            request->send(200, "application/json", resp);
        }
        // =========================================================
        // SET MANUAL ADC ZERO — User-supplied ADC zero point value
        // Validates against MANUAL_ADC_MIN/MAX (broader than CAL_RANGE for flexibility).
        // Returns the new adcZero value on success, or error on invalid range.
        // =========================================================
        else if (cmd == "set_manual_adc_zero") {
            float newAdcZero = val.toFloat();
            const float MANUAL_ADC_MIN = 500.0;   // Allow broader range for manual entry
            const float MANUAL_ADC_MAX = 2000.0;  // ADS1015 12-bit: 0-4095, pressure ~500-2000
            if (newAdcZero >= MANUAL_ADC_MIN && newAdcZero <= MANUAL_ADC_MAX) {
                executeRemoteCommand(cmd, val);
                char resp[128];
                snprintf(resp, sizeof(resp),
                         "{\"command\":\"set_manual_adc_zero\",\"adcZero\":%.2f,\"success\":true}",
                         adcZeroPressure);
                request->send(200, "application/json", resp);
            } else {
                Serial.printf("[WEB CMD] Manual ADC zero %.2f outside valid range (%.0f-%.0f)\r\n",
                             newAdcZero, MANUAL_ADC_MIN, MANUAL_ADC_MAX);
                char resp[192];
                snprintf(resp, sizeof(resp),
                         "{\"command\":\"set_manual_adc_zero\",\"success\":false,\"error\":\"ADC value outside valid range (%.0f-%.0f)\"}",
                         MANUAL_ADC_MIN, MANUAL_ADC_MAX);
                request->send(400, "application/json", resp);
            }
        }
        else if (cmd == "toggle_relay") {
            // Convert web relay names (CR1, CR2, CR5) to ESP32 GPIO pins and toggle them
            int pin = -1;
            if (val == "CR1") pin = CR1;
            else if (val == "CR2") pin = CR2;
            else if (val == "CR5") pin = CR5;

            if (pin != -1) {
                // Read current state and toggle it
                int currentState = digitalRead(pin);
                int newState = (currentState == HIGH) ? LOW : HIGH;

                // Set the new state directly on GPIO
                digitalWrite(pin, newState);

                // Also notify Python and set manual override flag
                manualRelayOverride = true;
                String relayName = val.substring(2);
                relayName.toLowerCase();
                notifyPythonOfCommand(relayName.c_str(), String(newState).c_str());
                Serial.printf("[WEB CMD] %s toggled to %d\r\n", val.c_str(), newState);
            }
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "restart") {
            request->send(200, "application/json", "{\"ok\":true}");
            delay(500);
            ESP.restart();
        }
        else {
            request->send(400, "application/json", "{\"ok\":false,\"error\":\"Unknown command\"}");
        }
    });
    
    // ---- CAPTIVE PORTAL DETECTION HANDLERS ----
    // Each OS/browser probes a specific URL to detect captive portals.
    // We serve a lightweight redirect page that works inside the captive portal popup.
    // Using inline HTML (no processor) to keep it tiny and fast.
    
    // Lightweight captive portal redirect page (used by all CP handlers below)
    // Served inline - no template processor needed, no %% escaping issues
    // Dynamic status display for captive portal (Rev 10.18)
    // Replaced static CP_PAGE with generateCaptivePortalHTML() function

    // Apple iOS / macOS captive portal detection
    // iOS probes http://captive.apple.com/hotspot-detect.html
    // If response is NOT "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY</HTML>"
    // then iOS opens the captive portal popup. We return our dynamic status page.
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });
    
    // Android captive portal detection
    // Android probes http://connectivitycheck.gstatic.com/generate_204
    // Expects HTTP 204 No Content. Any other response triggers captive portal popup.
    // We return 200 with our dynamic status page to force the popup open.
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });
    
    // Windows NCSI (Network Connectivity Status Indicator) probes
    server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });
    server.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });
    server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });
    server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });

    // Firefox captive portal detection
    server.on("/canonical.html", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });
    
    // Additional common captive portal probe URLs
    server.on("/success.txt", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });
    server.on("/chat", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = generateCaptivePortalHTML();
        request->send(200, "text/html", html);
    });
    
    // ---- CATCH-ALL HANDLER ----
    // Any URL not explicitly handled above gets redirected to the dashboard.
    // This catches all OS-specific probe URLs we might have missed.
    server.onNotFound([](AsyncWebServerRequest *request){
        // Use 302 redirect for standard HTTP requests
        if (request->method() == HTTP_GET) {
            request->redirect("http://192.168.4.1/");
        } else {
            request->send(200);  // Accept other methods silently
        }
    });
    
    server.begin();
    Serial.println("[WiFi] Web server started - captive portal active");
    Serial.print("[WiFi] AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.printf("[WiFi] AP SSID: %s\n", apSSID);
    delay(100);
}

void getHTTPinfo() {
    if (modem.httpConfigProfile(HTTP_PROFILE, "167.172.15.241", 5687, TLS_PROFILE)) {
        Serial.println("Successfully configured the http profile");
    } else {
        Serial.println("Failed to configure HTTP profile");
        return;
    }
    static char url[128];
    static char ctbuf[128];
    snprintf(url, sizeof(url), "lookup?mac=%s", macStr.c_str());
    if (modem.httpQuery(HTTP_PROFILE, url, WALTER_MODEM_HTTP_QUERY_CMD_GET, ctbuf, sizeof(ctbuf), NULL, &rsp)) {
        Serial.println("Request successful");
    } else {
        Serial.println("http query failed");
        delay(1000);
        ESP.restart();
        return;
    }
    bool responseReceived = false;
    unsigned long startTime = millis();
    const unsigned long timeout = 10000;
    while (!responseReceived && (millis() - startTime < timeout)) {
        while (modem.httpDidRing(HTTP_PROFILE, incomingBuf, sizeof(incomingBuf), &rsp)) {
            Serial.println("Webserver response received");
            Serial.printf("[%s]\r\n", incomingBuf);
            if (rsp.data.httpResponse.httpStatus == 200) {
                char* idStart = strstr((char*)incomingBuf, "\"id\":\"");
                char* timingStart = strstr((char*)incomingBuf, "\"timing\":");
                if (idStart && timingStart) {
                    idStart += 6;
                    char idBuf[20];
                    int i = 0;
                    while (idStart[i] != '"' && i < sizeof(idBuf) - 1) {
                        idBuf[i] = idStart[i];
                        i++;
                    }
                    idBuf[i] = '\0';
                    timingStart += 9;
                    int timing = atoi(timingStart);
                    String timingString = String(timing);
                    timing *= 1000;
                    String newId = String(idBuf);
                    // Display server device ID (informational only — does NOT overwrite ESP32's stored name)
                    // The ESP32 device name is set exclusively via the web portal.
                    if (DeviceName != newId) {
                        Serial.println("[HTTP] Server device ID: '" + newId + 
                                       "' (ESP32 name: '" + DeviceName + "' — NOT overwriting)");
                    }
                    if (sendInterval != timing) {
                        sendInterval = timing;
                        preferences.begin("RMS", false);
                        preferences.putString("sendFrequency", timingString);
                        preferences.end();
                    }
                }
            }
            responseReceived = true;
        }
        if (!responseReceived) delay(100);
    }
    if (!responseReceived) Serial.println("Timeout waiting for HTTP response");
}

// Removed extractDataBeyondComma3() - no longer needed with JSON parsing

bool initModemTime() {
    while (!modem.getClock(&rsp)) {
        if (rsp.result == WALTER_MODEM_STATE_OK) {
            currentTimestamp = rsp.data.clock.epochTime;
            lastMillis = millis();
            modemTimeFresh = true;  // REV 10.5: Mark time as fresh from modem
            Serial.print("Raw modem timestamp (UTC): ");
            Serial.println(rsp.data.clock.epochTime);
            return true;
        } else {
            Serial.println("Invalid timestamp from modem!");
            return false;
        }
    }
}

void updateTimestamp() {
    uint32_t nowMillis = millis();
    if (nowMillis < lastMillis) {
        currentTimestamp += (0xFFFFFFFF - lastMillis + nowMillis) / 1000;
    } else {
        currentTimestamp += (nowMillis - lastMillis) / 1000;
    }
    lastMillis = nowMillis;
}

/**
 * Check if the current UTC time falls within the firmware check schedule.
 * 
 * SCHEDULE (Eastern Standard Time, UTC-5):
 *   - First check at 7:00 AM EST (12:00 UTC)
 *   - Every 15 minutes: 7:00, 7:15, 7:30, 7:45, 8:00, ... 12:45 PM
 *   - Last check at 1:00 PM EST (18:00 UTC)
 *   - No checks outside this window
 * 
 * Returns true if we are inside a 15-minute check slot that hasn't been
 * serviced yet. Uses firmwareCheckDoneThisSlot to prevent duplicate checks
 * within the same slot.
 * 
 * @return true if a firmware check should run now
 * 
 * Usage: Called every loop iteration; only returns true once per 15-min slot
 *   if (isFirmwareCheckScheduled()) { checkFirmwareUpdate(); initModemTime(); }
 */
bool isFirmwareCheckScheduled() {
    // FAST POLLING MODE: Check every 15 seconds when active
    if (fastPollingActive) {
        // Check if fast polling has timed out (60 minutes max)
        if (millis() - fastPollingStartTime >= FAST_POLLING_TIMEOUT) {
            Serial.println("[FAST POLL] Timeout reached - disabling fast polling");
            fastPollingActive = false;
            return false;
        }

        // Use a simple time-based check instead of schedule slots
        static unsigned long lastFastPollTime = 0;
        const unsigned long FAST_POLL_INTERVAL = 15 * 1000; // 15 seconds

        if (millis() - lastFastPollTime >= FAST_POLL_INTERVAL) {
            lastFastPollTime = millis();
            Serial.println("[FAST POLL] Checking for BlueCherry messages...");
            return true;
        }
        return false;
    }

    // NORMAL SCHEDULED MODE: Need valid time from modem (currentTimestamp > 0 means modem time was set)
    if (currentTimestamp <= 0) return false;

    // Convert UTC epoch to broken-down time
    time_t rawTime = (time_t)currentTimestamp;
    struct tm *utcTime = gmtime(&rawTime);

    // Convert UTC hour/minute to EST (UTC - 5 hours)
    // Handle day wraparound (e.g., UTC 03:00 = EST 22:00 previous day)
    int estHour = utcTime->tm_hour - 5;
    int estMinute = utcTime->tm_min;
    if (estHour < 0) estHour += 24;

    // Calculate total minutes since midnight EST for easy comparison
    int estTotalMinutes = estHour * 60 + estMinute;

    // Schedule window: 7:00 AM EST (420 min) to 1:00 PM EST (780 min)
    const int SCHEDULE_START = 7 * 60;       // 7:00 AM = 420 minutes
    const int SCHEDULE_END   = 13 * 60;      // 1:00 PM = 780 minutes
    const int CHECK_INTERVAL = 15;           // Every 15 minutes

    // Outside the schedule window? No check needed.
    if (estTotalMinutes < SCHEDULE_START || estTotalMinutes > SCHEDULE_END) {
        firmwareCheckDoneThisSlot = false;  // Reset for next day's window
        return false;
    }

    // Calculate which 15-minute slot we're in (0 = 7:00, 1 = 7:15, 2 = 7:30, ...)
    int currentSlot = (estTotalMinutes - SCHEDULE_START) / CHECK_INTERVAL;

    // We're in a valid slot. Have we already checked this slot?
    // Use a static to track which slot was last serviced
    static int lastCheckedSlot = -1;

    if (currentSlot == lastCheckedSlot && firmwareCheckDoneThisSlot) {
        return false;  // Already checked this slot
    }

    // New slot! Mark it and return true
    lastCheckedSlot = currentSlot;
    firmwareCheckDoneThisSlot = true;

    Serial.printf("[SCHEDULE] Firmware check triggered at %02d:%02d EST (slot %d of %d)\r\n",
                  estHour, estMinute, currentSlot, (SCHEDULE_END - SCHEDULE_START) / CHECK_INTERVAL);

    return true;
}

String formatTimestamp(int64_t seconds) {
    time_t rawTime = (time_t)seconds;
    struct tm *timeInfo = gmtime(&rawTime);
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
             timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday,
             timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
    return String(buffer);
}

/**
 * Check if SD card is operational
 * Returns true if SD card is mounted and accessible, false otherwise
 * 
 * Usage example:
 *   if (isSDCardOK()) { ... }
 */
bool isSDCardOK() {
    // Check if SD card type is valid (not CARD_NONE)
    return (SD.cardType() != CARD_NONE);
}

/**
 * Send cellular status to Python device via Serial1 (RS-232).
 * 
 * REV 10.9: Sent every 10 seconds on a TIMER (not freshness-gated).
 * Previously (Rev 10.5): only sent when modemDataFresh/modemTimeFresh flags
 * were set. Problem: if modem queries fail silently, the cellular packet was
 * NEVER sent, leaving the Linux device with no signal/LTE info at all.
 * 
 * Now we always send cached values every 10 seconds so the Linux device
 * always has the latest known cellular state.
 * 
 * REMOVED from this packet (Rev 10.9):
 *   - "datetime" - sent separately via sendDateTimeToSerial() only when fresh
 *     from modem, avoiding stale/frozen timestamps.
 *   - "failsafe" - already included in the 5Hz fast sensor packet (Rev 10.8).
 *
 * Python modem.py parses each field with 'if "field" in data' - fully
 * compatible with this packet containing no datetime or failsafe fields.
 *
 * Packet format (sent every 10s on Serial1 to Linux):
 *   {"passthrough":0,"lte":1,"rsrp":"-87.2","rsrq":"-10.8",
 *    "operator":"AT&T","band":"17","mcc":310,"mnc":410,
 *    "cellId":20878097,"tac":10519,"profile":"CS8",
 *    "imei":"351234567890123","imsi":"310410123456789",
 *    "iccid":"8901260882310000000","technology":"LTE-M"}
 * 
 * Usage example:
 *   sendCellularStatusToSerial();  // Called every 10 seconds from loop()
 *
 * REV_HISTORY: Previously sendStatusToSerial() -- renamed and split in Rev 10.9
 *              Rev 10.9: Added imei, imsi, iccid, technology fields
 */
void sendCellularStatusToSerial() {
    // Check LTE connection status
    int lteStatus = lteConnected() ? 1 : 0;
    
    // Use cached signal values -- may be stale if modem queries are failing,
    // but sending stale values is better than sending nothing.
    String rsrpStr = modemRSRP.length() > 0 ? modemRSRP : "--";
    String rsrqStr = modemRSRQ.length() > 0 ? modemRSRQ : "--";
    String operatorStr = modemNetName.length() > 0 ? modemNetName : "--";
    String bandStr = modemBand.length() > 0 ? modemBand : "--";
    
    // REV 10.9: Cellular-only JSON -- no datetime, no failsafe.
    // datetime: sent separately via sendDateTimeToSerial() only when fresh.
    // failsafe: already in the 5Hz fast sensor packet (Rev 10.8).
    // REV 10.9: Added imei, imsi, iccid, technology for Linux device identification.
    // These are static values queried once at boot -- they don't change at runtime.
    char jsonBuffer[512];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
             "{\"passthrough\":%d,"
             "\"lte\":%d,\"rsrp\":\"%s\",\"rsrq\":\"%s\","
             "\"operator\":\"%s\",\"band\":\"%s\","
             "\"mcc\":%u,\"mnc\":%u,\"cellId\":%u,\"tac\":%u,"
             "\"profile\":\"%s\","
             "\"imei\":\"%s\",\"imsi\":\"%s\","
             "\"iccid\":\"%s\",\"technology\":\"%s\"}",
             passthroughValue,
             lteStatus,
             rsrpStr.c_str(),
             rsrqStr.c_str(),
             operatorStr.c_str(),
             bandStr.c_str(),
             modemCC, modemNC, modemCID, modemTAC,
             profileManager.getActiveProfileName().c_str(),
             imei.c_str(),
             modemIMSI.length() > 0 ? modemIMSI.c_str() : "--",
             modemICCID.length() > 0 ? modemICCID.c_str() : "--",
             modemTechnology.length() > 0 ? modemTechnology.c_str() : "--");
    
    // Send to Python device via Serial1 (RS-232)
    Serial1.println(jsonBuffer);
    
    // Debug output to Serial Monitor (verbose — only when debug mode ON)
    if (serialDebugMode) {
        Serial.print("[RS232-TX CELL @10s] ");
    Serial.println(jsonBuffer);
    }
}

/**
 * Send fresh datetime to Python device via Serial1 (RS-232).
 * 
 * REV 10.9: ONLY sent when modemTimeFresh is true -- i.e., when the modem
 * clock has been freshly queried and returned a new timestamp. This prevents
 * sending stale/frozen datetime values that confuse the Linux controller.
 * 
 * Separated from cellular packet so that:
 *   1. Cellular info (rsrp, rsrq, lte, etc.) is always available on a 10s timer
 *   2. Datetime only updates when genuinely fresh from modem
 * 
 * Packet format (sent only when modemTimeFresh, on Serial1 to Linux):
 *   {"datetime":"2026-02-10 13:23:51"}
 * 
 * Python modem.py parses "datetime" with 'if "datetime" in data' -- fully
 * compatible with this being a standalone single-field packet.
 * 
 * Usage example:
 *   if (modemTimeFresh) { sendDateTimeToSerial(); modemTimeFresh = false; }
 */
void sendDateTimeToSerial() {
    String dateTimeStr = formatTimestamp(currentTimestamp);
    
    char jsonBuffer[64];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
             "{\"datetime\":\"%s\"}",
             dateTimeStr.c_str());
    
    Serial1.println(jsonBuffer);
    
    // Debug output to Serial Monitor (verbose — only when debug mode ON)
    if (serialDebugMode) {
        Serial.print("[RS232-TX TIME FRESH] ");
        Serial.println(jsonBuffer);
    }
}

/**
 * Send a lightweight sensor-only packet to Python at 5Hz (every 200ms).
 * 
 * REV 10.5: Increased from 1Hz to 5Hz. Added sdcard field.
 * 
 * Contains the fast-changing values that the Linux controller needs in real-time:
 *   pressure (IWC), current (amps), overfill state, sdcard status, relay mode
 * 
 * This is much smaller (~80 bytes) than the full status (~500 bytes)
 * and avoids expensive modem queries. isSDCardOK() is a cheap SD.cardType() check.
 * 
 * Python modem.py parses "pressure", "current", "overfill" fields from
 * ANY incoming JSON — same parser handles both fast and full packets.
 * 
 * Usage example:
 *   sendFastSensorPacket();  // Called 5x/second from checkAndSendStatusToSerial()
 */
void sendFastSensorPacket() {
    // Lightweight JSON — real-time sensor data + SD status + system state, no modem/cell queries
    // SD card check (SD.cardType()) is fast — just reads cached card type, no I/O
    //
    // REV 10.8: Added "failsafe" and "shutdown" fields so Linux can see system state
    // without needing a new message type. Python ignores unrecognized JSON fields,
    // so this is fully backward-compatible — zero Python parsing changes required.
    //
    // REV 10.10: Stale data detection — if no successful ADC read for 60 seconds,
    // send -99.9 for pressure and 0.0 for current to signal a sensor fault.
    // The Linux program should interpret -99.9 as "ADC sensor fault — data unavailable."
    // Previously, stale values from the last good read persisted indefinitely,
    // making it look like the sensor was working when it wasn't.
    //
    // Packet format (sent at 5Hz on Serial1 to Linux):
    //   Normal:  {"pressure":-1.23,"current":5.67,"overfill":0,"sdcard":"OK","relayMode":1,"failsafe":0,"shutdown":0}
    //   Stale:   {"pressure":-99.90,"current":0.00,"overfill":0,"sdcard":"OK","relayMode":1,"failsafe":0,"shutdown":0}
    //
    // Fields:
    //   pressure  - ESP32 ADC pressure in IWC (60Hz rolling average), or -99.9 if stale
    //   current   - ESP32 ADC motor current in amps (differential, peak-detect), or 0.0 if stale
    //   overfill  - 0=normal, 1=overfill alarm active (GPIO38 LOW)
    //   sdcard    - "OK" or "FAULT"
    //   relayMode - Current relay mode applied to GPIO pins (0-9)
    //   failsafe  - 0=normal serial-controlled, 1=ESP32 autonomous (Comfile down)
    //   shutdown  - 0=DISP_SHUTDN HIGH (normal), 1=DISP_SHUTDN LOW (72-hour shutdown)
    
    // Check if ADC data is stale (no successful read for ADC_STALE_TIMEOUT_MS)
    float sendPressure = adcPressure;
    float sendCurrent = adcCurrent;
    
    if (lastSuccessfulAdcRead > 0 && (millis() - lastSuccessfulAdcRead >= ADC_STALE_TIMEOUT_MS)) {
        // ADC has been failing for > 60 seconds — send fault sentinel values
        if (!adcDataStale) {
            adcDataStale = true;
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            Serial.println("!! ADC DATA STALE — no successful read for 60 seconds            !!");
            Serial.printf("!!   Last good read: %lu ms ago  |  Total errors: %lu\r\n",
                          millis() - lastSuccessfulAdcRead, adcTotalErrors);
            Serial.println("!!   Sending pressure=-99.9 (FAULT) to Linux device               !!");
            Serial.println("!!   ADC will auto-recover when I2C bus responds again             !!");
            Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        }
        sendPressure = ADC_FAULT_PRESSURE;  // -99.9 = sensor fault
        sendCurrent = 0.0;                   // No valid current reading
    }
    
    char buf[200];
    unsigned long ms = millis();
    snprintf(buf, sizeof(buf),
             "{\"pressure\":%.2f,\"current\":%.2f,\"overfill\":%d,\"sdcard\":\"%s\","
             "\"relayMode\":%d,\"failsafe\":%d,\"shutdown\":%d,\"ms\":%lu}",
             sendPressure, sendCurrent, overfillAlarmActive ? 1 : 0,
             isSDCardOK() ? "OK" : "FAULT", currentRelayMode,
             failsafeMode ? 1 : 0, dispShutdownActive ? 0 : 1, ms);
    Serial1.println(buf);
    
    // Debug log to USB Serial Monitor — shows timestamp (ms) so you can verify 5Hz rate
    // Expect ~200ms between each line. Look for consistent spacing in the millis() values.
    // REV 10.9: Only prints when debug mode ON — this fires 5x/second and floods the monitor
    if (serialDebugMode) {
        Serial.printf("[FAST-TX @%lums] P=%.2f rawP=%d zero=%.2f C=%.2f rawC=%d OV=%d M=%d%s\r\n",
                      millis(), sendPressure, (int)adcRawPressure,
                      adcZeroPressure,
                      sendCurrent, (int)adcRawCurrent,
                      overfillAlarmActive ? 1 : 0, currentRelayMode,
                      adcDataStale ? " **STALE**" : "");
    }
}

/**
 * Check if it's time to send status/sensor data to Python and send if due.
 * Called from main loop - handles timing internally.
 * 
 * REV 10.9: Three send mechanisms:
 *   1. Every 200ms (5Hz): Fast sensor packet via sendFastSensorPacket()
 *      (pressure, current, overfill, sdcard, relayMode, failsafe, shutdown)
 *      Always sent on schedule -- these values change rapidly.
 *   2. Every 10 seconds: Cellular status via sendCellularStatusToSerial()
 *      (passthrough, lte, rsrp, rsrq, operator, band, mcc, mnc, cellId, tac, profile)
 *      Always sent on timer using cached values -- ensures Linux always receives
 *      cellular info even if modem queries fail. Datetime and failsafe REMOVED
 *      from this packet (Rev 10.9).
 *   3. On fresh time ONLY: Datetime via sendDateTimeToSerial()
 *      (datetime) -- only when modemTimeFresh flag is set, preventing stale timestamps.
 * 
 * REV 10.9 CHANGE: Cellular packet was previously freshness-gated (only sent when
 * modemDataFresh was true). If modem queries failed silently, the Linux device
 * never received cellular data. Now uses a 10-second timer with cached values.
 * modemDataFresh flag is cleared when new data arrives but no longer gates sending.
 * 
 * IMPORTANT: Skips ALL sending during passthrough mode to avoid interfering
 * with modem communication (PPP/AT commands)
 * 
 * Usage: Call in loop() - handles timing and freshness checks internally
 */
void checkAndSendStatusToSerial() {
    // Skip during passthrough mode - serial ports are dedicated to modem bridge
    if (passthroughMode) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // === 1. Fast sensor packet every 200ms (5Hz) ===
    // pressure, current, overfill, sdcard, relayMode, failsafe, shutdown
    // These are the CRITICAL real-time values the Linux controller uses for decisions.
    if (currentTime - lastSensorSendTime >= SENSOR_SEND_INTERVAL) {
        sendFastSensorPacket();
        lastSensorSendTime = currentTime;
    }
    
    // === 2. Cellular status every 10 seconds (timer-based, NOT freshness-gated) ===
    // REV 10.9: Always sends cached cellular values on a 10-second timer.
    // Previously (Rev 10.5): only sent when modemDataFresh was true, which meant
    // the Linux device got NOTHING if modem queries failed. Now the Linux device
    // always receives the last known cellular state every 10 seconds.
    // modemDataFresh is still set/cleared for internal tracking but does NOT gate this send.
    if (currentTime - lastCellularSendTime >= CELLULAR_SEND_INTERVAL) {
        sendCellularStatusToSerial();
        lastCellularSendTime = currentTime;
        // Clear modemDataFresh since we've sent whatever we have
        modemDataFresh = false;
    }
    
    // === 3. Datetime -- ONLY when fresh from modem ===
    // REV 10.9: Separated from cellular packet so stale timestamps are never sent.
    // modemTimeFresh is set by initModemTime() when modem clock returns new data.
    // Typically fires when modem time query succeeds (~every 60s depending on modem).
    if (modemTimeFresh) {
        sendDateTimeToSerial();
        modemTimeFresh = false;
    }
}

// =====================================================================
// QUALITY CHECK TEST MODE - ADC FUNCTIONS
// =====================================================================

// QC mode and ADS1015 ADC removed in Rev 9.4 (web simplification)
// Relay control is exclusively handled by I2C from the Linux master

// =====================================================================
// PASSTHROUGH MODE FUNCTIONS
// =====================================================================
// PASSTHROUGH MODE (Preference-based boot)
// =====================================================================
// When "remote XX" command is received, we store the request in preferences
// and restart. On boot, the firmware checks for this preference BEFORE
// loading WalterModem and runs a minimal passthrough-only mode.
// This ensures a clean serial port without WalterModem interference.
// =====================================================================

/**
 * Send passthrough request to Linux device and wait for "ready" confirmation
 * 
 * Sends JSON message: {"passthrough":"remote XX"} where XX is timeout in minutes
 * Waits up to 10 seconds for Linux to respond with "ready" (case-insensitive)
 * 
 * @param timeoutMinutes Passthrough duration to send to Linux
 * @return true if Linux confirmed ready, false if timeout or no response
 * 
 * Example:
 *   sendPassthroughRequestToLinux(60) sends {"passthrough":"remote 60"}
 *   Linux responds with "ready" or similar containing "ready"
 */
bool sendPassthroughRequestToLinux(unsigned long timeoutMinutes) {
    // Send passthrough request to Linux device
    String request = "{\"passthrough\":\"remote " + String(timeoutMinutes) + "\"}";
    Serial1.println(request);
    Serial.printf("[PASSTHROUGH] Sent to Linux: %s\r\n", request.c_str());
    
    // Wait for "ready" confirmation (up to 10 seconds)
    unsigned long startTime = millis();
    const unsigned long READY_TIMEOUT_MS = 10000;  // 10 second timeout
    String response = "";
    
    Serial.println("[PASSTHROUGH] Waiting for Linux 'ready' confirmation...");
    
    while (millis() - startTime < READY_TIMEOUT_MS) {
        while (Serial1.available()) {
            char c = Serial1.read();
            if (c == '\n' || c == '\r') {
                // Check if response contains "ready" (case-insensitive)
                response.toLowerCase();
                if (response.indexOf("ready") >= 0) {
                    Serial.println("[PASSTHROUGH] Linux confirmed ready");
                    return true;
                }
                response = "";  // Reset for next line
            } else {
                response += c;
                // Safety limit
                if (response.length() > 100) {
                    response = response.substring(response.length() - 50);
                }
            }
        }
        delay(10);  // Small delay to prevent tight loop
    }
    
    Serial.println("[PASSTHROUGH] WARNING: No 'ready' from Linux (timeout)");
    return false;
}

/**
 * Request passthrough mode - notifies Linux, saves state, and restarts IMMEDIATELY.
 * 
 * CRITICAL TIMING: Linux starts its chat script the instant it receives our
 * passthrough message, so the modem MUST be available with zero delay. We:
 *   1. Save any active relay cycle state (mode + remaining seconds) to preferences
 *   2. Send {"passthrough":"remote XX"} to Linux
 *   3. Restart into passthrough boot mode immediately (no confirmation wait)
 * 
 * The passthrough boot mode restores the saved relay state and finishes the
 * purge step quietly in the background while the modem bridge is already active.
 * 
 * @param timeoutMinutes Duration before auto-restart (default 60 minutes, max 1440 = 24hrs)
 * 
 * Example usage (from BlueCherry message):
 * - "remote" → 60 minutes (default)
 * - "remote 30" → 30 minutes
 * - "remote 120" → 120 minutes (2 hours)
 */
void enterPassthroughMode(unsigned long timeoutMinutes) {
    // Clamp timeout to reasonable range
    if (timeoutMinutes < 1) timeoutMinutes = 1;
    if (timeoutMinutes > 1440) timeoutMinutes = 1440;  // Max 24 hours
    
    Serial.printf("[PASSTHROUGH] Requesting %lu min timeout\r\n", timeoutMinutes);
    
    // =====================================================================
    // Rev 10 Step 5: START PPP IMMEDIATELY - modem must be ready for Linux.
    // Linux begins its chat script the instant it gets our passthrough message,
    // so any delay here means a failed PPP negotiation.
    //
    // If a relay cycle is running, save the current step's relay mode and
    // remaining seconds into preferences so the passthrough boot mode can
    // quietly finish the step AFTER the modem bridge is already running.
    // =====================================================================
    preferences.begin("RMS", false);
    preferences.putBool("ptBoot", true);
    preferences.putULong("ptTimeout", timeoutMinutes);

    // Save current datetime for backfill filtering
    String currentTime = getDateTimeString();
    preferences.putString("backfillStartTime", currentTime);
    preferences.putBool("backfillPending", true);

    // Set purge needed flag if cycle is running
    if (failsafeCycleRunning) {
        preferences.putBool("purgeNeeded", true);
        // Save current step so passthrough boot can finish it quietly
        uint8_t stepMode = activeCycleSequence[failsafeCycleStep].mode;
        unsigned long elapsed = millis() - failsafeCycleStepTimer;
        unsigned long totalMs = (unsigned long)activeCycleSequence[failsafeCycleStep].durationSeconds * 1000UL;
        unsigned long remainingMs = (elapsed < totalMs) ? (totalMs - elapsed) : 0;
        unsigned long remainingSec = remainingMs / 1000UL;

        preferences.putUChar("ptRelayMode", stepMode);
        preferences.putULong("ptRelaySec", remainingSec);

        Serial.printf("[PASSTHROUGH] Saving cycle state: mode %d, %lu sec remaining\r\n",
                      stepMode, remainingSec);
    } else {
        preferences.putBool("purgeNeeded", false);
        // No cycle running - clear any stale relay state
        preferences.putUChar("ptRelayMode", 0);
        preferences.putULong("ptRelaySec", 0);
    }

    Serial.printf("[BACKFILL] Passthrough start time: %s\r\n", currentTime.c_str());
    Serial.println("[BACKFILL] Backfill procedure will run after passthrough ends");
    preferences.end();
    
    // Notify Linux THEN restart immediately - no waiting for purge, no confirmation delay
    // Linux starts chat script as soon as it receives this message, so speed is critical
    String request = "{\"passthrough\":\"remote " + String(timeoutMinutes) + "\"}";
    Serial1.println(request);
    Serial.printf("[PASSTHROUGH] Sent to Linux: %s\r\n", request.c_str());
    
    Serial.println("[PASSTHROUGH] Restarting into passthrough mode NOW...");
    delay(100);  // Minimal delay - just enough for serial TX to flush
    ESP.restart();
}

/**
 * Exit passthrough mode - stub function (not used in preference-based boot)
 * 
 * In the preference-based approach, passthrough runs on boot and
 * auto-restarts when complete. This function is kept for compatibility.
 */
void exitPassthroughMode() {
    Serial.println("[Passthrough] Exit requested - restarting...");
    delay(1000);
    ESP.restart();
}

/**
 * Passthrough loop - stub function (not used in preference-based boot)
 * 
 * The passthrough loop now runs in runPassthroughBootMode() which is called
 * during setup() when the passthrough preference is set. This stub function
 * is kept for compatibility with existing main loop code.
 */
void runPassthroughLoop() {
    // Not used in preference-based boot approach
    // Passthrough runs from runPassthroughBootMode() on boot
}

/**
 * Get the current date/time as a formatted string from currentTimestamp.
 * Returns "YYYY-MM-DD HH:MM:SS" or "--" if timestamp is not set.
 */
String getDateTimeString() {
    if (currentTimestamp == 0) return "--";
    time_t rawTime = (time_t)currentTimestamp;
    struct tm *timeInfo = gmtime(&rawTime);
    char buf[20];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             timeInfo->tm_year + 1900, timeInfo->tm_mon + 1, timeInfo->tm_mday,
             timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
    return String(buf);
}

bool isInteger(const char* str) {
    if (str == NULL || *str == '\0') return false;
    for (int i = 0; str[i] != '\0'; i++) {
        if (!isdigit(str[i]) && (i != 0 || str[i] != '-')) return false;
    }
    return true;
}

/**
 * Read JSON data from Serial1
 * Expected format: {"type":"data","gmid":"RND-0007","press":-14.22,"mode":0,"current":0.07,"fault":0,"cycles":484}
 * This replaces the old CSV parser with a simpler JSON parser
 * 
 * IMPORTANT: Skips during passthrough mode - serial ports are dedicated to modem bridge
 */
void readSerialData() {
    // Skip during passthrough mode - serial ports are dedicated to modem bridge
    if (passthroughMode) {
        return;
    }
    
    static String jsonBuffer = "";
    static unsigned long lastRxTime = 0;
    static bool debugPrinted = false;
    static unsigned long lastDebugTime = 0;
    
    // Debug: Print once to confirm Serial1 is being checked
    if (!debugPrinted) {
        Serial.println("🔍 DEBUG: readSerialData() is active, Serial1 RX=GPIO44 TX=GPIO43");
        debugPrinted = true;
    }
    
    // Debug: Show Serial1.available() every 30 seconds
    if (millis() - lastDebugTime > 30000) {
        Serial.print("🔍 Serial1.available() = ");
        Serial.println(Serial1.available());
        lastDebugTime = millis();
    }
    
    while (Serial1.available() > 0) {
        char ch = Serial1.read();
        
        // Start of JSON object
        if (ch == '{') {
            jsonBuffer = "{";
            noSerialCount = 0;
            lastRxTime = millis();
            resetSerialWatchdog();  // Reset watchdog timer when data arrives
            Serial.print("→ Receiving JSON: ");
        }
        // Accumulate characters
        else if (jsonBuffer.length() > 0) {
            jsonBuffer += ch;
            
            // End of JSON object - process it immediately
            if (ch == '}') {
                Serial.println(jsonBuffer);  // Print complete JSON
                
                // Check for command messages (passthrough, set_profile, start/stop cycle)
                if (jsonBuffer.indexOf("\"command\"") >= 0) {
                    String cmdType = getJsonValue(jsonBuffer.c_str(), "command");
                    
                    if (cmdType == "passthrough") {
                    int timeoutIdx = jsonBuffer.indexOf("\"timeout\":");
                        unsigned long timeout = 60;
                    if (timeoutIdx >= 0) {
                        String timeoutStr = jsonBuffer.substring(timeoutIdx + 10);
                        timeout = timeoutStr.toInt();
                        if (timeout < 1) timeout = 60;
                    }
                        Serial.printf("[SERIAL] Passthrough command - %lu min\r\n", timeout);
                    enterPassthroughMode(timeout);
                    jsonBuffer = "";
                        return;
                    }
                    else if (cmdType == "set_profile") {
                        String profName = getJsonValue(jsonBuffer.c_str(), "profile");
                        if (profName.length() > 0) {
                            profileManager.setActiveProfile(profName.c_str());
                        }
                        jsonBuffer = "";
                        return;
                    }
                    else if (cmdType == "start_cycle") {
                        String cycleType = getJsonValue(jsonBuffer.c_str(), "type");
                        if (cycleType == "run") startFailsafeCycle(0);
                        else if (cycleType == "purge") startFailsafeCycle(1);
                        else if (cycleType == "clean") startFailsafeCycle(2);
                        jsonBuffer = "";
                        return;
                    }
                    else if (cmdType == "stop_cycle") {
                        stopFailsafeCycle("Serial command");
                        // REV 10.11: Linux stop_cycle does NOT clear web portal tests.
                        // The ESP32 owns relays during web portal tests — only the web
                        // portal Stop button or the auto-stop timer can end a test.
                        // Linux stop_cycle only stops failsafe cycles.
                        if (testRunning) {
                            Serial.printf("[Serial] stop_cycle from Linux IGNORED for web test '%s' "
                                          "— only web portal Stop or timer can end it\r\n",
                                          activeTestType.c_str());
                        }
                        jsonBuffer = "";
                        return;
                    }
                }
                
                strcpy(dataBuffer, jsonBuffer.c_str());
                diagData = jsonBuffer;
                
                // Parse the data immediately instead of waiting for parseSDInterval
                parsedSerialData();
                
                jsonBuffer = "";  // Reset for next message
                break;
            }
            
            // Safety limit
            if (jsonBuffer.length() >= MAX_BUFFER_SIZE - 1) {
                Serial.println("ERROR: JSON buffer overflow, resetting");
                jsonBuffer = "";
            }
        }
    }
    
    // If no data received, increment counter
    if (jsonBuffer.length() == 0 && Serial1.available() == 0) {
        noSerialCount++;
        
        // Debug: Print if we haven't received data in a while
        // Only warn every ~10 seconds (100 polls at 100ms each = 10s)
        if (noSerialCount % 100 == 0 && noSerialCount > 0) {
            Serial.print("⚠ No serial data received for ");
            Serial.print(noSerialCount / 10);  // readInterval is 100ms, so /10 = seconds
            Serial.println(" seconds");
        }
    }
}

/**
 * Simple JSON parser for Arduino
 * Extracts key-value pairs from JSON string
 * Usage: getValue(dataBuffer, "press") returns the pressure value as String
 */
String getJsonValue(const char* json, const char* key) {
    String jsonStr = String(json);
    String searchKey = "\"" + String(key) + "\":";
    
    int keyPos = jsonStr.indexOf(searchKey);
    if (keyPos == -1) return "";
    
    int valueStart = keyPos + searchKey.length();
    
    // Skip whitespace
    while (valueStart < jsonStr.length() && jsonStr.charAt(valueStart) == ' ') {
        valueStart++;
    }
    
    // Check if value is a string (starts with ")
    bool isString = (jsonStr.charAt(valueStart) == '"');
    if (isString) valueStart++; // Skip opening quote
    
    // Find end of value
    int valueEnd = valueStart;
    if (isString) {
        // Find closing quote
        valueEnd = jsonStr.indexOf('"', valueStart);
    } else {
        // Find comma or closing brace
        while (valueEnd < jsonStr.length()) {
            char c = jsonStr.charAt(valueEnd);
            if (c == ',' || c == '}' || c == ' ') break;
            valueEnd++;
        }
    }
    
    if (valueEnd == -1 || valueEnd <= valueStart) return "";
    
    return jsonStr.substring(valueStart, valueEnd);
}

/**
 * Parse JSON serial data
 * Expected format: {"type":"data","gmid":"CSX-1234","press":-14.22,"mode":0,"current":0.07,"fault":0,"cycles":484}
 * Much simpler than the old CSV parser!
 */
void parsedSerialData() {
    // Check if we have valid JSON data (must have both braces)
    if (strlen(dataBuffer) < 10 || strstr(dataBuffer, "{") == NULL || strstr(dataBuffer, "}") == NULL) {
        // Only print error if we actually received something
        if (strlen(dataBuffer) > 0) {
            Serial.print("Invalid JSON (buffer has ");
            Serial.print(strlen(dataBuffer));
            Serial.print(" chars): ");
            Serial.println(dataBuffer);
        }
        return;
    }
    
    // ─── REV 10.18: UNIFIED COMMAND FORMAT ─────────────────────────────
    // New format: {"command":"<name>","value":"<val>"}
    // Check for "command" field FIRST — this is the unified format used by
    // the Web Portal, BlueCherry, and Python for all commands.
    // Old format {"type":"cmd","cmd":"cal"} is still supported for backward
    // compatibility with older Python versions.
    // ─────────────────────────────────────────────────────────────────────
    String cmdField = getJsonValue(dataBuffer, "command");
    if (cmdField.length() > 0) {
        String valField = getJsonValue(dataBuffer, "value");
        Serial.printf("[SERIAL CMD] Unified command: %s value=%s\r\n", cmdField.c_str(), valField.c_str());
        executeRemoteCommand(cmdField, valField);
        return;  // Command handled — don't process as data packet
    }
    
    // Check message type (data packets from Linux)
    String msgType = getJsonValue(dataBuffer, "type");
    if (msgType.length() == 0) {
        Serial.println("No 'type' or 'command' field in JSON");
        return;
    }
    
    // ─── LEGACY COMMAND FORMAT (backward compatibility) ──────────────────
    // {"type":"cmd","cmd":"cal"}  — Old calibration command format.
    // Kept for backward compatibility with older Python versions.
    // New Python should send {"command":"calibrate_pressure"} instead.
    // ─────────────────────────────────────────────────────────────────────
    if (msgType == "cmd") {
        String cmdStr = getJsonValue(dataBuffer, "cmd");
        Serial.printf("[SERIAL CMD] Legacy command: %s\r\n", cmdStr.c_str());

        if (cmdStr == "cal") {
            calibratePressureSensorZeroPoint();
        } else if (cmdStr == "fast_poll") {
            // Enable fast BlueCherry polling for 60 minutes max
            String valStr = getJsonValue(dataBuffer, "val");
            int val = valStr.toInt();

            if (val == 1) {
                fastPollingActive = true;
                fastPollingStartTime = millis();
                Serial.println("[FAST POLL] Enabled - BlueCherry polling every 15 seconds for 60 minutes max");
            } else {
                fastPollingActive = false;
                Serial.println("[FAST POLL] Disabled - returning to scheduled polling");
            }
        } else {
            Serial.println("[SERIAL CMD] Unknown legacy command: " + cmdStr);
        }
        return;  // Command handled — don't process as data packet
    }
    
    if (msgType != "data") {
        Serial.println("Unknown message type: " + msgType);
        return;
    }
    
    // Extract values from JSON
    String gmidStr = getJsonValue(dataBuffer, "gmid");
    String pressStr = getJsonValue(dataBuffer, "press");
    String modeStr = getJsonValue(dataBuffer, "mode");
    String currentStr = getJsonValue(dataBuffer, "current");
    String faultStr = getJsonValue(dataBuffer, "fault");
    String cyclesStr = getJsonValue(dataBuffer, "cycles");
    
    // Check for shutdown/normal commands: string mode values (not integers)
    // {"mode":"shutdown"} — DISP_SHUTDN LOW (site shutdown, 72-hour alarm)
    // {"mode":"normal"}  — DISP_SHUTDN HIGH (clear 72-hour alarm, restore site)
    if (modeStr == "shutdown") {
        activateDispShutdown();
        return;  // Don't process as normal data packet
    }
    if (modeStr == "normal") {
        deactivateDispShutdown();
        return;  // Don't process as normal data packet
    }
    
    // If we're in failsafe mode and serial data resumes, exit failsafe
    if (failsafeMode) {
        exitFailsafeMode();
    }
    
    // Convert to appropriate types
    if (pressStr.length() > 0) pressure = pressStr.toFloat();
    if (modeStr.length() > 0) mode = modeStr.toInt();
    if (currentStr.length() > 0) current = currentStr.toFloat();
    if (faultStr.length() > 0) faults = faultStr.toInt();
    if (cyclesStr.length() > 0) cycles = cyclesStr.toInt();
    
    // Rev 10: Set relay outputs based on mode field from Linux
    // This replaces the old I2C register write approach
    //
    // REV 10.11: Web Portal Test Relay Override
    // When a web portal test is running (leak, func, eff, clean), the ESP32
    // has SOLE control of the relays. ALL mode commands from Linux are BLOCKED
    // — including mode 0. The web portal test can ONLY be stopped by:
    //   1. The user pressing Stop on the web portal
    //   2. The test auto-completing when its timer expires
    //
    // This prevents Linux's periodic mode messages (e.g. "mode":1 every 15s)
    // from interfering with maintenance tests. The ESP32 releases relay control
    // back to Linux when testRunning is cleared.
    //
    // Note: We still parse and store the mode value from Linux (for CBOR, etc.)
    // — we just don't apply it to the relay GPIOs while a test is active.
    //
    // REV 10.15: Skip redundant relay writes when mode hasn't changed.
    // Linux sends a data packet with "mode" every ~7 seconds. Previously, every
    // packet triggered setRelaysForMode() even if the mode was identical, causing
    // unnecessary GPIO writes that generate relay driver switching transients.
    // These transients couple EMI into the ADS1015 analog input, biasing pressure
    // readings. Now we only call setRelaysForMode() when the mode actually changes.
    // The 15-second relay refresh (loop) still force-writes as a safety net.
    // REV 10.18: Also block Linux mode commands during manual relay override
    if (modeStr.length() > 0 && !failsafeMode) {
        if (manualRelayOverride) {
            // Manual relay override in progress — BLOCK Linux mode commands
            Serial.printf("[Serial] Mode %d from Linux BLOCKED — manual relay override active\r\n", mode);
        } else if (testRunning) {
            // Web portal test in progress — BLOCK all Linux mode commands
            Serial.printf("[Serial] Mode %d from Linux BLOCKED — web test '%s' in progress "
                          "(ESP32 owns relays, elapsed %lus)\r\n",
                          mode, activeTestType.c_str(),
                          (millis() - testStartTime) / 1000);
        } else if ((uint8_t)mode != currentRelayMode) {
            // Mode changed — apply new relay state from Linux
            Serial.printf("[Serial] Mode change: %d → %d — applying to relays\r\n",
                          currentRelayMode, mode);
            setRelaysForMode((uint8_t)mode);
        }
        // else: same mode already active — skip redundant relay write to avoid EMI transient
    }
    
    // Display Linux device name (informational only — does NOT overwrite ESP32's stored name)
    // The ESP32 device name is set exclusively via the web portal /setdevicename endpoint.
    // Previously, Linux's gmid would overwrite web portal changes every 15 seconds.
    if (gmidStr.length() > 0 && gmidStr != String(deviceName)) {
        Serial.println("[Serial] Linux device name: '" + gmidStr + 
                       "' (ESP32 name: '" + String(deviceName) + "' — NOT overwriting)");
    }
    
    // Log received data from Linux (cycles, faults, GMID are used; pressure/current are informational only)
    // NOTE: ESP32 uses its own ADC for pressure/current (adcPressure/adcCurrent) not these values
    Serial.println("✓ JSON from Linux - Cycles:" + String(cycles) + " Faults:" + String(faults) +
                   " GMID:" + gmidStr + " (ADC press:" + String(adcPressure, 2) + " curr:" + String(adcCurrent, 2) + ")");
    
    // Save to SD if we have valid data
    if (noSerialCount <= 5) {
        // REV 10 FIX: Use adcPressure (ESP32's own ADC reading at 60Hz) instead of
        // the round-tripped 'pressure' variable from Python. Previously, pressure had
        // to travel: ESP32 ADC → serial → Python → serial back → parsedSerialData()
        // which added 15+ seconds of staleness. adcPressure is updated 60 times/sec.
        pres_scaled = static_cast<int>(round(adcPressure * 100.0));
        seq++;
        Serial.println("Saving to SD");
        // REV 10: Use ESP32's own adcCurrent and currentRelayMode (no Python round-trip)
        SaveToSD(deviceName, seq - 1, pres_scaled, cycles, faults, currentRelayMode, temp_scaled, adcCurrent);
    } else {
        NoSerial = 1;
    }
}

void buildSendDataArray(){
    Serial.println("Device Name");
    Serial.println(deviceName);
    String idStr = String(deviceName);
    
    // Extract numeric ID from device name (handles CSX-#### and RND-####)
    if (idStr.startsWith("CSX-")) {
        idStr = idStr.substring(4);
    } else if (idStr.startsWith("RND-")) {
        idStr = idStr.substring(4);
    }
    // If there are any other prefixes, try to find the last dash
    else if (idStr.indexOf("-") >= 0) {
        idStr = idStr.substring(idStr.lastIndexOf("-") + 1);
    }
    
    int id = idStr.toInt();
    Serial.print("ID String = ");
    Serial.print(idStr);
    Serial.print(" -> ID = ");
    Serial.println(id);
    Serial.println("***********************");

    readingCount++;
    
    // Calculate combined fault code: 1024 (watchdog) + 4096 (BlueCherry offline)
    // This is added to any existing faults from the Python device
    int systemFaultCode = getCombinedFaultCode();
    
    // Check if we're in watchdog state (no serial data for 30+ minutes)
    // If so, send zeroed data with fault codes to alert server
    if (isInWatchdogState()) {
        Serial.println("╔════════════════════════════════════════════════════════════╗");
        Serial.print("║  ⚠️ WATCHDOG STATE: Sending zeroed data with fault=");
        Serial.print(systemFaultCode);
        for (int i = String(systemFaultCode).length(); i < 5; i++) Serial.print(" ");
        Serial.println("  ║");
        Serial.println("╚════════════════════════════════════════════════════════════╝");
        
        // Zero out all values except ID
        readings[readingCount-1][0] = id;                          // Keep device ID
        readings[readingCount-1][1] = static_cast<int>(seq);       // Keep sequence number
        readings[readingCount-1][2] = 0;                           // pressure = 0
        readings[readingCount-1][3] = 0;                           // cycles = 0
        readings[readingCount-1][4] = systemFaultCode;             // fault = 1024 + 4096 if BC down
        readings[readingCount-1][5] = 0;                           // mode = 0
        readings[readingCount-1][6] = 0;                           // temp = 0
        readings[readingCount-1][7] = 0;                           // current = 0
        
        Serial.print("📤 Watchdog data packet: ID=");
        Serial.print(id);
        Serial.print(", seq=");
        Serial.print(seq);
        Serial.print(", fault=");
        Serial.print(systemFaultCode);
        if (!blueCherryConnected) Serial.print(" (includes 4096=BlueCherry offline)");
        Serial.println(" (all other values zeroed)");
        
    } else {
        // Normal operation — use ESP32's own sensor data (no Python round-trip)
        
        // Read ESP32's own temperature sensor
        float celsius = temperatureRead();
        float fahrenheit = (celsius * 9.0 / 5.0) + 32;
        temp_scaled = static_cast<int>(round(fahrenheit * 100.0));
        
        // REV 10 FIX: Use adcPressure and adcCurrent directly from ESP32's own
        // ADS1015 ADC (updated at 60Hz) instead of the round-tripped values from
        // Python. The old 'pressure' and 'current' variables required a 15-second
        // round trip through Python and back, making CBOR data stale.
        // Now: ESP32 ADC → CBOR (direct, real-time, no delay)
        pres_scaled = static_cast<int>(round(adcPressure * 100.0));
        int current_scaled = static_cast<int>(round(adcCurrent * 100.0));
        
        Serial.print("Data summary - Temp(ESP32): ");
        Serial.print(fahrenheit);
        Serial.print("°F, Press(ADC): ");
        Serial.print(adcPressure);
        Serial.print(", Current(ADC): ");
        Serial.print(adcCurrent);
        Serial.print(", Cycles(Linux): ");
        Serial.print(cycles);
        Serial.print(", Mode: ");
        Serial.println(currentRelayMode);
        
        // REV 10.17: Add all ESP32-detected fault codes to existing faults from Linux
        // SD card (1024) + Watchdog (2048) + Failsafe (4096) + BlueCherry (8192)
        int totalFaults = faults + getCombinedFaultCode();
        
        readings[readingCount-1][0] = id;
        readings[readingCount-1][1] = static_cast<int>(seq);
        readings[readingCount-1][2] = pres_scaled;
        readings[readingCount-1][3] = cycles;
        readings[readingCount-1][4] = totalFaults;  // Device faults + ESP32 fault codes (1024/2048/4096/8192)
        readings[readingCount-1][5] = currentRelayMode;  // Use ESP32's own mode, not round-tripped
        readings[readingCount-1][6] = temp_scaled;
        readings[readingCount-1][7] = current_scaled;
        
        // Log if any ESP32 fault codes are active
        int espFaults = getCombinedFaultCode();
        if (espFaults > 0) {
            Serial.printf("📤 Data includes ESP32 fault codes (%d): total fault=%d\r\n",
                          espFaults, totalFaults);
        }
    }
    
    // Increment sequence number
    seq++;
    
    if (readingCount >= 12) {
        Serial.print("=== Encoding ");
        Serial.print(readingCount);
        Serial.println(" readings as CBOR ===");
        
        if (isInWatchdogState()) {
            Serial.println("🚨 NOTE: This batch contains WATCHDOG DATA (fault includes 1024)");
        }
        if (!blueCherryConnected) {
            Serial.println("🚨 NOTE: Fault code includes 4096 (BlueCherry offline)");
        }
          
        size_t cborSize = buildCborFromReadings(cborBuffer, sizeof(cborBuffer), readings, readingCount);
        
        if (cborSize > 0) {
            Serial.print("CBOR encoded successfully, size: ");
            Serial.print(cborSize);
            Serial.println(" bytes");
                        
            if (sendCborArrayViaSocket(cborBuffer, cborSize)) {
                Serial.println("CBOR data sent successfully!");
            } else {
                Serial.println("Failed to send CBOR data");
            }
        } else {
            Serial.println("CBOR encoding failed");
        }
             
        readingCount = 0;
        Serial.println("=== Ready for next batch ===");
    }
} 

bool lteConnected() {
    WalterModemNetworkRegState regState = modem.getNetworkRegState();
    return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME || regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

bool lteConnect() {
    Serial.println("Starting LTE connection process...");
    
    // REV 10.10: All delay() calls replaced with serial-sending delay loops.
    // Previously, each delay() blocked the main loop completely — no serial data
    // was sent to Linux during the entire LTE reconnection process (up to 5+ min).
    // Now: checkAndSendStatusToSerial() is called every 200ms during each wait,
    // so pressure, current, and overfill data keep flowing to the Linux device.
    
    if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
        Serial.println("Failed to set operational state to minimum");
        return false;
    }
    // 2000ms wait — send serial data during delay
    for (int d = 0; d < 10; d++) { delay(200); checkAndSendStatusToSerial(); }
	
    Serial.printf("Defining PDP context with APN: %s\n", CELLULAR_APN);
    if (!modem.definePDPContext(1, CELLULAR_APN)) {
        Serial.println("Failed to define PDP context");
        Serial.println("Possible causes:");
        Serial.println("1. Incorrect APN for your cellular provider");
        Serial.println("2. Network not ready for data connection");
        Serial.println("3. SIM card issues");
        return false;
    }
    // 1000ms wait — send serial data during delay
    for (int d = 0; d < 5; d++) { delay(200); checkAndSendStatusToSerial(); }
    
    Serial.printf("Setting PDP authentication: Protocol=%d, User=%s\n", 
                  CELLULAR_AUTH_PROTOCOL, CELLULAR_USERNAME);
    if (!modem.setPDPAuthParams(CELLULAR_AUTH_PROTOCOL, CELLULAR_USERNAME, CELLULAR_PASSWORD)) {
        Serial.println("Failed to set PDP authentication parameters");
        Serial.println("Note: Some providers don't require authentication");
    }
    // 1000ms wait — send serial data during delay
    for (int d = 0; d < 5; d++) { delay(200); checkAndSendStatusToSerial(); }
    
    if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
        Serial.println("Failed to set operational state to full");
        return false;
    }
    // 2000ms wait — send serial data during delay
    for (int d = 0; d < 10; d++) { delay(200); checkAndSendStatusToSerial(); }
	
    if (!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
        Serial.println("Failed to set network selection mode");
        return false;
    }
    // 1000ms wait — send serial data during delay
    for (int d = 0; d < 5; d++) { delay(200); checkAndSendStatusToSerial(); }
    
    Serial.println("Waiting for network registration...");
    unsigned short timeout = 300, i = 0;
    while (!lteConnected() && i < timeout) {
        i++;
        // 1000ms wait — send serial data every 200ms during registration wait
        for (int d = 0; d < 5; d++) { delay(200); checkAndSendStatusToSerial(); }
        if (i % 10 == 0) {
            Serial.printf("Still waiting for network registration... (%d/%d)\n", i, timeout);
        }
    }
    
    if (i >= timeout) {
        Serial.println("Network registration timeout");
        return false;
    }
    
    Serial.println("Network registration successful");
    
    if (modem.getRAT(&rsp)) {
        Serial.printf("Connected to %s ", rsp.data.rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M");
    }
    
    if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
        Serial.printf("on band %u using operator %s (%u%02u)\r\n", rsp.data.cellInformation.band,
                      rsp.data.cellInformation.netName, rsp.data.cellInformation.cc, rsp.data.cellInformation.nc);
        Serial.printf("Signal strength: RSRP: %.2f, RSRQ: %.2f\r\n", rsp.data.cellInformation.rsrp, rsp.data.cellInformation.rsrq);
        Modem_Status = "Modem OK ";
        
        // Store cell info in globals for web dashboard and serial JSON output
        modemBand = String(rsp.data.cellInformation.band);
        modemNetName = String(rsp.data.cellInformation.netName);
        modemRSRP = String(rsp.data.cellInformation.rsrp, 1);   // e.g. "-89.5"
        modemRSRQ = String(rsp.data.cellInformation.rsrq, 1);   // e.g. "-11.2"
        modemCC = rsp.data.cellInformation.cc;                    // Country Code (e.g. 310)
        modemNC = rsp.data.cellInformation.nc;                    // Network Code (e.g. 260)
        modemCID = rsp.data.cellInformation.cid;                  // Cell Tower ID
        modemTAC = rsp.data.cellInformation.tac;
 
        // REV 10.5: Mark modem data as fresh for serial status output
        modemDataFresh = true;
 
        if (serialDebugMode) {
        Serial.printf("[CELL] Band=%s, Op=%s, RSRP=%s, RSRQ=%s, MCC=%u, MNC=%02u, CID=%u, TAC=%u\r\n",
                      modemBand.c_str(), modemNetName.c_str(), modemRSRP.c_str(), modemRSRQ.c_str(),
                      modemCC, modemNC, modemCID, modemTAC);
    }
    }  // end if (modem.getCellInformation)
    
    Serial.println("LTE connection established successfully");
    return true;
}

// =====================================================================
// REV 10.13: NON-BLOCKING CELLULAR INITIALIZATION STATE MACHINE
// =====================================================================
// Called from loop() on every iteration until cellularInitComplete is true.
// Each state performs ONE operation and returns immediately, allowing the
// main loop to continue (serial data, web portal, relay control, watchdog).
//
// On any failure, the state machine enters RETRY_COOLDOWN (60 seconds)
// then restarts from SET_MINIMUM. No ESP.restart() on failure.
//
// Usage example:
//   // In loop():
//   if (!cellularInitComplete) {
//       runCellularInitStateMachine();
//   }
void runCellularInitStateMachine() {
    if (cellularInitComplete) return;  // Already done
    
    switch (cellularInitState) {
        
        // ---- Step 1: Set modem to minimum operational state ----
        case CELL_INIT_SET_MINIMUM: {
            Serial.printf("[CELL-INIT] Starting cellular connection (attempt #%d)...\r\n",
                          cellularInitRetryCount + 1);
            if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
                Serial.println("[CELL-INIT] Failed to set op state to minimum — retrying in 60s");
                cellularInitState = CELL_INIT_RETRY_COOLDOWN;
                cellularInitTimer = millis();
                return;
            }
            Serial.println("[CELL-INIT] Op state → MINIMUM");
            cellularInitState = CELL_INIT_WAIT_MINIMUM;
            cellularInitTimer = millis();
            return;
        }
        
        // ---- Step 2: Wait 2 seconds for modem state change ----
        case CELL_INIT_WAIT_MINIMUM: {
            if (millis() - cellularInitTimer < 2000) return;  // Still waiting
            cellularInitState = CELL_INIT_DEFINE_PDP;
            return;
        }
        
        // ---- Step 3: Define APN/PDP context ----
        case CELL_INIT_DEFINE_PDP: {
            Serial.printf("[CELL-INIT] Defining PDP context with APN: %s\r\n", CELLULAR_APN);
            if (!modem.definePDPContext(1, CELLULAR_APN)) {
                Serial.println("[CELL-INIT] Failed to define PDP context — retrying in 60s");
                cellularInitState = CELL_INIT_RETRY_COOLDOWN;
                cellularInitTimer = millis();
                return;
            }
            cellularInitState = CELL_INIT_SET_AUTH;
            cellularInitTimer = millis();
            return;
        }
        
        // ---- Step 4: Set PDP authentication ----
        case CELL_INIT_SET_AUTH: {
            if (millis() - cellularInitTimer < 1000) return;  // Brief wait after PDP define
            Serial.printf("[CELL-INIT] Setting PDP auth: Protocol=%d, User=%s\r\n",
                          CELLULAR_AUTH_PROTOCOL, CELLULAR_USERNAME);
            // Auth failure is non-fatal — some providers don't require it
            if (!modem.setPDPAuthParams(CELLULAR_AUTH_PROTOCOL, CELLULAR_USERNAME, CELLULAR_PASSWORD)) {
                Serial.println("[CELL-INIT] PDP auth params failed (non-fatal, some providers don't need it)");
            }
            cellularInitState = CELL_INIT_SET_FULL;
            cellularInitTimer = millis();
            return;
        }
        
        // ---- Step 5: Set modem to full operational state ----
        case CELL_INIT_SET_FULL: {
            if (millis() - cellularInitTimer < 1000) return;  // Brief wait after auth
            if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
                Serial.println("[CELL-INIT] Failed to set op state to full — retrying in 60s");
                cellularInitState = CELL_INIT_RETRY_COOLDOWN;
                cellularInitTimer = millis();
                return;
            }
            Serial.println("[CELL-INIT] Op state → FULL");
            cellularInitState = CELL_INIT_WAIT_FULL;
            cellularInitTimer = millis();
            return;
        }
        
        // ---- Step 6: Wait 2 seconds for modem to go full ----
        case CELL_INIT_WAIT_FULL: {
            if (millis() - cellularInitTimer < 2000) return;
            cellularInitState = CELL_INIT_SET_AUTO_SELECT;
            return;
        }
        
        // ---- Step 7: Set automatic network selection ----
        case CELL_INIT_SET_AUTO_SELECT: {
            if (!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
                Serial.println("[CELL-INIT] Failed to set network selection — retrying in 60s");
                cellularInitState = CELL_INIT_RETRY_COOLDOWN;
                cellularInitTimer = millis();
                return;
            }
            Serial.println("[CELL-INIT] Network selection → AUTOMATIC");
            Serial.println("[CELL-INIT] Waiting for network registration (up to 300 seconds)...");
            cellularInitState = CELL_INIT_WAIT_REGISTRATION;
            cellularInitTimer = millis();
            cellularInitRegAttempts = 0;
            return;
        }
        
        // ---- Step 8: Poll for network registration (non-blocking, 1s intervals) ----
        case CELL_INIT_WAIT_REGISTRATION: {
            if (millis() - cellularInitTimer < 1000) return;  // Poll every 1 second
            cellularInitTimer = millis();
            cellularInitRegAttempts++;
            
            if (lteConnected()) {
                Serial.printf("[CELL-INIT] ===== NETWORK REGISTERED (%d seconds) =====\r\n",
                              cellularInitRegAttempts);
                Modem_Status = "Modem OK ";
                cellularInitState = CELL_INIT_GET_CELL_INFO;
                return;
            }
            
            if (cellularInitRegAttempts % 10 == 0) {
                Serial.printf("[CELL-INIT] Still waiting for registration... (%d/%d)\r\n",
                              cellularInitRegAttempts, CELL_INIT_MAX_REG_WAIT);
            }
            
            if (cellularInitRegAttempts >= CELL_INIT_MAX_REG_WAIT) {
                Serial.println("[CELL-INIT] Network registration TIMEOUT (300s) — retrying in 60s");
                Modem_Status = "Registration timeout";
                cellularInitState = CELL_INIT_RETRY_COOLDOWN;
                cellularInitTimer = millis();
            }
            return;
        }
        
        // ---- Step 9: Get cell tower info (RSRP, operator, band) ----
        case CELL_INIT_GET_CELL_INFO: {
            if (modem.getRAT(&rsp)) {
                Serial.printf("[CELL-INIT] Connected via %s\r\n",
                              rsp.data.rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M");
            }
            
            if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
                modemBand = String(rsp.data.cellInformation.band);
                modemNetName = String(rsp.data.cellInformation.netName);
                modemRSRP = String(rsp.data.cellInformation.rsrp, 1);
                modemRSRQ = String(rsp.data.cellInformation.rsrq, 1);
                modemCC = rsp.data.cellInformation.cc;
                modemNC = rsp.data.cellInformation.nc;
                modemCID = rsp.data.cellInformation.cid;
                modemTAC = rsp.data.cellInformation.tac;
                modemDataFresh = true;
                
                Serial.printf("[CELL-INIT] Band=%s, Op=%s, RSRP=%s dBm, RSRQ=%s dB\r\n",
                              modemBand.c_str(), modemNetName.c_str(),
                              modemRSRP.c_str(), modemRSRQ.c_str());
            }
            
            Serial.println("[CELL-INIT] LTE connection established — starting BlueCherry init");
            cellularInitState = CELL_INIT_BLUECHERRY;
            cellularInitBcAttempts = 0;
            cellularInitTimer = millis();
            return;
        }
        
        // ---- Step 10: BlueCherry initialization (up to 3 attempts) ----
        case CELL_INIT_BLUECHERRY: {
            // Brief pause between attempts
            if (cellularInitBcAttempts > 0 && millis() - cellularInitTimer < 5000) return;
            
            cellularInitBcAttempts++;
            Serial.printf("[CELL-INIT] BlueCherry attempt %d/%d\r\n",
                          cellularInitBcAttempts, CELL_INIT_MAX_BC_ATTEMPTS);
            
            systemStartTime = millis();
            blueCherryLastAttempt = millis();
            
            if (modem.blueCherryInit(BC_TLS_PROFILE, otaBuffer, &rsp)) {
                blueCherryConnected = true;
                Serial.println("[CELL-INIT] ✓ BlueCherry initialized successfully");
                cellularInitState = CELL_INIT_GET_IDENTITY;
                return;
            }
            
            // Handle not-provisioned: attempt ZTP on first try
            if (rsp.data.blueCherry.state == WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED
                && cellularInitBcAttempts == 1) {
                Serial.println("[CELL-INIT] Device not provisioned, attempting ZTP...");
                if (BlueCherryZTP::begin(BC_DEVICE_TYPE, BC_TLS_PROFILE, bc_ca_cert, &modem)) {
                    uint8_t mac[8] = {0};
                    esp_read_mac(mac, ESP_MAC_WIFI_STA);
                    BlueCherryZTP::addDeviceIdParameter(BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC, mac);
                    if (modem.getIdentity(&rsp)) {
                        BlueCherryZTP::addDeviceIdParameter(BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI, rsp.data.identity.imei);
                    }
                    if (BlueCherryZTP::requestDeviceId() && BlueCherryZTP::generateKeyAndCsr()
                        && BlueCherryZTP::requestSignedCertificate()
                        && modem.blueCherryProvision(BlueCherryZTP::getCert(), BlueCherryZTP::getPrivKey(), bc_ca_cert)) {
                        Serial.println("[CELL-INIT] ZTP provisioning succeeded — retrying BlueCherry init");
                    }
                }
            }
            
            if (cellularInitBcAttempts >= CELL_INIT_MAX_BC_ATTEMPTS) {
                blueCherryConnected = false;
                Serial.println("[CELL-INIT] BlueCherry unavailable — system continues without OTA");
                Serial.println("[CELL-INIT]   Will retry hourly, weekly restart if still offline");
                cellularInitState = CELL_INIT_GET_IDENTITY;
                return;
            }
            
            // Wait 5 seconds before next attempt
            cellularInitTimer = millis();
            return;
        }
        
        // ---- Step 11: Query modem identity (IMEI, ICCID, IMSI, RAT) ----
        case CELL_INIT_GET_IDENTITY: {
            // IMEI
            if (modem.getIdentity(&rsp)) {
                imei = rsp.data.identity.imei;
                Serial.print("[CELL-INIT] ✓ IMEI: ");
                Serial.println(imei);
            } else {
                Serial.println("[CELL-INIT] ⚠ Could not read IMEI");
            }
            
            // ICCID
            if (modem.getSIMCardID(&rsp)) {
                modemICCID = rsp.data.simCardID.iccid;
                Serial.print("[CELL-INIT] ✓ SIM ICCID: ");
                Serial.println(modemICCID);
            } else {
                Serial.println("[CELL-INIT] ⚠ Could not read SIM ICCID");
            }
            
            // IMSI
            if (modem.getSIMCardIMSI(&rsp)) {
                modemIMSI = rsp.data.imsi;
                Serial.print("[CELL-INIT] ✓ SIM IMSI: ");
                Serial.println(modemIMSI);
            } else {
                Serial.println("[CELL-INIT] ⚠ Could not read SIM IMSI");
            }
            
            // RAT
            if (modem.getRAT(&rsp)) {
                switch (rsp.data.rat) {
                    case WALTER_MODEM_RAT_LTEM:    modemTechnology = "LTE-M";   break;
                    case WALTER_MODEM_RAT_NBIOT:   modemTechnology = "NB-IoT";  break;
                    case WALTER_MODEM_RAT_AUTO:    modemTechnology = "Auto";    break;
                    default:                       modemTechnology = "Unknown"; break;
                }
                Serial.print("[CELL-INIT] ✓ RAT: ");
                Serial.println(modemTechnology);
            }
            
            // Display identity banner
            Serial.println("\n╔═══════════════════════════════════════════════════════╗");
            Serial.println("║              DEVICE IDENTIFICATION                    ║");
            Serial.println("╠═══════════════════════════════════════════════════════╣");
            Serial.print("║  MAC:  "); Serial.print(macStr);
            for (int i = macStr.length(); i < 47; i++) Serial.print(" "); Serial.println("║");
            Serial.print("║  IMEI: "); Serial.print(imei);
            for (int i = imei.length(); i < 47; i++) Serial.print(" "); Serial.println("║");
            Serial.print("║  IMSI: "); Serial.print(modemIMSI.length() > 0 ? modemIMSI : "--");
            for (int i = (modemIMSI.length() > 0 ? modemIMSI.length() : 2); i < 47; i++) Serial.print(" "); Serial.println("║");
            Serial.print("║  ICCID:"); Serial.print(modemICCID.length() > 0 ? modemICCID : "--");
            for (int i = (modemICCID.length() > 0 ? modemICCID.length() : 2); i < 47; i++) Serial.print(" "); Serial.println("║");
            Serial.print("║  RAT:  "); Serial.print(modemTechnology.length() > 0 ? modemTechnology : "--");
            for (int i = (modemTechnology.length() > 0 ? modemTechnology.length() : 2); i < 47; i++) Serial.print(" "); Serial.println("║");
            Serial.println("╚═══════════════════════════════════════════════════════╝\n");
            
            cellularInitState = CELL_INIT_GET_TIME;
            cellularInitTimer = millis();
            return;
        }
        
        // ---- Step 12: Get time from modem (with 10-second timeout) ----
        case CELL_INIT_GET_TIME: {
            if (millis() - cellularInitTimer < 1000) return;  // Brief pause
            
            if (modem.getClock(&rsp)) {
                if (rsp.result == WALTER_MODEM_STATE_OK) {
                    currentTimestamp = rsp.data.clock.epochTime;
                    lastMillis = millis();
                    modemTimeFresh = true;
                    Serial.printf("[CELL-INIT] ✓ Modem time (UTC epoch): %lu\r\n",
                                  (unsigned long)rsp.data.clock.epochTime);
                } else {
                    Serial.println("[CELL-INIT] ⚠ Invalid timestamp from modem (non-fatal)");
                }
            } else {
                Serial.println("[CELL-INIT] ⚠ Could not get modem clock (non-fatal)");
            }
            
            cellularInitState = CELL_INIT_COMPLETE;
            return;
        }
        
        // ---- Step 13: All done ----
        case CELL_INIT_COMPLETE: {
            cellularInitComplete = true;
            lteReconnectFailures = 0;
            
            Serial.println("\n╔═══════════════════════════════════════════════════════╗");
            Serial.println("║  ✅ CELLULAR INITIALIZATION COMPLETE                  ║");
            Serial.printf("║  LTE: Connected  |  BlueCherry: %-19s  ║\r\n",
                          blueCherryConnected ? "Connected" : "Offline");
            Serial.println("╚═══════════════════════════════════════════════════════╝\n");
            return;
        }
        
        // ---- Retry cooldown: wait 60 seconds then restart from step 1 ----
        case CELL_INIT_RETRY_COOLDOWN: {
            if (millis() - cellularInitTimer < CELL_INIT_RETRY_COOLDOWN_MS) return;
            cellularInitRetryCount++;
            Serial.printf("[CELL-INIT] Cooldown complete — restarting cellular init (retry #%d)\r\n",
                          cellularInitRetryCount);
            cellularInitState = CELL_INIT_SET_MINIMUM;
            return;
        }
    }
}

/**
 * Attempt to reconnect to BlueCherry platform
 * Called periodically when BlueCherry is not connected
 * Returns true if connection successful
 * 
 * Usage example:
 *   if (!blueCherryConnected && shouldRetryBlueCherry()) {
 *       retryBlueCherryConnection();
 *   }
 */
bool retryBlueCherryConnection() {
    Serial.println("\n🔄 Retrying BlueCherry connection...");
    blueCherryLastAttempt = millis();
    
    if (modem.blueCherryInit(BC_TLS_PROFILE, otaBuffer, &rsp)) {
        blueCherryConnected = true;
        Serial.println("✓ BlueCherry reconnected successfully!");
        return true;
    } else {
        Serial.print("BlueCherry still unavailable, state: ");
        Serial.println(rsp.data.blueCherry.state);
        return false;
    }
}

/**
 * Check if enough time has passed since system start to do weekly restart
 * Only restarts if BlueCherry has never connected (to retry OTA capability)
 * 
 * Usage: Call in main loop
 */
void checkWeeklyRestartForBlueCherry() {
    // Only restart if BlueCherry never connected during this session
    if (blueCherryConnected) {
        return;  // Already connected, no need for weekly restart
    }
    
    unsigned long uptime = millis() - systemStartTime;
    
    // Check for weekly restart (7 days without BlueCherry connection)
    if (uptime >= WEEKLY_RESTART_MS) {
        Serial.println("\n╔═══════════════════════════════════════════════════════╗");
        Serial.println("║  🔄 WEEKLY RESTART - BlueCherry Retry                 ║");
        Serial.println("║  System has been running 7 days without OTA platform  ║");
        Serial.println("║  Restarting to attempt fresh BlueCherry connection    ║");
        Serial.println("╚═══════════════════════════════════════════════════════╝\n");
        delay(2000);
        ESP.restart();
    }
}

/**
 * Check for firmware updates via BlueCherry platform
 * Only attempts sync if BlueCherry is connected
 * If not connected, attempts hourly reconnection
 */
void checkFirmwareUpdate() {
    // If BlueCherry not connected, try to reconnect periodically
    if (!blueCherryConnected) {
        unsigned long timeSinceLastAttempt = millis() - blueCherryLastAttempt;
        
        // Retry every hour
        if (timeSinceLastAttempt >= BC_RETRY_INTERVAL) {
            retryBlueCherryConnection();
        } else {
            // Calculate time until next retry
            unsigned long minutesUntilRetry = (BC_RETRY_INTERVAL - timeSinceLastAttempt) / 60000;
            Serial.print("BlueCherry offline - next retry in ");
            Serial.print(minutesUntilRetry);
            Serial.println(" minutes");
        }
        return;
    }
    
    // BlueCherry is connected - check for firmware updates AND incoming messages
    Serial.println("[BC Sync] Starting sync cycle...");
    do {
        if (!modem.blueCherrySync(&rsp)) {
            Serial.printf("[BC Sync] ERROR: sync failed, state=%d\r\n", rsp.data.blueCherry.state);
            // Mark as disconnected if sync fails - DO NOT reset modem (keeps LTE alive)
            if (rsp.data.blueCherry.state != 0) {
                blueCherryConnected = false;
                blueCherryLastAttempt = millis();  // Reset retry timer
                Serial.println("\n╔═══════════════════════════════════════════════════════╗");
                Serial.println("║  ⚠️ BlueCherry sync failed - platform may be down     ║");
                Serial.println("║  System continues running (fault code 4096 active)    ║");
                Serial.println("║  Will retry hourly, weekly restart if still offline   ║");
                Serial.println("╚═══════════════════════════════════════════════════════╝");
            }
            return;
        }
        
        // Debug: Show sync results
        Serial.printf("[BC Sync] OK - messageCount=%d, syncFinished=%s\r\n", 
            rsp.data.blueCherry.messageCount,
            rsp.data.blueCherry.syncFinished ? "true" : "false");
        
        // Process incoming messages from BlueCherry platform
        // Topic 0 = firmware update request, Topic != 0 = other MQTT messages
        for (uint8_t msgIdx = 0; msgIdx < rsp.data.blueCherry.messageCount; msgIdx++) {
            if (rsp.data.blueCherry.messages[msgIdx].topic == 0) {
                // Topic 0 = Firmware update available
                Serial.println("\n📥 Firmware update available - downloading...");
            } else {
                // Non-firmware message - extract payload and check for commands
                String payload = "";
                for (uint8_t byteIdx = 0; byteIdx < rsp.data.blueCherry.messages[msgIdx].dataSize; byteIdx++) {
                    payload += (char)rsp.data.blueCherry.messages[msgIdx].data[byteIdx];
                }
                
                // Print message to Serial Monitor
                Serial.println("\n╔═══════════════════════════════════════════════════════╗");
                Serial.println("║  📨 Incoming BlueCherry Message                       ║");
                Serial.println("╠═══════════════════════════════════════════════════════╣");
                Serial.printf("║  Message %d/%d\r\n", msgIdx + 1, rsp.data.blueCherry.messageCount);
                Serial.printf("║  Topic ID: 0x%02X\r\n", rsp.data.blueCherry.messages[msgIdx].topic);
                Serial.printf("║  Data size: %d bytes\r\n", rsp.data.blueCherry.messages[msgIdx].dataSize);
                Serial.println("║  Payload: " + payload);
                Serial.println("╚═══════════════════════════════════════════════════════╝");
                
                // Convert payload to lowercase for case-insensitive comparison
                String payloadLower = payload;
                payloadLower.toLowerCase();
                
                // Debug: show what we're checking
                Serial.printf("[BC Cmd] Checking payload: '%s' (len=%d)\r\n", payloadLower.c_str(), payloadLower.length());
                
                // Check for "remote" command - enter passthrough mode
                // Format: "remote" (default 60 min) or "remote XX" (XX minutes)
                int remoteIdx = payloadLower.indexOf("remote");
                Serial.printf("[BC Cmd] indexOf('remote') = %d\r\n", remoteIdx);
                if (remoteIdx >= 0) {
                    // Parse timeout from message (e.g., "remote 30" → 30 minutes)
                    unsigned long timeoutMinutes = 60;  // Default 60 minutes
                    
                    // Look for a number after "remote"
                    int numStart = remoteIdx + 6;  // Length of "remote"
                    while (numStart < (int)payload.length() && (payload[numStart] == ' ' || payload[numStart] == '\t')) {
                        numStart++;  // Skip whitespace
                    }
                    
                    // Extract number if present
                    String numStr = "";
                    while (numStart < (int)payload.length() && isDigit(payload[numStart])) {
                        numStr += payload[numStart];
                        numStart++;
                    }
                    
                    if (numStr.length() > 0) {
                        timeoutMinutes = numStr.toInt();
                        if (timeoutMinutes < 1) timeoutMinutes = 1;      // Minimum 1 minute
                        if (timeoutMinutes > 1440) timeoutMinutes = 1440; // Maximum 24 hours
                    }
                    
                    Serial.printf("\n🔄 BlueCherry REMOTE command - passthrough for %lu minutes\r\n", timeoutMinutes);
                    enterPassthroughMode(timeoutMinutes);
                    return;  // Exit immediately - don't continue with modem operations
                }
                
                // Check for "restart" command - perform ESP restart
                int restartIdx = payloadLower.indexOf("restart");
                Serial.printf("[BC Cmd] indexOf('restart') = %d\r\n", restartIdx);
                if (restartIdx >= 0) {
                    Serial.println("\n🔄 BlueCherry RESTART command received - restarting ESP32...");
                    Serial.println("Restarting in 3 seconds...");
                    delay(3000);
                    ESP.restart();
                }

                // =========================================================
                // REV 10.18: BlueCherry command keywords routed through
                // executeRemoteCommand() for unified behavior.
                //
                // BlueCherry messages are plain-text keywords (not JSON).
                // Each keyword maps directly to a command name.
                // For relay commands, format is "cr1 1" or "cr5 0".
                //
                // Supported keywords:
                //   stop_test, overfill_override, clear_press_alarm,
                //   clear_motor_alarm, enable_failsafe, exit_failsafe,
                //   cr0/cr1/cr2/cr5 <0|1>, exit_manual,
                //   calibrate_pressure
                // =========================================================

                // --- enable_failsafe (check before generic "failsafe") ---
                if (payloadLower.indexOf("enable_failsafe") >= 0) {
                    Serial.println("[BC Cmd] enable_failsafe command");
                    executeRemoteCommand("enable_failsafe", "");
                }
                // --- exit_failsafe ---
                else if (payloadLower.indexOf("exit_failsafe") >= 0) {
                    Serial.println("[BC Cmd] exit_failsafe command");
                    executeRemoteCommand("exit_failsafe", "");
                }
                // --- stop_test ---
                else if (payloadLower.indexOf("stop_test") >= 0) {
                    Serial.println("[BC Cmd] stop_test command");
                    executeRemoteCommand("stop_test", "");
                }
                // --- overfill_override ---
                else if (payloadLower.indexOf("overfill_override") >= 0) {
                    Serial.println("[BC Cmd] overfill_override command");
                    executeRemoteCommand("overfill_override", "1");
                }
                // --- clear_press_alarm ---
                else if (payloadLower.indexOf("clear_press_alarm") >= 0) {
                    Serial.println("[BC Cmd] clear_press_alarm command");
                    executeRemoteCommand("clear_press_alarm", "");
                }
                // --- clear_motor_alarm ---
                else if (payloadLower.indexOf("clear_motor_alarm") >= 0) {
                    Serial.println("[BC Cmd] clear_motor_alarm command");
                    executeRemoteCommand("clear_motor_alarm", "");
                }
                // --- calibrate_pressure ---
                else if (payloadLower.indexOf("calibrate_pressure") >= 0) {
                    Serial.println("[BC Cmd] calibrate_pressure command");
                    executeRemoteCommand("calibrate_pressure", "");
                }
                // --- fast_poll ---
                else if (payloadLower.indexOf("fast_poll") >= 0) {
                    Serial.println("[BC Cmd] fast_poll command");
                    // Enable fast BlueCherry polling for 60 minutes max
                    fastPollingActive = true;
                    fastPollingStartTime = millis();
                    Serial.println("[FAST POLL] Enabled via BlueCherry - BlueCherry polling every 15 seconds for 60 minutes max");
                }
                // --- exit_fast_poll ---
                else if (payloadLower.indexOf("exit_fast_poll") >= 0) {
                    Serial.println("[BC Cmd] exit_fast_poll command");
                    fastPollingActive = false;
                    Serial.println("[FAST POLL] Disabled via BlueCherry - returning to scheduled polling");
                }
                // --- exit_manual ---
                else if (payloadLower.indexOf("exit_manual") >= 0) {
                    Serial.println("[BC Cmd] exit_manual command");
                    executeRemoteCommand("exit_manual", "");
                }
                // --- Direct relay commands: "cr0 1", "cr1 0", "cr2 1", "cr5 0" ---
                // Format: keyword followed by space and 0 or 1
                else if (payloadLower.startsWith("cr0 ") || payloadLower.startsWith("cr1 ") ||
                         payloadLower.startsWith("cr2 ") || payloadLower.startsWith("cr5 ")) {
                    String relayCmd = payloadLower.substring(0, 3);  // "cr0", "cr1", etc.
                    String relayVal = payloadLower.substring(4);     // "0" or "1"
                    relayVal.trim();
                    Serial.printf("[BC Cmd] Relay command: %s = %s\n", relayCmd.c_str(), relayVal.c_str());
                    executeRemoteCommand(relayCmd, relayVal);
                }
            }
        }
    } while (!rsp.data.blueCherry.syncFinished);
    
    if (rsp.data.blueCherry.messageCount > 0) {
        Serial.println("✓ BlueCherry sync complete - messages processed");
    }
}

size_t buildCborFromReadings(uint8_t* buf, size_t bufSize, int data[][10], size_t rowCount) {
  CborEncoder encoder, arrayEncoder;
  cbor_encoder_init(&encoder, buf, bufSize, 0);

  cbor_encoder_create_array(&encoder, &arrayEncoder, rowCount);

  for (size_t i = 0; i < rowCount; i++) {
    CborEncoder rowEnc;
    cbor_encoder_create_array(&arrayEncoder, &rowEnc, 8);
    for (int j = 0; j < 8; j++) {
      cbor_encode_int(&rowEnc, data[i][j]);
    }
    cbor_encoder_close_container(&arrayEncoder, &rowEnc);
  }

  cbor_encoder_close_container(&encoder, &arrayEncoder);
  size_t len = cbor_encoder_get_buffer_size(&encoder, buf);
  return len;
}

/**
 * Refresh cell signal information from the modem.
 * Updates global strings: modemBand, modemNetName, modemRSRP, modemRSRQ
 * 
 * Called periodically (every 60s) to keep web dashboard and serial JSON current.
 * Also called by buildStatusCbor before sending CBOR data to BlueCherry.
 * 
 * Usage: refreshCellSignalInfo();  // Updates globals, no return value
 */
void refreshCellSignalInfo() {
    // REV 10.10: Send serial data BEFORE the modem call, which can block for seconds.
    // This ensures the Linux device gets at least one fresh packet before we stall.
    checkAndSendStatusToSerial();
    
    unsigned long cellStart = millis();
    if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
        modemBand = String(rsp.data.cellInformation.band);
        modemNetName = String(rsp.data.cellInformation.netName);
        modemRSRP = String(rsp.data.cellInformation.rsrp, 1);   // e.g. "-89.5"
        modemRSRQ = String(rsp.data.cellInformation.rsrq, 1);   // e.g. "-11.2"
        modemCC = rsp.data.cellInformation.cc;                    // Country Code (e.g. 310)
        modemNC = rsp.data.cellInformation.nc;                    // Network Code (e.g. 260)
        modemCID = rsp.data.cellInformation.cid;                  // Cell Tower ID
        modemTAC = rsp.data.cellInformation.tac;                  // Tracking Area Code
        
        // REV 10.5/10.9: Mark modem data as FRESH for internal tracking.
        // REV 10.9: Cellular is now sent every 10s on timer (not freshness-gated),
        // so this flag is informational -- it gets cleared on next 10s send cycle.
        // The cached values (modemRSRP, modemBand, etc.) updated above will be
        // included in the next sendCellularStatusToSerial() call automatically.
        modemDataFresh = true;
        if (serialDebugMode) {
            Serial.println("[CELL] Fresh cell info retrieved -- cached for next 10s send cycle");
        }
    }
    // On failure, keep previous values rather than overwriting with "Unknown"
    // This prevents dashboard from flickering between real data and "Unknown"
    
    // REV 10.10: Log how long the modem call took (helps diagnose stalls)
    unsigned long cellDuration = millis() - cellStart;
    if (cellDuration > 1000) {
        Serial.printf("[CELL] WARNING: getCellInformation() took %lu ms (>1s stall)\r\n", cellDuration);
    }
    
    // Send serial data immediately AFTER the modem call to minimize gap
    checkAndSendStatusToSerial();
}

size_t buildErrorCbor(uint8_t* cborBuf, size_t bufSize, int errorCode) {
    CborEncoder encoder, arrayEncoder;
    cbor_encoder_init(&encoder, cborBuf, bufSize, 0);
    String idStr = String(deviceName);
    
    // Extract numeric ID (handles CSX-####, RND-####, etc.)
    if (idStr.startsWith("CSX-") || idStr.startsWith("RND-")) {
        idStr = idStr.substring(4);
    } else if (idStr.indexOf("-") >= 0) {
        idStr = idStr.substring(idStr.lastIndexOf("-") + 1);
    }
    
    int id = idStr.toInt();
    cbor_encoder_create_array(&encoder, &arrayEncoder, 2);
    cbor_encode_int(&arrayEncoder, id);
    cbor_encode_int(&arrayEncoder, errorCode);
    cbor_encoder_close_container(&encoder, &arrayEncoder);
    Serial.println("Built error CBOR message");
    return cbor_encoder_get_buffer_size(&encoder, cborBuf);
}

size_t buildStatusCbor(uint8_t* cborBuf, size_t bufSize) {
    CborEncoder encoder, arrayEncoder;
    cbor_encoder_init(&encoder, cborBuf, bufSize, 0);
    String idStr = String(deviceName);
    
    // Extract numeric ID (handles CSX-####, RND-####, etc.)
    if (idStr.startsWith("CSX-") || idStr.startsWith("RND-")) {
        idStr = idStr.substring(4);
    } else if (idStr.indexOf("-") >= 0) {
        idStr = idStr.substring(idStr.lastIndexOf("-") + 1);
    }
    
    int id = idStr.toInt();
    String sdStatus = isSDCardOK() ? "OK" : "FAULT";
    
    // Create 15-element unified status array (matches RMS Board v0.1.33 format)
    // Both IO Board and RMS Board populate a common server-side table using this layout.
    // Server receives on port 5686 and distinguishes format by array length (15 = unified).
    cbor_encoder_create_array(&encoder, &arrayEncoder, 15);
    
    // [0] Device ID (integer) - numeric part of device name (e.g., 9199 from "CSX-9199")
    cbor_encode_int(&arrayEncoder, id);
    
    // [1] Device type (string) - "IO_Board" for this device, "RMS_Board" for RMS
    cbor_encode_text_stringz(&arrayEncoder, "IO_Board");
    
    // [2] Firmware version (string) - e.g., "9.4e"
    cbor_encode_text_stringz(&arrayEncoder, ver.c_str());
    
    // [3] LTE band (integer) - e.g., 17
    cbor_encode_int(&arrayEncoder, modemBand.toInt());
    
    // [4] Network/operator name (string) - e.g., "T-Mobile"
    cbor_encode_text_stringz(&arrayEncoder, modemNetName.c_str());
    
    // [5] RSRP - signal power in dBm (string) - e.g., "-89.5"
    cbor_encode_text_stringz(&arrayEncoder, modemRSRP.c_str());
    
    // [6] RSRQ - signal quality in dB (string) - e.g., "-10.2"
    cbor_encode_text_stringz(&arrayEncoder, modemRSRQ.c_str());
    
    // [7] MAC address (string) - e.g., "AA:BB:CC:DD:EE:FF"
    cbor_encode_text_stringz(&arrayEncoder, macStr.c_str());
    
    // [8] IMEI (string) - e.g., "351234567890123"
    cbor_encode_text_stringz(&arrayEncoder, imei.c_str());
    
    // [9] PLC type (integer) - always 0 for IO Board (no PLC attached)
    cbor_encode_int(&arrayEncoder, 0);
    
    // [10] SD card status (string) - "OK" or "FAULT"
    cbor_encode_text_stringz(&arrayEncoder, sdStatus.c_str());
    
    // [11] MCC - Mobile Country Code (integer) - e.g., 310 for US
    cbor_encode_int(&arrayEncoder, modemCC);
    
    // [12] MNC - Mobile Network Code (integer) - e.g., 410 for T-Mobile
    cbor_encode_int(&arrayEncoder, modemNC);
    
    // [13] Cell ID - E-UTRAN Cell Identity (integer) - e.g., 20878097
    cbor_encode_int(&arrayEncoder, modemCID);
    
    // [14] TAC - Tracking Area Code (integer) - e.g., 10519
    cbor_encode_int(&arrayEncoder, modemTAC);
    
    cbor_encoder_close_container(&encoder, &arrayEncoder);
    return cbor_encoder_get_buffer_size(&encoder, cborBuf);
}

// Build JSON array of last 5 CBOR payloads for web diagnostics
String buildCborPayloadHistoryJson() {
    String json = "[";

    for (int i = 0; i < CBOR_PAYLOAD_HISTORY_SIZE; i++) {
        int idx = (cborPayloadHistoryIdx - 1 - i + CBOR_PAYLOAD_HISTORY_SIZE) % CBOR_PAYLOAD_HISTORY_SIZE;

        if (cborPayloadHistory[idx].size > 0) {
            if (i > 0) json += ",";
            json += "{\"timestamp\":" + String(cborPayloadHistory[idx].timestamp) + ",";
            json += "\"data\":[";

            // Convert CBOR data to array of integers for JSON
            for (size_t j = 0; j < cborPayloadHistory[idx].size; j++) {
                if (j > 0) json += ",";
                json += String(cborPayloadHistory[idx].data[j]);
            }
            json += "]}";
        }
    }

    json += "]";
    return json;
}

bool sendCborArrayViaSocket(uint8_t* buffer, size_t size) {
    int retries = 3;
    int socketId = -1;
    
    Serial.print("Attempting to send CBOR data via UDP socket, size: ");
    Serial.print(size);
    Serial.println(" bytes");
    
    while (retries > 0) {
        Serial.print("UDP socket send attempt ");
        Serial.print(4 - retries);
        Serial.println("/3");
        
        if (!modem.socketConfig(&rsp, NULL, NULL, 1, 1024, 60, 30, 5000)) {
            Serial.println("Failed to configure UDP socket");
            retries--;
            continue;
        }
        socketId = rsp.data.socketId;
        Serial.print("Configured socket ID: ");
        Serial.println(socketId);

        if(!modem.socketConfigSecure(false, 1, socketId)) {
            Serial.println("Failed to disable TLS on the UDP socket");
            retries--;
            continue;
        }
        
        if (!modem.socketConfigExtended(&rsp, NULL, NULL, socketId)) {
            Serial.println("Failed to configure UDP socket extended parameters");
            retries--;
            continue;
        }
        Serial.println("Socket extended parameters configured successfully");
        
        if (!modem.socketDial("167.172.15.241", 5689, 0, &rsp, NULL, NULL, 
                                WALTER_MODEM_SOCKET_PROTO_UDP, 
                                WALTER_MODEM_ACCEPT_ANY_REMOTE_DISABLED, socketId)) {
            Serial.println("Failed to connect UDP socket to server");
            modem.socketClose(&rsp, NULL, NULL, socketId);
            retries--;
            continue;
        }
        Serial.println("Socket connected to server");
        
        if (!modem.socketSend(buffer, size, &rsp, NULL, NULL, 
                             WALTER_MODEM_RAI_ONLY_SINGLE_RXTX_EXPECTED, socketId)) {
            Serial.println("Failed to send data via UDP socket");
            modem.socketClose(&rsp, NULL, NULL, socketId);
            retries--;
            continue;
        }
        Serial.println("CBOR data sent successfully via UDP socket!");
        
        modem.socketClose(&rsp, NULL, NULL, socketId);
        lastSendTime = currentTime;
        return true;
        
        retries--;
        if (retries > 0) {
            Serial.print("Retrying in 1 second... (");
            Serial.print(retries);
            Serial.println(" attempts remaining)");
            delay(1000);
        }
    }
    Serial.println("All UDP socket send attempts failed");
    return false;
}

bool sendCborDataViaSocket(uint8_t* buffer, size_t size) {
    int socketId = -1;
    
    Serial.print("Sending status/error data via UDP socket, size: ");
    Serial.print(size);
    Serial.println(" bytes");
    
    if (!modem.socketConfig(&rsp, NULL, NULL, 1, 1024, 60, 30, 5000)) {
        Serial.println("Failed to configure UDP socket for status/error");
        return false;
    }
    socketId = rsp.data.socketId;

    if(!modem.socketConfigSecure(false, 1, socketId)) {
        Serial.println("Failed to disable TLS on the UDP socket for status/error");
        return false;
    }
    
    if (!modem.socketConfigExtended(&rsp, NULL, NULL, socketId)) {
        Serial.println("Failed to configure UDP socket extended parameters for status/error");
        return false;
    }
    
    if (!modem.socketDial("167.172.15.241", 5686, 0, &rsp, NULL, NULL, 
                            WALTER_MODEM_SOCKET_PROTO_UDP, 
                            WALTER_MODEM_ACCEPT_ANY_REMOTE_DISABLED, socketId)) {
        Serial.println("Failed to connect UDP socket to status/error server");
        modem.socketClose(&rsp, NULL, NULL, socketId);
        return false;
    }
    
    if (!modem.socketSend(buffer, size, &rsp, NULL, NULL, 
                         WALTER_MODEM_RAI_ONLY_SINGLE_RXTX_EXPECTED, socketId)) {
        Serial.println("Failed to send status/error data via UDP socket");
        modem.socketClose(&rsp, NULL, NULL, socketId);
        return false;
    }
    
    modem.socketClose(&rsp, NULL, NULL, socketId);
    lastSendTime = currentTime;
    buffer[0] = '\0';
    Serial.println("Status/error data sent successfully via UDP socket!");
    return true;
}

bool sendErrorMessageViaSocket() {
    size_t cborSize = buildErrorCbor(cborBuffer, sizeof(cborBuffer), 1);
    if (cborSize == 0) {
        Serial.println("Failed to build error CBOR message");
        return false;
    }
    return sendCborDataViaSocket(cborBuffer, cborSize);
}

bool sendStatusUpdateViaSocket() {
    refreshCellSignalInfo();
    size_t cborSize = buildStatusCbor(cborBuffer, sizeof(cborBuffer));
    if (cborSize == 0) {
        Serial.println("Failed to build status CBOR message");
        return false;
    }

    // Store CBOR payload in history for web diagnostics
    cborPayloadHistory[cborPayloadHistoryIdx].timestamp = millis();
    memcpy(cborPayloadHistory[cborPayloadHistoryIdx].data, cborBuffer, cborSize);
    cborPayloadHistory[cborPayloadHistoryIdx].size = cborSize;
    cborPayloadHistoryIdx = (cborPayloadHistoryIdx + 1) % CBOR_PAYLOAD_HISTORY_SIZE;

    return sendCborDataViaSocket(cborBuffer, cborSize);
}

// =====================================================================
// PASSTHROUGH BOOT MODE
// =====================================================================
// Minimal boot mode for serial passthrough - runs INSTEAD of normal firmware
// when passthrough preference is set. WalterModem library is NEVER loaded.
// Only runs: 3.3V power, ADC reader, and serial passthrough bridge.
// Rev 10: Buffers CBOR data during passthrough for later transmission.
// 
// Timer: Auto-restarts after timeout (stored in preferences, default 60 min).
// =====================================================================

// Escape sequence from Linux to stop passthrough: "+++STOPPPP\n"
const char PASSTHROUGH_STOP_SEQ[] = "+++STOPPPP";
const int PASSTHROUGH_STOP_SEQ_LEN = 10;

void runPassthroughBootMode(unsigned long timeoutMinutes) {
    Serial.printf("\n[PASSTHROUGH] Boot mode active - timeout %lu min\r\n", timeoutMinutes);
    Serial.println("[PASSTHROUGH] Exit: send +++STOPPPP or timeout");
    
    // Pre-initialize overfill state
    overfillAlarmActive = false;
    overfillValidationComplete = false;
    overfillStartupTime = 0;
    overfillLowCount = 0;
    overfillHighCount = 0;
    overfillCheckTimer = 0;
    
    // =====================================================================
    // RELAY INITIALIZATION: Restore saved cycle state if a purge was in progress.
    // The relay mode and remaining seconds were saved by enterPassthroughMode()
    // so the purge step can finish quietly while the modem bridge is running.
    // =====================================================================
    pinMode(CR0_MOTOR, OUTPUT);
    pinMode(CR1, OUTPUT);
    pinMode(CR2, OUTPUT);
    pinMode(CR5, OUTPUT);
    // REV 10.11: DISP_SHUTDN initialized with hold — use the hold-aware function
    initializeDispShutdownPinWithHold();  // CRITICAL: DISP_SHUTDN ON with pull-up + hold
    
    // Read saved relay state from preferences (written by enterPassthroughMode)
    preferences.begin("RMS", true);  // Read-only
    uint8_t savedRelayMode = preferences.getUChar("ptRelayMode", 0);
    unsigned long savedRelaySec = preferences.getULong("ptRelaySec", 0);
    preferences.end();
    
    // If a cycle step was saved, restore the relay state so it continues quietly
    // Otherwise all relays start idle (OFF)
    bool purgeFinishing = (savedRelayMode > 0 && savedRelaySec > 0);
    unsigned long purgeEndTime = 0;
    
    if (purgeFinishing) {
        Serial.printf("[PASSTHROUGH] Restoring relay mode %d for %lu sec (finishing purge quietly)\r\n",
                      savedRelayMode, savedRelaySec);
        purgeEndTime = millis() + (savedRelaySec * 1000UL);
    } else {
        // No saved cycle - start with all relays OFF
        Serial.println("[PASSTHROUGH] No active cycle - relays idle");
    }
    
    // Create mutex for thread-safe relay control
    relayMutex = xSemaphoreCreateMutex();
    if (relayMutex == NULL) {
        Serial.println("[PASSTHROUGH] ERROR: Failed to create mutex!");
        ESP.restart();
    }
    
    // Set relays to saved state (or idle if no saved state)
    // This happens BEFORE the serial bridge starts, but takes < 1ms
    if (purgeFinishing) {
        setRelaysForMode(savedRelayMode);
    } else {
        setRelaysForMode(0);
    }
    
    // Start ADC reader on Core 0 (handles overfill monitoring)
    xTaskCreatePinnedToCore(
        adcReaderTask,
        "ADC_Reader",
        8192,
        NULL,
        1,
        NULL,
        0
    );
    Serial.println("[PASSTHROUGH] ADC reader task started on Core 0");
    
    // Start CBOR buffering during passthrough
    allocateCborPassthroughBuffer();
    startCborBuffering();
    
    // Configure modem pins directly (matching walter-as-linux-modem.txt)
    pinMode(WALTER_MODEM_PIN_RX, INPUT);
    pinMode(WALTER_MODEM_PIN_TX, OUTPUT);
    pinMode(WALTER_MODEM_PIN_CTS, INPUT);
    pinMode(WALTER_MODEM_PIN_RTS, OUTPUT);
    
    // Initialize ModemSerial directly (NO WalterModem library)
    ModemSerial.begin(
        115200,
        SERIAL_8N1,
        WALTER_MODEM_PIN_RX,
        WALTER_MODEM_PIN_TX,
        false,
        WALTER_MODEM_PIN_RTS,
        WALTER_MODEM_PIN_CTS
    );
    
    // Initialize Serial1 for host (Linux device)
    Serial1.setTxBufferSize(512);   // REV 10.10: Double TX buffer to prevent blocking writes during loop stalls
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    
    Serial.println("[PASSTHROUGH] Bridge ready - Serial1 <-> ModemSerial");
    
    // Calculate timeout
    unsigned long timeoutMs = timeoutMinutes * 60UL * 1000UL;
    unsigned long startTime = millis();
    unsigned long lastStatusTime = 0;
    
    // Ring buffer to detect escape sequence from host
    char escapeBuffer[PASSTHROUGH_STOP_SEQ_LEN + 1] = {0};
    int escapeIdx = 0;
    
    // Main passthrough loop (runs until timeout or early exit)
    while (true) {
        // Check for timeout
        if (millis() - startTime >= timeoutMs) {
            Serial.println("[PASSTHROUGH] Timeout - restarting...");
            delay(500);
            ESP.restart();
        }
        
        // Buffer CBOR data every 15 seconds during passthrough
        if (cborBufferingActive && millis() - lastCborBufferTime >= 15000) {
            addSampleToCborBuffer();
            lastCborBufferTime = millis();
        }
        
        // =====================================================================
        // QUIET PURGE COMPLETION: If a purge step was in progress when passthrough
        // started, let its timer expire naturally then idle all relays.
        // This runs silently in the background while the modem bridge is active.
        // =====================================================================
        if (purgeFinishing && millis() >= purgeEndTime) {
            setRelaysForMode(0);  // Idle all relays - purge step time expired
            purgeFinishing = false;
            Serial.println("[PASSTHROUGH] Purge step completed quietly - relays idled");
        }
        
        // Emergency: if overfill trips during passthrough, stop relays immediately
        if (purgeFinishing && overfillAlarmActive) {
            setRelaysForMode(0);
            purgeFinishing = false;
            Serial.println("[PASSTHROUGH] Overfill alarm - relays idled immediately");
        }
        
        // Forward data from host (Serial1) to modem (ModemSerial)
        // Also watch for escape sequence "+++STOPPPP"
        if (Serial1.available()) {
            int x = Serial1.read();
            
            // Check for escape sequence
            if (x == PASSTHROUGH_STOP_SEQ[escapeIdx]) {
                escapeIdx++;
                if (escapeIdx >= PASSTHROUGH_STOP_SEQ_LEN) {
                    // Escape sequence detected - restart to normal mode
                    Serial.println("[PASSTHROUGH] Stop signal received - restarting...");
                    delay(500);
                    ESP.restart();
                }
                // Don't forward escape sequence characters to modem
            } else {
                // Not part of escape sequence - forward any buffered chars and this one
                for (int i = 0; i < escapeIdx; i++) {
                    ModemSerial.write(PASSTHROUGH_STOP_SEQ[i]);
                }
                escapeIdx = 0;
                
                // Check if this char starts a new escape sequence
                if (x == PASSTHROUGH_STOP_SEQ[0]) {
                    escapeIdx = 1;
                } else {
                    ModemSerial.write(x);
                }
            }
        }
        
        // Forward data from modem (ModemSerial) to host (Serial1)
        if (ModemSerial.available()) {
            int x = ModemSerial.read();
            Serial1.write(x);
        }
        
        // Flash red LED during passthrough mode (500ms on/500ms off)
        updatePassthroughLED();
        
        // Periodic status (every 5 minutes - less frequent)
        if (millis() - lastStatusTime >= 300000) {
            unsigned long remaining = (timeoutMs - (millis() - startTime)) / 60000;
            Serial.printf("[PASSTHROUGH] %lu min remaining\r\n", remaining);
            lastStatusTime = millis();
        }
        
        yield();  // Allow other tasks to run
    }
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
    // =====================================================================
    // CRITICAL: Enable 3.3V power FIRST (before anything else)
    // =====================================================================
    pinMode(ESP_POWER_EN, OUTPUT);
    digitalWrite(ESP_POWER_EN, LOW);
    
    // =====================================================================
    // REV 10.11: DISP_SHUTDN (GPIO13 / CR6) — initialize with pull-up + hold IMMEDIATELY.
    // This MUST happen before Serial.begin or any other initialization.
    // gpio_hold_en() latches the pin state through software resets (ESP.restart, OTA).
    // The internal pull-up provides backup protection during full power-on resets.
    // Without this, GPIO13 floats LOW for ~100-500ms during reboot, causing
    // an unwanted site shutdown on the relay board.
    // =====================================================================
    initializeDispShutdownPinWithHold();
    
    Serial.begin(115200);
    delay(100);  // Brief delay for serial to stabilize
    
    // Initialize WS2812B status LED early (before passthrough check)
    initializeStatusLED();
    
    // Check for passthrough boot mode (before loading WalterModem library)
    preferences.begin("RMS", false);
    bool passthroughBoot = preferences.getBool("ptBoot", false);
    unsigned long ptTimeout = preferences.getULong("ptTimeout", 60);
    
    if (passthroughBoot) {
        // Clear preference IMMEDIATELY so next boot is normal
        preferences.putBool("ptBoot", false);
        preferences.end();
        
        // Run passthrough mode (never returns - ends with ESP.restart)
        runPassthroughBootMode(ptTimeout);
    }
    preferences.end();
    
    // =====================================================================
    // Pre-initialize relay and overfill state to SAFE values
    // =====================================================================
    overfillAlarmActive = false;
    overfillValidationComplete = false;
    overfillStartupTime = 0;
    overfillLowCount = 0;
    overfillHighCount = 0;
    overfillCheckTimer = 0;
    dispShutdownActive = true;             // DISP_SHUTDN starts HIGH (ON)
    currentRelayMode = 0;                  // Idle mode
    
    // =====================================================================
    // Load pressure sensor calibration from EEPROM (before ADC task starts)
    // =====================================================================
    loadPressureCalibrationFromEeprom();
    
    // =====================================================================
    // Create mutex and start ADS1015 reader task on Core 0
    // =====================================================================
    relayMutex = xSemaphoreCreateMutex();
    if (relayMutex == NULL) {
        Serial.println("Failed to create relay mutex!");
        ESP.restart();
    }
    Serial.println("✓ Mutex created for thread-safe relay control");
    
    // Start ADC reader as FreeRTOS task on Core 0 (replaces MCP emulator)
    xTaskCreatePinnedToCore(
        adcReaderTask,
        "ADC_Reader",
        8192,
        NULL,
        1,
        NULL,
        0
    );
    Serial.println("✓ ADS1015 ADC reader started on Core 0 (60Hz polling)");
    
    // Now continue with rest of setup
    delay(500);
    
    Serial.println("\n╔═══════════════════════════════════════════════════════╗");
    Serial.println("║  Walter IO Board Firmware - Rev 10.19                ║");
    Serial.println("║  ADS1015 ADC + Failsafe Relay Control                ║");
    Serial.println("╚═══════════════════════════════════════════════════════╝\n");
    
    // Load active profile from EEPROM
    profileManager.loadActiveProfile();
    
    // Allocate CBOR passthrough buffer
    allocateCborPassthroughBuffer();
    
    // Initialize watchdog pin with internal pull-down
    // The pull-down ensures GPIO39 stays LOW during reboots/firmware updates,
    // preventing a floating pin from triggering an unwanted watchdog pulse.
    // Same approach as the DISP_SHUTDN pull-up — uses ESP-IDF GPIO functions.
    pinMode(ESP_WATCHDOG_PIN, OUTPUT);
    digitalWrite(ESP_WATCHDOG_PIN, LOW);
    gpio_pulldown_en((gpio_num_t)ESP_WATCHDOG_PIN);   // Internal pull-down (~45kΩ) keeps pin LOW
    gpio_pullup_dis((gpio_num_t)ESP_WATCHDOG_PIN);    // Disable pull-up (conflicting with pull-down)
    Serial.println("✓ Serial watchdog pin (GPIO39) initialized");
    Serial.println("  Internal pull-down: ENABLED (prevents floating during resets)");
    
    delay(500);
    
    // Manage SD Card mounting (with updated CS pin)
    initSdLogging(); 

    // Get device name and watchdog setting from EPROM
    // Send frequency is now controlled by Python (15 seconds), not stored here
    preferences.begin("RMS", false);
    DeviceName = preferences.getString("DeviceName", "CSX-9000");
    watchdogEnabled = preferences.getBool("watchdog", true);   // Default to ENABLED for safety
    failsafeEnabled = preferences.getBool("failsafe", false);  // Default to DISABLED
    serialDebugMode = preferences.getBool("debugMode", false); // Default to OFF (quiet Serial Monitor)
    preferences.end();
    
    Serial.print("✓ Loaded watchdog setting from EPROM: ");
    Serial.println(watchdogEnabled ? "ENABLED" : "DISABLED");
    Serial.print("✓ Loaded failsafe setting from EPROM: ");
    Serial.println(failsafeEnabled ? "ENABLED" : "DISABLED");
    Serial.print("✓ Loaded debug mode from EPROM: ");
    Serial.println(serialDebugMode ? "ON (verbose)" : "OFF (quiet)");

    // Check for pending backfill procedure from previous passthrough session
    preferences.begin("RMS", false);
    backfillPending = preferences.getBool("backfillPending", false);
    passthroughStartTime = preferences.getString("backfillStartTime", "");
    purgeNeeded = preferences.getBool("purgeNeeded", false);
    preferences.end();

    if (backfillPending) {
        Serial.println("[BACKFILL] Backfill procedure pending from previous passthrough session");
        Serial.printf("[BACKFILL] Passthrough start time: %s\r\n", passthroughStartTime.c_str());
        Serial.println("[BACKFILL] Executing data backfill to Linux device...");

        // Clear the flag immediately to prevent re-running on next boot
        preferences.begin("RMS", false);
        preferences.putBool("backfillPending", false);
        preferences.end();

        // Execute backfill procedure
        dataBackfill();

        // Clear the start time after backfill completes
        preferences.begin("RMS", false);
        preferences.putString("backfillStartTime", "");
        preferences.end();
    }

    // Fixed send interval (Python sends every 15 seconds)
    sendInterval = 15000;  // 15 seconds in milliseconds
    
    strncpy(deviceName, DeviceName.c_str(), sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';
    
    // Extract numeric ID from device name (handles CSX-####, RND-####, etc.)
    if (DeviceName.startsWith("CSX-") || DeviceName.startsWith("RND-")) {
        idStr = DeviceName.substring(4);
    } else if (DeviceName.indexOf("-") >= 0) {
        idStr = DeviceName.substring(DeviceName.lastIndexOf("-") + 1);
    }
    
    int id = idStr.toInt();
    
    Serial.println("Starting WiFi Access Point...");
    initializeAPName();  // Generate/load AP name before starting AP
    startConfigAP();
    Serial.print("WiFi AP '");
    Serial.print(apSSID);
    Serial.println("' ready");
    Serial.println("Connect and navigate to 192.168.4.1");
    
    Serial.println("#########################################");
    Serial.println("##   Walter IO Board Firmware " + ver + "   ##");
    Serial.println("#########################################");
    
    // Start SPI for SD Card (updated CS pin)
    SPI.begin(SCLK, MISO, MOSI, CS);
    if (!SD.begin(CS)) Serial.println("**** ERROR - SD Card not Mounted");
    else Serial.println("**** SD Card Mounted (CS=40)");
    
    // Initialize RS-232 Serial data
    Serial1.setTxBufferSize(512);   // REV 10.10: Double TX buffer to prevent blocking writes during loop stalls
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.println("✓ RS-232 Serial1 initialized (115200 baud)");

    // REV 10.13: modem.begin() can hang if the modem hardware isn't responding.
    // Print a message BEFORE the call so the Serial Monitor shows where we are.
    Serial.println("Initializing Walter modem (modem.begin)...");
    Serial.println("  If boot stops here, the modem hardware may not be responding.");
    Serial.println("  Common causes: modem not powered, UART wiring, modem firmware issue.");
    Serial.flush();  // Force all buffered serial out BEFORE the potentially blocking call
    
    if (!modem.begin(&ModemSerial)) {
        Serial.println("Could not initialize the modem, restarting in 10 seconds");
        Modem_Status = "Could Not Initialize Modem";
        delay(10000);
        ESP.restart();
    }
    Serial.println("✓ Modem UART initialized successfully");
    
    if (modem.getRAT(&rsp) && rsp.data.rat == WALTER_MODEM_RAT_NBIOT) {
        if (modem.setRAT(WALTER_MODEM_RAT_LTEM)) {
            Serial.println("Switched modem to LTE-M mode");
            modem.reset();
        }
    }
    
    // =====================================================================
    // REV 10.13: NON-BLOCKING CELLULAR INITIALIZATION
    // =====================================================================
    // LTE registration, BlueCherry, modem identity, and modem time are now
    // handled by runCellularInitStateMachine() in loop() instead of blocking
    // here for up to 5+ minutes. This allows serial to Linux, web portal,
    // relay control, and watchdog to start immediately.
    //
    // modem.begin() and RAT check stay here — they're fast and required
    // before any modem commands can be issued.
    // =====================================================================
    
    // Read MAC address early (needed for web dashboard, CBOR, and identity display)
    uint8_t mac[8] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buffer[18];
    snprintf(buffer, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    macStr = buffer;
    if (macStr == "00:00:00:00:00:00" || macStr.length() == 0) {
        Serial.println("⚠️ WARNING: MAC address is blank or invalid!");
    } else {
        Serial.println("✓ MAC address: " + macStr);
    }
    
    // Initialize cellular state tracking
    systemStartTime = millis();
    blueCherryConnected = false;
    cellularInitComplete = false;
    cellularInitState = CELL_INIT_SET_MINIMUM;
    Modem_Status = "Connecting...";
    
    Serial.println("✓ Modem hardware initialized — cellular connection will proceed in background");
    Serial.println("  (LTE registration, BlueCherry, and modem time run non-blocking in loop)");
    
    // Initialize watchdog timer
    resetSerialWatchdog();
    Serial.println("✓ Serial watchdog timer initialized");
    
    Serial.println("\n✅ Walter IO Board Firmware Rev 10.19 initialization complete!");
    Serial.println("✅ ADS1015 ADC reader running on Core 0 (60Hz, address 0x48)");
    Serial.println("✅ SPA web interface active with " + String(PROFILE_COUNT) + " profiles");
    Serial.println("✅ Active profile: " + profileManager.getActiveProfileName());
    Serial.println("✅ Serial watchdog ready");
    Serial.println("✅ Failsafe relay control standby (activates after 2 restart attempts)");
    Serial.printf("✅ CBOR buffer: %d samples (%d min capacity)\r\n", CBOR_BUFFER_MAX_SAMPLES, CBOR_BUFFER_MAX_SAMPLES * 15 / 60);
    Serial.println("⏳ Cellular connecting in background (LTE + BlueCherry + time)");
    Serial.println("✅ System ready — IO operational!\n");
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
    // REV 10.10: Loop stall detector — prints if the loop takes >2 seconds between iterations
    // This catches any blocking operation (modem calls, I2C hangs, SD card retries, etc.)
    // that prevents serial data from being sent to the Linux device.
    static unsigned long lastLoopTime = 0;
    if (lastLoopTime > 0) {
        unsigned long loopDelta = millis() - lastLoopTime;
        if (loopDelta > 2000) {
            Serial.println("####################################################################");
            Serial.printf("## LOOP STALL DETECTED: %lu ms since last iteration\r\n", loopDelta);
            Serial.printf("## Serial data was NOT sent to Linux for %.1f seconds!\r\n", loopDelta / 1000.0);
            Serial.printf("## Free heap: %lu bytes  |  @%lums\r\n", (unsigned long)ESP.getFreeHeap(), millis());
            Serial.println("####################################################################");
        }
    }
    lastLoopTime = millis();
    
    // Debug: confirm loop is running
    static unsigned long lastLoopDebug = 0;
    static bool firstLoop = true;
    if (firstLoop || (passthroughMode && millis() - lastLoopDebug > 2000)) {
        Serial.printf("[LOOP] Running, passthroughMode=%d\r\n", passthroughMode);
        lastLoopDebug = millis();
        firstLoop = false;
    }
    
    // =====================================================================
    // PASSTHROUGH MODE - Bridge Serial1 (RS-232) to modem
    // When active, normal firmware operations are suspended (INCLUDING LTE check)
    // ADC reader continues running on Core 0
    // Web server continues running (handled by AsyncWebServer)
    // MUST be checked FIRST - before any modem operations
    // =====================================================================
    if (passthroughMode) {
        // Buffer CBOR data during passthrough
        if (cborBufferingActive && millis() - lastCborBufferTime >= 15000) {
            addSampleToCborBuffer();
            lastCborBufferTime = millis();
        }
        runPassthroughLoop();
        delay(1);
        return;
    }
    
    // =====================================================================
    // REV 10.13: NON-BLOCKING CELLULAR INITIALIZATION
    // Run the cellular state machine until initialization is complete.
    // This replaces the blocking lteConnect() + BlueCherry + initModemTime()
    // that previously ran in setup(). Each call advances one state and returns
    // immediately, so serial data, web portal, and relay control keep working.
    // =====================================================================
    if (!cellularInitComplete) {
        runCellularInitStateMachine();
    }
    
    // =====================================================================
    // LTE CONNECTION CHECK — SCHEDULED (every 60 seconds)
    // Only runs AFTER cellular initialization is complete.
    // If LTE drops, serial data keeps flowing. Reconnection is attempted on
    // the next scheduled check. After 5 consecutive failures, ESP restarts.
    // =====================================================================
    if (cellularInitComplete && currentTime - lastLteCheckTime >= LTE_CHECK_INTERVAL) {
        lastLteCheckTime = currentTime;
        if (!lteConnected()) {
            Serial.println("####################################################################");
            Serial.printf("## LTE CONNECTION LOST — attempting reconnect (failure #%d)\r\n",
                          lteReconnectFailures + 1);
            Serial.println("## Serial data to Linux continues during reconnection attempt");
            Serial.println("####################################################################");
            if (!lteConnect()) {
                lteReconnectFailures++;
                Serial.printf("[LTE] Reconnect FAILED — will retry in 60 seconds (failures: %d/5)\r\n",
                              lteReconnectFailures);
                if (lteReconnectFailures >= 5) {
                    Serial.println("[LTE] 5 consecutive failures — restarting ESP32");
                    delay(1000);
                    ESP.restart();
                }
            } else {
                lteReconnectFailures = 0;
                Serial.println("[LTE] Reconnection successful — resuming normal operation");
            }
        } else {
            lteReconnectFailures = 0;
        }
    }
    
    // =====================================================================
    // NORMAL OPERATION MODE
    // =====================================================================
    currentTime = millis();
    dataBuf[6] = counter >> 8;
    dataBuf[7] = counter & 0xFF;
    
    if (currentTime - readTime >= readInterval) {
        updateTimestamp();
        readSerialData();
        checkSerialWatchdog();  // Check if watchdog timeout has been exceeded
        readTime = currentTime;
    }
    
    if (currentTime - lastParseTime >= parseSDInterval) {
        Serial.print("*** Parse/SD write interval *****");
        parsedSerialData();
        lastParseTime = currentTime;
    }
    // Send normal CBOR Payload
    if (currentTime - lastSendTime >= sendInterval) {
        buildSendDataArray();
        Serial.print("*** Reading Count= ");
        Serial.println(readingCount);
        lastSendTime = currentTime;
    }

    if ((currentTime - lastSendTime >= noComInterval && NoSerial == 1) || (firstLostComm == 1 && NoSerial == 1)) {
        Serial.print("#################################\n############### Sending Serial Error Message#################\n####################################");
        sendErrorMessageViaSocket();
        firstLostComm = 0;
    }
    
    // =====================================================================
    // SCHEDULED FIRMWARE CHECK (7:00 AM - 1:00 PM EST, every 15 min)
    // Also runs once on first boot to sync modem time and check for updates
    // REV 10.13: Only runs after cellular init completes (modem time, BlueCherry
    // are handled by the state machine during boot).
    // =====================================================================
    if (cellularInitComplete && (firstTime || isFirmwareCheckScheduled())) {
        Serial.println("[FIRMWARE] Scheduled check - syncing modem time & checking updates");
        checkFirmwareUpdate();
        initModemTime();
        lastFirmwareCheckMillis = currentTime;
        firstTime = false;
        
        // Check if weekly restart is needed (only if BlueCherry never connected)
        checkWeeklyRestartForBlueCherry();
    }
    
    // =====================================================================
    // DAILY STATUS MESSAGE (Unified 15-element format - sends to port 5686)
    // Sends once on first boot and then every 24 hours.
    // REV 10.13: Gated on cellularInitComplete — needs LTE connection for socket send.
    // =====================================================================
    if (cellularInitComplete && (lastDailyStatusTime == 0 || currentTime - lastDailyStatusTime >= STATUS_INTERVAL)) {
        Serial.println("##################################################");
        Serial.println("##### DAILY STATUS - IO_Board (Unified Format) ####");
        Serial.println("##################################################");
        refreshCellSignalInfo();
        if (sendStatusUpdateViaSocket()) {
            Serial.println("##### Status sent successfully #####");
        } else {
            Serial.println("##### Status send failed #####");
        }
        Serial.println("##################################################");
        lastDailyStatusTime = currentTime;
    }
    
    // Refresh cell signal info every 60 seconds for web dashboard and serial JSON
    // This updates modemRSRP, modemRSRQ, modemBand, modemNetName globals
    // REV 10.13: refreshCellSignalInfo only runs after cellular init completes
    if (currentTime - lastSDCheck >= 60000) {
        if (cellularInitComplete) refreshCellSignalInfo();
        Serial.print("@@@@@@  Free heap: ");
        Serial.println(ESP.getFreeHeap());
        Serial.println("@@@@@@");
        if (!SD.cardType()) {
            Serial.println("SD card not mounted, attempting to remount");
            int attempts = 0;
            const int maxAttempts = 3;
            while (attempts < maxAttempts) {
                if (SD.begin(CS)) {
                    Serial.println("SD card remounted successfully on attempt " + String(attempts + 1));
                    time_t rawTime = (time_t)currentTimestamp;
                    struct tm *timeInfo = gmtime(&rawTime);
                    int currentYear = timeInfo->tm_year + 1900;
                    String logFileName = "/logfile" + String(currentYear) + ".log";
                    logFile = SD.open(logFileName.c_str(), FILE_APPEND);
                    if (!logFile) {
                        Serial.print(F("Error: could not open logfile after remount "));
                        Serial.print(logFileName);
                        Serial.println(F(" for appending"));
                    } else {
                        Serial.println("Log file opened after remount: " + logFileName);
                    }
                    break;
                }
                attempts++;
                Serial.println("Failed to remount SD card, attempt " + String(attempts) + " of " + String(maxAttempts));
                delay(1000);
            }
            if (attempts >= maxAttempts) {
                Serial.println("Failed to remount SD card after " + String(maxAttempts) + " attempts");
            }
        }
        lastSDCheck = currentTime;
    }
    
    // REV 10.5: Send serial data to Python device via RS-232
    // Fast sensors (pressure, current, overfill, sdcard) at 5Hz (every 200ms)
    // Cellular status (datetime, lte, rsrp, rsrq) only when fresh from modem
    checkAndSendStatusToSerial();
    
    // =====================================================================
    // REV 10.8: PERIODIC RELAY STATE REFRESH (every 15 seconds)
    // Re-applies currentRelayMode to physical GPIO pins to compensate for:
    //   - Missed mode commands (serial packet lost)
    //   - Relay driver glitch or failed GPIO write
    //   - Drift between currentRelayMode variable and actual pin states
    // Skipped during failsafe mode (failsafe manages its own relay cycle).
    // Also skipped during passthrough (relays should be idle).
    //
    // The Linux 15-second payload also re-sends the mode field, so even if
    // this refresh re-applies an old mode, the next Linux packet will correct
    // it. Worst-case recovery from a missed mode command: 15 seconds.
    // =====================================================================
    // REV 10.18: Skip relay refresh during manual relay override — the user
    // is controlling individual relay pins directly via cr0/cr1/cr2/cr5 commands.
    // Refreshing would overwrite their manual settings with the current mode.
    if (!failsafeMode && !passthroughMode && !manualRelayOverride &&
        (currentTime - lastRelayRefreshTime >= RELAY_REFRESH_INTERVAL)) {
        setRelaysForMode(currentRelayMode);
        if (serialDebugMode) {
            Serial.printf("[RELAY-REFRESH] Re-applied mode %d to relay pins (every %lus)\r\n",
                          currentRelayMode, RELAY_REFRESH_INTERVAL / 1000);
        }
        lastRelayRefreshTime = currentTime;
    }
    
    // =====================================================================
    // REV 10.11: WEB PORTAL TEST ENGINE
    // When a web portal test is running, the ESP32 owns the relays.
    //
    // Two types of tests:
    //   1. Single-mode (leak, eff, clean): Hold one relay mode, auto-stop on timer.
    //   2. Multi-step (func): Step through a CycleStep sequence, auto-stop on completion.
    //
    // Both types block ALL Linux mode commands via the testRunning guard.
    // =====================================================================
    if (testRunning) {
        if (webTestMultiStep && webTestCycleSequence != NULL) {
            // --- MULTI-STEP TEST (functionality test) ---
            // Step through the CycleStep sequence, advancing when each step's timer expires.
            // Mirrors the failsafe cycle engine but runs independently.
            unsigned long stepDurationMs = (unsigned long)webTestCycleSequence[webTestCycleStep].durationSeconds * 1000UL;
            
            if (millis() - webTestCycleStepTimer >= stepDurationMs) {
                // Current step complete — advance to next
                webTestCycleStep++;
                
                if (webTestCycleStep >= webTestCycleLength) {
                    // All steps complete — test finished
                    Serial.printf("[WEB TEST] ===== MULTI-STEP TEST AUTO-COMPLETED =====\r\n");
                    Serial.printf("[WEB TEST]   Type: %s  |  Steps: %d  |  COMPLETED\r\n",
                                  activeTestType.c_str(), webTestCycleLength);
                    Serial.printf("[WEB TEST]   Relays → idle (mode 0)  |  Linux resumes relay control\r\n");
                    
                    setRelaysForMode(0);
                    
                    // Notify Linux
                    char notifyBuf[128];
                    snprintf(notifyBuf, sizeof(notifyBuf),
                             "{\"command\":\"test_complete\",\"type\":\"%s\"}", activeTestType.c_str());
                    Serial1.println(notifyBuf);
                    
                    testRunning = false;
                    activeTestType = "";
                    webTestMultiStep = false;
                } else {
                    // Start the next step
                    uint8_t nextMode = webTestCycleSequence[webTestCycleStep].mode;
                    uint16_t nextDur = webTestCycleSequence[webTestCycleStep].durationSeconds;
                    setRelaysForMode(nextMode);
                    webTestCycleStepTimer = millis();
                    
                    // Log step transition
                    const char* modeName = (nextMode == 1) ? "Run" : (nextMode == 2) ? "Purge" : 
                                           (nextMode == 3) ? "Burp" : "?";
                    Serial.printf("[WEB TEST] Step %d/%d: mode %d (%s) for %d sec  [rep %d of %d]\r\n",
                                  webTestCycleStep, webTestCycleLength,
                                  nextMode, modeName, nextDur,
                                  (webTestCycleStep / 2) + 1, webTestCycleLength / 2);
                }
            }
        } else {
            // --- SINGLE-MODE TEST (leak, eff, clean) ---
            // Hold one relay mode, auto-stop when the timer expires.
            unsigned long testDuration = getTestDurationSeconds(activeTestType);
            unsigned long testElapsed = (millis() - testStartTime) / 1000;
            
            if (testDuration > 0 && testElapsed >= testDuration) {
                Serial.printf("[WEB TEST] ===== WEB PORTAL TEST AUTO-COMPLETED =====\r\n");
                Serial.printf("[WEB TEST]   Type: %s  |  Duration: %lu sec  |  COMPLETED\r\n",
                              activeTestType.c_str(), testDuration);
                Serial.printf("[WEB TEST]   Relays → idle (mode 0)  |  Linux resumes relay control\r\n");
                
                setRelaysForMode(0);
                
                // Notify Linux
                char notifyBuf[128];
                snprintf(notifyBuf, sizeof(notifyBuf),
                         "{\"command\":\"test_complete\",\"type\":\"%s\"}", activeTestType.c_str());
                Serial1.println(notifyBuf);
                
                testRunning = false;
                activeTestType = "";
            }
        }
    }
    
    // =====================================================================
    // FAILSAFE MODE CHECK (Rev 10)
    // After 2 watchdog restart attempts with no serial response, ESP32
    // takes over relay control autonomously using ADC sensor data.
    // =====================================================================
    if (shouldEnterFailsafeMode()) {
        enterFailsafeMode();
    }
    
    // Run failsafe cycle logic (only executes when failsafeMode is true)
    runFailsafeCycleLogic();
    
    // Update WS2812B status LED color based on current operating mode
    // Blue breathing=Idle, Green=Run, Orange=Purge, Yellow=Burp
    updateStatusLED();
    
    delay(100);
}
