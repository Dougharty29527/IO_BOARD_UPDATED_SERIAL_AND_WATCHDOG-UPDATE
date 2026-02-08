/* ********************************************
 *  
 *  Walter IO Board Firmware - Rev 10.0
 *  Date: 2/8/2026
 *  Written By: Todd Adams & Doug Harty
 *  
 *  Based on:
 *  - Rev 9.4f (MCP23017 Emulator + Diagnostic Dashboard)
 *  - RMS_CBOR_1_16 (Walter RMS w/OTA and ZTA)
 *  - Python control.py (Relay cycle logic, profile management, alarm monitoring)
 *  
 *  =====================================================================
 *  REVISION HISTORY
 *  =====================================================================
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
 *  - Added fault code 4096 when BlueCherry is offline
 *  - Fault codes: 1024=watchdog, 4096=BlueCherry, 5120=both
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
 *  Rev 10.0 (2/8/2026) - MAJOR REWRITE: MCP Emulation Removed, Serial Relay Control
 *  - REMOVED: MCP23017 I2C slave emulation (all 22 registers, handlers, task)
 *  - NEW: ESP32 is now I2C MASTER reading ADS1015 ADC at 0x48
 *    * Channel 0: Pressure sensor (single-ended, 200-sample rolling average)
 *    * Channels 2&3: Current monitoring (abs differential, 20-sample rolling avg)
 *    * 60Hz polling on Core 0 FreeRTOS task with error recovery
 *  - NEW: Relay control via serial JSON mode field (replaces I2C register writes)
 *    * Linux sends {"type":"data",...,"mode":0,...} → ESP32 sets relays for mode
 *    * setRelaysForMode(): Mode 0=Idle, 1=Run, 2=Purge, 3=Burp, 8=FreshAir
 *  - NEW: Failsafe relay control when Comfile is down (after 2 restart attempts)
 *    * Full autonomous cycle logic: pressure auto-start, run/purge/burp sequences
 *    * Current monitoring: low (<3A for 9 readings), high (>25A for 2s) protection
 *    * Adds 8192 to fault code when in failsafe mode
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
 *  FAULT CODES
 *  =====================================================================
 *  - 1024: Serial watchdog triggered (no serial data for 30+ minutes)
 *  - 4096: BlueCherry platform offline (OTA updates unavailable)
 *  - 5120: Both watchdog AND BlueCherry faults active
 *  - 8192: Panel down - ESP32 in failsafe mode (Comfile not responding)
 *  - Note: These are ADDED to any existing fault codes from the device
 *  
 ***********************************************/

// Define the software version as a macro
#define VERSION "Rev 10.0"
String ver = VERSION;

// Password required to change device name or toggle watchdog via web portal
// Change this to your desired password. Case-sensitive, sent as URL parameter.
#define CONFIG_PASSWORD "1793"

// ### Libraries ###
#include <esp_mac.h>
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
bool   sendCborArrayViaSocket(uint8_t* buffer, size_t size);
void   cleanupOldLogFiles(int currentYear);
String formatTimestamp(int64_t seconds);
bool   lteConnected();
bool   isSDCardOK();
String getDateTimeString();
String getJsonValue(const char* json, const char* key);
void   parsedSerialData();
void   sendFastSensorPacket();

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
volatile float adcPressure = 0.0;       // Pressure in IWC (200-sample rolling average)
volatile float adcCurrent = 0.0;        // Abs current in Amps (20-sample rolling average)
volatile float adcPeakCurrent = 0.0;    // Peak current reading (for high-current detection)
volatile int16_t adcRawPressure = 0;    // Raw ADC value for channel 0 (diagnostics)
volatile int16_t adcRawCurrent = 0;     // Raw abs(ch2-ch3) value (diagnostics)
volatile uint32_t adcErrorCount = 0;    // Consecutive read errors (resets on success)
volatile uint32_t adcTotalErrors = 0;   // Total lifetime read errors (diagnostics)
volatile bool adcInitialized = false;   // True once ADS1015 responds successfully

// ADC calibration constants (from Python control.py)
// Pressure: mapit(adc_value, adc_zero, 22864, 0.0, 20.8)
// Note: ADS1015 is 12-bit (0-2047) vs ADS1115 16-bit (0-32767)
// Scale factor: 12-bit values are ~16x smaller than 16-bit
float adcZeroPressure = 964.0;    // 15422/16 ≈ 964 (12-bit equivalent of 15422)
float adcFullPressure = 1429.0;   // 22864/16 ≈ 1429 (12-bit equivalent of 22864)
float pressureRangeIWC = 20.8;    // Full scale pressure in IWC

// Current: mapit(adc_rms, 1248, 4640, 2.1, 8.0) for 16-bit
// 12-bit equivalents: 1248/16=78, 4640/16=290
float adcZeroCurrent = 78.0;      // 12-bit equivalent
float adcFullCurrent = 290.0;     // 12-bit equivalent
float currentRangeLow = 2.1;      // Amps at zero ADC
float currentRangeHigh = 8.0;     // Amps at full ADC

// Rolling average buffers
#define PRESSURE_AVG_SAMPLES 200
#define CURRENT_AVG_SAMPLES 20
float pressureBuffer[PRESSURE_AVG_SAMPLES] = {0};
float currentBuffer[CURRENT_AVG_SAMPLES] = {0};
int pressureBufferIdx = 0;
int currentBufferIdx = 0;
int pressureSampleCount = 0;       // Tracks how many samples collected (for initial fill)
int currentSampleCount = 0;

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
// FAILSAFE MODE GLOBALS (Rev 10 - autonomous relay control when Comfile is down)
// =====================================================================
bool failsafeMode = false;                  // True when ESP32 controls relays autonomously
const int FAILSAFE_FAULT_CODE = 8192;       // Added to fault code when in failsafe
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
unsigned long lastCborBufferTime = 0; // Last time a sample was buffered

// Thread safety
SemaphoreHandle_t relayMutex = NULL;

// Current mode tracking (read-only from web dashboard, set by I2C master)
RunMode current_mode = IDLE_MODE;

// Serial watchdog variables
bool watchdogEnabled = true;                     // Watchdog feature enable/disable flag (default: enabled)
unsigned long lastSerialDataTime = 0;            // Timestamp of last received serial data
bool watchdogTriggered = false;                  // Flag to track if we're in watchdog state (no serial)
int watchdogRebootAttempts = 0;                  // Counter for reboot attempts (0 = no attempts yet)
unsigned long lastWatchdogPulseTime = 0;         // Timestamp of last watchdog pulse
const unsigned long WATCHDOG_TIMEOUT = 1800000;  // 30 minutes in milliseconds (30 * 60 * 1000)
const unsigned long WATCHDOG_FIRST_PULSE = 1000; // First reboot attempt: 1 second pulse
const unsigned long WATCHDOG_LONG_PULSE = 30000; // Subsequent attempts: 30 second pulse
const int WATCHDOG_FAULT_CODE = 1024;            // Fault code to send when no serial data
const int BLUECHERRY_FAULT_CODE = 4096;          // Fault code to send when BlueCherry is disconnected

// BlueCherry connection tracking variables
// Allows system to run without BlueCherry, but schedules weekly restart for OTA retry
bool blueCherryConnected = false;                // True if BlueCherry initialized successfully
unsigned long blueCherryLastAttempt = 0;         // Timestamp of last BlueCherry connection attempt
unsigned long systemStartTime = 0;               // Timestamp when system started (for weekly restart)
const unsigned long WEEKLY_RESTART_MS = 604800000UL;  // 7 days in milliseconds (7 * 24 * 60 * 60 * 1000)
const unsigned long BC_RETRY_INTERVAL = 3600000; // Retry BlueCherry every hour (60 * 60 * 1000)

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
String imei = "";
String macStr = "";
int64_t currentTimestamp = 0;
uint32_t lastMillis = 0;
bool firstTime = true;
static unsigned long lastSendTime = 0;
static unsigned long lastFirmwareCheckMillis = 0;   // millis() of last firmware check
static bool firmwareCheckDoneThisSlot = false;       // Prevents re-checking within same 15-min slot
static unsigned long readTime = 0;
static unsigned int readInterval = 5000;
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

// Status JSON output to Python device (via Serial1/RS-232)
// FULL status (datetime, SD, LTE, cell info) sent every 15 seconds
// FAST sensor packet (pressure, current, overfill, mode) sent every 1 second
unsigned long lastStatusSendTime = 0;
const unsigned long STATUS_SEND_INTERVAL = 15000;  // 15 seconds - full status
unsigned long lastSensorSendTime = 0;
const unsigned long SENSOR_SEND_INTERVAL = 1000;   // 1 second - fast sensor updates

// Passthrough mode - allows IO Board to act as a serial bridge to the modem
// When enabled, Serial1 (RS-232) data is forwarded directly to/from the modem
// This allows Linux host to use AT commands and PPP connections via the modem
// NOTE: Passthrough mode is NOT persisted - always returns to normal on reboot
bool passthroughMode = false;      // True when passthrough is active
int passthroughValue = 0;          // 0=normal operation, 1=passthrough active (sent in JSON status)

// Forward declarations for passthrough functions (needed for lambda in web server)
bool sendPassthroughRequestToLinux(unsigned long timeoutMinutes);
void enterPassthroughMode(unsigned long timeoutMinutes = 60);
void exitPassthroughMode();
void runPassthroughLoop();

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
 * During no-serial state, data is sent with zeroed values and fault code 1024
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
            Serial.printf("[WATCHDOG] ACTIVATED - no data for %lu min, fault code 1024\r\n", 
                          timeSinceLastData / 60000);
        }
        
        // Check if it's time for another reboot attempt (every 30 minutes)
        unsigned long timeSinceLastPulse = millis() - lastWatchdogPulseTime;
        
        if (lastWatchdogPulseTime == 0 || timeSinceLastPulse >= WATCHDOG_TIMEOUT) {
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
            
            Serial.printf("[WATCHDOG] Next attempt in 30 min if no serial data\r\n");
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
 *       // Send zeroed data with fault code 1024
 *   }
 */
bool isInWatchdogState() {
    return watchdogTriggered && watchdogEnabled;
}

/**
 * Get the watchdog fault code (1024) for use in data packets
 * Returns 0 if not in watchdog state
 * 
 * Usage example:
 *   int fault = getWatchdogFaultCode();  // Returns 1024 if in watchdog state
 */
int getWatchdogFaultCode() {
    return isInWatchdogState() ? WATCHDOG_FAULT_CODE : 0;
}

/**
 * Get the BlueCherry fault code (4096) for use in data packets
 * Returns 4096 if BlueCherry is not connected, 0 if connected
 * This alerts the user that OTA updates are unavailable
 * 
 * Usage example:
 *   int fault = getBlueCherryFaultCode();  // Returns 4096 if BC disconnected
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
        switch (modeNum) {
            case 0:  // IDLE - All relay outputs OFF
                digitalWrite(CR0_MOTOR, LOW);
                digitalWrite(CR1, LOW);
                digitalWrite(CR2, LOW);
                digitalWrite(CR5, LOW);
                break;
            case 1:  // RUN - Motor + CR1 + CR5 ON, CR2 OFF
                digitalWrite(CR0_MOTOR, HIGH);
                digitalWrite(CR1, HIGH);
                digitalWrite(CR2, LOW);
                digitalWrite(CR5, HIGH);
                break;
            case 2:  // PURGE - Motor + CR2 ON, CR1 + CR5 OFF
                digitalWrite(CR0_MOTOR, HIGH);
                digitalWrite(CR1, LOW);
                digitalWrite(CR2, HIGH);
                digitalWrite(CR5, LOW);
                break;
            case 3:  // BURP - Only CR5 ON, everything else OFF
                digitalWrite(CR0_MOTOR, LOW);
                digitalWrite(CR1, LOW);
                digitalWrite(CR2, LOW);
                digitalWrite(CR5, HIGH);
                break;
            case 8:  // FRESH AIR / SPECIAL BURP - CR2 + CR5 ON
                digitalWrite(CR0_MOTOR, LOW);
                digitalWrite(CR1, LOW);
                digitalWrite(CR2, HIGH);
                digitalWrite(CR5, HIGH);
                break;
            case 9:  // LEAK TEST - CR1 + CR2 + CR5 ON (no motor)
                digitalWrite(CR0_MOTOR, LOW);
                digitalWrite(CR1, HIGH);
                digitalWrite(CR2, HIGH);
                digitalWrite(CR5, HIGH);
                break;
            default:
                // Unknown mode - safe state (all off)
                digitalWrite(CR0_MOTOR, LOW);
                digitalWrite(CR1, LOW);
                digitalWrite(CR2, LOW);
                digitalWrite(CR5, LOW);
                break;
        }
        currentRelayMode = modeNum;
        // Update current_mode enum for LED color display
        if (modeNum <= 3) {
            current_mode = (RunMode)modeNum;
        }
        xSemaphoreGive(relayMutex);
    } else {
        // Mutex unavailable - write directly (safety fallback)
        switch (modeNum) {
            case 0: digitalWrite(CR0_MOTOR, LOW); digitalWrite(CR1, LOW); digitalWrite(CR2, LOW); digitalWrite(CR5, LOW); break;
            case 1: digitalWrite(CR0_MOTOR, HIGH); digitalWrite(CR1, HIGH); digitalWrite(CR2, LOW); digitalWrite(CR5, HIGH); break;
            case 2: digitalWrite(CR0_MOTOR, HIGH); digitalWrite(CR1, LOW); digitalWrite(CR2, HIGH); digitalWrite(CR5, LOW); break;
            case 3: digitalWrite(CR0_MOTOR, LOW); digitalWrite(CR1, LOW); digitalWrite(CR2, LOW); digitalWrite(CR5, HIGH); break;
            default: digitalWrite(CR0_MOTOR, LOW); digitalWrite(CR1, LOW); digitalWrite(CR2, LOW); digitalWrite(CR5, LOW); break;
        }
        currentRelayMode = modeNum;
        if (modeNum <= 3) current_mode = (RunMode)modeNum;
    }
    // DISP_SHUTDN is NOT changed here - managed separately via serial shutdown command
    digitalWrite(DISP_SHUTDN, dispShutdownActive ? HIGH : LOW);
}

/**
 * Activate DISP_SHUTDN (site shutdown) - sets GPIO13 LOW.
 * Called ONLY when {"mode":"shutdown"} is received via serial.
 * This is the ONLY way to shut down the site in Rev 10.
 */
void activateDispShutdown() {
    dispShutdownActive = false;
    digitalWrite(DISP_SHUTDN, LOW);
    Serial.println("[SHUTDOWN] DISP_SHUTDN set LOW - site shutdown activated");
}

/**
 * Deactivate DISP_SHUTDN (restore site) - sets GPIO13 HIGH.
 * Called when a non-shutdown mode is received after a shutdown.
 */
void deactivateDispShutdown() {
    dispShutdownActive = true;
    digitalWrite(DISP_SHUTDN, HIGH);
    Serial.println("[SHUTDOWN] DISP_SHUTDN set HIGH - site restored");
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
 * Ported from Python control.py mapit() function.
 * 
 * Usage example:
 *   float pressure = mapFloat(adcValue, 964.0, 1429.0, 0.0, 20.8);
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
 * Add a sample to the current rolling average buffer.
 * Returns the current rolling average.
 * 
 * Usage: Called from ADS1015 task at 60Hz.
 */
float addCurrentSample(float sample) {
    currentBuffer[currentBufferIdx] = sample;
    currentBufferIdx = (currentBufferIdx + 1) % CURRENT_AVG_SAMPLES;
    if (currentSampleCount < CURRENT_AVG_SAMPLES) currentSampleCount++;
    
    // Calculate average over available samples
    float sum = 0;
    int count = min(currentSampleCount, CURRENT_AVG_SAMPLES);
    for (int i = 0; i < count; i++) {
        sum += currentBuffer[i];
    }
    return sum / count;
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
    Wire.setClock(100000);  // 100kHz I2C clock
    
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
 *   - Channels 2 & 3: Current monitoring (abs differential)
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
    pinMode(DISP_SHUTDN, OUTPUT);
    
    digitalWrite(CR0_MOTOR, LOW);     // Motor OFF
    digitalWrite(CR1, LOW);           // Relay 1 OFF
    digitalWrite(CR2, LOW);           // Relay 2 OFF
    digitalWrite(CR5, LOW);           // Relay 5 OFF
    digitalWrite(DISP_SHUTDN, HIGH);  // CRITICAL: DISP_SHUTDN ON (safety)
    
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
            
            if (adcInitialized) {
                // Read Channel 0: Pressure sensor (single-ended)
                int16_t rawPressure = ads.readADC_SingleEnded(0);
                
                if (rawPressure >= 0) {
                    // Success - convert to pressure IWC using rolling average
                    adcRawPressure = rawPressure;
                    float pressureIWC = mapFloat((float)rawPressure, adcZeroPressure, adcFullPressure, 0.0, pressureRangeIWC);
                    // Negate because negative IWC is the normal operating range
                    adcPressure = addPressureSample(-pressureIWC);
                    adcErrorCount = 0;  // Reset consecutive error count
                } else {
                    adcErrorCount++;
                    adcTotalErrors++;
                }
                
                // Read Channels 2 & 3: Current monitoring (abs differential)
                int16_t rawCh2 = ads.readADC_SingleEnded(2);
                int16_t rawCh3 = ads.readADC_SingleEnded(3);
                
                if (rawCh2 >= 0 && rawCh3 >= 0) {
                    // Calculate absolute differential current
                    int16_t rawDiff = abs(rawCh2 - rawCh3);
                    adcRawCurrent = rawDiff;
                    float currentAmps = mapFloat((float)rawDiff, adcZeroCurrent, adcFullCurrent, currentRangeLow, currentRangeHigh);
                    if (currentAmps < 0) currentAmps = 0;  // Clamp negative values
                    adcCurrent = addCurrentSample(currentAmps);
                    
                    // Track peak current for high-current detection
                    if (currentAmps > adcPeakCurrent) {
                        adcPeakCurrent = currentAmps;
                    }
                } else {
                    adcErrorCount++;
                    adcTotalErrors++;
                }
                
                // Error recovery: After 10 consecutive errors, attempt reinit
                if (adcErrorCount >= 10) {
                    adcInitialized = false;
                    Wire.end();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    if (initializeADS1015()) {
                        adcErrorCount = 0;
                    } else {
                        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait 1 second before retry
                    }
                }
            } else {
                // ADS1015 not initialized - try periodically
                static unsigned long lastReinitAttempt = 0;
                if (now - lastReinitAttempt >= 5000) {  // Try every 5 seconds
                    lastReinitAttempt = now;
                    if (initializeADS1015()) {
                        adcErrorCount = 0;
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
 * Get combined fault code including failsafe indicator.
 * Returns base faults + 8192 if in failsafe mode.
 * 
 * Usage example:
 *   int faultCode = getCombinedFaultCode();  // e.g., 1024 + 4096 + 8192 = 13312
 */
int getCombinedFaultCode() {
    int code = 0;
    if (isInWatchdogState()) code += WATCHDOG_FAULT_CODE;
    if (!blueCherryConnected) code += BLUECHERRY_FAULT_CODE;
    if (failsafeMode) code += FAILSAFE_FAULT_CODE;
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
    <title>Walter IO Board - Rev 10.0</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        html { -webkit-text-size-adjust: 100%%; }
        body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; max-width: 600px; margin: 0 auto; padding: 0; background: #f0f2f5; color: #222; }
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
        .hdr h1 { font-size: 1.2em; color: #1a1a2e; }
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
        /* Padding at bottom for fixed nav */
        .content-wrap { padding-bottom: 60px; }
    </style>
</head>
<body>
    <!-- Top bar with status -->
    <div class="topbar">
        <div><span class="title">Walter IO Board</span><br><span class="info" id="topVer">Rev 10.0</span></div>
        <div style="text-align:right"><span class="info" id="topTime">--</span><br><span class="badge ok" id="connBadge" style="position:static;font-size:10px">Connected</span></div>
    </div>
    <div class="content-wrap">

    <!-- ============ MAIN SCREEN ============ -->
    <div class="screen active" id="scr-main">
        <div class="hdr"><h1>%DEVICENAME%</h1>
        <p id="mainProfile">--</p></div>
        <div class="card"><div class="card-title bg-blue">System Status</div><div class="card-body">
            <div class="row"><span class="lbl">Pressure (IWC)</span><span class="val" id="pressure">--</span></div>
            <div class="row"><span class="lbl">Current (A)</span><span class="val" id="mainCurrent">--</span></div>
            <div class="row"><span class="lbl">Mode</span><span class="val" id="mainMode">Idle</span></div>
            <div class="row"><span class="lbl">Run Cycles</span><span class="val" id="mainCycles">0</span></div>
            <div class="row"><span class="lbl">Overfill</span><span class="val" id="mainOverfill">Normal</span></div>
            <div class="row"><span class="lbl">Status</span><span class="val" id="mainStatus">--</span></div>
            <div class="row"><span class="lbl">Failsafe</span><span class="val" id="mainFailsafe">Inactive</span></div>
        </div></div>
        <div style="display:flex;gap:8px;margin:10px 0">
            <button class="btn" style="flex:1;padding:12px;background:#2e7d32;color:#fff;border:none;border-radius:8px;font-size:0.9em;font-weight:700" onclick="sendCmd('start_cycle','run')">Start Cycle</button>
            <button class="btn" style="flex:1;padding:12px;background:#c62828;color:#fff;border:none;border-radius:8px;font-size:0.9em;font-weight:700" onclick="sendCmd('stop_cycle')">Stop</button>
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
        <div id="maintLock" style="text-align:center;padding:30px 10px">
            <p style="color:#666;font-size:0.9em;margin-bottom:12px">Enter maintenance password to continue</p>
            <input type="password" id="maintPwd" maxlength="10" placeholder="Password" style="padding:10px 14px;border:1px solid #ccc;border-radius:6px;font-size:1em;width:140px;text-align:center">
            <div style="margin-top:12px">
                <button class="btn" style="padding:10px 28px;background:#1565c0;color:#fff;border:none;border-radius:6px;font-weight:700" onclick="checkMaintPw()">Unlock</button>
            </div>
            <p id="maintPwMsg" style="color:#c62828;font-size:0.85em;margin-top:8px"></p>
        </div>
        <!-- Maintenance menu (shown when unlocked) -->
        <div id="maintMenu" style="display:none">
            <div class="menu-grid">
                <button class="menu-btn" onclick="sendCmd('clear_press_alarm')">Clear Press Alarm</button>
                <button class="menu-btn" onclick="sendCmd('clear_motor_alarm')">Clear Motor Alarm</button>
                <button class="menu-btn" onclick="nav('tests')">Run Tests</button>
                <button class="menu-btn" onclick="nav('diagnostics')">Diagnostics</button>
                <button class="menu-btn" onclick="nav('clean')">Clean Canister</button>
                <button class="menu-btn" onclick="showPtModal()">Passthrough</button>
            </div>
        </div>
    </div><!-- end scr-maint -->

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
    <div class="screen" id="scr-func">
        <div class="hdr"><h1>Functionality Test</h1></div>
        <div class="card"><div class="card-body">
            <div class="countdown" id="funcTimer">--</div>
            <div class="test-status" id="funcStatus">Ready</div>
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
        </div></div>
    </div><!-- end scr-config -->

    <!-- ============ DIAGNOSTICS SCREEN ============ -->
    <div class="screen" id="scr-diagnostics">
        <div class="hdr"><h1>Diagnostics</h1></div>
        <div class="card"><div class="card-title bg-blue">Serial Data</div><div class="card-body">
            <div class="row"><span class="lbl">Serial Status</span><span class="val" id="serialStatus">--</span></div>
            <div class="row"><span class="lbl">Device ID</span><span class="val" id="deviceId">--</span></div>
            <div class="row"><span class="lbl">Fault Code</span><span class="val" id="fault">0</span></div>
        </div></div>
        <div class="card"><div class="card-title bg-green">ADC Readings</div><div class="card-body">
            <div class="row"><span class="lbl">ADC Pressure (raw)</span><span class="val" id="adcRawP">--</span></div>
            <div class="row"><span class="lbl">ADC Current (raw)</span><span class="val" id="adcRawC">--</span></div>
            <div class="row"><span class="lbl">ADC Initialized</span><span class="val" id="adcInit">--</span></div>
            <div class="row"><span class="lbl">ADC Errors</span><span class="val" id="adcErrors">0</span></div>
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
        </div></div>
        <div class="card"><div class="card-title bg-gray">IO Board</div><div class="card-body">
            <div class="row"><span class="lbl">Firmware</span><span class="val" id="version">--</span></div>
            <div class="row"><span class="lbl">Board Temp</span><span class="val" id="temperature">--</span></div>
            <div class="row"><span class="lbl">SD Card</span><span class="val" id="sdCard">--</span></div>
            <div class="row"><span class="lbl">Overfill Sensor</span><span class="val" id="overfill">--</span></div>
            <div class="row"><span class="lbl">GPIO38 Raw</span><span class="val" id="overfillDebug" style="font-size:0.78em;color:#888">--</span></div>
            <div class="row"><span class="lbl">Watchdog</span><span class="val" id="watchdog">--</span></div>
            <div class="row"><span class="lbl">Uptime</span><span class="val" id="uptime">--</span></div>
            <div class="row"><span class="lbl">MAC</span><span class="val" id="mac">--</span></div>
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

    <!-- Bottom Navigation -->
    <div class="nav-footer">
        <button id="nb-main" class="active" onclick="nav('main')">Main</button>
        <button id="nb-alarms" onclick="nav('alarms')">Alarms</button>
        <button id="nb-maint" onclick="nav('maint')">Maint</button>
        <button id="nb-settings" onclick="nav('settings')">Settings</button>
    </div>

    <script>
    (function(){
        var errs=0,IP='192.168.4.1',curScr='main';
        var navStack=['main'];
        function $(id){return document.getElementById(id);}
        function upd(id,v){var e=$(id);if(e&&e.textContent!==String(v))e.textContent=v;}
        function conn(ok){var b=$('connBadge');if(b){b.textContent=ok?'OK':'ERR';b.className='badge '+(ok?'ok':'err');}if(ok)errs=0;}
        function setInd(id,alarm){var e=$(id);if(e)e.className='ind '+(alarm?'ind-r':'ind-g');}

        // SPA navigation: show one screen, hide others
        window.nav=function(scr){
            var screens=document.querySelectorAll('.screen');
            for(var i=0;i<screens.length;i++)screens[i].classList.remove('active');
            var target=$('scr-'+scr);if(target)target.classList.add('active');
            // Update nav footer active state
            var btns=document.querySelectorAll('.nav-footer button');
            for(var i=0;i<btns.length;i++)btns[i].classList.remove('active');
            var nb=$('nb-'+scr);if(nb)nb.classList.add('active');
            if(scr!==curScr){navStack.push(scr);curScr=scr;}
        };

        // Send command to ESP32 via POST /api/command
        window.sendCmd=function(cmd,val){
            var x=new XMLHttpRequest();
            x.open('POST','http://'+IP+'/api/command',true);
            x.setRequestHeader('Content-Type','application/json');
            var body=JSON.stringify({command:cmd,value:val||''});
            x.send(body);
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
                    setTimeout(function(){$('profConfirm').style.display='none';$('profPwMsg').textContent='';$('profPwMsg').style.color='#c62828';},2000);
                } else if(x.status===403){
                    $('profPwMsg').style.color='#c62828';
                    $('profPwMsg').textContent='Wrong password';
                } else {
                    $('profPwMsg').style.color='#c62828';
                    $('profPwMsg').textContent='Error: '+x.status;
                }
            };
            x.onerror=function(){$('profPwMsg').style.color='#c62828';$('profPwMsg').textContent='Connection error';};
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
        // Intercept nav to maint: if already unlocked, show menu directly
        var origNav=window.nav;
        window.nav=function(scr){
            origNav(scr);
            if(scr==='maint'){
                if(maintUnlocked){
                    $('maintLock').style.display='none';
                    $('maintMenu').style.display='block';
                } else {
                    $('maintLock').style.display='block';
                    $('maintMenu').style.display='none';
                    $('maintPwd').value='';
                    $('maintPwMsg').textContent='';
                }
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
                        // Main screen
                        upd('pressure',d.pressure);upd('mainCurrent',d.current);
                        upd('mainMode',modeNames[d.mode]||d.mode);upd('mainCycles',d.cycles);
                        upd('mainProfile',d.profile||'--');
                        var ovEl=$('mainOverfill');if(ovEl){ovEl.textContent=d.overfillAlarm?'ALARM':'Normal';ovEl.style.color=d.overfillAlarm?'#c62828':'#2e7d32';}
                        var fsEl=$('mainFailsafe');if(fsEl){fsEl.textContent=d.failsafe?'ACTIVE':'Inactive';fsEl.style.color=d.failsafe?'#c62828':'#2e7d32';}
                        upd('mainStatus',d.serialActive?'Online':'No Serial');
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
                        // Tests
                        upd('testPressure',d.pressure);
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
                        // Diagnostics
                        var ss=$('serialStatus');if(ss){ss.textContent=d.serialActive?'Receiving':'No Data';ss.style.color=d.serialActive?'#2e7d32':'#c62828';}
                        upd('deviceId',d.deviceName);upd('fault',d.fault||'0');
                        upd('adcRawP',d.adcRawPressure||'--');upd('adcRawC',d.adcRawCurrent||'--');
                        upd('adcInit',d.adcInitialized?'Yes':'No');upd('adcErrors',d.adcErrors||'0');
                        var le=$('lteStatus');if(le)le.innerHTML=d.lteConnected?'<span style=color:#2e7d32>Connected</span>':'<span style=color:#c62828>Disconnected</span>';
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
                        upd('uptime',d.uptime);upd('mac',d.macAddress);
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
        
        // Get signal quality strings (updated by refreshCellSignalInfo every 60s)
        String rsrpStr = modemRSRP.length() > 0 ? modemRSRP : "--";
        String rsrqStr = modemRSRQ.length() > 0 ? modemRSRQ : "--";
        String operatorStr = modemNetName.length() > 0 ? modemNetName : "--";
        String bandStr = modemBand.length() > 0 ? modemBand : "--";
        
        // Build JSON response with all status fields (Rev 10)
        String json = "{";
        // Serial data
        json += "\"serialActive\":" + String(serialActive ? "true" : "false") + ",";
        json += "\"deviceName\":\"" + DeviceName + "\",";
        // ADC readings (from ADS1015, replaces Linux-sourced pressure/current)
        json += "\"pressure\":\"" + String(adcPressure, 2) + "\",";
        json += "\"current\":\"" + String(adcCurrent, 2) + "\",";
        json += "\"mode\":" + String(currentRelayMode) + ",";
        json += "\"cycles\":\"" + String(cycles) + "\",";
        json += "\"fault\":\"" + String(faults + getCombinedFaultCode()) + "\",";
        // Profile
        json += "\"profile\":\"" + profileManager.getActiveProfileName() + "\",";
        // Failsafe
        json += "\"failsafe\":" + String(failsafeMode ? "true" : "false") + ",";
        // Cellular modem
        json += "\"lteConnected\":" + String(lteConnected() ? "true" : "false") + ",";
        json += "\"rsrp\":\"" + rsrpStr + "\",";
        json += "\"rsrq\":\"" + rsrqStr + "\",";
        json += "\"operator\":\"" + operatorStr + "\",";
        json += "\"band\":\"" + bandStr + "\",";
        json += "\"mcc\":" + String(modemCC) + ",";
        json += "\"mnc\":" + String(modemNC) + ",";
        json += "\"cellId\":" + String(modemCID) + ",";
        json += "\"tac\":" + String(modemTAC) + ",";
        json += "\"blueCherryConnected\":" + String(blueCherryConnected ? "true" : "false") + ",";
        json += "\"modemStatus\":\"" + Modem_Status + "\",";
        json += "\"imei\":\"" + imei + "\",";
        // IO Board status
        json += "\"version\":\"" + ver + "\",";
        json += "\"temperature\":\"" + String(fahrenheit, 1) + "\",";
        json += "\"sdCardOK\":" + String(isSDCardOK() ? "true" : "false") + ",";
        // Overfill (direct GPIO38 read)
        json += "\"overfillAlarm\":" + String(overfillAlarmActive ? "true" : "false") + ",";
        json += "\"gpio38Raw\":" + String(digitalRead(ESP_OVERFILL)) + ",";
        json += "\"overfillLowCnt\":" + String(overfillLowCount) + ",";
        json += "\"overfillHighCnt\":" + String(overfillHighCount) + ",";
        json += "\"overfillValidated\":" + String(overfillValidationComplete ? "true" : "false") + ",";
        // ADC debug
        json += "\"adcRawPressure\":" + String(adcRawPressure) + ",";
        json += "\"adcRawCurrent\":" + String(adcRawCurrent) + ",";
        json += "\"adcInitialized\":" + String(adcInitialized ? "true" : "false") + ",";
        json += "\"adcErrors\":" + String(adcTotalErrors) + ",";
        // Alarm states for web display
        json += "\"alarmLowPress\":" + String((adcPressure < profileManager.getActiveProfile()->lowPressThreshold && currentRelayMode > 0) ? "true" : "false") + ",";
        json += "\"alarmHighPress\":" + String((adcPressure > profileManager.getActiveProfile()->highPressThreshold && currentRelayMode > 0) ? "true" : "false") + ",";
        json += "\"alarmZeroPress\":" + String((profileManager.getActiveProfile()->hasZeroPressAlarm && fabs(adcPressure) < 0.15 && currentRelayMode > 0) ? "true" : "false") + ",";
        json += "\"alarmVarPress\":false,";
        json += "\"alarmLowCurrent\":" + String((adcCurrent > 0 && adcCurrent < LOW_CURRENT_THRESHOLD && currentRelayMode > 0) ? "true" : "false") + ",";
        json += "\"alarmHighCurrent\":" + String((adcCurrent > HIGH_CURRENT_THRESHOLD) ? "true" : "false") + ",";
        json += "\"watchdogTriggered\":" + String(watchdogTriggered ? "true" : "false") + ",";
        json += "\"watchdogEnabled\":" + String(watchdogEnabled ? "true" : "false") + ",";
        // Datetime
        json += "\"datetime\":\"" + getDateTimeString() + "\",";
        json += "\"uptime\":\"" + String(uptimeStr) + "\",";
        json += "\"macAddress\":\"" + macStr + "\"";
        json += "}";
        
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
            } else {
                // =========================================================
                // NORMAL MODE: Linux is connected and controls relays.
                // Forward the start_cycle command to Linux via Serial1.
                // Linux's IOManager will run the cycle through its normal
                // cycle management (alarms, DB logging, pause/resume, etc.)
                //
                // Previously this called startFailsafeCycle() which would
                // set relays, but Linux would override them within 500ms
                // because its ModeManager was still in rest mode.
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
        else if (cmd == "overfill_override") {
            overfillAlarmActive = false;
            overfillLowCount = 0;
            Serial.println("[WEB] Overfill override activated");
            request->send(200, "application/json", "{\"ok\":true}");
        }
        else if (cmd == "clear_press_alarm" || cmd == "clear_motor_alarm") {
            failsafeLowCurrentCount = 0;
            failsafeHighCurrentCount = 0;
            Serial.printf("[WEB] Cleared alarm: %s\r\n", cmd.c_str());
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
    static const char CP_PAGE[] PROGMEM = 
        "<!DOCTYPE html><html><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Walter IO Board</title>"
        "<style>body{font-family:-apple-system,Arial,sans-serif;text-align:center;padding:40px 20px;background:#f0f2f5;}"
        "h2{color:#1a1a2e;margin-bottom:10px;}p{color:#666;margin:8px 0;}"
        "a{display:inline-block;margin-top:15px;padding:12px 30px;background:#1565c0;color:#fff;"
        "text-decoration:none;border-radius:6px;font-weight:bold;font-size:1.1em;}"
        "a:active{background:#0d47a1;}.hint{font-size:12px;color:#999;margin-top:20px;}</style>"
        "</head><body>"
        "<h2>Walter IO Board</h2>"
        "<p>Service Diagnostic Dashboard</p>"
        "<a href='http://192.168.4.1/'>Open Dashboard</a>"
        "<p class='hint'>If the page doesn't load, open your browser<br>and go to <b>192.168.4.1</b></p>"
        "</body></html>";
    
    // Apple iOS / macOS captive portal detection
    // iOS probes http://captive.apple.com/hotspot-detect.html
    // If response is NOT "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>"
    // then iOS opens the captive portal popup. We return our redirect page.
    server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
    });
    
    // Android captive portal detection
    // Android probes http://connectivitycheck.gstatic.com/generate_204
    // Expects HTTP 204 No Content. Any other response triggers captive portal popup.
    // We return 200 with our redirect page to force the popup open.
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
    });
    
    // Windows NCSI (Network Connectivity Status Indicator) probes
    server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
    });
    server.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
    });
    server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
    });
    server.on("/redirect", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
    });
    
    // Firefox captive portal detection
    server.on("/canonical.html", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
    });
    
    // Additional common captive portal probe URLs
    server.on("/success.txt", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
    });
    server.on("/chat", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", CP_PAGE);
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
                    if (DeviceName != newId) {
                        DeviceName = newId;
                        strncpy(deviceName, DeviceName.c_str(), sizeof(deviceName) - 1);
                        deviceName[sizeof(deviceName) - 1] = '\0';
                        preferences.begin("RMS", false);
                        preferences.putString("DeviceName", DeviceName);
                        preferences.putString("DeviceNameBackup", DeviceName);
                        preferences.end();
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
    // Need valid time from modem (currentTimestamp > 0 means modem time was set)
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
 * Send status JSON to Python device via Serial1 (RS-232)
 * Called every 15 seconds from main loop
 * 
 * JSON Format:
 *   {"datetime":"2026-01-30 12:34:56","sdcard":"OK","passthrough":0,"lte":1,"rsrp":"-85.5","rsrq":"-10.2"}
 * 
 * Fields:
 *   - datetime: Current date/time from modem (UTC)
 *   - sdcard: "OK" or "FAULT" based on SD card status
 *   - passthrough: 0 or 1 (passthrough mode status)
 *   - lte: 1 if connected to LTE, 0 if not
 *   - rsrp: Signal strength in dBm (typical: -80 excellent, -100 poor)
 *   - rsrq: Signal quality in dB (typical: -10 good, -20 poor)
 * 
 * Usage example:
 *   sendStatusToSerial();  // Call every 15 seconds
 */
void sendStatusToSerial() {
    // Get current timestamp from modem
    String dateTimeStr = formatTimestamp(currentTimestamp);
    
    // Check SD card status
    String sdStatus = isSDCardOK() ? "OK" : "FAULT";
    
    // Check LTE connection status
    int lteStatus = lteConnected() ? 1 : 0;
    
    // Get signal quality - use stored values (updated by refreshCellSignalInfo every 60s)
    String rsrpStr = modemRSRP.length() > 0 ? modemRSRP : "--";
    String rsrqStr = modemRSRQ.length() > 0 ? modemRSRQ : "--";
    String operatorStr = modemNetName.length() > 0 ? modemNetName : "--";
    String bandStr = modemBand.length() > 0 ? modemBand : "--";
    
    // Build JSON string with all status info (Rev 10 - includes pressure, current, overfill, profile)
    // Sent every 15 seconds to modem.py on Linux device via Serial1 (RS-232)
    // New Rev 10 fields: pressure, current (from ADS1015), overfill (GPIO38), profile, failsafe
    char jsonBuffer[512];
    snprintf(jsonBuffer, sizeof(jsonBuffer),
             "{\"datetime\":\"%s\",\"sdcard\":\"%s\",\"passthrough\":%d,"
             "\"lte\":%d,\"rsrp\":\"%s\",\"rsrq\":\"%s\","
             "\"operator\":\"%s\",\"band\":\"%s\","
             "\"mcc\":%u,\"mnc\":%u,\"cellId\":%u,\"tac\":%u,"
             "\"pressure\":%.2f,\"current\":%.2f,\"overfill\":%d,"
             "\"profile\":\"%s\",\"failsafe\":%d}",
             dateTimeStr.c_str(),
             sdStatus.c_str(),
             passthroughValue,
             lteStatus,
             rsrpStr.c_str(),
             rsrqStr.c_str(),
             operatorStr.c_str(),
             bandStr.c_str(),
             modemCC, modemNC, modemCID, modemTAC,
             adcPressure, adcCurrent, overfillAlarmActive ? 1 : 0,
             profileManager.getActiveProfileName().c_str(),
             failsafeMode ? 1 : 0);
    
    // Send to Python device via Serial1 (RS-232)
    Serial1.println(jsonBuffer);
    
    // Debug output to Serial Monitor
    Serial.print("[RS232-TX] ");
    Serial.println(jsonBuffer);
}

/**
 * Send a lightweight sensor-only packet to Python every 1 second.
 * Contains ONLY the fast-changing values that the UI needs in real-time:
 *   pressure (IWC), current (amps), overfill state, relay mode
 * 
 * This is much smaller (~60 bytes) than the full status (~500 bytes)
 * and avoids expensive modem/SD card queries.
 * 
 * Python modem.py parses "pressure", "current", "overfill" fields from
 * ANY incoming JSON — same parser handles both fast and full packets.
 * 
 * Usage example:
 *   sendFastSensorPacket();  // Call every 1 second from main loop
 */
void sendFastSensorPacket() {
    // Lightweight JSON — only real-time sensor data, no modem/SD/cell queries
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"pressure\":%.2f,\"current\":%.2f,\"overfill\":%d,\"relayMode\":%d}",
             adcPressure, adcCurrent, overfillAlarmActive ? 1 : 0, currentRelayMode);
    Serial1.println(buf);
}

/**
 * Check if it's time to send status/sensor data to Python and send if due.
 * Called from main loop - handles timing internally.
 * 
 * Two send rates:
 *   - Every 1 second:  Fast sensor packet (pressure, current, overfill, mode)
 *   - Every 15 seconds: Full status (datetime, SD, LTE, cell info, profile, etc.)
 * 
 * IMPORTANT: Skips sending during passthrough mode to avoid interfering
 * with modem communication (PPP/AT commands)
 * 
 * Usage: Call in loop() - handles both intervals internally
 */
void checkAndSendStatusToSerial() {
    // Skip during passthrough mode - serial ports are dedicated to modem bridge
    if (passthroughMode) {
        return;
    }
    
    unsigned long currentTime = millis();
    
    // Fast sensor packet every 1 second — pressure/current for UI updates
    if (currentTime - lastSensorSendTime >= SENSOR_SEND_INTERVAL) {
        sendFastSensorPacket();
        lastSensorSendTime = currentTime;
    }
    
    // Full status packet every 15 seconds — includes cell info, SD, datetime
    if (currentTime - lastStatusSendTime >= STATUS_SEND_INTERVAL) {
        sendStatusToSerial();
        lastStatusSendTime = currentTime;
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
    
    if (failsafeCycleRunning) {
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
        // No cycle running - clear any stale relay state
        preferences.putUChar("ptRelayMode", 0);
        preferences.putULong("ptRelaySec", 0);
    }
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
        if (noSerialCount % 20 == 0 && noSerialCount > 0) {
            Serial.print("⚠ No serial data received for ");
            Serial.print(noSerialCount * 5);  // readInterval is 5 seconds
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
    
    // Check message type
    String msgType = getJsonValue(dataBuffer, "type");
    if (msgType.length() == 0) {
        Serial.println("No 'type' field in JSON");
        return;
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
    
    // Check for shutdown command: {"mode":"shutdown"} (string value, not integer)
    // This is the ONLY way to trigger DISP_SHUTDN LOW in Rev 10
    if (modeStr == "shutdown") {
        activateDispShutdown();
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
    if (modeStr.length() > 0 && !failsafeMode) {
        setRelaysForMode((uint8_t)mode);
    }
    
    // Update device name if provided and different
    if (gmidStr.length() > 0 && gmidStr != String(deviceName)) {
        Serial.println("[Serial] Updating device name from '" + String(deviceName) + "' to '" + gmidStr + "'");
        DeviceName = gmidStr;
        strncpy(deviceName, gmidStr.c_str(), sizeof(deviceName) - 1);
        deviceName[sizeof(deviceName) - 1] = '\0';
        
        // Save to preferences/EPROM
        preferences.begin("RMS", false);
        preferences.putString("DeviceName", DeviceName);
        preferences.putString("APSuffix", DeviceName);  // Also update AP suffix
        preferences.end();
        Serial.println("✓ Device name saved to EPROM: " + DeviceName);
        
        // Update WiFi AP name with the new device ID
        updateAPName(DeviceName, true);  // Restart AP with new name
        apNameFromSerial = true;  // Mark as received from serial
        Serial.println("[Serial] WiFi AP name updated to: " + String(apSSID));
    }
    
    // Log received data
    Serial.println("✓ JSON parsed - Press:" + String(pressure) + " Cycles:" + String(cycles) + 
                   " Mode:" + String(mode) + " Current:" + String(current) + " GMID:" + gmidStr);
    
    // Save to SD if we have valid data
    if (noSerialCount <= 5) {
        pres_scaled = static_cast<int>(round(pressure * 100.0));
        seq++;
        Serial.println("Saving to SD");
        SaveToSD(deviceName, seq - 1, pres_scaled, cycles, faults, mode, temp_scaled, current);
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
        // Normal operation - use live data from serial
        
        // Read ESP32's own temperature sensor
        float celsius = temperatureRead();
        float fahrenheit = (celsius * 9.0 / 5.0) + 32;
        temp_scaled = static_cast<int>(round(fahrenheit * 100.0));
        
        // Scale current from Python JSON
        int current_scaled = static_cast<int>(round(current * 100.0));
        
        Serial.print("Data summary - Temp(ESP32): ");
        Serial.print(fahrenheit);
        Serial.print("°F, Press(Python): ");
        Serial.print(pres_scaled / 100.0);
        Serial.print(", Cycles(Python): ");
        Serial.print(cycles);
        Serial.print(", Current(Python): ");
        Serial.println(current);
        
        // Add BlueCherry fault code (4096) to existing faults if BC is offline
        int totalFaults = faults + getBlueCherryFaultCode();
        
        readings[readingCount-1][0] = id;
        readings[readingCount-1][1] = static_cast<int>(seq);
        readings[readingCount-1][2] = pres_scaled;
        readings[readingCount-1][3] = cycles;
        readings[readingCount-1][4] = totalFaults;  // Device faults + 4096 if BC offline
        readings[readingCount-1][5] = mode;
        readings[readingCount-1][6] = temp_scaled;
        readings[readingCount-1][7] = current_scaled;
        
        // Log if BlueCherry fault code is active
        if (!blueCherryConnected) {
            Serial.print("📤 Data includes BlueCherry fault (4096): total fault=");
            Serial.println(totalFaults);
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
    
    if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
        Serial.println("Failed to set operational state to minimum");
        return false;
    }
    delay(2000);
	
    Serial.printf("Defining PDP context with APN: %s\n", CELLULAR_APN);
    if (!modem.definePDPContext(1, CELLULAR_APN)) {
        Serial.println("Failed to define PDP context");
        Serial.println("Possible causes:");
        Serial.println("1. Incorrect APN for your cellular provider");
        Serial.println("2. Network not ready for data connection");
        Serial.println("3. SIM card issues");
        return false;
    }
    delay(1000);
    
    Serial.printf("Setting PDP authentication: Protocol=%d, User=%s\n", 
                  CELLULAR_AUTH_PROTOCOL, CELLULAR_USERNAME);
    if (!modem.setPDPAuthParams(CELLULAR_AUTH_PROTOCOL, CELLULAR_USERNAME, CELLULAR_PASSWORD)) {
        Serial.println("Failed to set PDP authentication parameters");
        Serial.println("Note: Some providers don't require authentication");
    }
    delay(1000);
    
    if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
        Serial.println("Failed to set operational state to full");
        return false;
    }
    delay(2000);
	
    if (!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
        Serial.println("Failed to set network selection mode");
        return false;
    }
    delay(1000);
    
    Serial.println("Waiting for network registration...");
    unsigned short timeout = 300, i = 0;
    while (!lteConnected() && i < timeout) {
        i++;
        delay(1000);
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
 
        Serial.printf("[CELL] Band=%s, Op=%s, RSRP=%s, RSRQ=%s, MCC=%u, MNC=%02u, CID=%u, TAC=%u\r\n",
                      modemBand.c_str(), modemNetName.c_str(), modemRSRP.c_str(), modemRSRQ.c_str(),
                      modemCC, modemNC, modemCID, modemTAC);
    }
    
    Serial.println("LTE connection established successfully");
    return true;
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
    if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
        modemBand = String(rsp.data.cellInformation.band);
        modemNetName = String(rsp.data.cellInformation.netName);
        modemRSRP = String(rsp.data.cellInformation.rsrp, 1);   // e.g. "-89.5"
        modemRSRQ = String(rsp.data.cellInformation.rsrq, 1);   // e.g. "-11.2"
        modemCC = rsp.data.cellInformation.cc;                    // Country Code (e.g. 310)
        modemNC = rsp.data.cellInformation.nc;                    // Network Code (e.g. 260)
        modemCID = rsp.data.cellInformation.cid;                  // Cell Tower ID
        modemTAC = rsp.data.cellInformation.tac;                  // Tracking Area Code
    }
    // On failure, keep previous values rather than overwriting with "Unknown"
    // This prevents dashboard from flickering between real data and "Unknown"
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
    pinMode(DISP_SHUTDN, OUTPUT);
    digitalWrite(DISP_SHUTDN, HIGH);  // CRITICAL: DISP_SHUTDN ON (safety)
    
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
    Serial.println("║  Walter IO Board Firmware - Rev 10.0                 ║");
    Serial.println("║  ADS1015 ADC + Failsafe Relay Control                ║");
    Serial.println("╚═══════════════════════════════════════════════════════╝\n");
    
    // Load active profile from EEPROM
    profileManager.loadActiveProfile();
    
    // Allocate CBOR passthrough buffer
    allocateCborPassthroughBuffer();
    
    // Initialize watchdog pin
    pinMode(ESP_WATCHDOG_PIN, OUTPUT);
    digitalWrite(ESP_WATCHDOG_PIN, LOW);
    Serial.println("✓ Serial watchdog pin (GPIO39) initialized");
    
    delay(500);
    
    // Manage SD Card mounting (with updated CS pin)
    initSdLogging(); 

    // Get device name and watchdog setting from EPROM
    // Send frequency is now controlled by Python (15 seconds), not stored here
    preferences.begin("RMS", false);
    DeviceName = preferences.getString("DeviceName", "CSX-9000");
    watchdogEnabled = preferences.getBool("watchdog", true);   // Default to ENABLED for safety
    preferences.end();
    
    Serial.print("✓ Loaded watchdog setting from EPROM: ");
    Serial.println(watchdogEnabled ? "ENABLED" : "DISABLED");
    
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
    Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    if (!modem.begin(&ModemSerial)) {
        Serial.println("Could not initialize the modem, restarting in 10 seconds");
        Modem_Status = "Could Not Initialize Modem";
        delay(10000);
        ESP.restart();
    }
    
    if (modem.getRAT(&rsp) && rsp.data.rat == WALTER_MODEM_RAT_NBIOT) {
        if (modem.setRAT(WALTER_MODEM_RAT_LTEM)) {
            Serial.println("Switched modem to LTE-M mode");
            modem.reset();
        }
    }
    
    if (!lteConnect()) {
        Serial.println("Unable to connect to cellular network, restarting in 10 seconds");
        Modem_Status = "Unable to connect to cellular network";
        delay(10000);
        ESP.restart();
    }
    
    // BlueCherry initialization - NON-FATAL (system continues if platform is down)
    // BlueCherry is used for OTA firmware updates - core IO functions work without it
    // If BlueCherry fails, system will retry hourly and do a weekly restart to retry
    Serial.println("\nInitializing BlueCherry cloud connection...");
    Serial.println("(BlueCherry is for OTA updates - system will continue if unavailable)");
    
    systemStartTime = millis();  // Record start time for weekly restart tracking
    blueCherryLastAttempt = millis();
    blueCherryConnected = false;
    
    unsigned short attempt = 0;
    const unsigned short maxAttempts = 3;  // Try 3 times before giving up
    
    while (!modem.blueCherryInit(BC_TLS_PROFILE, otaBuffer, &rsp) && attempt < maxAttempts) {
        Serial.print("BlueCherry attempt ");
        Serial.print(attempt + 1);
        Serial.print("/");
        Serial.print(maxAttempts);
        Serial.print(", state: ");
        Serial.println(rsp.data.blueCherry.state);
        
        if (rsp.data.blueCherry.state == WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED && attempt <= 2) {
            Serial.println("Device not provisioned, attempting ZTP...");
            if (attempt++ == 0) {
                if (!BlueCherryZTP::begin(BC_DEVICE_TYPE, BC_TLS_PROFILE, bc_ca_cert, &modem)) continue;
                // Read MAC address HERE - inside ZTP block, exactly as original code
                uint8_t mac[8] = {0};
                esp_read_mac(mac, ESP_MAC_WIFI_STA);
                BlueCherryZTP::addDeviceIdParameter(BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC, mac);
                if (modem.getIdentity(&rsp)) {
                    BlueCherryZTP::addDeviceIdParameter(BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI, rsp.data.identity.imei);
                }
            }
            if (!BlueCherryZTP::requestDeviceId() || !BlueCherryZTP::generateKeyAndCsr() || !BlueCherryZTP::requestSignedCertificate() ||
                !modem.blueCherryProvision(BlueCherryZTP::getCert(), BlueCherryZTP::getPrivKey(), bc_ca_cert)) continue;
        } else {
            attempt++;
            if (attempt < maxAttempts) {
                Serial.println("BlueCherry init failed, retrying in 5 seconds...");
                delay(5000);
            }
        }
    }
    
    // Check if BlueCherry connected successfully
    if (modem.blueCherryInit(BC_TLS_PROFILE, otaBuffer, &rsp)) {
        blueCherryConnected = true;
        Serial.println("✓ BlueCherry initialized successfully");
    } else {
        blueCherryConnected = false;
        Serial.println("\n╔═══════════════════════════════════════════════════════╗");
        Serial.println("║  ⚠️ BlueCherry platform unavailable                   ║");
        Serial.println("║  System will continue without OTA update capability   ║");
        Serial.println("║  Will retry hourly, weekly restart if still offline   ║");
        Serial.println("╚═══════════════════════════════════════════════════════╝\n");
    }
    
    if (modem.getIdentity(&rsp)) {
        imei = rsp.data.identity.imei;
    }
    
    // Read MAC address AFTER BlueCherry init (matching original code structure)
    // This is for display and for macStr global variable used elsewhere
    uint8_t mac[8] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char buffer[18];
    snprintf(buffer, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    macStr = buffer;
    
    // Display MAC address prominently in Serial Monitor
    Serial.println("\n╔═══════════════════════════════════════════════════════╗");
    Serial.println("║              DEVICE IDENTIFICATION                    ║");
    Serial.println("╠═══════════════════════════════════════════════════════╣");
    Serial.print("║  MAC:  ");
    Serial.print(macStr);
    for (int i = macStr.length(); i < 47; i++) Serial.print(" ");
    Serial.println("║");
    Serial.print("║  IMEI: ");
    Serial.print(imei);
    for (int i = imei.length(); i < 47; i++) Serial.print(" ");
    Serial.println("║");
    Serial.println("╚═══════════════════════════════════════════════════════╝\n");
    
    if (macStr == "00:00:00:00:00:00" || macStr.length() == 0) {
        Serial.println("⚠️ WARNING: MAC address is blank or invalid!");
    }
    
    initModemTime();
    delay(1000);
    
    // Initialize watchdog timer
    resetSerialWatchdog();
    Serial.println("✓ Serial watchdog timer initialized");
    
    Serial.println("\n✅ Walter IO Board Firmware Rev 10.0 initialization complete!");
    Serial.println("✅ ADS1015 ADC reader running on Core 0 (60Hz, address 0x48)");
    Serial.println("✅ SPA web interface active with " + String(PROFILE_COUNT) + " profiles");
    Serial.println("✅ Active profile: " + profileManager.getActiveProfileName());
    Serial.println("✅ Serial watchdog ready");
    Serial.println("✅ Failsafe relay control standby (activates after 2 restart attempts)");
    if (blueCherryConnected) {
        Serial.println("✅ BlueCherry OTA platform connected");
    } else {
        Serial.println("⚠️ BlueCherry OTA offline - will retry hourly, weekly restart scheduled");
    }
    Serial.printf("✅ CBOR buffer: %d samples (%d min capacity)\r\n", CBOR_BUFFER_MAX_SAMPLES, CBOR_BUFFER_MAX_SAMPLES * 15 / 60);
    Serial.println("✅ System ready!\n");
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
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
    
    // LTE connection check - only runs in normal mode (not passthrough)
    if (!lteConnected() && !lteConnect()) {
        Serial.println("Unable to connect to cellular network, restarting in 10 seconds");
        delay(10000);
        ESP.restart();
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
    // =====================================================================
    if (firstTime || isFirmwareCheckScheduled()) {
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
    // Uses sendStatusUpdateViaSocket() which calls refreshCellSignalInfo(),
    // buildStatusCbor() (15-element unified array), and sendCborDataViaSocket().
    // =====================================================================
    if (lastDailyStatusTime == 0 || currentTime - lastDailyStatusTime >= STATUS_INTERVAL) {
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
    if (currentTime - lastSDCheck >= 60000) {
        refreshCellSignalInfo();
        
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
    
    // Send status JSON to Python device every 15 seconds via RS-232
    checkAndSendStatusToSerial();
    
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
