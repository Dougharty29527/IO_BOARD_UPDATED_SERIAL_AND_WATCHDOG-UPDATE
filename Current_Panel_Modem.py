'''
Serial data transmission module for ESP32 communication.

Sends sensor data to Walter ESP32 via RS-232 Serial.
Formats data as JSON compatible with IO_BOARD_FIRMWARE.ino.

JSON Format (Outgoing to ESP32):
    {"type":"data","gmid":"CSX-1234","press":-14.22,"mode":0,"current":0.07,"fault":0,"cycles":484}

JSON Format (Incoming from ESP32 - Status):
    {"datetime":"2026-01-29 12:34:56","sdcard":"OK","passthrough":0,"lte":1,"rsrp":"-85.5","rsrq":"-10.2"}
    
    - datetime: Current timestamp from modem (UTC)
    - sdcard: "OK" or "FAULT" 
    - passthrough: 0=normal, 1=passthrough mode active
    - lte: 1=connected to LTE network, 0=not connected
    - rsrp: Signal strength in dBm (typical: -80 excellent, -100 poor)
    - rsrq: Signal quality in dB (typical: -10 good, -20 poor)

JSON Format (Incoming from ESP32 - Passthrough Request):
    {"passthrough":"remote 60"}
    
    - passthrough: "remote XX" where XX is timeout in minutes
    - Linux responds with "ready" then starts PPP
    - PPP runs for XX minutes then stops automatically

JSON Format (Outgoing to ESP32 - Passthrough Command):
    {"command":"passthrough","timeout":60}
    
    - command: "passthrough" to request passthrough mode
    - timeout: Duration in minutes
'''

import json
import serial
import time
import subprocess
import threading
import re

from kivy.logger import Logger
from kivy.clock import Clock

from .data_handler import DataHandler


class SerialManager:
    '''Manages serial data transmission to ESP32'''

    # Serial configuration
    SERIAL_PORT = '/dev/serial0'
    BAUD_RATE = 115200
    UPDATE_INTERVAL = 15      # Send data to ESP32 every 15 seconds (CBOR payload only)
    RECEIVE_INTERVAL = 1      # Check for incoming ESP32 data every 1 second
    RECONNECT_DELAY = 5

    # Alarm bitmask mapping
    ALARM_MAP = {
        'vac_pump': 1,
        'panel_power': 2,
        'overfill': 4,
        'digital_storage': 8,
        'under_pressure': 16,
        'over_pressure': 32,
        'zero_pressure': 64,
        'variable_pressure': 128,
        'pressure_sensor': 256,
        '72_hour_shutdown': 512
    }

    def __init__(self, data_handler):
        self.data_handler = data_handler
        self.serial_port = None
        self.online = False
        self._send_event = None      # Scheduled event for sending data (15s)
        self._receive_event = None   # Scheduled event for receiving data (1s)
        
        # ESP32 status variables (received from IO Board)
        self.esp32_datetime = None       # Timestamp from modem (string)
        self.esp32_sdcard_status = None  # "OK" or "FAULT"
        self.esp32_passthrough = 0       # 0=normal, 1=passthrough mode active
        self.esp32_last_update = None    # time.time() of last received message
        
        # Cellular signal quality (received from IO Board)
        self.esp32_lte_connected = False # True if connected to LTE network
        self.esp32_rsrp = None           # Signal strength in dBm (e.g., -85.5)
        self.esp32_rsrq = None           # Signal quality in dB (e.g., -10.2)
        
        # =====================================================================
        # SERIAL-ONLY: ESP32 sensor data received via serial.
        # The ESP32 reads the ADS1015 ADC at 60Hz with rolling averages and
        # sends computed values in its 15-second status JSON. These variables
        # hold the latest received values — they are the ONLY source of
        # pressure, current, and overfill data (no I2C fallback exists).
        # =====================================================================
        self.esp32_pressure = 0.0        # Pressure in IWC from ESP32 ADS1015 (float)
        self.esp32_current = 0.0         # Motor current in amps from ESP32 ADS1015 (float)
        self.esp32_overfill = False      # Overfill alarm state from ESP32 GPIO38 (bool)
        self.esp32_profile = ''          # Active profile name on ESP32 (string)
        self.esp32_failsafe = False      # True when ESP32 is in failsafe relay control (bool)
        
        # SERIAL-ONLY: Mode-change detection for immediate relay control.
        # IOManager updates ModeManager mmap (even from child processes).
        # This serial manager polls it every 0.5s and sends mode to ESP32.
        self._last_sent_mode = None      # Last mode number sent to ESP32 (for change detection)
        self._mode_check_event = None    # Kivy Clock event for mode polling
        
        # Passthrough mode callback (called when passthrough value changes)
        # Set this to a function that takes (old_value, new_value) as arguments
        # Example: manager.on_passthrough_change = my_ppp_handler
        self.on_passthrough_change = None
        
        # PPP (Point-to-Point Protocol) management
        self.ppp_process = None          # subprocess.Popen object for pppd
        self.ppp_timeout_minutes = 0     # Requested timeout in minutes
        self.ppp_start_time = None       # time.time() when PPP started
        self.ppp_timer_thread = None     # Background thread for timeout
        self.ppp_active = False          # True while PPP is running

    def _log(self, level, message):
        '''Log message with specified level'''
        level = level.lower()
        if hasattr(Logger, level):
            getattr(Logger, level)(f'Serial: {message}')
        else:
            Logger.warning(f'Serial: Invalid log level "{level}". Message: {message}')

    # -------------------------------------------------------------------------
    # Serial Port Management
    # -------------------------------------------------------------------------

    def _initialize_serial(self):
        '''Initialize serial port connection'''
        try:
            if self.serial_port and self.serial_port.is_open:
                return True

            self.serial_port = serial.Serial(
                port=self.SERIAL_PORT,
                baudrate=self.BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            time.sleep(0.1)  # Brief stabilization
            self.online = True
            self._log('info', f'Serial port {self.SERIAL_PORT} opened at {self.BAUD_RATE} baud')
            return True

        except serial.SerialException as e:
            self._log('error', f'Failed to open serial port: {e}')
            self.online = False
            return False
        except Exception as e:
            self._log('error', f'Unexpected error initializing serial: {e}')
            self.online = False
            return False

    def _close_serial(self):
        '''Close serial port connection'''
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
                self._log('info', 'Serial port closed')
            except Exception as e:
                self._log('error', f'Error closing serial port: {e}')
        self.serial_port = None
        self.online = False

    # -------------------------------------------------------------------------
    # PPP (Point-to-Point Protocol) Management
    # -------------------------------------------------------------------------
    
    # Escape sequence to signal ESP32 to exit passthrough mode
    # ESP32 watches for this sequence and restarts to normal mode when detected
    PPP_STOP_SEQUENCE = b'+++STOPPPP\n'

    def connect_ppp(self, timeout_minutes=60):
        '''
        Initiate PPP connection from Linux side.
        
        Sends command to ESP32 to enter passthrough mode, waits for ESP32 to
        restart, then starts PPP connection.
        
        Args:
            timeout_minutes: Duration for PPP connection (default 60 min)
            
        Returns:
            bool: True if PPP started successfully, False otherwise
            
        Example:
            manager.connect_ppp(30)  # Start 30-minute PPP session
        '''
        if self.ppp_active:
            self._log('warning', 'PPP already active')
            return False
            
        self._log('info', f'Initiating PPP connection ({timeout_minutes} min)...')
        
        # Ensure serial port is open
        if not self.serial_port or not self.serial_port.is_open:
            if not self._initialize_serial():
                self._log('error', 'Cannot open serial port')
                return False
        
        # Send passthrough request to ESP32
        # Format: {"command":"passthrough","timeout":XX}
        try:
            request = json.dumps({
                'command': 'passthrough',
                'timeout': timeout_minutes
            })
            self.serial_port.write((request + '\n').encode('ascii'))
            self.serial_port.flush()
            self._log('info', f'Sent passthrough request to ESP32: {request}')
        except Exception as e:
            self._log('error', f'Failed to send passthrough request: {e}')
            return False
        
        # Wait for ESP32 to process and restart (up to 5 seconds)
        self._log('info', 'Waiting for ESP32 to restart into passthrough mode...')
        time.sleep(5)
        
        # Close serial port - pppd will take over
        self._close_serial()
        
        # Wait for ESP32 passthrough boot (additional 3 seconds)
        time.sleep(3)
        
        # Start PPP
        self.ppp_timeout_minutes = timeout_minutes
        self._start_ppp()
        
        return self.ppp_active

    def disconnect_ppp(self):
        '''
        Disconnect PPP and signal ESP32 to return to normal mode.
        
        Stops pppd process, reopens serial port, sends escape sequence
        to ESP32 to trigger restart to normal operation.
        
        Returns:
            bool: True if disconnect successful, False otherwise
            
        Example:
            manager.disconnect_ppp()  # Stop PPP and return to normal
        '''
        if not self.ppp_active:
            self._log('info', 'PPP not active, nothing to disconnect')
            return True
            
        self._log('info', 'Disconnecting PPP...')
        
        # Stop pppd process
        self._stop_ppp()
        
        # Wait for serial port to be released
        time.sleep(2)
        
        # Reopen serial port to send stop signal
        if self._initialize_serial():
            try:
                # Send escape sequence to ESP32 to trigger restart
                self._log('info', 'Sending stop signal to ESP32...')
                self.serial_port.write(self.PPP_STOP_SEQUENCE)
                self.serial_port.flush()
                self._log('info', 'Stop signal sent, ESP32 will restart to normal mode')
                
                # Wait for ESP32 to restart
                time.sleep(3)
                
                # Reopen serial for normal operation
                self._close_serial()
                time.sleep(2)
                self._initialize_serial()
                
            except Exception as e:
                self._log('error', f'Error sending stop signal: {e}')
        
        return not self.ppp_active

    def handle_passthrough_request(self, timeout_minutes):
        '''
        Handle passthrough request from ESP32 - start PPP connection.
        
        Called when ESP32 sends: {"passthrough":"remote XX"}
        
        Sequence:
        1. Close serial port (PPP will use it)
        2. Send "ready" confirmation to ESP32
        3. Start pppd process
        4. Start timeout timer thread
        5. After timeout, stop PPP and resume normal operation
        
        Args:
            timeout_minutes: Duration for PPP connection in minutes
        '''
        self._log('info', f'Passthrough request received: {timeout_minutes} minutes')
        
        # Send "ready" confirmation to ESP32 before closing serial
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.write(b'ready\n')
                self.serial_port.flush()
                self._log('info', 'Sent "ready" confirmation to ESP32')
                time.sleep(0.5)  # Brief delay to ensure message is sent
        except Exception as e:
            self._log('error', f'Error sending ready confirmation: {e}')
        
        # Close serial port - pppd will take over
        self._close_serial()
        
        # Store timeout and start PPP
        self.ppp_timeout_minutes = timeout_minutes
        self._start_ppp()

    def _start_ppp(self):
        '''
        Start PPP daemon process.
        
        Runs: sudo pppd call walter nodetach debug
        
        The "nodetach" option keeps pppd in foreground so we can monitor it.
        The "debug" option provides verbose logging to syslog.
        '''
        if self.ppp_active:
            self._log('warning', 'PPP already active, ignoring start request')
            return
            
        self._log('info', 'Starting PPP connection...')
        
        try:
            # Start pppd process
            self.ppp_process = subprocess.Popen(
                ['sudo', 'pppd', 'call', 'walter', 'nodetach', 'debug'],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.ppp_active = True
            self.ppp_start_time = time.time()
            
            self._log('info', f'PPP started (PID: {self.ppp_process.pid}), '
                             f'timeout: {self.ppp_timeout_minutes} min')
            
            # Start timeout timer in background thread
            self.ppp_timer_thread = threading.Thread(
                target=self._ppp_timeout_monitor,
                daemon=True
            )
            self.ppp_timer_thread.start()
            
        except Exception as e:
            self._log('error', f'Failed to start PPP: {e}')
            self.ppp_active = False
            self._resume_normal_operation()

    def _ppp_timeout_monitor(self):
        '''
        Background thread to monitor PPP timeout and process status.
        
        Checks every 10 seconds:
        - If timeout has expired
        - If pppd process has terminated unexpectedly
        
        When timeout expires or process ends, calls _stop_ppp().
        '''
        timeout_seconds = self.ppp_timeout_minutes * 60
        check_interval = 10  # Check every 10 seconds
        
        self._log('info', f'PPP timeout monitor started: {timeout_seconds}s')
        
        while self.ppp_active:
            time.sleep(check_interval)
            
            # Check if pppd process has terminated
            if self.ppp_process and self.ppp_process.poll() is not None:
                exit_code = self.ppp_process.returncode
                self._log('warning', f'PPP process terminated (exit code: {exit_code})')
                self._stop_ppp()
                return
            
            # Check if timeout expired
            elapsed = time.time() - self.ppp_start_time
            remaining = timeout_seconds - elapsed
            
            if remaining <= 0:
                self._log('info', 'PPP timeout expired')
                self._stop_ppp()
                return
            
            # Log remaining time every minute
            if int(elapsed) % 60 < check_interval:
                self._log('debug', f'PPP running, {int(remaining/60)} min remaining')

    def _stop_ppp(self):
        '''
        Stop PPP daemon process and resume normal operation.
        
        Terminates pppd gracefully (SIGTERM), then forcefully (SIGKILL) if needed.
        After PPP stops, reopens serial port for normal communication.
        '''
        if not self.ppp_active:
            return
            
        self._log('info', 'Stopping PPP connection...')
        
        try:
            if self.ppp_process:
                # Try graceful termination first
                self.ppp_process.terminate()
                
                # Wait up to 5 seconds for graceful shutdown
                try:
                    self.ppp_process.wait(timeout=5)
                    self._log('info', 'PPP terminated gracefully')
                except subprocess.TimeoutExpired:
                    # Force kill if still running
                    self._log('warning', 'PPP not responding, forcing kill')
                    self.ppp_process.kill()
                    self.ppp_process.wait(timeout=2)
                    
        except Exception as e:
            self._log('error', f'Error stopping PPP: {e}')
            # Try to kill via sudo as fallback
            try:
                subprocess.run(['sudo', 'pkill', '-9', 'pppd'], timeout=5)
            except Exception:
                pass
        
        self.ppp_process = None
        self.ppp_active = False
        self.ppp_start_time = None
        
        # Resume normal serial operation
        self._resume_normal_operation()

    def _resume_normal_operation(self):
        '''
        Resume normal serial communication after PPP ends.
        
        Waits briefly for serial port to be released, then reopens it.
        '''
        self._log('info', 'Resuming normal serial operation...')
        
        # Wait for serial port to be released
        time.sleep(2)
        
        # Reopen serial port
        if self._initialize_serial():
            self._log('info', 'Serial port reopened, normal operation resumed')
        else:
            self._log('error', 'Failed to reopen serial port after PPP')
            # Schedule retry
            Clock.schedule_once(lambda dt: self._initialize_serial(), 5)

    def is_ppp_active(self):
        '''Check if PPP connection is currently active'''
        return self.ppp_active

    # -------------------------------------------------------------------------
    # ESP32 Data Reception
    # -------------------------------------------------------------------------

    def receive_esp32_status(self):
        '''
        Check for and parse incoming JSON status from ESP32.
        
        Expected JSON format:
            {"datetime":"2026-01-29 12:34:56","sdcard":"OK","passthrough":0}
        
        Returns:
            dict: Parsed status data, or None if no data available
            
        Updates instance variables:
            - self.esp32_datetime
            - self.esp32_sdcard_status  
            - self.esp32_passthrough
            - self.esp32_last_update
            
        Triggers on_passthrough_change callback if passthrough value changes.
        '''
        if not self.serial_port or not self.serial_port.is_open:
            return None
            
        try:
            # Check if data is waiting
            if self.serial_port.in_waiting == 0:
                return None
            
            # =================================================================
            # SPEED FIX: Drain the ENTIRE serial buffer and use only the LATEST
            # message. Previously, readline() read the oldest queued message.
            # ESP32 sends a packet every 1 second, and any timing drift causes
            # messages to pile up. Reading only the oldest meant pressure values
            # were always stale (seconds behind the actual ADC reading).
            #
            # Now: read all available lines, keep only the last valid JSON.
            # This ensures pressure/current values are always fresh (<1 second old).
            # =================================================================
            latest_line = None
            lines_drained = 0
            while self.serial_port.in_waiting > 0:
                try:
                    raw_data = self.serial_port.readline()
                    if raw_data:
                        decoded = raw_data.decode('ascii', errors='ignore').strip()
                        if decoded.startswith('{'):
                            latest_line = decoded
                        lines_drained += 1
                except Exception:
                    break  # Stop draining on any read error
                # Safety limit — don't loop forever on a flooded buffer
                if lines_drained > 50:
                    break
            
            if lines_drained > 2:
                self._log('debug', f'Drained {lines_drained} buffered lines (using latest)')
            
            if not latest_line:
                return None
            
            line = latest_line
                
            # Parse JSON
            try:
                data = json.loads(line)
            except json.JSONDecodeError as e:
                self._log('warning', f'Invalid JSON from ESP32: {line} - {e}')
                return None
                
            # =================================================================
            # SERIAL-ONLY: Handle commands forwarded from ESP32 web portal.
            # When Linux is connected, the ESP32 web portal's "Start Cycle"
            # and "Stop" buttons forward commands to Linux via serial instead
            # of running the failsafe cycle engine. This lets Linux manage
            # cycles through its normal IOManager path (with alarms, DB, etc.)
            #
            # Expected formats:
            #   {"command":"start_cycle","type":"run"}
            #   {"command":"start_cycle","type":"manual_purge"}
            #   {"command":"start_cycle","type":"clean"}
            #   {"command":"stop_cycle"}
            # =================================================================
            if 'command' in data:
                cmd = data['command']
                if cmd == 'start_cycle':
                    cycle_type = data.get('type', 'run')
                    self._log('info', f'ESP32 web portal requested start_cycle: {cycle_type}')
                    try:
                        app = self.data_handler.app
                        if hasattr(app, 'io') and app.io:
                            # from_web=True bypasses the Kivy screen guard — the web portal
                            # has its own interface and isn't tied to a touchscreen screen.
                            if cycle_type == 'run':
                                app.io.run_cycle(from_web=True)
                            elif cycle_type == 'manual_purge':
                                app.io.run_cycle(is_manual=True, from_web=True)
                            elif cycle_type == 'clean':
                                if hasattr(app.io, 'canister_clean'):
                                    app.io.canister_clean(from_web=True)
                                else:
                                    app.io.run_cycle()  # Fallback to normal cycle
                            self._log('info', f'Started {cycle_type} cycle from web portal')
                    except Exception as e:
                        self._log('error', f'Error starting cycle from web portal: {e}')
                    return data
                    
                elif cmd == 'stop_cycle':
                    self._log('info', 'ESP32 web portal requested stop_cycle')
                    try:
                        app = self.data_handler.app
                        if hasattr(app, 'io') and app.io:
                            app.io.stop_cycle()
                            self._log('info', 'Stopped cycle from web portal')
                    except Exception as e:
                        self._log('error', f'Error stopping cycle from web portal: {e}')
                    return data
                
                # =================================================================
                # SERIAL-ONLY: Web portal test commands forwarded by ESP32.
                # The web portal's Tests screens (Leak, Functionality, Efficiency)
                # send start_test with a type, which ESP32 forwards here as JSON:
                #   {"command":"start_test","type":"leak"}
                #   {"command":"start_test","type":"func"}
                #   {"command":"start_test","type":"eff"}
                # Each maps to the corresponding IOManager test function.
                # stop_test is handled by stop_cycle above (same Python function).
                # =================================================================
                elif cmd == 'start_test':
                    test_type = data.get('type', '')
                    self._log('info', f'ESP32 web portal requested start_test: {test_type}')
                    try:
                        app = self.data_handler.app
                        if hasattr(app, 'io') and app.io:
                            if test_type == 'leak':
                                # Leak test: 30 min in leak mode (CR1+CR2+CR5 ON, no motor)
                                # from_web=True bypasses the Kivy screen guard — the web portal
                                # has its own password protection and isn't tied to a screen.
                                if hasattr(app.io, 'leak_test'):
                                    app.io.leak_test(from_web=True)
                                else:
                                    self._log('warning', 'IOManager has no leak_test() method')
                            elif test_type == 'func':
                                # Functionality test: 10x (60s run + 60s purge)
                                if hasattr(app.io, 'functionality_test'):
                                    app.io.functionality_test(from_web=True)
                                else:
                                    self._log('warning', 'IOManager has no functionality_test() method')
                            elif test_type == 'eff':
                                # Efficiency test: 120s fill/run phase
                                if hasattr(app.io, 'efficiency_test_fill_run'):
                                    app.io.efficiency_test_fill_run(from_web=True)
                                else:
                                    self._log('warning', 'IOManager has no efficiency_test_fill_run() method')
                            else:
                                self._log('warning', f'Unknown test type: {test_type}')
                            self._log('info', f'Started {test_type} test from web portal')
                    except Exception as e:
                        self._log('error', f'Error starting test from web portal: {e}')
                    return data
            
            # Check for passthrough REQUEST (string like "remote 60")
            # vs passthrough STATUS (integer 0 or 1)
            if 'passthrough' in data:
                pt_value = data['passthrough']
                
                # Passthrough REQUEST: {"passthrough":"remote XX"}
                if isinstance(pt_value, str) and 'remote' in pt_value.lower():
                    # Extract timeout from "remote XX" format
                    match = re.search(r'remote\s*(\d+)?', pt_value, re.IGNORECASE)
                    if match:
                        timeout_str = match.group(1)
                        timeout_minutes = int(timeout_str) if timeout_str else 60
                        self._log('info', f'Passthrough REQUEST detected: {timeout_minutes} min')
                        
                        # Handle the passthrough request (sends ready, starts PPP)
                        self.handle_passthrough_request(timeout_minutes)
                        return data  # Exit early - serial will be closed
                
                # Passthrough STATUS: {"passthrough":0} or {"passthrough":1}
                else:
                    try:
                        self.esp32_passthrough = int(pt_value)
                    except (ValueError, TypeError):
                        self._log('warning', f'Invalid passthrough value: {pt_value}')
            
            # Extract and store other values (normal status message)
            old_passthrough = self.esp32_passthrough
            
            if 'datetime' in data:
                self.esp32_datetime = data['datetime']
                
            if 'sdcard' in data:
                self.esp32_sdcard_status = data['sdcard']
            
            # Parse LTE connection status
            if 'lte' in data:
                self.esp32_lte_connected = bool(int(data['lte']))
            
            # Parse signal quality (RSRP and RSRQ)
            if 'rsrp' in data:
                try:
                    self.esp32_rsrp = float(data['rsrp'])
                except (ValueError, TypeError):
                    self.esp32_rsrp = None
                    
            if 'rsrq' in data:
                try:
                    self.esp32_rsrq = float(data['rsrq'])
                except (ValueError, TypeError):
                    self.esp32_rsrq = None
            
            # =================================================================
            # SERIAL-ONLY: Parse sensor data from ESP32 status packet.
            # ESP32 reads the ADS1015 at 60Hz with rolling averages and sends
            # computed values. This is the ONLY source of sensor data —
            # no I2C ADC reads exist in this build.
            # =================================================================
            if 'pressure' in data:
                try:
                    self.esp32_pressure = float(data['pressure'])
                except (ValueError, TypeError):
                    pass  # Keep last good value
            
            if 'current' in data:
                try:
                    self.esp32_current = float(data['current'])
                except (ValueError, TypeError):
                    pass  # Keep last good value
            
            # SERIAL-ONLY: Overfill from ESP32 GPIO38 with hysteresis validation.
            # This is the ONLY overfill source — MCP23017 TLS pin read removed.
            if 'overfill' in data:
                try:
                    self.esp32_overfill = bool(int(data['overfill']))
                except (ValueError, TypeError):
                    pass  # Keep last good value
            
            # SERIAL-ONLY: Profile and failsafe status from ESP32
            if 'profile' in data:
                self.esp32_profile = str(data['profile'])
            
            if 'failsafe' in data:
                try:
                    self.esp32_failsafe = bool(int(data['failsafe']))
                except (ValueError, TypeError):
                    pass
            
            # ================================================================
            # CALIBRATION RESPONSE: ESP32 sends {"type":"data","ps_cal":964.5}
            # after completing a pressure sensor zero-point calibration.
            # Save the calibration factor to the Python database (gm_db) so
            # the Linux program has a persistent copy of the ADC zero point.
            # Key: 'adc_zero' — matches original pressure_sensor.py convention.
            #
            # Usage: Triggered automatically when ESP32 finishes calibration.
            #   ESP32 sends → modem.py parses → saves to gm_db → logs result.
            # ================================================================
            if 'ps_cal' in data:
                try:
                    cal_value = float(data['ps_cal'])
                    self._log('info', f'Received pressure calibration from ESP32: ps_cal={cal_value}')
                    # Save to Python database using same key as original pressure_sensor.py
                    if hasattr(self, 'data_handler') and self.data_handler:
                        app = getattr(self.data_handler, 'app', None)
                        if app and hasattr(app, 'gm_db'):
                            app.gm_db.add_setting('adc_zero', cal_value)
                            self._log('info', f'Saved pressure calibration to database: adc_zero={cal_value}')
                        else:
                            self._log('warning', 'Cannot save ps_cal — gm_db not available')
                    else:
                        self._log('warning', 'Cannot save ps_cal — data_handler not available')
                except (ValueError, TypeError) as e:
                    self._log('error', f'Invalid ps_cal value: {data["ps_cal"]} — {e}')
                
            self.esp32_last_update = time.time()
            
            # REV 10.5: Log ONLY the fields that were actually in THIS packet.
            # Fast sensor packets (5Hz) contain: pressure, current, overfill, sdcard, relayMode
            # Fresh cellular packets (~60s) contain: datetime, lte, rsrp, rsrq, passthrough, etc.
            # Previously this logged ALL cached values on every packet, making it look
            # like datetime/rsrp/rsrq were being repeated — they were just cached.
            received_fields = ', '.join(f'{k}={v}' for k, v in data.items())
            self._log('debug', f'ESP32 packet: {received_fields}')
            
            # Trigger callback if passthrough status changed
            if self.esp32_passthrough != old_passthrough:
                self._log('warning', f'PASSTHROUGH STATUS CHANGED: {old_passthrough} -> {self.esp32_passthrough}')
                if self.on_passthrough_change:
                    try:
                        self.on_passthrough_change(old_passthrough, self.esp32_passthrough)
                    except Exception as e:
                        self._log('error', f'Error in passthrough callback: {e}')
                        
            return data
            
        except serial.SerialException as e:
            self._log('error', f'Serial read error: {e}')
            return None
        except Exception as e:
            self._log('error', f'Error receiving ESP32 status: {e}')
            return None

    def is_passthrough_active(self):
        '''Check if ESP32 is currently in passthrough mode'''
        return self.esp32_passthrough == 1
    
    def is_lte_connected(self):
        '''Check if ESP32 reports LTE connection is active'''
        return self.esp32_lte_connected
    
    def get_signal_quality(self):
        '''
        Get cellular signal quality information.
        
        Returns:
            dict: Signal quality data, or None if not available
            
        Example:
            {
                'rsrp': -85.5,       # dBm (signal strength)
                'rsrq': -10.2,       # dB (signal quality)
                'quality': 'Good',   # Human-readable quality
                'lte_connected': True
            }
            
        Signal Quality Interpretation:
            RSRP (Reference Signal Received Power):
                >= -80 dBm: Excellent
                -80 to -90 dBm: Good
                -90 to -100 dBm: Fair
                < -100 dBm: Poor
                
            RSRQ (Reference Signal Received Quality):
                >= -10 dB: Good
                -10 to -15 dB: Fair
                < -15 dB: Poor
        '''
        if self.esp32_rsrp is None and self.esp32_rsrq is None:
            return None
            
        # Determine quality description
        quality = 'Unknown'
        if self.esp32_rsrp is not None:
            if self.esp32_rsrp >= -80:
                quality = 'Excellent'
            elif self.esp32_rsrp >= -90:
                quality = 'Good'
            elif self.esp32_rsrp >= -100:
                quality = 'Fair'
            else:
                quality = 'Poor'
        
        return {
            'rsrp': self.esp32_rsrp,
            'rsrq': self.esp32_rsrq,
            'quality': quality,
            'lte_connected': self.esp32_lte_connected
        }
        
    def get_esp32_status(self):
        '''
        Get current ESP32 status as dictionary.
        
        Returns:
            dict: Current status values, or None if never received
            
        Example:
            {
                'datetime': '2026-01-29 12:34:56',
                'sdcard': 'OK',
                'passthrough': 0,
                'lte_connected': True,
                'rsrp': -85.5,
                'rsrq': -10.2,
                'signal_quality': 'Good',
                'last_update': 1738171234.567,
                'age_seconds': 5.2
            }
        '''
        if self.esp32_last_update is None:
            return None
        
        # Determine signal quality description based on RSRP
        # RSRP: -80 or better = Excellent, -80 to -90 = Good, -90 to -100 = Fair, below -100 = Poor
        signal_quality = 'Unknown'
        if self.esp32_rsrp is not None:
            if self.esp32_rsrp >= -80:
                signal_quality = 'Excellent'
            elif self.esp32_rsrp >= -90:
                signal_quality = 'Good'
            elif self.esp32_rsrp >= -100:
                signal_quality = 'Fair'
            else:
                signal_quality = 'Poor'
            
        return {
            'datetime': self.esp32_datetime,
            'sdcard': self.esp32_sdcard_status,
            'passthrough': self.esp32_passthrough,
            'lte_connected': self.esp32_lte_connected,
            'rsrp': self.esp32_rsrp,
            'rsrq': self.esp32_rsrq,
            'signal_quality': signal_quality,
            'last_update': self.esp32_last_update,
            'age_seconds': round(time.time() - self.esp32_last_update, 1)
        }

    # -------------------------------------------------------------------------
    # Data Collection (from Kivy app)
    # -------------------------------------------------------------------------

    def _get_device_id(self):
        '''Get device name from app'''
        return self.data_handler.app.device_name or 'RND-9999'

    def _get_run_cycles(self):
        '''Get run cycle count'''
        try:
            return int(self.data_handler.app.current_run_cycle_count or 0)
        except (ValueError, TypeError, AttributeError):
            return 0

    def _get_pressure(self):
        '''Get pressure value as float — reads DIRECTLY from ESP32 serial data.
        
        SPEED FIX: Previously read from self.data_handler.app.current_pressure,
        which is a Kivy StringProperty ("X.XX IWC") updated by a 1-second Clock.
        That added up to 1 second of extra staleness and required string parsing.
        Now reads self.esp32_pressure directly — the freshest value from the
        last serial packet received from the ESP32 (updated every ~1 second).
        '''
        try:
            return round(float(self.esp32_pressure), 2)
        except (ValueError, TypeError, AttributeError):
            return 0.0

    def _get_current(self):
        '''Get motor current as float — reads DIRECTLY from ESP32 serial data.
        
        SPEED FIX: Previously read from self.data_handler.app.current_amps,
        which is a Kivy StringProperty ("X.XX A") updated by a 1-second Clock.
        Now reads self.esp32_current directly for freshest possible value.
        '''
        try:
            return round(float(self.esp32_current), 2)
        except (ValueError, TypeError, AttributeError):
            return 0.0

    def _get_cpu_temp(self):
        '''Get Raspberry Pi CPU temperature'''
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return round(float(f.read()) / 1000.0, 1)
        except Exception:
            return 0.0

    def _get_active_alarms(self):
        '''Get active alarms as bitmask'''
        try:
            active_alarms = self.data_handler.app.get_active_alarm_names()
            return sum(
                num for name, num in self.ALARM_MAP.items()
                if name in active_alarms
            )
        except Exception:
            return 0

    def _get_mode(self):
        '''Get current operating mode'''
        return self.data_handler.get_current_mode()

    # -------------------------------------------------------------------------
    # Payload Creation
    # -------------------------------------------------------------------------

    def create_payload(self):
        '''Create payload dict from sensor data'''
        try:
            return {
                'gmid': self._get_device_id(),
                'press': self._get_pressure(),
                'mode': self._get_mode(),
                'current': self._get_current(),
                'fault': self._get_active_alarms(),
                'cycles': self._get_run_cycles(),
                'temp': self._get_cpu_temp()
            }
        except Exception as e:
            self._log('error', f'Error creating payload: {e}')
            return None

    def _format_json(self, payload):
        '''Format payload as compact JSON string for ESP32'''
        data = {
            'type': 'data',
            'gmid': payload['gmid'],
            'press': payload['press'],
            'mode': payload['mode'],
            'current': payload['current'],
            'fault': payload['fault'],
            'cycles': payload['cycles']
        }
        return json.dumps(data, separators=(',', ':'))

    # -------------------------------------------------------------------------
    # Data Transmission
    # -------------------------------------------------------------------------

    def send_data(self, payload):
        '''Send payload over serial to ESP32'''
        if not payload:
            return False

        # Ensure serial port is open
        if not self.serial_port or not self.serial_port.is_open:
            if not self._initialize_serial():
                return False

        try:
            json_string = self._format_json(payload)
            bytes_written = self.serial_port.write(json_string.encode('ascii'))
            self.serial_port.flush()
            self._log('debug', f'Sent {bytes_written} bytes: {json_string}')
            return True

        except serial.SerialException as e:
            self._log('error', f'Serial write error: {e}')
            self._close_serial()
            return False
        except Exception as e:
            self._log('error', f'Error sending data: {e}')
            return False

    # -------------------------------------------------------------------------
    # Update Cycle
    # -------------------------------------------------------------------------

    def _receive_cycle(self, *args):
        '''Check for incoming data from ESP32 (runs every 1 second)'''
        # Skip if PPP is active (serial port is in use by pppd)
        if self.ppp_active:
            return
            
        try:
            self.receive_esp32_status()
        except Exception as e:
            self._log('error', f'Error in receive cycle: {e}')
    
    def _send_cycle(self, *args):
        '''Send sensor data to ESP32 every 15 seconds (feeds CBOR payload builder)'''
        # Skip if PPP is active (serial port is in use by pppd)
        if self.ppp_active:
            return
            
        try:
            # If passthrough mode is active, skip sending data
            # (ESP32 is bridging serial to modem, don't interfere)
            if self.is_passthrough_active():
                self._log('debug', 'Passthrough active - skipping data send')
                return
            
            # Normal operation - send sensor data to ESP32
            payload = self.create_payload()
            if payload:
                self.send_data(payload)
        except Exception as e:
            self._log('error', f'Error in send cycle: {e}')

    # -------------------------------------------------------------------------
    # SERIAL-ONLY: Immediate Mode Relay Control via Serial
    # -------------------------------------------------------------------------
    # ESP32 is the sole relay controller. These methods detect mode changes
    # from the ModeManager mmap file (written by IOManager child processes)
    # and send them to ESP32 immediately. ESP32 calls setRelaysForMode()
    # to set physical relay outputs.
    #
    # Mode number mapping (matches ESP32 setRelaysForMode()):
    #   rest=0, run=1, purge=2, burp=3, bleed=8, leak=9
    # -------------------------------------------------------------------------

    # Map mode strings (from ModeManager) to ESP32 relay mode numbers
    MODE_STRING_TO_NUMBER = {
        'rest': 0, 'run': 1, 'purge': 2, 'burp': 3,
        'bleed': 8, 'leak': 9
    }

    def send_mode_immediate(self, mode_num):
        '''
        Send a relay mode command to ESP32 immediately (non-periodic).
        
        SERIAL-ONLY: Called when a mode change is detected via ModeManager
        polling. The ESP32 sets physical relay outputs based on the mode
        number. This is the ONLY relay control mechanism in this build.
        
        Args:
            mode_num: Integer mode (0=rest, 1=run, 2=purge, 3=burp, 8=bleed, 9=leak)
            
        Example:
            send_mode_immediate(1)  # ESP32 turns on Motor + CR1 + CR5 (RUN mode)
        '''
        if self.ppp_active or self.is_passthrough_active():
            return  # Don't send during PPP — serial port in use by pppd
        
        if not self.serial_port or not self.serial_port.is_open:
            if not self._initialize_serial():
                return
        
        try:
            # Minimal JSON — ESP32 only needs "type" and "mode" to set relays
            msg = json.dumps({'type': 'data', 'mode': mode_num}, separators=(',', ':'))
            self.serial_port.write((msg + '\n').encode('ascii'))
            self.serial_port.flush()
            self._last_sent_mode = mode_num
            self._log('debug', f'Sent immediate mode {mode_num} to ESP32')
        except Exception as e:
            self._log('error', f'Error sending immediate mode: {e}')

    def send_shutdown_command(self):
        '''
        Send shutdown command to ESP32 — sets DISP_SHUTDN (GPIO13) LOW.
        
        SERIAL-ONLY: ESP32 recognizes {"mode":"shutdown"} to trigger
        DISP_SHUTDN LOW. Use send_normal_command() to restore it HIGH.
        
        Example:
            send_shutdown_command()  # ESP32 sets GPIO13 LOW (site shutdown)
        '''
        if self.ppp_active or self.is_passthrough_active():
            return
        
        if not self.serial_port or not self.serial_port.is_open:
            if not self._initialize_serial():
                return
        
        try:
            # ESP32 checks: if (modeStr == "shutdown") activateDispShutdown();
            msg = json.dumps({'type': 'data', 'mode': 'shutdown'}, separators=(',', ':'))
            self.serial_port.write((msg + '\n').encode('ascii'))
            self.serial_port.flush()
            self._log('info', 'Sent shutdown command to ESP32 — DISP_SHUTDN LOW')
        except Exception as e:
            self._log('error', f'Error sending shutdown command: {e}')

    def send_normal_command(self):
        '''
        Send normal command to ESP32 — restores DISP_SHUTDN (GPIO13) HIGH.
        
        SERIAL-ONLY: Clears a previous 72-hour shutdown. ESP32 recognizes
        {"mode":"normal"} and calls deactivateDispShutdown() to set GPIO13 HIGH.
        
        Example:
            send_normal_command()  # ESP32 sets GPIO13 HIGH (site restored)
        '''
        if self.ppp_active or self.is_passthrough_active():
            return
        
        if not self.serial_port or not self.serial_port.is_open:
            if not self._initialize_serial():
                return
        
        try:
            # ESP32 checks: if (modeStr == "normal") deactivateDispShutdown();
            msg = json.dumps({'type': 'data', 'mode': 'normal'}, separators=(',', ':'))
            self.serial_port.write((msg + '\n').encode('ascii'))
            self.serial_port.flush()
            self._log('info', 'Sent normal command to ESP32 — DISP_SHUTDN HIGH (site restored)')
        except Exception as e:
            self._log('error', f'Error sending normal command: {e}')

    def send_calibration_command(self):
        '''
        Send pressure calibration command to ESP32 — zeros the pressure sensor.
        
        SERIAL-ONLY: Assumes the pressure sensor is currently at atmospheric
        pressure (0.0 IWC). ESP32 collects 60 ADC samples, computes a trimmed
        mean, and saves the new zero point to EEPROM. The calibration persists
        across reboots.
        
        Mirrors Python pressure_sensor.py calibrate() behavior.
        
        Example:
            send_calibration_command()  # ESP32 recalibrates pressure zero point
        '''
        if self.ppp_active or self.is_passthrough_active():
            self._log('warning', 'Cannot calibrate during PPP/passthrough')
            return False
        
        if not self.serial_port or not self.serial_port.is_open:
            if not self._initialize_serial():
                return False
        
        try:
            # {"type":"cmd","cmd":"cal"} — command message, not a data packet
            msg = json.dumps({'type': 'cmd', 'cmd': 'cal'}, separators=(',', ':'))
            self.serial_port.write((msg + '\n').encode('ascii'))
            self.serial_port.flush()
            self._log('info', 'Sent calibration command to ESP32 — pressure zero point will be recalculated')
            return True
        except Exception as e:
            self._log('error', f'Error sending calibration command: {e}')
            return False

    def _mode_check_cycle(self, *args):
        '''
        Poll ModeManager for mode changes and send to ESP32 immediately.
        
        SERIAL-ONLY: Runs every 0.1 seconds (100ms) via Kivy Clock. Reads the
        current mode from the memory-mapped file (written by IOManager or
        its child processes) and sends it to ESP32 only when it changes.
        This bridges the gap between multiprocessing mode writes and the
        serial relay control on ESP32 — the ONLY relay control path.
        '''
        if self.ppp_active:
            return  # Don't interfere during PPP
        
        try:
            # Read current mode from ModeManager mmap (cross-process safe)
            mode_str = self.data_handler.app.io.mode_manager.get_mode()
            if mode_str is None:
                return
            
            mode_num = self.MODE_STRING_TO_NUMBER.get(mode_str, 0)
            
            # Only send if mode actually changed (avoids flooding serial)
            if mode_num != self._last_sent_mode:
                self.send_mode_immediate(mode_num)
                self._log('info', f'Mode changed: {self._last_sent_mode} → {mode_num} ({mode_str})')
        except Exception as e:
            self._log('error', f'Error in mode check cycle: {e}')

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def start(self):
        '''Start periodic send/receive using Kivy Clock'''
        # Initialize serial port
        self._initialize_serial()

        # Initial send after brief delay
        Clock.schedule_once(self._send_cycle, 5)

        # Schedule periodic data sending (every 15 seconds — feeds CBOR builder on ESP32)
        self._send_event = Clock.schedule_interval(
            self._send_cycle,
            self.UPDATE_INTERVAL
        )
        
        # Schedule frequent receive checking (every 1 second)
        # This ensures passthrough mode changes are detected quickly
        self._receive_event = Clock.schedule_interval(
            self._receive_cycle,
            self.RECEIVE_INTERVAL
        )
        
        # SERIAL-ONLY: Poll ModeManager for mode changes and send to ESP32.
        # SPEED FIX: Reduced from 500ms to 100ms. When a cycle sequence runs
        # in a child process, this is the bridge that detects mmap mode changes
        # and sends them to ESP32. At 100ms, relay commands reach ESP32 within
        # ~200ms total (100ms poll + 100ms ESP32 read interval).
        self._mode_check_event = Clock.schedule_interval(
            self._mode_check_cycle,
            0.1  # 100ms polling — fast relay response, still light on CPU
        )
        
        self._log('info', f'Started: send every {self.UPDATE_INTERVAL}s, receive check every {self.RECEIVE_INTERVAL}s, mode poll every 0.1s')

    def stop(self):
        '''Stop periodic updates, PPP if running, and close serial'''
        if self._send_event:
            Clock.unschedule(self._send_event)
            self._send_event = None
        if self._receive_event:
            Clock.unschedule(self._receive_event)
            self._receive_event = None
        # SERIAL-ONLY: Stop mode polling
        if self._mode_check_event:
            Clock.unschedule(self._mode_check_event)
            self._mode_check_event = None
        
        # Stop PPP if running
        if self.ppp_active:
            self._log('info', 'Stopping PPP before shutdown...')
            self._stop_ppp()
        
        self._close_serial()
        self._log('info', 'Stopped')


def main():
    '''Initialize and start serial manager (standalone test)'''
    data_handler = DataHandler()
    manager = SerialManager(data_handler)
    manager.start()


if __name__ == '__main__':
    main()
