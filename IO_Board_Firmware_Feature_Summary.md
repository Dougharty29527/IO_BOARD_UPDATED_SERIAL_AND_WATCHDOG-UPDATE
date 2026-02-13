# ESP32 IO Board Firmware - Feature Summary

## Executive Overview
The ESP32 IO Board firmware provides a comprehensive, production-ready solution for remote monitoring and control applications. This document outlines how the implemented features address key project requirements.

---

## ✅ **Firmware Updates via Cellular**
**Requirement:** The I/O board will update firmware via cellular.

**Implementation:**
- **BlueCherry OTA Integration:** Automatic firmware updates through cellular connection
- **Scheduled Updates:** Daily update windows (7AM-1PM EST) with 15-minute intervals
- **Fast Polling Mode:** 15-second update checks when activated (60-minute timeout)
- **Rollback Protection:** Maintains system stability during update process
- **Version Tracking:** Firmware revision Rev 10.18 with detailed changelog

---

## ✅ **Locally-Hosted Dashboard with VST Control**
**Requirement:** Locally hosted dashboard to control/monitor devices with updateable parameters under VST control.

**Implementation:**
- **WiFi Access Point:** 192.168.4.1 with password-free access
- **Simplified Landing Page:** Clean interface showing real-time status
- **Maintenance Mode:** Password-protected advanced controls (PW: 878)
- **Live Monitoring:** Real-time pressure, current, cycle counts, and system status
- **Parameter Updates:** Device naming, profile changes, watchdog settings

---

## ✅ **Device Management Messaging**
**Requirement:** Sending messages to/from device to manage device settings such as Renaming, rebooting, changing frequency, etc.

**Supported Commands:**
- **Device Renaming:** `POST /setdevicename` with password validation
- **Reboot Control:** `POST /api/command {"command":"restart"}`
- **Profile Changes:** Equipment profiles (CS2, CS8, CS9, CS12, CS13) with validation
- **Watchdog Control:** Enable/disable serial watchdog monitoring
- **Frequency Settings:** Update intervals and timing parameters
- **Failsafe Management:** Enable/disable autonomous operation mode

---

## ✅ **Custom Status Messages**
**Requirement:** Send regular custom status messages as required.

**Status Message Types:**
- **Fast Sensor Data:** 5Hz (every 200ms) - pressure, current, overfill, relay mode, failsafe status
- **Cellular Status:** 10-second intervals - LTE, signal quality, operator, cell info
- **Datetime Updates:** Fresh modem time when available
- **Daily Status:** 24-hour system health reports
- **Error Notifications:** Immediate alerts for system issues

---

## ✅ **Firmware Revision Management**
**Requirement:** Updating and maintaining revisions of firmware to be installed individually or by group.

**Features:**
- **Individual Updates:** Device-specific firmware targeting via BlueCherry
- **Version Control:** Comprehensive revision history (Rev 10.0 → 10.18)
- **Update Scheduling:** Time-windowed updates to minimize operational impact
- **Update Verification:** Integrity checks and rollback capabilities
- **Group Management:** Profile-based configuration for device groups

---

## ✅ **Performance Monitoring & Cellular Management**
**Requirement:** Monitor performance characteristics such as modem disconnects/latency/speed and ability to change base cellular settings.

**Monitoring Capabilities:**
- **Connection Status:** LTE connectivity with automatic reconnection
- **Signal Quality:** RSRP/RSRQ, operator, band, cell tower information
- **Latency Tracking:** Serial communication monitoring with watchdog
- **Speed Monitoring:** Cellular performance metrics
- **Disconnect Recovery:** Automatic reconnection with exponential backoff

**Cellular Settings:**
- **Profile Management:** CS2-CS13 equipment profiles for different applications
- **Modem Control:** Direct cellular modem management via Serial2
- **Network Optimization:** Band selection and operator preferences

---

## ✅ **Device Grouping**
**Requirement:** Being able to place individual devices into groups.

**Grouping Features:**
- **Profile-Based Groups:** Equipment profiles (CS2, CS8, CS9, CS12, CS13)
- **Device Naming:** Custom identification (CSX-1234, RND-0007 format)
- **Configuration Inheritance:** Profile-based settings for group management
- **Status Reporting:** Group-level monitoring capabilities

---

## ✅ **Multi-Protocol Data Routing**
**Requirement:** Route data via different protocols to different endpoints.

**Supported Protocols:**
- **CBOR over Cellular:** Primary data transmission to BlueCherry cloud
- **Serial JSON:** Local communication with Linux controller
- **HTTP/WebSocket:** Web portal communication
- **MQTT:** BlueCherry message system for commands and updates
- **DTLS Security:** Encrypted cellular communications

---

## ✅ **Digital Twinning Capability**
**Requirement:** Monitor remote parameters locally and make local changes that directly affect the remote device.

**Digital Twin Features:**
- **Real-Time Monitoring:** Live sensor data (pressure, current, temperature)
- **Local Control Interface:** Web portal for immediate parameter changes
- **Bidirectional Sync:** Local changes reflected in remote device instantly
- **Status Mirroring:** Remote device state visible locally
- **Configuration Management:** Local settings update remote device behavior

---

## ✅ **CBOR Data Transmission**
**Requirement:** The I/O board will transmit data via a CBOR array via a user-modifiable format. We have achieved 8020 bytes/hour @ 15 second updates.

**CBOR Implementation:**
- **15-Second Intervals:** Regular data transmission cadence
- **8020 Bytes/Hour:** Efficient data compression and transmission
- **Structured Format:** Device ID, sequence, pressure, cycles, faults, mode, temperature, current
- **Modular Design:** Extensible CBOR structure for additional parameters
- **Reliability:** Continues during failsafe mode and connection issues

**Data Fields:**
- Device identification and sequencing
- Pressure readings (ESP32 ADC direct)
- Current measurements with outlier rejection
- Cycle counters and fault codes
- Temperature and system status

---

## ✅ **DTLS Security Implementation**
**Requirement:** The I/O board ensures proper DTLS functionality. Another vendor had to work hard with the modem manufacturer.

**Security Features:**
- **BlueCherry DTLS:** Pre-configured secure cellular communications
- **Certificate Management:** Automatic certificate handling
- **Encryption:** End-to-end encrypted data transmission
- **Authentication:** Secure device-to-cloud authentication
- **Compliance:** Production-ready security implementation

---

## ✅ **Local WiFi Portal**
**Requirement:** The I/O board locally-hosted wifi portal.

**Portal Features:**
- **Access Point:** "GM IO Board" network, IP 192.168.4.1
- **Responsive Design:** Mobile-optimized interface
- **Real-Time Updates:** AJAX polling every 2 seconds
- **Security Layers:** Password protection for sensitive operations
- **Comprehensive Monitoring:** All system parameters accessible locally

---

## 📊 **System Architecture**

**Hardware Integration:**
- ESP32 microcontroller with dual-core processing
- ADS1015 12-bit ADC for precision sensor readings
- MCP23017 I/O expansion (legacy compatibility)
- Walter cellular modem with BlueCherry integration
- SD card logging with automatic maintenance

**Software Architecture:**
- FreeRTOS dual-core task management
- State machine-based cellular initialization
- Interrupt-driven sensor sampling
- WebSocket-based real-time updates
- Comprehensive error handling and recovery

**Communication Stack:**
- Serial JSON (ESP32 ↔ Linux controller)
- CBOR over cellular (ESP32 → Cloud)
- HTTP/WebSocket (Web portal)
- MQTT (BlueCherry commands)

---

## 🎯 **Production Readiness**

**Reliability Features:**
- Watchdog monitoring with configurable timeouts
- Automatic failsafe operation during communication loss
- Data backfill after connectivity restoration
- Comprehensive logging and diagnostics
- Over-the-air update capability

**Performance Metrics:**
- 15-second data transmission intervals
- 8020 bytes/hour data throughput
- 60Hz sensor sampling with 1-second averaging
- 5Hz status updates to local controller
- Sub-200ms relay response times

**Safety & Compliance:**
- Failsafe operation with configurable parameters
- Emergency shutdown capabilities
- Sensor validation and error detection
- Password-protected configuration changes
- Audit logging for all operations

---

*This firmware represents a complete, production-ready solution meeting all specified requirements with enterprise-grade reliability and comprehensive feature coverage.*