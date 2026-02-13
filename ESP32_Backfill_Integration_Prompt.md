# ESP32 IO Board Firmware Update - Backfill Integration

## Context
Your Python program currently communicates with ESP32 IO Board firmware versions 10.11 and 10.13. The device has been updated with new backfill functionality that ensures no sensor data is lost during PPP passthrough sessions.

## Current Serial Protocol (Unchanged)
- **Baud Rate:** 115200, 8N1, no flow control
- **Format:** JSON messages, one per line, terminated with `\n`
- **Direction ESP32 → Python:**
  - Fast sensor data (5Hz): `{"pressure":X,"current":X,"overfill":X,"sdcard":"OK","relayMode":X,"failsafe":0,"shutdown":0}`
  - Cellular status (10s): `{"passthrough":0,"lte":1,"rsrp":"-85.5","rsrq":"-10.2","operator":"T-Mobile","band":"B2",...}`
  - Datetime: `{"datetime":"2026-02-13 10:30:15"}`
- **Direction Python → ESP32:**
  - Data payload (15s): `{"type":"data","gmid":"CSX-1234","press":-14.22,"mode":0,"current":0.07,"fault":0,"cycles":484}`
  - Immediate mode: `{"type":"data","mode":1}`
  - Commands: `{"command":"passthrough","timeout":60}`

## NEW: Data Backfill Functionality

### When It Happens
- Automatically sent by ESP32 after exiting passthrough mode
- Occurs during device boot when backfill flag is detected in EEPROM
- Sends sensor data collected during PPP session that couldn't be sent to cloud

### Message Format
```json
{"backfill":[
  {"pressure":-14.22,"current":0.07,"mode":0,"fault":0,"cycles":484},
  {"pressure":-14.25,"current":0.06,"mode":1,"fault":0,"cycles":485}
]}
```

### Required Python Changes

1. **Add backfill message parser** in your serial receive loop:
   ```python
   if "backfill" in data:
       # Process backfill data
       backfill_entries = data["backfill"]
       for entry in backfill_entries:
           # Add timestamp (current time or interpolate)
           # Send to cloud/database
           # Log the backfill event
   ```

2. **Handle the message appropriately:**
   - Add timestamps to each entry (server-side timing)
   - Forward to cloud platform (BlueCherry)
   - Update local database with the backfilled data
   - Log successful backfill completion

3. **No response needed** - ESP32 doesn't expect acknowledgment

## Integration Checklist

- [ ] Parse incoming `{"backfill": [...]}` messages
- [ ] Add timestamps to backfill entries
- [ ] Forward backfill data to cloud
- [ ] Update local database
- [ ] Log backfill events
- [ ] Test with actual passthrough cycle
- [ ] Verify no duplicate data in cloud

## Testing

1. Start passthrough mode on ESP32
2. Let it run for a few minutes (data collects on SD card)
3. Exit passthrough (timeout or +++STOPPPP)
4. ESP32 reboots and sends backfill message
5. Verify Python receives and processes the data

## Files to Reference

- `SERIAL_PROTOCOL_GUIDE.md` - Complete protocol specification
- `README.md` - High-level overview of backfill functionality
- Your existing `modem.py` or serial handling code

The backfill ensures complete data continuity during remote access sessions while maintaining all existing functionality.