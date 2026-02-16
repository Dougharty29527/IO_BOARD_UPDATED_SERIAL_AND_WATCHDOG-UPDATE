# Linux Device Setup Guide for Walter IO Board System

## Overview

This guide provides comprehensive instructions for setting up a Linux device (Raspberry Pi or similar) to work with the Walter IO Board ESP32 firmware. The setup includes serial communication, PPP internet connectivity, and remote management capabilities.

## System Requirements

- **Hardware**: Raspberry Pi 3, 4, or 5 (or similar Linux SBC with GPIO serial port)
- **OS**: Debian-based Linux distribution (Raspberry Pi OS, Ubuntu, etc.)
- **Access**: Root/administrator access to the Linux device
- **Network**: Internet connection for initial setup and updates

## 1. Prepare the Linux Device Hardware Serial Port

### Raspberry Pi Serial Port Configuration

By default, Raspberry Pi disables the hardware serial port and uses UART-Lite for Bluetooth. We need to enable the full hardware UART for reliable serial communication with the ESP32.

1. **Enable Serial Hardware**:
   ```bash
   sudo raspi-config
   ```
   Navigate to:
   - `3 Interface Options`
   - `I6 Serial Port`
   - `Would you like a login shell to be accessible over serial?` → **No**
   - `Would you like the serial port hardware to be enabled?` → **Yes**

2. **Disable Bluetooth** (if present):
   Add these lines to `/boot/firmware/config.txt`:
   ```bash
   dtoverlay=disable-bt
   dtoverlay=pi3-disable-bt
   ```

3. **Reboot the system**:
   ```bash
   sudo reboot
   ```

4. **Verify Serial Port**:
   ```bash
   ls -la /dev/serial*
   ```
   You should see `/dev/serial0` and `/dev/serial1` (or similar).

## 2. Install Required Software

Update your package list and install the necessary software:

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y ppp picocom git python3 python3-pip
```

### Additional Python Dependencies

If you're running the Python control panel:

```bash
pip3 install kivy pyyaml requests
```

## 3. Configure PPP Connection

### Install Configuration Files

Copy the PPP configuration files from the project repository:

```bash
# Create necessary directories
sudo mkdir -p /etc/chatscripts
sudo mkdir -p /etc/ppp/peers

# Copy configuration files (adjust paths as needed)
sudo cp etc/chatscripts/walter /etc/chatscripts/walter
sudo cp etc/chatscripts/walter-disconnect /etc/chatscripts/walter-disconnect
sudo cp etc/ppp/peers/walter /etc/ppp/peers/walter
```

### PPP Configuration Files

**`/etc/ppp/peers/walter`** (example configuration):
```
# Example PPP peer configuration for Walter modem
/dev/serial0          # Serial device
115200                # Baud rate
noauth               # No authentication required
defaultroute         # Use as default route
persist              # Reconnect on disconnect
holdoff 10           # Wait 10 seconds before reconnect
maxfail 0            # Retry indefinitely
noipdefault          # Don't use default IP
ipcp-accept-local    # Accept local IP from peer
ipcp-accept-remote   # Accept remote IP from peer
lcp-echo-interval 30 # Send LCP echo every 30 seconds
lcp-echo-failure 4   # Fail after 4 missed echoes
connect "/usr/sbin/chat -v -f /etc/chatscripts/walter"
disconnect "/usr/sbin/chat -v -f /etc/chatscripts/walter-disconnect"
```

**`/etc/chatscripts/walter`** (connection script):
```
# Chat script for Walter modem connection
ABORT "NO CARRIER"
ABORT "ERROR"
ABORT "NO DIALTONE"
ABORT "BUSY"
ABORT "NO ANSWER"
"" "AT"
OK "AT+CGDCONT=1,\"IP\",\"your_apn_here\""
OK "ATD*99#"
CONNECT ""
```

**`/etc/chatscripts/walter-disconnect`** (disconnection script):
```
# Chat script for Walter modem disconnection
"" "\K"
"" "+++ATH"
OK "ATH"
```

## 4. Set Up Remote Management (BlueCherry.io)

### Install BlueCherry Client

1. **Install required software**:
   ```bash
   sudo apt install -y ttyd
   ```

2. **Install BlueCherry client**:
   ```bash
   # Determine your architecture
   uname -m

   # For ARM32 (Raspberry Pi 3/4):
   sudo cp usr/sbin/bluecherry-client /usr/sbin/bluecherry-client

   # For ARM64 (Raspberry Pi 4/5):
   sudo cp usr/sbin/bluecherry-client_arm64 /usr/sbin/bluecherry-client
   ```

3. **Copy configuration files**:
   ```bash
   sudo mkdir -p /etc/bluecherry
   sudo mkdir -p /etc/ttyd

   sudo cp etc/bluecherry/config.yaml /etc/bluecherry/config.yaml
   sudo cp etc/ttyd/index.html /etc/ttyd/index.html
   ```

### Configure BlueCherry

1. **Get your BlueCherry Type ID**:
   - Register at BlueCherry.io
   - Request a Type ID from your BlueCherry representative

2. **Update configuration**:
   Edit `/etc/bluecherry/config.yaml`:
   ```yaml
   type_id: "your_bluecherry_type_id_here"
   server: "api.bluecherry.io"
   interval: 30
   ```

## 5. Set Up Python Control Panel

### Clone and Install

```bash
# Clone the control panel repository
git clone https://github.com/your-repo/vst_gm_control_panel.git
cd vst_gm_control_panel

# Install Python dependencies
pip3 install -r requirements.txt
```

### Configure the Control Panel

1. **Update device configuration** in the Python code
2. **Configure serial port** (typically `/dev/serial0`)
3. **Set up device identification** and credentials

## 6. System Services Configuration

### Create Systemd Services

**PPP Auto-connect service** (`/etc/systemd/system/ppp-walter.service`):
```ini
[Unit]
Description=Walter PPP Connection
After=network.target
Wants=network.target

[Service]
Type=forking
ExecStart=/usr/sbin/pppd call walter
ExecStop=/usr/sbin/pppd call walter-disconnect
Restart=always
RestartSec=30
User=root

[Install]
WantedBy=multi-user.target
```

**Python Control Panel service** (`/etc/systemd/system/walter-control.service`):
```ini
[Unit]
Description=Walter IO Board Control Panel
After=network.target ppp-walter.service
Requires=ppp-walter.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/vst_gm_control_panel
ExecStart=/usr/bin/python3 main.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**BlueCherry service** (`/etc/systemd/system/bluecherry.service`):
```ini
[Unit]
Description=BlueCherry Remote Management
After=network.target

[Service]
Type=simple
ExecStart=/usr/sbin/bluecherry-client
Restart=always
RestartSec=60
User=root

[Install]
WantedBy=multi-user.target
```

### Enable and Start Services

```bash
# Reload systemd
sudo systemctl daemon-reload

# Enable services
sudo systemctl enable ppp-walter
sudo systemctl enable walter-control
sudo systemctl enable bluecherry

# Start services
sudo systemctl start ppp-walter
sudo systemctl start walter-control
sudo systemctl start bluecherry
```

## 7. Testing and Verification

### Test Serial Communication

1. **Check serial port**:
   ```bash
   dmesg | grep serial
   ls -la /dev/serial*
   ```

2. **Test with picocom**:
   ```bash
   picocom -b 115200 /dev/serial0
   ```

### Test PPP Connection

1. **Manual PPP connection**:
   ```bash
   sudo pppd call walter nodetach debug
   ```

2. **Check PPP status**:
   ```bash
   ip route show
   ping -c 4 8.8.8.8
   ```

### Test Remote Access

1. **Check BlueCherry connection**:
   ```bash
   sudo systemctl status bluecherry
   journalctl -u bluecherry -f
   ```

2. **Test web terminal**:
   ```bash
   # Access via BlueCherry platform
   # Should show terminal interface
   ```

## 8. Monitoring and Maintenance

### Log Monitoring

```bash
# PPP logs
journalctl -u ppp-walter -f

# Python control panel logs
journalctl -u walter-control -f

# BlueCherry logs
journalctl -u bluecherry -f

# Serial communication logs
journalctl -f | grep chat
```

### System Health Checks

```bash
# Check all services
sudo systemctl status ppp-walter walter-control bluecherry

# Network connectivity
ip addr show
ip route show

# Serial port status
ls -la /dev/serial*
dmesg | grep -i serial
```

## 9. Troubleshooting

### Common Issues

**Serial Port Not Available**:
```bash
# Check if serial port is enabled
raspi-config nonint get_serial
# Should return 0 for enabled

# Check device permissions
ls -la /dev/serial0
sudo usermod -a -G dialout $USER
```

**PPP Connection Fails**:
```bash
# Check chat scripts
sudo chat -v -f /etc/chatscripts/walter

# Debug PPP
sudo pppd call walter debug nodetach
```

**BlueCherry Connection Issues**:
```bash
# Check configuration
cat /etc/bluecherry/config.yaml

# Test client manually
sudo /usr/sbin/bluecherry-client --test
```

### Emergency Access

If remote access fails, you can still access the device via:
- Direct HDMI/keyboard connection
- Serial console: `picocom -b 115200 /dev/serial0`
- SSH if local network access is available

## 10. Security Considerations

- **Change default passwords** in all services
- **Use strong authentication** for BlueCherry access
- **Regularly update** system packages: `sudo apt update && sudo apt upgrade`
- **Monitor logs** for unauthorized access attempts
- **Restrict physical access** to the device

## 11. Backup and Recovery

### Configuration Backup

```bash
# Backup important configurations
tar -czf ~/walter_config_backup.tar.gz \
  /etc/ppp/peers/walter \
  /etc/chatscripts/ \
  /etc/bluecherry/ \
  /etc/ttyd/ \
  ~/vst_gm_control_panel/
```

### Firmware Updates

```bash
# Update ESP32 firmware via serial
# Use the Arduino IDE or platformio to flash new firmware

# Update Python control panel
cd ~/vst_gm_control_panel
git pull
pip3 install -r requirements.txt --upgrade
sudo systemctl restart walter-control
```

---

**Document Version**: 1.0
**Last Updated**: February 16, 2026
**Compatible Firmware**: Walter IO Board Rev 10.25+