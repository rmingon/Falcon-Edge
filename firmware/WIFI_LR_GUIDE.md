# WiFi Long Range Guide

## Overview

Your ESP32 firmware now supports **WiFi Long Range (LR) mode** alongside LoRa for redundant telemetry and configuration.

## Features Added

### 1. **WiFi Long Range Mode**
- Range: Up to **1 km** line-of-sight
- Frequency: 5 Hz telemetry updates
- Dual-band support with LoRa as backup

### 2. **UDP Telemetry**
- Sends flight data to ground station at `192.168.4.1:5000`
- Same packet format as LoRa telemetry
- Lower latency than LoRa

### 3. **Remote WiFi Configuration**
- Configure WiFi via LoRa commands
- No need to reflash firmware to change networks

## Configuration

### In [config.h](include/config.h)

```c
#define WIFI_ENABLED              true        // Enable/disable WiFi
#define WIFI_DEFAULT_SSID         "DroneGCS"  // Your WiFi network
#define WIFI_DEFAULT_PASSWORD     "drone123"  // Your WiFi password
```

### Ground Station IP

Change in [wifi_lr.c](src/wifi_lr.c:10):
```c
#define GROUND_STATION_IP       "192.168.4.1"  // Your ground station IP
```

## Usage

### Auto-Connect on Startup

If you set `WIFI_DEFAULT_SSID` and `WIFI_DEFAULT_PASSWORD` in config.h, the drone will automatically connect on boot.

### Connect via LoRa Command

Send command `'W'` via LoRa with this format:
```
Byte 0: 'W' (0x57)
Byte 1: SSID length (N)
Bytes 2 to N+1: SSID string
Bytes N+2 to end: Password string
```

**Example Python code to send WiFi config:**
```python
import struct

ssid = "MyDrone"
password = "password123"

packet = bytearray()
packet.append(ord('W'))           # Command
packet.append(len(ssid))          # SSID length
packet.extend(ssid.encode())      # SSID
packet.extend(password.encode())  # Password

# Send packet via your LoRa module
lora.send(packet)
```

## Ground Station Setup

### Simple UDP Receiver (Python)

```python
import socket
import struct

UDP_IP = "0.0.0.0"
UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for telemetry on {UDP_IP}:{UDP_PORT}")

while True:
    data, addr = sock.recvfrom(1024)

    if len(data) >= 29:
        # Unpack telemetry
        roll, pitch, yaw, alt, lat, lon, sats, armed = struct.unpack('<ffffffBB', data[:29])

        print(f"Drone at {addr[0]}:")
        print(f"  Attitude: R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}°")
        print(f"  Altitude: {alt:.2f}m")
        print(f"  GPS: {lat:.6f}, {lon:.6f} ({sats} sats)")
        print(f"  Armed: {bool(armed)}")
        print()
```

## Serial Monitor Output

When WiFi is working, you'll see:
```
I (12345) WiFi_LR: WiFi Long Range mode initialized
I (12456) WiFi_LR: Connecting to WiFi: DroneGCS
I (15234) WiFi_LR: Got IP: 192.168.4.2
I (15345) Main: WiFi task started
I (20000) Main: WiFi connected, RSSI: -45 dBm
```

## Troubleshooting

### WiFi won't connect
1. Check SSID and password in config.h
2. Ensure AP supports 802.11n or 802.11b/g
3. Check serial monitor for error messages
4. Verify ESP32 is in range (~1km max)

### No telemetry received
1. Check ground station IP in wifi_lr.c
2. Verify UDP port 5000 is not blocked by firewall
3. Ensure ground station is on same network
4. Check RSSI in serial monitor (should be > -80 dBm)

### High packet loss
- Move closer to AP (check RSSI)
- Reduce obstacles between drone and AP
- Lower `WIFI_TX_RATE_HZ` in config.h to reduce bandwidth

## Disable WiFi

To disable WiFi completely:

In [config.h](include/config.h):
```c
#define WIFI_ENABLED    false
```

This will remove WiFi task and save memory.

## Comparison: WiFi LR vs LoRa

| Feature | WiFi LR | LoRa |
|---------|---------|------|
| **Range** | ~1 km | 2-15 km |
| **Data Rate** | 5 Hz | 1 Hz |
| **Latency** | ~10 ms | ~100 ms |
| **Power** | Higher | Lower |
| **Best For** | Near-field ops | Long-range |

**Recommendation**: Use both! WiFi for low-latency control near base, LoRa for long-range missions.

## Advanced: Bidirectional Commands via WiFi

You can easily extend the WiFi module to receive commands (not just send telemetry):

1. Add TCP server in wifi_lr.c
2. Parse incoming command packets
3. Call flight controller functions

This would allow real-time parameter tuning over WiFi!
