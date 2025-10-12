# Falcon-Edge Flight Controller Firmware

FreeRTOS-based ESP32 firmware for a drone flight controller with altitude and attitude control.

## Hardware Configuration

### ESP32 Pinout

#### I2C Bus (Sensors)
- **SDA**: GPIO 21
- **SCL**: GPIO 22

Connected sensors:
- **BMP280** - Barometric pressure/altitude sensor (I2C address: 0x76)
- **HMC5883L** - 3-axis magnetometer (I2C address: 0x1E)
- **MPU6050** - 6-axis IMU (accelerometer + gyroscope) (I2C address: 0x68)

#### SPI Bus (LoRa)
- **MISO**: GPIO 12
- **MOSI**: GPIO 13
- **SCK**: GPIO 14
- **SS/CS**: GPIO 15
- **RST**: GPIO 16
- **DIO0**: GPIO 17 (interrupt line)
- **DIO1**: GPIO 18 (interrupt line)

Connected device:
- **RFM95** - LoRa transceiver module (915 MHz)

#### UART (GPS)
- **TX**: GPIO 4
- **RX**: GPIO 5

Connected device:
- **ATGM336H** - GPS module (9600 baud, NMEA protocol)

## Features

### Sensors
- **MPU6050**: 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
- **HMC5883L**: 3-axis magnetometer for heading measurement
- **BMP280**: Barometric pressure sensor for altitude measurement
- **ATGM336H**: GPS module for position tracking

### Control System
- **PID Controllers**:
  - Altitude control (using BMP280 barometer)
  - Roll control (using IMU data)
  - Pitch control (using IMU data)
  - Yaw control (using magnetometer + gyro)

- **Sensor Fusion**:
  - Complementary filter for roll/pitch (combines accelerometer and gyroscope)
  - Magnetometer-based yaw with tilt compensation
  - Gyroscope integration with magnetometer correction

### Communication
- **LoRa telemetry**: Sends real-time flight data (attitude, altitude, GPS position)
- **Command reception**: Receives commands via LoRa (arm, disarm, set setpoints)

### FreeRTOS Tasks
1. **Sensor Task** (100 Hz) - Reads all sensors
2. **Control Task** (100 Hz) - Updates attitude estimation and PID controllers
3. **GPS Task** (10 Hz) - Processes GPS data
4. **LoRa Task** (1 Hz) - Sends telemetry
5. **Command Task** - Receives and processes LoRa commands

## Building and Flashing

### Prerequisites
- ESP-IDF v4.4 or later
- PlatformIO (optional)

### Using PlatformIO
```bash
cd firmware
pio run -t upload
pio device monitor
```

### Using ESP-IDF
```bash
cd firmware
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Configuration

All configuration parameters are in [include/config.h](include/config.h):

- **Sensor pins and I2C settings**
- **LoRa frequency and SPI pins**
- **GPS UART settings**
- **PID tuning parameters** (Kp, Ki, Kd)
- **Task priorities and stack sizes**
- **Sensor sample rates**

### PID Tuning

Default PID parameters are set in [config.h](include/config.h). You can tune them for your specific setup:

```c
// Altitude PID
#define ALT_PID_KP    2.0f
#define ALT_PID_KI    0.5f
#define ALT_PID_KD    1.0f

// Roll PID
#define ROLL_PID_KP   1.5f
#define ROLL_PID_KI   0.3f
#define ROLL_PID_KD   0.8f

// Pitch PID
#define PITCH_PID_KP  1.5f
#define PITCH_PID_KI  0.3f
#define PITCH_PID_KD  0.8f

// Yaw PID
#define YAW_PID_KP    2.0f
#define YAW_PID_KI    0.1f
#define YAW_PID_KD    1.2f
```

## LoRa Commands

The system accepts the following commands via LoRa:

### Arm Command
Send single byte: `'A'` (0x41)
- Arms the flight controller and enables control outputs

### Disarm Command
Send single byte: `'D'` (0x44)
- Disarms the flight controller and zeros all outputs

### Set Setpoints Command
Send 17 bytes: `'S'` + 4 floats (altitude, roll, pitch, yaw)
- Format: `['S'][altitude][roll][pitch][yaw]`
- Each float is 4 bytes (little-endian)
- Example: Set altitude to 10m, level attitude (0° roll/pitch), 90° heading

## Telemetry Data

Telemetry is sent via LoRa every second containing:
- Roll (degrees)
- Pitch (degrees)
- Yaw (degrees)
- Altitude (meters)
- GPS latitude (degrees)
- GPS longitude (degrees)
- Number of GPS satellites
- Armed status (1 byte boolean)

Total packet size: ~30 bytes

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                    ESP32 Main                        │
│  ┌────────────────────────────────────────────────┐ │
│  │         FreeRTOS Task Scheduler                │ │
│  └────────────────────────────────────────────────┘ │
│                                                       │
│  ┌──────────┐  ┌───────────┐  ┌─────────┐          │
│  │  Sensor  │  │  Control  │  │  LoRa   │          │
│  │   Task   │→│   Task    │→│  Task   │          │
│  │ (100 Hz) │  │ (100 Hz)  │  │ (1 Hz)  │          │
│  └──────────┘  └───────────┘  └─────────┘          │
│       ↓              ↓                                │
│  ┌──────────────────────────────────────────────┐   │
│  │         Flight Controller Module             │   │
│  │  • Sensor Fusion (Complementary Filter)     │   │
│  │  • 4x PID Controllers                       │   │
│  │  • Attitude Estimation                      │   │
│  └──────────────────────────────────────────────┘   │
│       ↓                                               │
│  ┌──────────┬──────────┬──────────┐                 │
│  │  MPU6050 │ HMC5883L │  BMP280  │  (I2C)         │
│  └──────────┴──────────┴──────────┘                 │
│  ┌──────────┐  ┌─────────────┐                      │
│  │ ATGM336  │  │   RFM95     │                      │
│  │   GPS    │  │   LoRa      │                      │
│  └──────────┘  └─────────────┘                      │
│    (UART)          (SPI)                             │
└─────────────────────────────────────────────────────┘
```

## Safety Features

- **Calibration check**: System must be calibrated before arming
- **Armed state**: Control outputs are zero when disarmed
- **Mutex protection**: Thread-safe access to shared data
- **PID anti-windup**: Prevents integral term from growing unbounded
- **Output limiting**: All PID outputs are clamped to safe ranges

## Serial Monitor Output

When running, you'll see output like:
```
I (1234) Main: Attitude: R=2.3° P=-1.5° Y=45.2° | Alt=123.45m | GPS: 37.774929,-122.419418 (8 sats)
I (1234) Main: Control outputs: Alt=15.2 Roll=-3.4 Pitch=2.1 Yaw=0.8
```

## License

See the LICENSE file in the repository root.

## Troubleshooting

### Sensor initialization fails
- Check I2C connections and pull-up resistors
- Verify sensor I2C addresses (some modules use alternate addresses)
- Check power supply (3.3V for all sensors)

### GPS not getting fix
- Ensure antenna has clear view of sky
- GPS may take several minutes for first fix
- Check UART baud rate (9600 for ATGM336)

### LoRa not working
- Verify antenna is connected
- Check SPI connections
- Ensure frequency matches your region (915 MHz US, 868 MHz EU, 433 MHz Asia)
- Check DIO0/DIO1 interrupt pins are connected

### Attitude estimation drifts
- Recalibrate IMU (keep device stationary during calibration)
- Adjust complementary filter alpha values
- Check for magnetic interference affecting magnetometer

## Future Improvements

- [ ] Kalman filter for better sensor fusion
- [ ] Motor/ESC control outputs
- [ ] Battery voltage monitoring
- [ ] Failsafe modes
- [ ] SD card logging
- [ ] Web interface for configuration
- [ ] OTA firmware updates
