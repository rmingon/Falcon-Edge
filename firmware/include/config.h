#ifndef CONFIG_H
#define CONFIG_H

// I2C Configuration
#define I2C_MASTER_SCL_IO 22 // GPIO22 for I2C clock
#define I2C_MASTER_SDA_IO 21 // GPIO21 for I2C data
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000 // 400kHz
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

// SPI Configuration for RFM95 LoRa
#define SPI_MISO_PIN 12
#define SPI_MOSI_PIN 13
#define SPI_SCK_PIN 14
#define SPI_SS_PIN 15
#define LORA_RST_PIN 16
#define LORA_DIO0_PIN 17
#define LORA_DIO1_PIN 18

// UART Configuration for GPS
#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN 5
#define GPS_RX_PIN 4
#define GPS_BAUD_RATE 9600
#define GPS_RX_BUF_SIZE 1024

// Sensor I2C Addresses
#define BMP280_I2C_ADDR 0x77 // or 0x77
#define HMC5883L_I2C_ADDR 0x1E
#define MPU6050_I2C_ADDR 0x68 // or 0x69

// PID Configuration - Altitude
#define ALT_PID_KP 2.0f
#define ALT_PID_KI 0.5f
#define ALT_PID_KD 1.0f
#define ALT_PID_MAX_OUTPUT 100.0f
#define ALT_PID_MIN_OUTPUT -100.0f
#define ALT_PID_MAX_INTEGRAL 50.0f

// PID Configuration - Roll
#define ROLL_PID_KP 1.5f
#define ROLL_PID_KI 0.3f
#define ROLL_PID_KD 0.8f
#define ROLL_PID_MAX_OUTPUT 100.0f
#define ROLL_PID_MIN_OUTPUT -100.0f
#define ROLL_PID_MAX_INTEGRAL 50.0f

// PID Configuration - Pitch
#define PITCH_PID_KP 1.5f
#define PITCH_PID_KI 0.3f
#define PITCH_PID_KD 0.8f
#define PITCH_PID_MAX_OUTPUT 100.0f
#define PITCH_PID_MIN_OUTPUT -100.0f
#define PITCH_PID_MAX_INTEGRAL 50.0f

// PID Configuration - Yaw
#define YAW_PID_KP 2.0f
#define YAW_PID_KI 0.1f
#define YAW_PID_KD 1.2f
#define YAW_PID_MAX_OUTPUT 100.0f
#define YAW_PID_MIN_OUTPUT -100.0f
#define YAW_PID_MAX_INTEGRAL 30.0f

// WiFi Long Range Configuration
#define WIFI_ENABLED true                // Set to false to disable WiFi
#define WIFI_DEFAULT_SSID "DroneGCS"     // Change to your network
#define WIFI_DEFAULT_PASSWORD "drone123" // Change to your password

// FreeRTOS Task Priorities
#define SENSOR_TASK_PRIORITY 5
#define CONTROL_TASK_PRIORITY 6
#define GPS_TASK_PRIORITY 3
#define LORA_TASK_PRIORITY 2
#define WIFI_TASK_PRIORITY 2

// FreeRTOS Task Stack Sizes
#define SENSOR_TASK_STACK_SIZE 4096
#define CONTROL_TASK_STACK_SIZE 4096
#define GPS_TASK_STACK_SIZE 3072
#define LORA_TASK_STACK_SIZE 3072
#define WIFI_TASK_STACK_SIZE 4096

// Control Loop Rates (Hz)
#define SENSOR_SAMPLE_RATE_HZ 100
#define CONTROL_LOOP_RATE_HZ 100
#define GPS_READ_RATE_HZ 10
#define LORA_TX_RATE_HZ 1
#define WIFI_TX_RATE_HZ 5

// Complementary Filter
#define ALPHA_GYRO 0.98f
#define ALPHA_ACCEL 0.02f

#endif // CONFIG_H
