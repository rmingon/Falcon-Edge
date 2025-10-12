#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c.h"
#include "esp_err.h"

// MPU6050 Registers
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_ACCEL_XOUT_L    0x3C
#define MPU6050_REG_ACCEL_YOUT_H    0x3D
#define MPU6050_REG_ACCEL_YOUT_L    0x3E
#define MPU6050_REG_ACCEL_ZOUT_H    0x3F
#define MPU6050_REG_ACCEL_ZOUT_L    0x40
#define MPU6050_REG_TEMP_OUT_H      0x41
#define MPU6050_REG_TEMP_OUT_L      0x42
#define MPU6050_REG_GYRO_XOUT_H     0x43
#define MPU6050_REG_GYRO_XOUT_L     0x44
#define MPU6050_REG_GYRO_YOUT_H     0x45
#define MPU6050_REG_GYRO_YOUT_L     0x46
#define MPU6050_REG_GYRO_ZOUT_H     0x47
#define MPU6050_REG_GYRO_ZOUT_L     0x48
#define MPU6050_REG_WHO_AM_I        0x75

// MPU6050 Scale factors
#define MPU6050_ACCEL_SCALE_2G      16384.0f
#define MPU6050_GYRO_SCALE_250      131.0f

typedef struct {
    float x;
    float y;
    float z;
} vector3_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    vector3_t accel_offset;
    vector3_t gyro_offset;
} mpu6050_t;

typedef struct {
    vector3_t accel;    // Acceleration in g
    vector3_t gyro;     // Angular velocity in deg/s
    float temperature;  // Temperature in degrees C
} mpu6050_data_t;

/**
 * @brief Initialize MPU6050 sensor
 */
esp_err_t mpu6050_init(mpu6050_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr);

/**
 * @brief Read accelerometer and gyroscope data
 */
esp_err_t mpu6050_read_data(mpu6050_t *dev, mpu6050_data_t *data);

/**
 * @brief Calibrate MPU6050 (device must be stationary)
 */
esp_err_t mpu6050_calibrate(mpu6050_t *dev);

#endif // MPU6050_H
