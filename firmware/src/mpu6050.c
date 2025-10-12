#include "mpu6050.h"
#include "config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "MPU6050";

static esp_err_t mpu6050_write_reg(mpu6050_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr,
                                      write_buf, sizeof(write_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6050_read_reg(mpu6050_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(dev->i2c_port, dev->i2c_addr,
                                        &reg, 1, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t mpu6050_init(mpu6050_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr)
{
    dev->i2c_port = i2c_port;
    dev->i2c_addr = i2c_addr;
    memset(&dev->accel_offset, 0, sizeof(vector3_t));
    memset(&dev->gyro_offset, 0, sizeof(vector3_t));

    // Check WHO_AM_I register
    uint8_t who_am_i;
    esp_err_t ret = mpu6050_read_reg(dev, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register");
        return ret;
    }

    if (who_am_i != dev->i2c_addr) {
        ESP_LOGE(TAG, "Invalid WHO_AM_I: 0x%02X (expected 0x%02X)", who_am_i, dev->i2c_addr);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "MPU6050 detected, WHO_AM_I: 0x%02X", who_am_i);

    // Wake up device (clear sleep bit)
    ret = mpu6050_write_reg(dev, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up device");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set gyroscope range to ±250°/s
    ret = mpu6050_write_reg(dev, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }

    // Set accelerometer range to ±2g
    ret = mpu6050_write_reg(dev, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6050_read_data(mpu6050_t *dev, mpu6050_data_t *data)
{
    uint8_t raw_data[14];
    esp_err_t ret = mpu6050_read_reg(dev, MPU6050_REG_ACCEL_XOUT_H, raw_data, 14);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return ret;
    }

    // Parse accelerometer data
    int16_t accel_x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t accel_y_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t accel_z_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    // Parse temperature data
    int16_t temp_raw = (int16_t)((raw_data[6] << 8) | raw_data[7]);

    // Parse gyroscope data
    int16_t gyro_x_raw = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    int16_t gyro_y_raw = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    int16_t gyro_z_raw = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    // Convert to physical units
    data->accel.x = (float)accel_x_raw / MPU6050_ACCEL_SCALE_2G - dev->accel_offset.x;
    data->accel.y = (float)accel_y_raw / MPU6050_ACCEL_SCALE_2G - dev->accel_offset.y;
    data->accel.z = (float)accel_z_raw / MPU6050_ACCEL_SCALE_2G - dev->accel_offset.z;

    data->gyro.x = (float)gyro_x_raw / MPU6050_GYRO_SCALE_250 - dev->gyro_offset.x;
    data->gyro.y = (float)gyro_y_raw / MPU6050_GYRO_SCALE_250 - dev->gyro_offset.y;
    data->gyro.z = (float)gyro_z_raw / MPU6050_GYRO_SCALE_250 - dev->gyro_offset.z;

    data->temperature = (float)temp_raw / 340.0f + 36.53f;

    return ESP_OK;
}

esp_err_t mpu6050_calibrate(mpu6050_t *dev)
{
    ESP_LOGI(TAG, "Starting calibration... Keep device stationary!");

    const int samples = 100;
    vector3_t accel_sum = {0};
    vector3_t gyro_sum = {0};

    for (int i = 0; i < samples; i++) {
        mpu6050_data_t data;
        esp_err_t ret = mpu6050_read_data(dev, &data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read data during calibration");
            return ret;
        }

        accel_sum.x += data.accel.x;
        accel_sum.y += data.accel.y;
        accel_sum.z += data.accel.z;

        gyro_sum.x += data.gyro.x;
        gyro_sum.y += data.gyro.y;
        gyro_sum.z += data.gyro.z;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Calculate average offsets
    dev->accel_offset.x = accel_sum.x / samples;
    dev->accel_offset.y = accel_sum.y / samples;
    dev->accel_offset.z = (accel_sum.z / samples) - 1.0f; // Remove gravity

    dev->gyro_offset.x = gyro_sum.x / samples;
    dev->gyro_offset.y = gyro_sum.y / samples;
    dev->gyro_offset.z = gyro_sum.z / samples;

    ESP_LOGI(TAG, "Calibration complete");
    ESP_LOGI(TAG, "Accel offset: X=%.3f Y=%.3f Z=%.3f",
             dev->accel_offset.x, dev->accel_offset.y, dev->accel_offset.z);
    ESP_LOGI(TAG, "Gyro offset: X=%.3f Y=%.3f Z=%.3f",
             dev->gyro_offset.x, dev->gyro_offset.y, dev->gyro_offset.z);

    return ESP_OK;
}
