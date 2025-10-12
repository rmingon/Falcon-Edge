#include "bmp280.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "BMP280";

#define SEA_LEVEL_PRESSURE 101325.0f  // Pa

static esp_err_t bmp280_write_reg(bmp280_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr,
                                      write_buf, sizeof(write_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t bmp280_read_reg(bmp280_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(dev->i2c_port, dev->i2c_addr,
                                        &reg, 1, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t bmp280_read_calibration(bmp280_t *dev)
{
    uint8_t calib_data[24];
    esp_err_t ret = bmp280_read_reg(dev, BMP280_REG_CALIB, calib_data, 24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ret;
    }

    dev->calib.dig_T1 = (calib_data[1] << 8) | calib_data[0];
    dev->calib.dig_T2 = (calib_data[3] << 8) | calib_data[2];
    dev->calib.dig_T3 = (calib_data[5] << 8) | calib_data[4];
    dev->calib.dig_P1 = (calib_data[7] << 8) | calib_data[6];
    dev->calib.dig_P2 = (calib_data[9] << 8) | calib_data[8];
    dev->calib.dig_P3 = (calib_data[11] << 8) | calib_data[10];
    dev->calib.dig_P4 = (calib_data[13] << 8) | calib_data[12];
    dev->calib.dig_P5 = (calib_data[15] << 8) | calib_data[14];
    dev->calib.dig_P6 = (calib_data[17] << 8) | calib_data[16];
    dev->calib.dig_P7 = (calib_data[19] << 8) | calib_data[18];
    dev->calib.dig_P8 = (calib_data[21] << 8) | calib_data[20];
    dev->calib.dig_P9 = (calib_data[23] << 8) | calib_data[22];

    return ESP_OK;
}

esp_err_t bmp280_init(bmp280_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr)
{
    dev->i2c_port = i2c_port;
    dev->i2c_addr = i2c_addr;

    // Check chip ID
    uint8_t chip_id;
    esp_err_t ret = bmp280_read_reg(dev, BMP280_REG_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read chip ID");
        return ret;
    }

    if (chip_id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "Invalid chip ID: 0x%02X (expected 0x%02X)", chip_id, BMP280_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "BMP280 detected, chip ID: 0x%02X", chip_id);

    // Reset device
    ret = bmp280_write_reg(dev, BMP280_REG_RESET, 0xB6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset device");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read calibration data
    ret = bmp280_read_calibration(dev);
    if (ret != ESP_OK) {
        return ret;
    }

    // Configure: normal mode, temp oversampling x2, pressure oversampling x16
    ret = bmp280_write_reg(dev, BMP280_REG_CTRL_MEAS, 0x57);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ctrl_meas");
        return ret;
    }

    // Configure: standby time 0.5ms, filter coefficient 16
    ret = bmp280_write_reg(dev, BMP280_REG_CONFIG, 0x10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure config");
        return ret;
    }

    ESP_LOGI(TAG, "BMP280 initialized successfully");
    return ESP_OK;
}

esp_err_t bmp280_read_temperature(bmp280_t *dev, float *temperature)
{
    uint8_t data[3];
    esp_err_t ret = bmp280_read_reg(dev, BMP280_REG_TEMP_MSB, data, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    int32_t adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    // Temperature compensation (from BMP280 datasheet)
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1))) *
                    ((int32_t)dev->calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dev->calib.dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dev->calib.dig_T1))) >> 12) *
                    ((int32_t)dev->calib.dig_T3)) >> 14;

    dev->t_fine = var1 + var2;
    *temperature = (dev->t_fine * 5 + 128) / 25600.0f;

    return ESP_OK;
}

esp_err_t bmp280_read_pressure(bmp280_t *dev, float *pressure)
{
    // Read temperature first to update t_fine
    float temp;
    esp_err_t ret = bmp280_read_temperature(dev, &temp);
    if (ret != ESP_OK) {
        return ret;
    }

    uint8_t data[3];
    ret = bmp280_read_reg(dev, BMP280_REG_PRESS_MSB, data, 3);
    if (ret != ESP_OK) {
        return ret;
    }

    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);

    // Pressure compensation (from BMP280 datasheet)
    int64_t var1 = ((int64_t)dev->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) +
           ((var1 * (int64_t)dev->calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calib.dig_P1) >> 33;

    if (var1 == 0) {
        return ESP_FAIL;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calib.dig_P7) << 4);

    *pressure = (float)p / 256.0f;

    return ESP_OK;
}

esp_err_t bmp280_read_altitude(bmp280_t *dev, float *altitude)
{
    float pressure;
    esp_err_t ret = bmp280_read_pressure(dev, &pressure);
    if (ret != ESP_OK) {
        return ret;
    }

    // Barometric formula
    *altitude = 44330.0f * (1.0f - powf(pressure / SEA_LEVEL_PRESSURE, 0.1903f));

    return ESP_OK;
}

esp_err_t bmp280_read_data(bmp280_t *dev, float *temperature, float *pressure, float *altitude)
{
    esp_err_t ret = bmp280_read_temperature(dev, temperature);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = bmp280_read_pressure(dev, pressure);
    if (ret != ESP_OK) {
        return ret;
    }

    *altitude = 44330.0f * (1.0f - powf(*pressure / SEA_LEVEL_PRESSURE, 0.1903f));

    return ESP_OK;
}
