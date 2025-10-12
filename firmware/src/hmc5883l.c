#include "hmc5883l.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "HMC5883L";

static esp_err_t hmc5883l_write_reg(hmc5883l_t *dev, uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr,
                                      write_buf, sizeof(write_buf),
                                      I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t hmc5883l_read_reg(hmc5883l_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(dev->i2c_port, dev->i2c_addr,
                                        &reg, 1, data, len,
                                        I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t hmc5883l_init(hmc5883l_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr)
{
    dev->i2c_port = i2c_port;
    dev->i2c_addr = i2c_addr;
    dev->scale = 0.92f; // Default scale for ±1.3 Ga

    // Check identification registers
    uint8_t id[3];
    esp_err_t ret = hmc5883l_read_reg(dev, HMC5883L_REG_ID_A, id, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ID registers");
        return ret;
    }

    if (id[0] != 'H' || id[1] != '4' || id[2] != '3') {
        ESP_LOGE(TAG, "Invalid device ID: %c%c%c", id[0], id[1], id[2]);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "HMC5883L detected, ID: %c%c%c", id[0], id[1], id[2]);

    // Configure: 8 samples averaged, 15 Hz data output rate, normal measurement
    ret = hmc5883l_write_reg(dev, HMC5883L_REG_CONFIG_A, 0x70);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Config A");
        return ret;
    }

    // Configure: Gain = 1.3 Ga (±1.3 Ga range)
    ret = hmc5883l_write_reg(dev, HMC5883L_REG_CONFIG_B, 0x20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure Config B");
        return ret;
    }

    // Set continuous measurement mode
    ret = hmc5883l_write_reg(dev, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set measurement mode");
        return ret;
    }

    ESP_LOGI(TAG, "HMC5883L initialized successfully");
    return ESP_OK;
}

esp_err_t hmc5883l_read_data(hmc5883l_t *dev, vector3_t *mag)
{
    uint8_t raw_data[6];
    esp_err_t ret = hmc5883l_read_reg(dev, HMC5883L_REG_DATA_X_MSB, raw_data, 6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read magnetometer data");
        return ret;
    }

    // HMC5883L data order is X, Z, Y
    int16_t x_raw = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    int16_t z_raw = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    int16_t y_raw = (int16_t)((raw_data[4] << 8) | raw_data[5]);

    // Convert to micro-Tesla (uT)
    mag->x = (float)x_raw * dev->scale;
    mag->y = (float)y_raw * dev->scale;
    mag->z = (float)z_raw * dev->scale;

    return ESP_OK;
}

float hmc5883l_get_heading(vector3_t *mag)
{
    // Calculate heading (yaw) from magnetometer
    // Assumes the device is level (no roll/pitch compensation)
    float heading = atan2f(mag->y, mag->x);

    // Convert from radians to degrees
    heading = heading * 180.0f / M_PI;

    // Normalize to 0-360 degrees
    if (heading < 0) {
        heading += 360.0f;
    }

    return heading;
}
