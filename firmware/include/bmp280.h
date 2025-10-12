#ifndef BMP280_H
#define BMP280_H

#include "driver/i2c.h"
#include "esp_err.h"

// BMP280 Registers
#define BMP280_REG_TEMP_XLSB        0xFC
#define BMP280_REG_TEMP_LSB         0xFB
#define BMP280_REG_TEMP_MSB         0xFA
#define BMP280_REG_PRESS_XLSB       0xF9
#define BMP280_REG_PRESS_LSB        0xF8
#define BMP280_REG_PRESS_MSB        0xF7
#define BMP280_REG_CONFIG           0xF5
#define BMP280_REG_CTRL_MEAS        0xF4
#define BMP280_REG_STATUS           0xF3
#define BMP280_REG_RESET            0xE0
#define BMP280_REG_ID               0xD0
#define BMP280_REG_CALIB            0x88

// BMP280 Chip ID
#define BMP280_CHIP_ID              0x58

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    bmp280_calib_t calib;
    int32_t t_fine;
} bmp280_t;

/**
 * @brief Initialize BMP280 sensor
 */
esp_err_t bmp280_init(bmp280_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr);

/**
 * @brief Read temperature from BMP280
 *
 * @param dev BMP280 device
 * @param temperature Temperature in degrees Celsius
 * @return ESP_OK on success
 */
esp_err_t bmp280_read_temperature(bmp280_t *dev, float *temperature);

/**
 * @brief Read pressure from BMP280
 *
 * @param dev BMP280 device
 * @param pressure Pressure in Pa
 * @return ESP_OK on success
 */
esp_err_t bmp280_read_pressure(bmp280_t *dev, float *pressure);

/**
 * @brief Read altitude from BMP280
 *
 * @param dev BMP280 device
 * @param altitude Altitude in meters (relative to sea level)
 * @return ESP_OK on success
 */
esp_err_t bmp280_read_altitude(bmp280_t *dev, float *altitude);

/**
 * @brief Read both temperature and pressure
 */
esp_err_t bmp280_read_data(bmp280_t *dev, float *temperature, float *pressure, float *altitude);

#endif // BMP280_H
