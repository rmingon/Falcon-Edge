#ifndef HMC5883L_H
#define HMC5883L_H

#include "driver/i2c.h"
#include "esp_err.h"
#include "mpu6050.h"

// HMC5883L Registers
#define HMC5883L_REG_CONFIG_A       0x00
#define HMC5883L_REG_CONFIG_B       0x01
#define HMC5883L_REG_MODE           0x02
#define HMC5883L_REG_DATA_X_MSB     0x03
#define HMC5883L_REG_DATA_X_LSB     0x04
#define HMC5883L_REG_DATA_Z_MSB     0x05
#define HMC5883L_REG_DATA_Z_LSB     0x06
#define HMC5883L_REG_DATA_Y_MSB     0x07
#define HMC5883L_REG_DATA_Y_LSB     0x08
#define HMC5883L_REG_STATUS         0x09
#define HMC5883L_REG_ID_A           0x0A
#define HMC5883L_REG_ID_B           0x0B
#define HMC5883L_REG_ID_C           0x0C

// HMC5883L Modes
#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    float scale;
} hmc5883l_t;

/**
 * @brief Initialize HMC5883L magnetometer
 */
esp_err_t hmc5883l_init(hmc5883l_t *dev, i2c_port_t i2c_port, uint8_t i2c_addr);

/**
 * @brief Read magnetometer data
 *
 * @param dev HMC5883L device
 * @param mag Magnetic field vector in uT (micro-Tesla)
 * @return ESP_OK on success
 */
esp_err_t hmc5883l_read_data(hmc5883l_t *dev, vector3_t *mag);

/**
 * @brief Calculate heading (yaw) from magnetometer data
 *
 * @param mag Magnetic field vector
 * @return Heading in degrees (0-360)
 */
float hmc5883l_get_heading(vector3_t *mag);

#endif // HMC5883L_H
