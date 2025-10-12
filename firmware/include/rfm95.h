#ifndef RFM95_H
#define RFM95_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

// RFM95 Registers
#define RFM95_REG_FIFO                  0x00
#define RFM95_REG_OP_MODE               0x01
#define RFM95_REG_FRF_MSB               0x06
#define RFM95_REG_FRF_MID               0x07
#define RFM95_REG_FRF_LSB               0x08
#define RFM95_REG_PA_CONFIG             0x09
#define RFM95_REG_FIFO_ADDR_PTR         0x0D
#define RFM95_REG_FIFO_TX_BASE_ADDR     0x0E
#define RFM95_REG_FIFO_RX_BASE_ADDR     0x0F
#define RFM95_REG_FIFO_RX_CURRENT_ADDR  0x10
#define RFM95_REG_IRQ_FLAGS             0x12
#define RFM95_REG_RX_NB_BYTES           0x13
#define RFM95_REG_PKT_RSSI_VALUE        0x1A
#define RFM95_REG_PKT_SNR_VALUE         0x1B
#define RFM95_REG_MODEM_CONFIG_1        0x1D
#define RFM95_REG_MODEM_CONFIG_2        0x1E
#define RFM95_REG_PREAMBLE_MSB          0x20
#define RFM95_REG_PREAMBLE_LSB          0x21
#define RFM95_REG_PAYLOAD_LENGTH        0x22
#define RFM95_REG_MODEM_CONFIG_3        0x26
#define RFM95_REG_RSSI_WIDEBAND         0x2C
#define RFM95_REG_DIO_MAPPING_1         0x40
#define RFM95_REG_VERSION               0x42
#define RFM95_REG_PA_DAC                0x4D

// Operating modes
#define RFM95_MODE_SLEEP                0x00
#define RFM95_MODE_STDBY                0x01
#define RFM95_MODE_TX                   0x03
#define RFM95_MODE_RX_CONTINUOUS        0x05
#define RFM95_MODE_RX_SINGLE            0x06

// LoRa mode
#define RFM95_LONG_RANGE_MODE           0x80

// IRQ Flags
#define RFM95_IRQ_TX_DONE               0x08
#define RFM95_IRQ_RX_DONE               0x40
#define RFM95_IRQ_CAD_DONE              0x04

// Frequency (Hz)
#define RFM95_FREQ_915MHZ               915000000
#define RFM95_FREQ_868MHZ               868000000
#define RFM95_FREQ_433MHZ               433000000

#define RFM95_MAX_PAYLOAD               255

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t cs_pin;
    gpio_num_t rst_pin;
    gpio_num_t dio0_pin;
    gpio_num_t dio1_pin;
    uint32_t frequency;
} rfm95_t;

/**
 * @brief Initialize RFM95 LoRa module
 *
 * @param dev RFM95 device structure
 * @param spi_host SPI host (SPI2_HOST or SPI3_HOST)
 * @param frequency Operating frequency in Hz
 * @return ESP_OK on success
 */
esp_err_t rfm95_init(rfm95_t *dev, spi_host_device_t spi_host, uint32_t frequency);

/**
 * @brief Set transmission power
 *
 * @param dev RFM95 device
 * @param power Power in dBm (5 to 23)
 * @return ESP_OK on success
 */
esp_err_t rfm95_set_tx_power(rfm95_t *dev, int8_t power);

/**
 * @brief Send data via LoRa
 *
 * @param dev RFM95 device
 * @param data Data buffer
 * @param len Data length (max 255 bytes)
 * @return ESP_OK on success
 */
esp_err_t rfm95_send(rfm95_t *dev, const uint8_t *data, uint8_t len);

/**
 * @brief Receive data via LoRa
 *
 * @param dev RFM95 device
 * @param data Buffer to store received data
 * @param len Maximum buffer length
 * @param timeout_ms Timeout in milliseconds
 * @return Number of bytes received, or -1 on error
 */
int rfm95_receive(rfm95_t *dev, uint8_t *data, uint8_t len, uint32_t timeout_ms);

/**
 * @brief Get RSSI of last received packet
 *
 * @param dev RFM95 device
 * @return RSSI in dBm
 */
int16_t rfm95_get_rssi(rfm95_t *dev);

/**
 * @brief Set LoRa spreading factor
 *
 * @param dev RFM95 device
 * @param sf Spreading factor (6-12)
 * @return ESP_OK on success
 */
esp_err_t rfm95_set_spreading_factor(rfm95_t *dev, uint8_t sf);

/**
 * @brief Set LoRa bandwidth
 *
 * @param dev RFM95 device
 * @param bw Bandwidth (0=7.8kHz, 1=10.4kHz, 2=15.6kHz, 3=20.8kHz,
 *                     4=31.25kHz, 5=41.7kHz, 6=62.5kHz, 7=125kHz, 8=250kHz, 9=500kHz)
 * @return ESP_OK on success
 */
esp_err_t rfm95_set_bandwidth(rfm95_t *dev, uint8_t bw);

#endif // RFM95_H
