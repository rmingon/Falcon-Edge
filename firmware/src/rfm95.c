#include "rfm95.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "RFM95";

static esp_err_t rfm95_write_reg(rfm95_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {reg | 0x80, value}; // Set write bit
    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
    };
    return spi_device_transmit(dev->spi, &trans);
}

static esp_err_t rfm95_read_reg(rfm95_t *dev, uint8_t reg, uint8_t *value)
{
    uint8_t tx_data[2] = {reg & 0x7F, 0x00}; // Clear write bit
    uint8_t rx_data[2];
    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
    esp_err_t ret = spi_device_transmit(dev->spi, &trans);
    if (ret == ESP_OK) {
        *value = rx_data[1];
    }
    return ret;
}

static esp_err_t rfm95_set_mode(rfm95_t *dev, uint8_t mode)
{
    return rfm95_write_reg(dev, RFM95_REG_OP_MODE, RFM95_LONG_RANGE_MODE | mode);
}

esp_err_t rfm95_init(rfm95_t *dev, spi_host_device_t spi_host, uint32_t frequency)
{
    dev->cs_pin = SPI_SS_PIN;
    dev->rst_pin = LORA_RST_PIN;
    dev->dio0_pin = LORA_DIO0_PIN;
    dev->dio1_pin = LORA_DIO1_PIN;
    dev->frequency = frequency;

    // Configure reset pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << dev->rst_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Configure DIO pins as inputs
    io_conf.pin_bit_mask = (1ULL << dev->dio0_pin) | (1ULL << dev->dio1_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    // Reset RFM95
    gpio_set_level(dev->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(dev->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };

    esp_err_t ret = spi_bus_initialize(spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000,  // 1 MHz
        .mode = 0,
        .spics_io_num = dev->cs_pin,
        .queue_size = 7,
    };

    ret = spi_bus_add_device(spi_host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device");
        return ret;
    }

    // Check version
    uint8_t version;
    ret = rfm95_read_reg(dev, RFM95_REG_VERSION, &version);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read version register");
        return ret;
    }

    if (version != 0x12) {
        ESP_LOGE(TAG, "Invalid version: 0x%02X (expected 0x12)", version);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "RFM95 detected, version: 0x%02X", version);

    // Set sleep mode
    rfm95_set_mode(dev, RFM95_MODE_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set frequency
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
    rfm95_write_reg(dev, RFM95_REG_FRF_MSB, (uint8_t)(frf >> 16));
    rfm95_write_reg(dev, RFM95_REG_FRF_MID, (uint8_t)(frf >> 8));
    rfm95_write_reg(dev, RFM95_REG_FRF_LSB, (uint8_t)(frf >> 0));

    // Set FIFO base addresses
    rfm95_write_reg(dev, RFM95_REG_FIFO_TX_BASE_ADDR, 0x00);
    rfm95_write_reg(dev, RFM95_REG_FIFO_RX_BASE_ADDR, 0x00);

    // Set LNA boost
    rfm95_write_reg(dev, 0x0C, 0x23);

    // Set auto AGC
    rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_3, 0x04);

    // Set default configuration (BW=125kHz, CR=4/5, SF=7)
    rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_1, 0x72);
    rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_2, 0x74);

    // Set preamble length
    rfm95_write_reg(dev, RFM95_REG_PREAMBLE_MSB, 0x00);
    rfm95_write_reg(dev, RFM95_REG_PREAMBLE_LSB, 0x08);

    // Set TX power to 17 dBm
    rfm95_set_tx_power(dev, 17);

    // Set standby mode
    rfm95_set_mode(dev, RFM95_MODE_STDBY);

    ESP_LOGI(TAG, "RFM95 initialized at %.1f MHz", frequency / 1000000.0f);
    return ESP_OK;
}

esp_err_t rfm95_set_tx_power(rfm95_t *dev, int8_t power)
{
    if (power < 5) power = 5;
    if (power > 23) power = 23;

    if (power > 20) {
        // High power +20 dBm operation
        rfm95_write_reg(dev, RFM95_REG_PA_DAC, 0x87);
        power -= 3;
    } else {
        rfm95_write_reg(dev, RFM95_REG_PA_DAC, 0x84);
    }

    // PA_BOOST pin, max power
    uint8_t pa_config = 0x80 | (power - 5);
    return rfm95_write_reg(dev, RFM95_REG_PA_CONFIG, pa_config);
}

esp_err_t rfm95_send(rfm95_t *dev, const uint8_t *data, uint8_t len)
{
    if (len > RFM95_MAX_PAYLOAD) {
        ESP_LOGE(TAG, "Payload too large: %d bytes", len);
        return ESP_ERR_INVALID_SIZE;
    }

    // Set standby mode
    rfm95_set_mode(dev, RFM95_MODE_STDBY);

    // Set FIFO pointer to TX base
    rfm95_write_reg(dev, RFM95_REG_FIFO_ADDR_PTR, 0x00);

    // Write data to FIFO
    for (int i = 0; i < len; i++) {
        rfm95_write_reg(dev, RFM95_REG_FIFO, data[i]);
    }

    // Set payload length
    rfm95_write_reg(dev, RFM95_REG_PAYLOAD_LENGTH, len);

    // Start transmission
    rfm95_set_mode(dev, RFM95_MODE_TX);

    // Wait for TX done (DIO0)
    uint32_t start = xTaskGetTickCount();
    while (gpio_get_level(dev->dio0_pin) == 0) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(1000)) {
            ESP_LOGE(TAG, "TX timeout");
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Clear IRQ flags
    rfm95_write_reg(dev, RFM95_REG_IRQ_FLAGS, 0xFF);

    // Set standby mode
    rfm95_set_mode(dev, RFM95_MODE_STDBY);

    ESP_LOGI(TAG, "Sent %d bytes", len);
    return ESP_OK;
}

int rfm95_receive(rfm95_t *dev, uint8_t *data, uint8_t len, uint32_t timeout_ms)
{
    // Set RX continuous mode
    rfm95_set_mode(dev, RFM95_MODE_RX_CONTINUOUS);

    // Wait for RX done (DIO0)
    uint32_t start = xTaskGetTickCount();
    while (gpio_get_level(dev->dio0_pin) == 0) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            rfm95_set_mode(dev, RFM95_MODE_STDBY);
            return 0; // Timeout, no data received
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Read IRQ flags
    uint8_t irq_flags;
    rfm95_read_reg(dev, RFM95_REG_IRQ_FLAGS, &irq_flags);

    // Clear IRQ flags
    rfm95_write_reg(dev, RFM95_REG_IRQ_FLAGS, 0xFF);

    // Check if RX done
    if (irq_flags & RFM95_IRQ_RX_DONE) {
        // Read received length
        uint8_t rx_len;
        rfm95_read_reg(dev, RFM95_REG_RX_NB_BYTES, &rx_len);

        if (rx_len > len) {
            rx_len = len;
        }

        // Get FIFO RX current address
        uint8_t fifo_addr;
        rfm95_read_reg(dev, RFM95_REG_FIFO_RX_CURRENT_ADDR, &fifo_addr);

        // Set FIFO pointer
        rfm95_write_reg(dev, RFM95_REG_FIFO_ADDR_PTR, fifo_addr);

        // Read data from FIFO
        for (int i = 0; i < rx_len; i++) {
            rfm95_read_reg(dev, RFM95_REG_FIFO, &data[i]);
        }

        rfm95_set_mode(dev, RFM95_MODE_STDBY);
        ESP_LOGI(TAG, "Received %d bytes", rx_len);
        return rx_len;
    }

    rfm95_set_mode(dev, RFM95_MODE_STDBY);
    return 0;
}

int16_t rfm95_get_rssi(rfm95_t *dev)
{
    uint8_t rssi;
    rfm95_read_reg(dev, RFM95_REG_PKT_RSSI_VALUE, &rssi);
    return -157 + (int16_t)rssi;
}

esp_err_t rfm95_set_spreading_factor(rfm95_t *dev, uint8_t sf)
{
    if (sf < 6 || sf > 12) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t config2;
    rfm95_read_reg(dev, RFM95_REG_MODEM_CONFIG_2, &config2);
    config2 = (config2 & 0x0F) | ((sf << 4) & 0xF0);
    return rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_2, config2);
}

esp_err_t rfm95_set_bandwidth(rfm95_t *dev, uint8_t bw)
{
    if (bw > 9) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t config1;
    rfm95_read_reg(dev, RFM95_REG_MODEM_CONFIG_1, &config1);
    config1 = (config1 & 0x0F) | (bw << 4);
    return rfm95_write_reg(dev, RFM95_REG_MODEM_CONFIG_1, config1);
}
