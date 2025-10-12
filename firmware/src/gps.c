#include "gps.h"
#include "config.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "GPS";

esp_err_t gps_init(gps_t *gps, uart_port_t uart_port, int tx_pin, int rx_pin)
{
    gps->uart_port = uart_port;
    gps->rx_index = 0;
    memset(&gps->data, 0, sizeof(gps_data_t));

    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(uart_port, GPS_RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver");
        return ret;
    }

    ret = uart_param_config(uart_port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART");
        return ret;
    }

    ret = uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins");
        return ret;
    }

    ESP_LOGI(TAG, "GPS initialized on UART%d", uart_port);
    return ESP_OK;
}

static float parse_coordinate(const char *coord, const char *dir)
{
    if (!coord || !dir || strlen(coord) == 0) {
        return 0.0f;
    }

    // Parse DDMM.MMMM format
    float value = atof(coord);
    int degrees = (int)(value / 100);
    float minutes = value - (degrees * 100);
    float decimal = degrees + (minutes / 60.0f);

    // Apply direction
    if (dir[0] == 'S' || dir[0] == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

esp_err_t gps_parse_nmea(gps_t *gps, const char *sentence)
{
    if (!sentence || strlen(sentence) < 10) {
        return ESP_ERR_INVALID_ARG;
    }

    // Parse GPGGA sentence (Global Positioning System Fix Data)
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        char *token;
        char *saveptr;
        char buffer[128];
        strncpy(buffer, sentence, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';

        int field = 0;
        char time_str[16] = {0};
        char lat_str[16] = {0};
        char lat_dir[2] = {0};
        char lon_str[16] = {0};
        char lon_dir[2] = {0};
        char quality_str[2] = {0};
        char sats_str[4] = {0};
        char alt_str[16] = {0};

        token = strtok_r(buffer, ",", &saveptr);
        while (token != NULL && field <= 10) {
            switch (field) {
                case 1: strncpy(time_str, token, sizeof(time_str) - 1); break;
                case 2: strncpy(lat_str, token, sizeof(lat_str) - 1); break;
                case 3: strncpy(lat_dir, token, sizeof(lat_dir) - 1); break;
                case 4: strncpy(lon_str, token, sizeof(lon_str) - 1); break;
                case 5: strncpy(lon_dir, token, sizeof(lon_dir) - 1); break;
                case 6: strncpy(quality_str, token, sizeof(quality_str) - 1); break;
                case 7: strncpy(sats_str, token, sizeof(sats_str) - 1); break;
                case 9: strncpy(alt_str, token, sizeof(alt_str) - 1); break;
            }
            field++;
            token = strtok_r(NULL, ",", &saveptr);
        }

        // Parse time (HHMMSS.sss)
        if (strlen(time_str) >= 6) {
            gps->data.hour = (time_str[0] - '0') * 10 + (time_str[1] - '0');
            gps->data.minute = (time_str[2] - '0') * 10 + (time_str[3] - '0');
            gps->data.second = (time_str[4] - '0') * 10 + (time_str[5] - '0');
        }

        // Parse position
        gps->data.latitude = parse_coordinate(lat_str, lat_dir);
        gps->data.longitude = parse_coordinate(lon_str, lon_dir);
        gps->data.altitude = atof(alt_str);
        gps->data.satellites = atoi(sats_str);
        gps->data.fix = (atoi(quality_str) > 0);

        return ESP_OK;
    }

    // Parse GPRMC sentence (Recommended Minimum)
    if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        char *token;
        char *saveptr;
        char buffer[128];
        strncpy(buffer, sentence, sizeof(buffer) - 1);
        buffer[sizeof(buffer) - 1] = '\0';

        int field = 0;
        char speed_str[16] = {0};
        char course_str[16] = {0};

        token = strtok_r(buffer, ",", &saveptr);
        while (token != NULL && field <= 8) {
            if (field == 7) strncpy(speed_str, token, sizeof(speed_str) - 1);
            if (field == 8) strncpy(course_str, token, sizeof(course_str) - 1);
            field++;
            token = strtok_r(NULL, ",", &saveptr);
        }

        // Speed in knots, convert to m/s
        gps->data.speed = atof(speed_str) * 0.514444f;
        gps->data.course = atof(course_str);

        return ESP_OK;
    }

    return ESP_OK;
}

esp_err_t gps_read(gps_t *gps, uint32_t timeout_ms)
{
    uint8_t data[128];
    int len = uart_read_bytes(gps->uart_port, data, sizeof(data) - 1,
                              pdMS_TO_TICKS(timeout_ms));

    if (len > 0) {
        for (int i = 0; i < len; i++) {
            if (data[i] == '\n' || data[i] == '\r') {
                if (gps->rx_index > 0) {
                    gps->rx_buffer[gps->rx_index] = '\0';

                    // Debug: log received sentence
                    static int sentence_count = 0;
                    if (++sentence_count % 20 == 0) {  // Log every 20th sentence
                        ESP_LOGI(TAG, "GPS RX: %s", gps->rx_buffer);
                    }

                    gps_parse_nmea(gps, (char *)gps->rx_buffer);
                    gps->rx_index = 0;
                }
            } else if (gps->rx_index < sizeof(gps->rx_buffer) - 1) {
                gps->rx_buffer[gps->rx_index++] = data[i];
            } else {
                ESP_LOGW(TAG, "GPS buffer overflow, resetting");
                gps->rx_index = 0;
            }
        }
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

void gps_get_data(gps_t *gps, gps_data_t *data)
{
    memcpy(data, &gps->data, sizeof(gps_data_t));
}
