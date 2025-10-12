#ifndef GPS_H
#define GPS_H

#include "driver/uart.h"
#include "esp_err.h"
#include <stdbool.h>

typedef struct {
    float latitude;       // Decimal degrees
    float longitude;      // Decimal degrees
    float altitude;       // Meters above sea level
    float speed;          // Speed in m/s
    float course;         // Course in degrees
    uint8_t satellites;   // Number of satellites
    bool fix;             // GPS fix status
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} gps_data_t;

typedef struct {
    uart_port_t uart_port;
    gps_data_t data;
    uint8_t rx_buffer[256];
    int rx_index;
} gps_t;

/**
 * @brief Initialize GPS module
 */
esp_err_t gps_init(gps_t *gps, uart_port_t uart_port, int tx_pin, int rx_pin);

/**
 * @brief Parse NMEA sentence
 *
 * @param gps GPS device
 * @param sentence NMEA sentence string
 * @return ESP_OK if parsed successfully
 */
esp_err_t gps_parse_nmea(gps_t *gps, const char *sentence);

/**
 * @brief Read and parse GPS data
 *
 * @param gps GPS device
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK if data was read
 */
esp_err_t gps_read(gps_t *gps, uint32_t timeout_ms);

/**
 * @brief Get current GPS data
 */
void gps_get_data(gps_t *gps, gps_data_t *data);

#endif // GPS_H
