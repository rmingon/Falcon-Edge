#ifndef WIFI_LR_H
#define WIFI_LR_H

#include "esp_err.h"
#include "esp_wifi.h"
#include <stdbool.h>

#define WIFI_SSID_MAX_LEN       32
#define WIFI_PASS_MAX_LEN       64

typedef struct {
    char ssid[WIFI_SSID_MAX_LEN];
    char password[WIFI_PASS_MAX_LEN];
    bool connected;
    bool enabled;
    int rssi;
} wifi_lr_t;

esp_err_t wifi_lr_init(wifi_lr_t *wifi);

esp_err_t wifi_lr_connect(wifi_lr_t *wifi, const char *ssid, const char *password);

esp_err_t wifi_lr_disconnect(wifi_lr_t *wifi);

bool wifi_lr_is_connected(wifi_lr_t *wifi);

int wifi_lr_get_rssi(wifi_lr_t *wifi);

esp_err_t wifi_lr_send_telemetry(wifi_lr_t *wifi, const uint8_t *data, size_t len);

#endif // WIFI_LR_H
