#include "wifi_lr.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include <string.h>

static const char *TAG = "WiFi_LR";

#define TELEMETRY_UDP_PORT      5000
#define GROUND_STATION_IP       "192.168.4.1"

static int udp_socket = -1;
static struct sockaddr_in ground_station_addr;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    wifi_lr_t *wifi = (wifi_lr_t *)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi->connected = false;
        ESP_LOGW(TAG, "Disconnected from AP, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi->connected = true;

        // Setup UDP socket for telemetry
        if (udp_socket < 0) {
            udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if (udp_socket >= 0) {
                memset(&ground_station_addr, 0, sizeof(ground_station_addr));
                ground_station_addr.sin_family = AF_INET;
                ground_station_addr.sin_port = htons(TELEMETRY_UDP_PORT);
                inet_pton(AF_INET, GROUND_STATION_IP, &ground_station_addr.sin_addr);
                ESP_LOGI(TAG, "UDP socket created for telemetry");
            }
        }
    }
}

esp_err_t wifi_lr_init(wifi_lr_t *wifi)
{
    memset(wifi, 0, sizeof(wifi_lr_t));
    wifi->enabled = false;
    wifi->connected = false;

    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to init netif");
        return ret;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop");
        return ret;
    }

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi");
        return ret;
    }

    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, wifi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register WiFi event handler");
        return ret;
    }

    ret = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, wifi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IP event handler");
        return ret;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode");
        return ret;
    }

    // Enable Long Range mode
    ret = esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable LR mode");
        return ret;
    }

    ESP_LOGI(TAG, "WiFi Long Range mode initialized");
    return ESP_OK;
}

esp_err_t wifi_lr_connect(wifi_lr_t *wifi, const char *ssid, const char *password)
{
    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(wifi->ssid, ssid, WIFI_SSID_MAX_LEN - 1);
    wifi->ssid[WIFI_SSID_MAX_LEN - 1] = '\0';

    if (password) {
        strncpy(wifi->password, password, WIFI_PASS_MAX_LEN - 1);
        wifi->password[WIFI_PASS_MAX_LEN - 1] = '\0';
    } else {
        wifi->password[0] = '\0';
    }

    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, wifi->ssid);
    strcpy((char *)wifi_config.sta.password, wifi->password);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_err_t ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi config");
        return ret;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_STATE) {
        ESP_LOGE(TAG, "Failed to start WiFi");
        return ret;
    }

    wifi->enabled = true;
    ESP_LOGI(TAG, "Connecting to WiFi: %s", wifi->ssid);
    return ESP_OK;
}

esp_err_t wifi_lr_disconnect(wifi_lr_t *wifi)
{
    if (udp_socket >= 0) {
        close(udp_socket);
        udp_socket = -1;
    }

    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi");
        return ret;
    }

    wifi->connected = false;
    wifi->enabled = false;
    ESP_LOGI(TAG, "WiFi disconnected");
    return ESP_OK;
}

bool wifi_lr_is_connected(wifi_lr_t *wifi)
{
    if (!wifi->connected) {
        return false;
    }

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        wifi->rssi = ap_info.rssi;
        return true;
    }

    wifi->connected = false;
    return false;
}

int wifi_lr_get_rssi(wifi_lr_t *wifi)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        wifi->rssi = ap_info.rssi;
        return wifi->rssi;
    }
    return -100;
}

esp_err_t wifi_lr_send_telemetry(wifi_lr_t *wifi, const uint8_t *data, size_t len)
{
    if (!wifi->connected || udp_socket < 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int sent = sendto(udp_socket, data, len, 0,
                      (struct sockaddr *)&ground_station_addr,
                      sizeof(ground_station_addr));

    if (sent < 0) {
        ESP_LOGW(TAG, "Failed to send telemetry");
        return ESP_FAIL;
    }

    return ESP_OK;
}
