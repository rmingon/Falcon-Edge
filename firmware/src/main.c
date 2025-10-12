#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "config.h"
#include "flight_controller.h"

static const char *TAG = "Main";

static flight_controller_t flight_controller;
static SemaphoreHandle_t fc_mutex;

static void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started");
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000 / SENSOR_SAMPLE_RATE_HZ);

    while (1) {
        if (xSemaphoreTake(fc_mutex, portMAX_DELAY) == pdTRUE) {
            esp_err_t ret = flight_controller_update_sensors(&flight_controller);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to update sensors");
            }
            xSemaphoreGive(fc_mutex);
        }
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

static void control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Control task started");
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000 / CONTROL_LOOP_RATE_HZ);
    const float dt = 1.0f / CONTROL_LOOP_RATE_HZ;

    while (1) {
        if (xSemaphoreTake(fc_mutex, portMAX_DELAY) == pdTRUE) {
            flight_controller_update_attitude(&flight_controller, dt);
            flight_controller_update_control(&flight_controller, dt);

            // Log every second
            if (flight_controller.loop_count % CONTROL_LOOP_RATE_HZ == 0) {
                ESP_LOGI(TAG, "Attitude: R=%.1f° P=%.1f° Y=%.1f° | Alt=%.2fm | GPS: %.6f,%.6f (%d sats)",
                         flight_controller.attitude.roll,
                         flight_controller.attitude.pitch,
                         flight_controller.attitude.yaw,
                         flight_controller.altitude,
                         flight_controller.gps_data.latitude,
                         flight_controller.gps_data.longitude,
                         flight_controller.gps_data.satellites);

                if (flight_controller.armed) {
                    ESP_LOGI(TAG, "Control outputs: Alt=%.1f Roll=%.1f Pitch=%.1f Yaw=%.1f",
                             flight_controller.altitude_output,
                             flight_controller.roll_output,
                             flight_controller.pitch_output,
                             flight_controller.yaw_output);
                }
            }
            xSemaphoreGive(fc_mutex);
        }
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

static void gps_task(void *pvParameters)
{
    ESP_LOGI(TAG, "GPS task started");

    while (1) {
        // Read GPS continuously without mutex (GPS has its own buffer)
        gps_read(&flight_controller.gps, 50);

        // Update shared data with mutex
        if (xSemaphoreTake(fc_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            gps_get_data(&flight_controller.gps, &flight_controller.gps_data);

            // Log GPS status occasionally
            static int gps_log_counter = 0;
            if (++gps_log_counter >= 50) {  // Every ~5 seconds at 10Hz
                if (flight_controller.gps_data.fix) {
                    ESP_LOGI(TAG, "GPS: %.6f,%.6f Alt:%.1fm Sats:%d Speed:%.1fm/s",
                             flight_controller.gps_data.latitude,
                             flight_controller.gps_data.longitude,
                             flight_controller.gps_data.altitude,
                             flight_controller.gps_data.satellites,
                             flight_controller.gps_data.speed);
                } else {
                    ESP_LOGW(TAG, "GPS: No fix, satellites: %d", flight_controller.gps_data.satellites);
                }
                gps_log_counter = 0;
            }
            xSemaphoreGive(fc_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // 10 Hz
    }
}

static void lora_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LoRa task started");
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000 / LORA_TX_RATE_HZ);

    while (1) {
        if (xSemaphoreTake(fc_mutex, portMAX_DELAY) == pdTRUE) {
            esp_err_t ret = flight_controller_send_telemetry(&flight_controller);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send telemetry");
            }
            xSemaphoreGive(fc_mutex);
        }
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

static void wifi_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WiFi task started");
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1000 / WIFI_TX_RATE_HZ);

    while (1) {
        if (xSemaphoreTake(fc_mutex, portMAX_DELAY) == pdTRUE) {
            if (wifi_lr_is_connected(&flight_controller.wifi)) {
                // Send telemetry via WiFi
                uint8_t packet[64];
                int offset = 0;

                memcpy(&packet[offset], &flight_controller.attitude.roll, sizeof(float)); offset += sizeof(float);
                memcpy(&packet[offset], &flight_controller.attitude.pitch, sizeof(float)); offset += sizeof(float);
                memcpy(&packet[offset], &flight_controller.attitude.yaw, sizeof(float)); offset += sizeof(float);
                memcpy(&packet[offset], &flight_controller.altitude, sizeof(float)); offset += sizeof(float);
                memcpy(&packet[offset], &flight_controller.gps_data.latitude, sizeof(float)); offset += sizeof(float);
                memcpy(&packet[offset], &flight_controller.gps_data.longitude, sizeof(float)); offset += sizeof(float);
                memcpy(&packet[offset], &flight_controller.gps_data.satellites, sizeof(uint8_t)); offset += sizeof(uint8_t);
                packet[offset++] = flight_controller.armed ? 1 : 0;

                wifi_lr_send_telemetry(&flight_controller.wifi, packet, offset);

                // Log WiFi status occasionally
                static int wifi_log_counter = 0;
                if (++wifi_log_counter >= 25) {  // Every ~5 seconds at 5Hz
                    int rssi = wifi_lr_get_rssi(&flight_controller.wifi);
                    ESP_LOGI(TAG, "WiFi connected, RSSI: %d dBm", rssi);
                    wifi_log_counter = 0;
                }
            }
            xSemaphoreGive(fc_mutex);
        }
        vTaskDelayUntil(&last_wake_time, frequency);
    }
}

static void command_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Command task started");
    uint8_t rx_buffer[64];

    while (1) {
        int len = rfm95_receive(&flight_controller.lora, rx_buffer, sizeof(rx_buffer), 100);

        if (len > 0) {
            ESP_LOGI(TAG, "Received %d bytes via LoRa", len);

            if (xSemaphoreTake(fc_mutex, portMAX_DELAY) == pdTRUE) {
                if (len >= 1) {
                    switch (rx_buffer[0]) {
                    case 'A':
                        ESP_LOGI(TAG, "Arm command received");
                        flight_controller_arm(&flight_controller);
                        break;

                    case 'D':
                        ESP_LOGI(TAG, "Disarm command received");
                        flight_controller_disarm(&flight_controller);
                        break;

                    case 'S':
                        if (len >= 17) {
                            float altitude, roll, pitch, yaw;
                            memcpy(&altitude, &rx_buffer[1], sizeof(float));
                            memcpy(&roll, &rx_buffer[5], sizeof(float));
                            memcpy(&pitch, &rx_buffer[9], sizeof(float));
                            memcpy(&yaw, &rx_buffer[13], sizeof(float));

                            ESP_LOGI(TAG, "Setpoint command: Alt=%.1f R=%.1f P=%.1f Y=%.1f",
                                     altitude, roll, pitch, yaw);

                            flight_controller_set_altitude(&flight_controller, altitude);
                            flight_controller_set_attitude(&flight_controller, roll, pitch, yaw);
                        }
                        break;

                    case 'W':  // WiFi connect command
                        if (len >= 2) {
                            char ssid[32] = {0};
                            char password[64] = {0};
                            int ssid_len = rx_buffer[1];
                            if (ssid_len > 0 && ssid_len < 32 && len >= 2 + ssid_len) {
                                memcpy(ssid, &rx_buffer[2], ssid_len);
                                ssid[ssid_len] = '\0';

                                if (len > 2 + ssid_len) {
                                    int pass_len = len - 2 - ssid_len;
                                    if (pass_len < 64) {
                                        memcpy(password, &rx_buffer[2 + ssid_len], pass_len);
                                        password[pass_len] = '\0';
                                    }
                                }

                                ESP_LOGI(TAG, "WiFi connect command: %s", ssid);
                                wifi_lr_connect(&flight_controller.wifi, ssid, password);
                            }
                        }
                        break;

                    case 'P':
                        ESP_LOGI(TAG, "PID update command (not implemented)");
                        break;

                    default:
                        ESP_LOGW(TAG, "Unknown command: 0x%02X", rx_buffer[0]);
                        break;
                    }
                }
                xSemaphoreGive(fc_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Falcon-Edge Flight Controller");
    ESP_LOGI(TAG, "FreeRTOS-based ESP32 Firmware");
    ESP_LOGI(TAG, "========================================");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    fc_mutex = xSemaphoreCreateMutex();
    if (fc_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    ESP_LOGI(TAG, "Initializing flight controller...");
    ret = flight_controller_init(&flight_controller);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize flight controller: %s", esp_err_to_name(ret));
        return;
    }

    flight_controller_set_altitude(&flight_controller, flight_controller.altitude);
    flight_controller_set_attitude(&flight_controller, 0.0f, 0.0f, flight_controller.attitude.yaw);

    ESP_LOGI(TAG, "Creating FreeRTOS tasks...");

    xTaskCreate(sensor_task, "sensor_task", SENSOR_TASK_STACK_SIZE, NULL,
                SENSOR_TASK_PRIORITY, NULL);

    xTaskCreate(control_task, "control_task", CONTROL_TASK_STACK_SIZE, NULL,
                CONTROL_TASK_PRIORITY, NULL);

    xTaskCreate(gps_task, "gps_task", GPS_TASK_STACK_SIZE, NULL,
                GPS_TASK_PRIORITY, NULL);

    xTaskCreate(lora_task, "lora_task", LORA_TASK_STACK_SIZE, NULL,
                LORA_TASK_PRIORITY, NULL);

    xTaskCreate(command_task, "command_task", LORA_TASK_STACK_SIZE, NULL,
                LORA_TASK_PRIORITY, NULL);

#if WIFI_ENABLED
    xTaskCreate(wifi_task, "wifi_task", WIFI_TASK_STACK_SIZE, NULL,
                WIFI_TASK_PRIORITY, NULL);
#endif

    ESP_LOGI(TAG, "System initialized successfully!");
    ESP_LOGI(TAG, "Waiting for commands...");
    ESP_LOGI(TAG, "Send 'A' via LoRa to arm the system");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
