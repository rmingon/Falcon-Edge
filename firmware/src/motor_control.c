#include "motor_control.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "MotorControl";

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY          50

static uint32_t angle_to_pulse_us(float angle)
{
    // Map -45 to +45 degrees to 1000-2000us
    if (angle < -45.0f) angle = -45.0f;
    if (angle > 45.0f) angle = 45.0f;

    float pulse = SERVO_CENTER_PULSE_US + (angle / 45.0f) * 500.0f;
    return (uint32_t)pulse;
}

static uint32_t throttle_to_pulse_us(float throttle)
{
    // Map 0-100% to ESC_MIN to ESC_MAX
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 100.0f) throttle = 100.0f;

    uint32_t pulse = ESC_MIN_PULSE_US + (uint32_t)((throttle / 100.0f) * (ESC_MAX_PULSE_US - ESC_MIN_PULSE_US));
    return pulse;
}

static uint32_t pulse_us_to_duty(uint32_t pulse_us)
{
    // Calculate duty cycle for LEDC
    // duty = (pulse_us / period_us) * (2^duty_resolution)
    uint32_t period_us = 1000000 / LEDC_FREQUENCY;
    uint32_t max_duty = (1 << LEDC_DUTY_RES);
    uint32_t duty = (pulse_us * max_duty) / period_us;
    return duty;
}

esp_err_t motor_control_init(motor_control_t *motor)
{
    motor->initialized = false;
    motor->servo_left_angle = 0.0f;
    motor->servo_right_angle = 0.0f;
    motor->throttle = 0.0f;

    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LEDC timer");
        return ret;
    }

    // Configure left servo channel
    ledc_channel_config_t left_servo = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_LEFT_GPIO,
        .duty           = pulse_us_to_duty(SERVO_CENTER_PULSE_US),
        .hpoint         = 0
    };
    ret = ledc_channel_config(&left_servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure left servo");
        return ret;
    }
    motor->servo_left_channel = LEDC_CHANNEL_0;

    // Configure right servo channel
    ledc_channel_config_t right_servo = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = SERVO_RIGHT_GPIO,
        .duty           = pulse_us_to_duty(SERVO_CENTER_PULSE_US),
        .hpoint         = 0
    };
    ret = ledc_channel_config(&right_servo);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure right servo");
        return ret;
    }
    motor->servo_right_channel = LEDC_CHANNEL_1;

    // Configure ESC channel (start at min throttle)
    ledc_channel_config_t esc = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = ESC_GPIO,
        .duty           = pulse_us_to_duty(ESC_MIN_PULSE_US),
        .hpoint         = 0
    };
    ret = ledc_channel_config(&esc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ESC");
        return ret;
    }
    motor->esc_channel = LEDC_CHANNEL_2;

    motor->initialized = true;
    ESP_LOGI(TAG, "Motor control initialized - Left:%d Right:%d ESC:%d",
             SERVO_LEFT_GPIO, SERVO_RIGHT_GPIO, ESC_GPIO);

    return ESP_OK;
}

esp_err_t motor_set_servo_left(motor_control_t *motor, float angle)
{
    if (!motor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    motor->servo_left_angle = angle;
    uint32_t pulse = angle_to_pulse_us(angle);
    uint32_t duty = pulse_us_to_duty(pulse);

    return ledc_set_duty(LEDC_MODE, motor->servo_left_channel, duty) ||
           ledc_update_duty(LEDC_MODE, motor->servo_left_channel);
}

esp_err_t motor_set_servo_right(motor_control_t *motor, float angle)
{
    if (!motor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    motor->servo_right_angle = angle;
    uint32_t pulse = angle_to_pulse_us(angle);
    uint32_t duty = pulse_us_to_duty(pulse);

    return ledc_set_duty(LEDC_MODE, motor->servo_right_channel, duty) ||
           ledc_update_duty(LEDC_MODE, motor->servo_right_channel);
}

esp_err_t motor_set_throttle(motor_control_t *motor, float throttle)
{
    if (!motor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    motor->throttle = throttle;
    uint32_t pulse = throttle_to_pulse_us(throttle);
    uint32_t duty = pulse_us_to_duty(pulse);

    return ledc_set_duty(LEDC_MODE, motor->esc_channel, duty) ||
           ledc_update_duty(LEDC_MODE, motor->esc_channel);
}

esp_err_t motor_set_all(motor_control_t *motor, float left_angle, float right_angle, float throttle)
{
    esp_err_t ret;

    ret = motor_set_servo_left(motor, left_angle);
    if (ret != ESP_OK) return ret;

    ret = motor_set_servo_right(motor, right_angle);
    if (ret != ESP_OK) return ret;

    ret = motor_set_throttle(motor, throttle);
    return ret;
}

esp_err_t motor_arm_esc(motor_control_t *motor)
{
    if (!motor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Arming ESC sequence...");

    // ESC arming sequence: min throttle for 2 seconds
    uint32_t duty = pulse_us_to_duty(ESC_MIN_PULSE_US);
    ledc_set_duty(LEDC_MODE, motor->esc_channel, duty);
    ledc_update_duty(LEDC_MODE, motor->esc_channel);

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "ESC armed");
    return ESP_OK;
}

esp_err_t motor_disarm(motor_control_t *motor)
{
    if (!motor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Disarming motors");

    // Center servos
    motor_set_servo_left(motor, 0.0f);
    motor_set_servo_right(motor, 0.0f);

    // Min throttle
    motor_set_throttle(motor, 0.0f);

    return ESP_OK;
}
