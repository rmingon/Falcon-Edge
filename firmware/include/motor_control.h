#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#define SERVO_LEFT_GPIO         26
#define SERVO_RIGHT_GPIO        25
#define ESC_GPIO                33

#define SERVO_MIN_PULSE_US      1000    // 1ms
#define SERVO_MAX_PULSE_US      2000    // 2ms
#define SERVO_CENTER_PULSE_US   1500    // 1.5ms

#define ESC_MIN_PULSE_US        1000    // 1ms (stop)
#define ESC_MAX_PULSE_US        2000    // 2ms (full throttle)
#define ESC_IDLE_PULSE_US       1100    // Idle throttle

#define PWM_FREQUENCY           50      // 50Hz for servos/ESC

typedef struct {
    uint32_t pwm_timer_num;
    uint32_t servo_left_channel;
    uint32_t servo_right_channel;
    uint32_t esc_channel;

    float servo_left_angle;     // -45 to +45 degrees
    float servo_right_angle;    // -45 to +45 degrees
    float throttle;             // 0 to 100%

    bool initialized;
} motor_control_t;

esp_err_t motor_control_init(motor_control_t *motor);

esp_err_t motor_set_servo_left(motor_control_t *motor, float angle);

esp_err_t motor_set_servo_right(motor_control_t *motor, float angle);

esp_err_t motor_set_throttle(motor_control_t *motor, float throttle);

esp_err_t motor_set_all(motor_control_t *motor, float left_angle, float right_angle, float throttle);

esp_err_t motor_arm_esc(motor_control_t *motor);

esp_err_t motor_disarm(motor_control_t *motor);

#endif // MOTOR_CONTROL_H
