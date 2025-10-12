#include "pid_controller.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float max_output, float min_output, float max_integral)
{
    memset(pid, 0, sizeof(pid_controller_t));
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->max_integral = max_integral;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

float pid_update(pid_controller_t *pid, float measurement, float dt)
{
    // Calculate error
    float error = pid->setpoint - measurement;

    // Proportional term
    float p_term = pid->kp * error;

    // Integral term with anti-windup
    pid->integral += error * dt;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    float i_term = pid->ki * pid->integral;

    // Derivative term
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;

    // Calculate output
    float output = p_term + i_term + d_term;

    // Apply output limits
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }

    // Store error for next iteration
    pid->prev_error = error;

    return output;
}

void pid_set_setpoint(pid_controller_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->last_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void pid_set_tunings(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
