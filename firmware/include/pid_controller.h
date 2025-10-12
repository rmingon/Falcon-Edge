#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct {
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain
    float setpoint;        // Desired value
    float integral;        // Accumulated integral
    float prev_error;      // Previous error for derivative
    float max_output;      // Maximum output limit
    float min_output;      // Minimum output limit
    float max_integral;    // Anti-windup limit
    uint32_t last_time_ms; // Last update time
} pid_controller_t;

/**
 * @brief Initialize a PID controller
 */
void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float max_output, float min_output, float max_integral);

/**
 * @brief Update PID controller with new measurement
 *
 * @param pid PID controller instance
 * @param measurement Current measured value
 * @param dt Time delta in seconds
 * @return Control output
 */
float pid_update(pid_controller_t *pid, float measurement, float dt);

/**
 * @brief Set the setpoint for the PID controller
 */
void pid_set_setpoint(pid_controller_t *pid, float setpoint);

/**
 * @brief Reset the PID controller state
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief Update PID tuning parameters
 */
void pid_set_tunings(pid_controller_t *pid, float kp, float ki, float kd);

#endif // PID_CONTROLLER_H
