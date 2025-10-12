#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "pid_controller.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "bmp280.h"
#include "gps.h"
#include "rfm95.h"
#include "wifi_lr.h"
#include "motor_control.h"

typedef struct {
    float roll;     // degrees
    float pitch;    // degrees
    float yaw;      // degrees (heading)
} attitude_t;

typedef struct {
    // Sensors
    mpu6050_t mpu6050;
    hmc5883l_t hmc5883l;
    bmp280_t bmp280;
    gps_t gps;
    rfm95_t lora;
    wifi_lr_t wifi;
    motor_control_t motor;

    // Sensor data
    mpu6050_data_t imu_data;
    vector3_t mag_data;
    float altitude;
    float pressure;
    float temperature;
    gps_data_t gps_data;

    // Attitude (fused sensor data)
    attitude_t attitude;

    // PID Controllers
    pid_controller_t altitude_pid;
    pid_controller_t roll_pid;
    pid_controller_t pitch_pid;
    pid_controller_t yaw_pid;

    // Control outputs (0-100%)
    float altitude_output;
    float roll_output;
    float pitch_output;
    float yaw_output;

    // Setpoints
    float altitude_setpoint;
    float roll_setpoint;
    float pitch_setpoint;
    float yaw_setpoint;

    // System state
    bool armed;
    bool calibrated;
    bool has_bmp280;       // BMP280 availability flag
    uint32_t loop_count;
} flight_controller_t;

/**
 * @brief Initialize flight controller system
 */
esp_err_t flight_controller_init(flight_controller_t *fc);

/**
 * @brief Update sensor readings
 */
esp_err_t flight_controller_update_sensors(flight_controller_t *fc);

/**
 * @brief Update attitude estimation (sensor fusion)
 */
void flight_controller_update_attitude(flight_controller_t *fc, float dt);

/**
 * @brief Update control loops
 */
void flight_controller_update_control(flight_controller_t *fc, float dt);

/**
 * @brief Set altitude setpoint
 */
void flight_controller_set_altitude(flight_controller_t *fc, float altitude);

/**
 * @brief Set attitude setpoints
 */
void flight_controller_set_attitude(flight_controller_t *fc, float roll, float pitch, float yaw);

/**
 * @brief Arm the flight controller
 */
void flight_controller_arm(flight_controller_t *fc);

/**
 * @brief Disarm the flight controller
 */
void flight_controller_disarm(flight_controller_t *fc);

/**
 * @brief Send telemetry via LoRa
 */
esp_err_t flight_controller_send_telemetry(flight_controller_t *fc);

#endif // FLIGHT_CONTROLLER_H
