#include "flight_controller.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "FlightController";

esp_err_t flight_controller_init(flight_controller_t *fc)
{
    memset(fc, 0, sizeof(flight_controller_t));

    ESP_LOGI(TAG, "Initializing I2C...");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C");
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver");
        return ret;
    }

    ESP_LOGI(TAG, "Initializing MPU6050...");
    ret = mpu6050_init(&fc->mpu6050, I2C_MASTER_NUM, MPU6050_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050");
        return ret;
    }

    ESP_LOGI(TAG, "Initializing HMC5883L...");
    ret = hmc5883l_init(&fc->hmc5883l, I2C_MASTER_NUM, HMC5883L_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HMC5883L");
        return ret;
    }

    ESP_LOGI(TAG, "Initializing BMP280...");
    ret = bmp280_init(&fc->bmp280, I2C_MASTER_NUM, BMP280_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BMP280 not found (altitude control disabled)");
        fc->has_bmp280 = false;
    } else {
        fc->has_bmp280 = true;
        ESP_LOGI(TAG, "BMP280 initialized successfully");
    }

    ESP_LOGI(TAG, "Initializing GPS...");
    ret = gps_init(&fc->gps, GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize GPS");
        return ret;
    }

    ESP_LOGI(TAG, "Initializing LoRa...");
    ret = rfm95_init(&fc->lora, SPI2_HOST, RFM95_FREQ_915MHZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LoRa");
        return ret;
    }

#if WIFI_ENABLED
    ESP_LOGI(TAG, "Initializing WiFi Long Range...");
    ret = wifi_lr_init(&fc->wifi);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi init failed, continuing without WiFi");
    } else {
        // Auto-connect if default credentials are set
        if (strlen(WIFI_DEFAULT_SSID) > 0) {
            wifi_lr_connect(&fc->wifi, WIFI_DEFAULT_SSID, WIFI_DEFAULT_PASSWORD);
        }
    }
#endif

    ESP_LOGI(TAG, "Initializing motor control...");
    ret = motor_control_init(&fc->motor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motor control");
        return ret;
    }

    ESP_LOGI(TAG, "Initializing PID controllers...");
    pid_init(&fc->altitude_pid, ALT_PID_KP, ALT_PID_KI, ALT_PID_KD,
             ALT_PID_MAX_OUTPUT, ALT_PID_MIN_OUTPUT, ALT_PID_MAX_INTEGRAL);

    pid_init(&fc->roll_pid, ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD,
             ROLL_PID_MAX_OUTPUT, ROLL_PID_MIN_OUTPUT, ROLL_PID_MAX_INTEGRAL);

    pid_init(&fc->pitch_pid, PITCH_PID_KP, PITCH_PID_KI, PITCH_PID_KD,
             PITCH_PID_MAX_OUTPUT, PITCH_PID_MIN_OUTPUT, PITCH_PID_MAX_INTEGRAL);

    pid_init(&fc->yaw_pid, YAW_PID_KP, YAW_PID_KI, YAW_PID_KD,
             YAW_PID_MAX_OUTPUT, YAW_PID_MIN_OUTPUT, YAW_PID_MAX_INTEGRAL);

    ESP_LOGI(TAG, "Calibrating IMU...");
    ret = mpu6050_calibrate(&fc->mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calibrate MPU6050");
        return ret;
    }

    fc->calibrated = true;
    fc->armed = false;

    ESP_LOGI(TAG, "Flight controller initialized successfully");
    return ESP_OK;
}

esp_err_t flight_controller_update_sensors(flight_controller_t *fc)
{
    esp_err_t ret = mpu6050_read_data(&fc->mpu6050, &fc->imu_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read MPU6050");
        return ret;
    }

    ret = hmc5883l_read_data(&fc->hmc5883l, &fc->mag_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read HMC5883L");
        return ret;
    }

    if (fc->has_bmp280) {
        ret = bmp280_read_data(&fc->bmp280, &fc->temperature, &fc->pressure, &fc->altitude);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read BMP280");
        }
    }

    return ESP_OK;
}

void flight_controller_update_attitude(flight_controller_t *fc, float dt)
{
    // Get roll/pitch from accelerometer
    float accel_roll = atan2f(fc->imu_data.accel.y, fc->imu_data.accel.z) * 180.0f / M_PI;
    float accel_pitch = atan2f(-fc->imu_data.accel.x,
                               sqrtf(fc->imu_data.accel.y * fc->imu_data.accel.y +
                                     fc->imu_data.accel.z * fc->imu_data.accel.z)) * 180.0f / M_PI;

    // Integrate gyro
    fc->attitude.roll += fc->imu_data.gyro.x * dt;
    fc->attitude.pitch += fc->imu_data.gyro.y * dt;
    fc->attitude.yaw += fc->imu_data.gyro.z * dt;

    // Complementary filter to fuse accel and gyro
    fc->attitude.roll = ALPHA_GYRO * fc->attitude.roll + ALPHA_ACCEL * accel_roll;
    fc->attitude.pitch = ALPHA_GYRO * fc->attitude.pitch + ALPHA_ACCEL * accel_pitch;

    // Tilt-compensated compass heading
    float mag_x = fc->mag_data.x * cosf(fc->attitude.pitch * M_PI / 180.0f) +
                  fc->mag_data.z * sinf(fc->attitude.pitch * M_PI / 180.0f);

    float mag_y = fc->mag_data.x * sinf(fc->attitude.roll * M_PI / 180.0f) *
                      sinf(fc->attitude.pitch * M_PI / 180.0f) +
                  fc->mag_data.y * cosf(fc->attitude.roll * M_PI / 180.0f) -
                  fc->mag_data.z * sinf(fc->attitude.roll * M_PI / 180.0f) *
                      cosf(fc->attitude.pitch * M_PI / 180.0f);

    float mag_heading = atan2f(mag_y, mag_x) * 180.0f / M_PI;
    if (mag_heading < 0) {
        mag_heading += 360.0f;
    }

    // Slowly correct yaw drift with compass
    float yaw_diff = mag_heading - fc->attitude.yaw;
    if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
    if (yaw_diff < -180.0f) yaw_diff += 360.0f;
    fc->attitude.yaw += yaw_diff * 0.01f;

    // Keep yaw in 0-360 range
    if (fc->attitude.yaw < 0) fc->attitude.yaw += 360.0f;
    if (fc->attitude.yaw >= 360.0f) fc->attitude.yaw -= 360.0f;
}

void flight_controller_update_control(flight_controller_t *fc, float dt)
{
    if (!fc->armed) {
        fc->altitude_output = 0.0f;
        fc->roll_output = 0.0f;
        fc->pitch_output = 0.0f;
        fc->yaw_output = 0.0f;
        motor_disarm(&fc->motor);
        return;
    }

    fc->altitude_output = pid_update(&fc->altitude_pid, fc->altitude, dt);
    fc->roll_output = pid_update(&fc->roll_pid, fc->attitude.roll, dt);
    fc->pitch_output = pid_update(&fc->pitch_pid, fc->attitude.pitch, dt);

    // Handle yaw wraparound
    float yaw_error = fc->yaw_setpoint - fc->attitude.yaw;
    if (yaw_error > 180.0f) yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;

    fc->yaw_output = pid_update(&fc->yaw_pid, fc->attitude.yaw, dt);

    // Fixed-wing mixing: convert PID outputs to servo angles and throttle
    // Roll: differential aileron (left wing down = negative roll)
    // Pitch: both servos move together (elevator effect)
    // Yaw: rudder effect via differential aileron
    // Altitude: throttle control

    float left_servo = 0.0f;
    float right_servo = 0.0f;
    float throttle = 50.0f;  // Base throttle

    // Aileron mixing (roll control)
    // Left wing down = left servo up, right servo down
    left_servo += fc->roll_output * 0.3f;   // Scale PID output to servo angle
    right_servo -= fc->roll_output * 0.3f;

    // Elevator mixing (pitch control)
    // Pitch up = both servos up
    left_servo += fc->pitch_output * 0.3f;
    right_servo += fc->pitch_output * 0.3f;

    // Rudder mixing via differential aileron (yaw control)
    left_servo += fc->yaw_output * 0.1f;
    right_servo -= fc->yaw_output * 0.1f;

    // Throttle from altitude PID
    throttle += fc->altitude_output * 0.5f;

    // Clamp values
    if (left_servo < -45.0f) left_servo = -45.0f;
    if (left_servo > 45.0f) left_servo = 45.0f;
    if (right_servo < -45.0f) right_servo = -45.0f;
    if (right_servo > 45.0f) right_servo = 45.0f;
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 100.0f) throttle = 100.0f;

    // Apply to motors
    motor_set_all(&fc->motor, left_servo, right_servo, throttle);

    fc->loop_count++;
}

void flight_controller_set_altitude(flight_controller_t *fc, float altitude)
{
    fc->altitude_setpoint = altitude;
    pid_set_setpoint(&fc->altitude_pid, altitude);
}

void flight_controller_set_attitude(flight_controller_t *fc, float roll, float pitch, float yaw)
{
    fc->roll_setpoint = roll;
    fc->pitch_setpoint = pitch;
    fc->yaw_setpoint = yaw;

    pid_set_setpoint(&fc->roll_pid, roll);
    pid_set_setpoint(&fc->pitch_pid, pitch);
    pid_set_setpoint(&fc->yaw_pid, yaw);
}

void flight_controller_arm(flight_controller_t *fc)
{
    if (!fc->calibrated) {
        ESP_LOGW(TAG, "Cannot arm: system not calibrated");
        return;
    }

    ESP_LOGI(TAG, "Arming flight controller");

    // Arm ESC first (safety sequence)
    motor_arm_esc(&fc->motor);

    fc->armed = true;

    pid_reset(&fc->altitude_pid);
    pid_reset(&fc->roll_pid);
    pid_reset(&fc->pitch_pid);
    pid_reset(&fc->yaw_pid);

    ESP_LOGI(TAG, "Flight controller armed - ready to fly");
}

void flight_controller_disarm(flight_controller_t *fc)
{
    ESP_LOGI(TAG, "Disarming flight controller");
    fc->armed = false;

    fc->altitude_output = 0.0f;
    fc->roll_output = 0.0f;
    fc->pitch_output = 0.0f;
    fc->yaw_output = 0.0f;
}

esp_err_t flight_controller_send_telemetry(flight_controller_t *fc)
{
    uint8_t packet[64];
    int offset = 0;

    memcpy(&packet[offset], &fc->attitude.roll, sizeof(float)); offset += sizeof(float);
    memcpy(&packet[offset], &fc->attitude.pitch, sizeof(float)); offset += sizeof(float);
    memcpy(&packet[offset], &fc->attitude.yaw, sizeof(float)); offset += sizeof(float);
    memcpy(&packet[offset], &fc->altitude, sizeof(float)); offset += sizeof(float);
    memcpy(&packet[offset], &fc->gps_data.latitude, sizeof(float)); offset += sizeof(float);
    memcpy(&packet[offset], &fc->gps_data.longitude, sizeof(float)); offset += sizeof(float);
    memcpy(&packet[offset], &fc->gps_data.satellites, sizeof(uint8_t)); offset += sizeof(uint8_t);
    packet[offset++] = fc->armed ? 1 : 0;

    return rfm95_send(&fc->lora, packet, offset);
}
