#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BNO055_support.h>

#define EULER_TO_DEG 16

struct bno055_t myBNO;
struct bno055_gyro gyroData;
struct bno055_accel accelData;
struct bno055_euler myEulerData;

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
float yaw_sp = 0.0, last_yaw = 0.0;

unsigned char sys_calib_status = 0;
unsigned char mag_calib_status = 0;
unsigned long last_time = 0;

void check_imu_calibration() {
    if (mag_calib_status != 3 || sys_calib_status !=3){
        Serial2.println("[IMU] Sensor is not calibrated. Calibrating now");
        Serial1.println("[IMU] Sensor is not calibrated. Calibrating now");
    }
    while (mag_calib_status != 3 || sys_calib_status != 3) {
        if ((millis() - last_time) > 200) {
            bno055_get_magcalib_status(&mag_calib_status);
            bno055_get_syscalib_status(&sys_calib_status);
            Serial2.print("Time Stamp: ");
            Serial2.println(last_time);
            Serial2.print("Magnetometer Calibration Status: ");
            Serial2.println(mag_calib_status);
            Serial2.print("System Calibration Status: ");
            Serial2.println(sys_calib_status);

            Serial.print("Time Stamp: ");
            Serial.println(last_time);
            Serial.print("Magnetometer Calibration Status: ");
            Serial.println(mag_calib_status);
            Serial.print("System Calibration Status: ");
            Serial.println(sys_calib_status);
            last_time = millis();
        }
    }
    Serial2.println("[IMU] IMU Calibrated");
}

void imu_init() {
    // Initialize I2C communication
    Wire.begin();
    // Initialization of the BNO055
    BNO_Init(&myBNO);  // Assigning the structure to hold information about the device
    // Configuration to IMUPLUS mode
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(50);
    check_imu_calibration();
}

void imu_update(){
    // gyro : rad/s -> deg/s
    bno055_read_gyro_xyz(&gyroData); // rad/s
    gyro_x = float(gyroData.x) / EULER_TO_DEG;
    gyro_y = float(gyroData.y) / EULER_TO_DEG;
    gyro_z = float(gyroData.z) / EULER_TO_DEG;
    

    // euler : rad/s -> deg/s
    bno055_read_euler_hrp(&myEulerData); // rad/s
    roll = float(myEulerData.r) / EULER_TO_DEG;
    pitch = float(myEulerData.p) / EULER_TO_DEG;
    yaw = float(myEulerData.h) / EULER_TO_DEG;
    if (yaw > 180) yaw -= 360.0; // invert heading to -180 - 180    

    // Additional Low-Pass Filter for Yaw
    yaw = 0.95 * last_yaw + 0.05 * yaw;
    last_yaw = yaw;
    
    yaw_sp = yaw - last_yaw;
}