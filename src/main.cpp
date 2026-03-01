#include <Arduino.h>
#include <bno055.h>
// #include <ekf.h>
// #include <uart.h>
#include <actu_setup.h>
#include <flight_controller.h>
#include <radio.h>
#include<datastream.h>

// #define USB Serial
// #define UART Serial1
// #define TELEMETRY Serial2
#define LED 2
#define M_CALIB false

float actual_roll = 0.0f;
float actual_pitch = 0.0f;
float actual_yaw = 0.0f;

float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

float gyro_x = 0.0f;
float gyro_y = 0.0f;
float gyro_z = 0.0f;

float gyro_x_control = 0.0f;
float gyro_y_control = 0.0f;
float gyro_z_control = 0.0f;

float accel_x = 0.0f;
float accel_y = 0.0f;
float accel_z = 0.0f;

float mag_x = 0.0f;
float mag_y = 0.0f;
float mag_z = 0.0f;

float setpoint_roll = 0.0f;
float setpoint_pitch = 0.0f;
float setpoint_yaw = 0.0f;

// float roll_ekf = 0.0f;
// float pitch_ekf = 0.0f;
// float yaw_ekf = 0.0f;

float error_roll = 0.0f;
float error_pitch = 0.0f;
float error_yaw = 0.0f;

// int16_t ch_throttle = 1000;
bool arming;

float u1, u2, u3, u4;
float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
extern int16_t raw_throttle;

unsigned long prevMillis = 0;
unsigned long previousMicros = 0;
const long loopInterval = 10000; 

void setup() {
    datastream_setup();    
    
    // scan i2c
    Serial.println("Scanning...");
    for (byte addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("Found at 0x");
        Serial.println(addr, HEX);
      }
    }
    imu_hardware_init();
    remote_setup();
    motor_init();
    // motor_calibration();
    Serial.println("[SYSTEM READY]");
    delay(3000);
}

void loop() {
//FAST LOOP WITHOUT DELAY
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros <= loopInterval)return;
    // float dt = (currentMicros - previousMicros) / 1e6f;
    previousMicros = currentMicros;
    imu_hardware_read();  
    remote_loop();
    // ekf_predict(dt);
    // ekf_update();
    // get_euler();
    // uart_loop();
    drone_controller();
    if (arming) {
       writeMotors(motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm);
       digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
       writeMotors(1000, 1000, 1000, 1000);
       digitalWrite(LED_BUILTIN, LOW);
    }

  //SLOWER LOOP WITH DELAY
    unsigned long currentMillis = millis();
    if (currentMillis - prevMillis >= 20) {
      prevMillis = currentMillis;
      // datastream_via_wire();
      datastream_via_telem();
    }
}