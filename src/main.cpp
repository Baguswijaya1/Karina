#include <Arduino.h>
#include <bno055.h>
// #include <ekf.h>
// #include <uart.h>
#include <actu_setup.h>
#include <flight_controller.h>
#include <radio.h>

#define USB Serial
// #define UART Serial1
#define TELEMETRY Serial2
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

void debug_data() {
    USB.print("Act_Roll:");
    USB.print(actual_roll);
    USB.print("\tAct_Pitch:");
    USB.print(actual_pitch);
    USB.print("\tAct_Yaw:");
    USB.print(actual_yaw);
    // USB.print("\tGyro_X:");
    // USB.print(gyro_x); 
    // USB.print("\tGyro_Y:");
    // USB.print(gyro_y);
    // USB.print("\tGyro_Z:");
    // USB.print(gyro_z);
    // USB.print("\tAccel_X:");
    // USB.print(accel_x); 
    // USB.print("\tAccel_Y:");
    // USB.print(accel_y);
    // USB.print("\tAccel_Z:");
    // USB.print(accel_z);
    // USB.print("\tMag_X:");
    // USB.print(mag_x);
    // USB.print("\tMag_Y:");
    // USB.print(mag_y);
    // USB.print("\tMag_Z:");
    // USB.print(mag_z);
    // USB.print("\tSet_Roll:");
    // USB.print(setpoint_roll);
    // USB.print("\tSet_Pitch:");
    // USB.print(setpoint_pitch);
    // USB.print("\tSet_Yaw:");
    // USB.print(setpoint_yaw);
    // USB.print("\tErr_Roll:");
    // USB.print(error_roll);
    // USB.print("\tErr_Pitch:");
    // USB.print(error_pitch);
    // USB.print("\tErr_Yaw:");
    // USB.print(error_yaw);
    // USB.print("\tErr_Gyro_Roll:");
    // USB.print(error_gyro_roll);
    // USB.print("\tErr_Gyro_Pitch:");
    // USB.print(error_gyro_pitch);
    // USB.print("\tErr_Gyro_Yaw:");
    // USB.print(error_gyro_yaw);
    // USB.print("\tCh_Thr:");
    // USB.print(ch_throttle);
    // USB.print("\tIncomng_throttle:");
    // USB.print(raw_throttle);
    // USB.print("\tArming:");
    USB.print(arming);
    // USB.print("\tMotor1_PWM:");
    // USB.print(motor1_pwm);
    // USB.print("\tMotor2_PWM:");
    // USB.print(motor2_pwm);
    // USB.print("\tMotor3_PWM:");
    // USB.print(motor3_pwm);
    // USB.print("\tMotor4_PWM:");
    // USB.print(motor4_pwm);

    // USB.print("\tError_yaw:");
    // USB.print(error_yaw);
    USB.println();
}

void telemetry_data() {
    TELEMETRY.println();
}

void setup() {
    // UART.begin(115200);
    TELEMETRY.begin(57600);
    USB.begin(115200);
    // scan i2c
    Serial.println("Scanning...");
      for (byte addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
          Serial.print("Found at 0x");
          Serial.println(addr, HEX);
        }
      }
    Serial.println("initializing imu");
    imu_hardware_init();
    // remote_setup();
    // ekf_init();
    // uart_init();
    motor_init();
    // motor_calibration();
    // pinMode(LED, OUTPUT);
    // pinMode(LED_BUILTIN, OUTPUT);
    // digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("[SYSTEM READY]");
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
    debug_data();

// //SLOWER LOOP WITH DELAY
//   unsigned long currentMillis = millis();
//   if (currentMillis - prevMillis >= 20) {
//     prevMillis = currentMillis;
//     // telemetry_data();
//     // debug_data();
//   }
}