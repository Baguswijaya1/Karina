#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#define MIN_PWM 1000
#define MAX_PWM 1800
#define MAX_ROLL 35.0f
#define MAX_PITCH 35.0f
#define MAX_YAW 35.0f
#define YAW_RATE_DEADBAND 50

#include <Arduino.h>
#include <math.h>
#include <radio.h>

extern float gyro_x, gyro_y, gyro_z;
extern float gyro_x_control, gyro_y_control, gyro_z_control;
float altitude;
float alt_last, alt_now, alt_vel;

float setpoint_roll, setpoint_pitch, setpoint_yaw, target_alt;
float setpoint_roll_last, setpoint_pitch_last, setpoint_yaw_last, target_alt_last;
float setpoint_roll_now, setpoint_pitch_now, setpoint_yaw_now, target_alt_now;
float setpoint_roll_rate, setpoint_pitch_rate, setpoint_yaw_rate;
float yaw_rate_sp = 0.0f;   // target yaw rate dari stick


extern int16_t ch_throttle;
extern float motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm;
extern float u1, u2, u3, u4;
float motor_speed_squared[4];


extern float error_roll, error_pitch, error_yaw;
float error_roll_rate, error_pitch_rate, error_yaw_rate;
float p_roll, d_roll, p_pitch, d_pitch, p_yaw, d_yaw;
float prev_error_roll, prev_error_pitch, prev_error_yaw;

float locked_yaw = 0.0f;
float null_yaw = false;
float prev_null_yaw = false;

uint8_t t_now, t_last, dt;

// copter lama
const double A_invers[4][4] = {{292600,  1300300,  1300300, -6283300},
                               {292600, -1300300,  1300300, 6283300},
                               {292600, -1300300, -1300300,  -6283300},
                               {292600,  1300300, -1300300, 6283300}};

// tuning bang ikhlas
// struct Gains {
//     float roll = 5.477;//5.916; //5.477 (FINAL)
//     float p = 3.027;//4.144; //3.027 (FINAL)
//     float pitch = 5.196; //4.8 //2.1 //5.196 (FINAL)
//     float q = 3.341; //1.05 //2.7 //max 1.7 dengan p 3.00 //1.3 oke //3.341 (FINAL)
//     float yaw = 2.000; // 6.324
//     float r = 1.079; // 1.160
// } gain;

struct Gains {
    float roll = 5.9; //5.477;  // 5.477     6.38
    float p = 2.0; //2.482; // 1.6           2
    float pitch = 5.9; //5.291; // 5.5       6.4
    float q = 2.0; //2.676; // -1.6         -2
    float yaw = 3.0; //3.0;                
    float r = 1.0;//1.079;
    float max_rate_y = 100.0f;  // tambahkan ini
    float iy = 0.0006; //trial
} gain;

float set_yaw(){
    null_yaw = (ch_yaw >= 1450 && ch_yaw <= 1550);
    if (null_yaw && !prev_null_yaw){
        // enter heading lock
        locked_yaw = yaw;
        prev_null_yaw = null_yaw;
        Serial2.println("\nheading lock\n");
        return locked_yaw;
    } else if (!null_yaw && prev_null_yaw){
        // exit heading lock, manual control
        prev_null_yaw = null_yaw;
        Serial2.println("\nyawing\n");
        return (yaw_scaler() * MAX_YAW) + yaw;
    } else if (null_yaw){
        // locked_yaw = yaw;
        prev_null_yaw = null_yaw;
        return locked_yaw;
    } else {
        // in manual control
        return (yaw_scaler() * MAX_YAW) + yaw;
    }
}

void yaw_compute() {
    // yaw stick = kondisi kontrol manual
    const bool yaw_stick = (abs(ch_yaw - 1500) > YAW_RATE_DEADBAND);

    if (yaw_stick) {
        // Stick position → desired yaw rate
        yaw_rate_sp = constrain(
            map(ch_yaw, 1000, 2000, -gain.max_rate_y, gain.max_rate_y),
            -gain.max_rate_y,
            gain.max_rate_y
        );
    } else {
        yaw_rate_sp = 0.0f;
    }
    float yaw_rate_actual = -gyro_z;

    // Yaw-rate error
    error_yaw_rate = yaw_rate_actual - yaw_rate_sp;
}

void set_control_reference() {
    setpoint_roll = roll_scaler() * MAX_ROLL;
    setpoint_pitch = pitch_scaler() * MAX_PITCH;
    setpoint_yaw = set_yaw();
    if (setpoint_yaw > 180.0f) { setpoint_yaw -= 360.0f; }
    if (setpoint_yaw < -180.0f) { setpoint_yaw += 360.0f; }
    yaw_compute();
}

void drone_controller(){
  t_now = micros();
  dt = t_now - t_last;
  t_last = t_now;
    // derivative error properties
    setpoint_roll_last = setpoint_roll_now;
    setpoint_roll_now = setpoint_roll;

    setpoint_pitch_last = setpoint_pitch_now;
    setpoint_pitch_now = setpoint_pitch;

    setpoint_yaw_last = setpoint_yaw_now;
    setpoint_yaw_now = setpoint_yaw;

    if (dt > 0) {
        setpoint_roll_rate = (setpoint_roll_now - setpoint_roll_last) / dt;
        setpoint_pitch_rate = (setpoint_pitch_now - setpoint_pitch_last) / dt;
        // setpoint_yaw_rate = (setpoint_yaw_now - setpoint_yaw_last) / dt;
        setpoint_yaw_rate = yaw_rate_sp;
    } else {
        setpoint_roll_rate = 0.0f;
        setpoint_pitch_rate = 0.0f;
        setpoint_yaw_rate = 0.0f;
    }

    alt_last = alt_now;
    alt_now = altitude;
    alt_vel = (alt_now - alt_last) / dt;
    

  // proportional error
    error_roll = roll - setpoint_roll;
    error_pitch = pitch - setpoint_pitch;
    error_yaw = yaw - setpoint_yaw;
    if (error_yaw < -180) error_yaw += 360;
    if (error_yaw > 180) error_yaw -= 360;    

  // derivative error
    error_roll_rate = -gyro_y; // x y dibalik
    error_pitch_rate = -gyro_x;
    // error yaw rate sdh dihitung di yaw_compute

  // u = -k * (state - setpoint)
    p_roll = -gain.roll * error_roll;
    d_roll = -gain.p * error_roll_rate;
    p_pitch = -gain.pitch * error_pitch;
    d_pitch = -gain.q * error_pitch_rate;
    p_yaw = -gain.yaw * error_yaw;       
    d_yaw = -gain.r   * error_yaw_rate;    

  // action vector
    u1 = 0.0f;
    u2 = (p_roll + d_roll)/10'000'000.0f;
    u3 = (p_pitch + d_pitch)/10'000'000.0f;
    u4 = (p_yaw + d_yaw)/10'000'000.0f;
    // u4 = 0; // abaikan yaw

    motor_speed_squared[0] = ((A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4));
    motor_speed_squared[1] = ((A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4));
    motor_speed_squared[2] = ((A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4));
    motor_speed_squared[3] = ((A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4)); 
    
    motor1_pwm = ch_throttle + (int)(motor_speed_squared[0]);
    motor2_pwm = ch_throttle + (int)(motor_speed_squared[1]);
    motor3_pwm = ch_throttle + (int)(motor_speed_squared[2]);
    motor4_pwm = ch_throttle + (int)(motor_speed_squared[3]);   
    
    motor1_pwm = constrain(motor1_pwm, MIN_PWM, MAX_PWM);
    motor2_pwm = constrain(motor2_pwm, MIN_PWM, MAX_PWM);
    motor3_pwm = constrain(motor3_pwm, MIN_PWM, MAX_PWM);
    motor4_pwm = constrain(motor4_pwm, MIN_PWM, MAX_PWM);
}


#endif


// Proll 6.38 
// Droll 2 2.1

// Ppitch 6.3 6.4
// Dpitch -2 -1.84

// Pyaw 1  0.5 0.7 0.8
// Dyaw 0.2 0.3 0.2