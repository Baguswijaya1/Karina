#ifndef DATASTREAM
#define DATASTREAM

#include<Arduino.h>
#include "flight_controller.h"

#define USB Serial
#define TELEMETRY Serial2

void datastream_setup(){
    USB.begin(115200);
    TELEMETRY.begin(57600);
};

void datastream_via_wire() {
    USB.print("Act_Roll:");
    USB.print(roll);
    USB.print("\tAct_Pitch:");
    USB.print(pitch);
    USB.print("\tAct_Yaw:");
    USB.print(yaw);
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
    USB.print("\tArming:");
    USB.print(arming);
    USB.print("\tMotor1_PWM:");
    USB.print(motor1_pwm);
    USB.print("\tMotor2_PWM:");
    USB.print(motor2_pwm);
    USB.print("\tMotor3_PWM:");
    USB.print(motor3_pwm);
    USB.print("\tMotor4_PWM:");
    USB.print(motor4_pwm);

    // USB.print("\tError_yaw:");
    // USB.print(error_yaw);
    USB.println();
}

void datastream_via_telem() {
    // TELEMETRY.println();
    TELEMETRY.print("Roll:"); TELEMETRY.print(roll);
    TELEMETRY.print("| pitch:");TELEMETRY.print(pitch);
    TELEMETRY.print("| yaw:"); TELEMETRY.print(yaw);

    TELEMETRY.print("| Arming:"); TELEMETRY.print(arming);
    TELEMETRY.print("| M1:"); TELEMETRY.print(motor1_pwm);
    TELEMETRY.print("| M2:"); TELEMETRY.print(motor2_pwm);
    TELEMETRY.print("| M3:"); TELEMETRY.print(motor3_pwm);
    TELEMETRY.print("| M4:"); TELEMETRY.print(motor4_pwm);

    TELEMETRY.print("| Kp_roll: "); TELEMETRY.print(gain.roll);
    TELEMETRY.print("| Kd_roll: "); TELEMETRY.print(gain.p);
    TELEMETRY.print("| Kp_pitch: "); TELEMETRY.print(gain.pitch);
    TELEMETRY.print("| Kd_pitch: "); TELEMETRY.println(gain.q);
    
}

void tune_gain(){
    while (Serial2.available()){
        char selector = Serial2.read();
        if (selector == '\r' || selector == '\n' || selector == ' ') continue;
        if (true){
            switch (selector)
            {
            case 'q': gain.roll += 0.1; break;
            case 'a': gain.roll -= 0.1; break;
            case 'w': gain.p += 0.1; break;
            case 's': gain.p -= 0.1; break;
            case 'e': gain.pitch += 0.1; break;
            case 'd': gain.pitch -= 0.1; break;
            case 'r': gain.q += 0.01; break;
            case 'f': gain.q -= 0.01; break;
            case 't': gain.yaw += 0.1; break;
            case 'g': gain.yaw -= 0.1; break;
            case 'y': gain.r += 0.1; break;
            case 'h': gain.r -= 0.1; break;
            default: break;
            }
        }
    }
}

#endif