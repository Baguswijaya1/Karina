#ifndef DATASTREAM
#define DATASTREAM

#include<Arduino.h>

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
    TELEMETRY.print("| Motor1_PWM:"); TELEMETRY.print(motor1_pwm);
    TELEMETRY.print("| Motor2_PWM:"); TELEMETRY.print(motor2_pwm);
    TELEMETRY.print("| Motor3_PWM:"); TELEMETRY.print(motor3_pwm);
    TELEMETRY.print("| Motor4_PWM:"); TELEMETRY.println(motor4_pwm);
}

#endif