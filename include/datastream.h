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
    USB.print(roll); USB.print(",");
    USB.print(pitch); USB.print(",");
    USB.print(yaw); USB.print(",");
    USB.println(arming);
}

void datastream_via_telem() {
    // TELEMETRY.println();
    TELEMETRY.print("Roll:"); TELEMETRY.print(roll);
    TELEMETRY.print("| pitch:");TELEMETRY.print(pitch);
    TELEMETRY.print("| yaw:"); TELEMETRY.print(yaw);

    TELEMETRY.print("| Arm:"); TELEMETRY.print(arming);
    // TELEMETRY.print("| M1:"); TELEMETRY.print(motor1_pwm);
    // TELEMETRY.print("| M2:"); TELEMETRY.print(motor2_pwm);
    // TELEMETRY.print("| M3:"); TELEMETRY.print(motor3_pwm);
    // TELEMETRY.print("| M4:"); TELEMETRY.print(motor4_pwm);

    TELEMETRY.print("| Proll: "); TELEMETRY.print(gain.roll);
    TELEMETRY.print("| Droll: "); TELEMETRY.print(gain.p);
    TELEMETRY.print("| Ppitch: "); TELEMETRY.print(gain.pitch);
    TELEMETRY.print("| Dpitch: "); TELEMETRY.print(gain.q);

    TELEMETRY.print("| Pyaw: "); TELEMETRY.print(gain.yaw);
    TELEMETRY.print("| Dyaw: "); TELEMETRY.print(gain.r);
    TELEMETRY.print("| yaw_sp: "); TELEMETRY.println(setpoint_yaw);
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