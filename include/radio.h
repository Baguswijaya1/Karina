#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <sbus.h>

#define THROTTLE_SAFETY_THRESHOLD 1100 // Batas aman throttle (sedikit di atas 1000)
#define LED_INDICATOR_PIN 2

#define FMODE_LIMIT1 1300
#define FMODE_LIMIT2 1600

bfs::SbusRx sbus_rx(&Serial8);

bfs::SbusData data;

int16_t ch_roll, ch_pitch, ch_throttle, ch_yaw, ch_fmode;

bool signal_lost = false;
extern bool arming;
bool alt_hold_mode = false;

bool flight_mode[3] = {0,0,0};

unsigned long arrival_time;

void failsafe() {
    // ch_roll = 1500;
    // ch_pitch = 1500;
    // ch_throttle = 1000;
    // ch_yaw = 1500;
    // arming = false;
    Serial2.println(
        "FAILSAFE TRIGGERED\nFAILSAFE TRIGGERED\nFAILSAFE TRIGGERED\nFAILSAFE TRIGGERED\n");
}

void remote_setup() {
    Serial.println("setting up remote");
    Serial2.println("waiting for remote");
    sbus_rx.Begin();

    while (!sbus_rx.Read()) {
    delay(10);
    }

    if (sbus_rx.Read()) {
        data = sbus_rx.data();
        arming = data.ch[4] > 1500 ? 1 : 0;
        signal_lost = data.lost_frame;

        int temp_throttle = data.ch[2] * 0.611f + 895; 


        if (arming) {
            Serial.println("Please disarm the remote for safety");
            while (arming) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    arming = data.ch[4] > 1500 ? 1 : 0;
                    // signal_lost = data.lost_frame;
                }
                delay(10);
            }
        }

        if (sbus_rx.Read()) data = sbus_rx.data();
        temp_throttle = data.ch[2] * 0.611f + 895;

        if (temp_throttle > THROTTLE_SAFETY_THRESHOLD) {
             Serial.println("SAFETY: Lower Throttle to 0!");
             while (temp_throttle > THROTTLE_SAFETY_THRESHOLD) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    temp_throttle = data.ch[2] * 0.611f + 895;
                }
                delay(10);
             }
        }    

        if (signal_lost) {
            Serial.println("Signal lost, please reconnect");
            while (signal_lost) {
                if (sbus_rx.Read()) {
                    data = sbus_rx.data();
                    arming = data.ch[4] > 1500 ? 1 : 0;
                    signal_lost = data.lost_frame;
                }
            }
        }
    }

    Serial.println("Remote setup complete");
}

void remote_loop() {
  if (sbus_rx.Read()) {
    arrival_time = millis();
    data = sbus_rx.data();
    signal_lost = data.lost_frame;

    if (signal_lost) {
      failsafe();
      return;
    }

    // for (int i=0;i<5;i++) {
    //     Serial.print(data.ch[i]);
    //     Serial.print(" ");
    // }
    // Serial.println();

    ch_roll = data.ch[0] * 0.611f + 895;
    ch_pitch = data.ch[1] * 0.611f + 895;
    ch_throttle = data.ch[2] * 0.611f + 895;
    ch_yaw = data.ch[3] * 0.611f + 895;
    ch_fmode = data.ch[6] * 0.611f + 895;

    ch_roll = constrain(ch_roll, 1000, 2000);
    ch_pitch = constrain(ch_pitch, 1000, 2000);
    ch_throttle = constrain(ch_throttle, 1000, 2000);
    ch_yaw = constrain(ch_yaw, 1000, 2000);
    ch_fmode = constrain(ch_fmode, 1000, 2000);

    flight_mode[0] = ch_fmode <= FMODE_LIMIT1; // stablize
    flight_mode[1] = ch_fmode > FMODE_LIMIT1; // althold
    // flight_mode[2] = ch_fmode > FMODE_LIMIT2; // poshold


    bool switch_arm_position = data.ch[4] > 1500; // Cek posisi switch fisik
    
    if (switch_arm_position) {
        // Jika sistem belum ARM, kita cek apakah aman untuk ARM?
        if (!arming) {
            if (ch_throttle < THROTTLE_SAFETY_THRESHOLD) {  
                // Aman, throttle rendah, izinkan arming
                arming = true;
            } else {
                // TIDAK AMAN, throttle tinggi, tolak arming
                arming = false;
            }
        }
        // Jika sudah arming (arming == true), biarkan tetap true walaupun throttle naik (sedang terbang)
    } else {
        // Switch mati, maka sistem disarm
        if (arming) arming = false;
    }

    if (!arming) {
        ch_throttle = 1000; 
    }
    // Indikator LED (Nyala jika Armed)
    if (arming) { digitalWrite(LED_INDICATOR_PIN, HIGH); }
    else { digitalWrite(LED_INDICATOR_PIN, LOW); }
    
  }
}


float roll_scaler() {
    return (ch_roll - 1500) / 500.0f;
}

float throttle_scaler() {
    return (ch_throttle - 1500) / 500.0f;
}

float pitch_scaler() {
    return -(ch_pitch - 1500) / 500.0f;
}

float yaw_scaler() {
    return (ch_yaw - 1500) / 500.0f;
}

#endif
