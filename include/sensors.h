#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

// float alt;
int16_t alt = 0;   // dalam cm
uint16_t lidar_strength = 0;  // signal strength (opsional, untuk validasi kualitas data)
bool lidar_data_valid = false; // flag apakah data terakhir valid

void lidar_init(){
    Serial1.begin(115200);
    Serial2.println("Lidar ready");
}

void read_lidar() {
    static uint8_t buf[9];
    static uint8_t idx = 0;

    while (Serial1.available()) {
        uint8_t byte_in = Serial1.read();

        // Cari header 2x 0x59 di awal frame
        if (idx == 0) {
            if (byte_in == 0x59) {
                buf[idx++] = byte_in;
            }
            // kalau bukan 0x59, byte dibuang (belum mulai frame)
            continue;
        } 
        else if (idx == 1) {
            if (byte_in == 0x59) {
                buf[idx++] = byte_in;
            } else {
                idx = 0; // header salah, reset pencarian
            }
            continue;
        }

        // Header sudah valid (2 byte 0x59 0x59), lanjut isi buffer
        buf[idx++] = byte_in;

        if (idx >= 9) {
            // Frame lengkap (9 byte), verifikasi checksum
            uint8_t checksum = 0;
            for (uint8_t i = 0; i < 8; i++) {
                checksum += buf[i];
            }

            if (checksum == buf[8]) {
                // Checksum valid, parse data
                alt = buf[2] | (buf[3] << 8);
                lidar_strength = buf[4] | (buf[5] << 8);
                lidar_data_valid = true;
            } else {
                lidar_data_valid = false;
            }

            idx = 0; // reset untuk frame berikutnya
        }
    }
}