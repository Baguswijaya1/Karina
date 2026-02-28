#ifndef BNO055_H
#define BNO055_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// ================= KONFIGURASI =================
// Set 'true' jika ingin kalibrasi ulang (Putar-putar sensor)
// Set 'false' untuk pemakaian normal
bool CALIBRATION_MODE = false;

// Deklarasi objek BNO
// Menggunakan Wire default (SDA Pin 18, SCL Pin 19 pada Teensy 4.1)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Variabel Data Sensor (Pastikan variabel ini didefinisikan di main.cpp/ino)
extern float gyro_x, gyro_y, gyro_z;
extern float accel_x, accel_y, accel_z;
extern float mag_x, mag_y, mag_z;
extern float actual_roll, actual_pitch, actual_yaw;
extern float gyro_x_control, gyro_y_control, gyro_z_control;
extern float roll, pitch, yaw;

// --- EEPROM HELPERS (Teensy Version) ---
void _save_calib() {
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  
  // Teensy EEPROM tidak butuh commit() atau begin()
  EEPROM.put(0, newCalib); 
  
  Serial.println("[BNO] Kalibrasi Tersimpan di EEPROM Teensy!");
}

void _load_calib() {
  adafruit_bno055_offsets_t calibData;
  
  // Baca langsung dari EEPROM
  EEPROM.get(0, calibData);
  
  // Cek validitas data (Teensy EEPROM baru biasanya 0xFF atau 0x00)
  if (calibData.accel_radius == -1 || calibData.accel_radius == 0xFFFF) {
    Serial.println("[BNO] EEPROM Kosong/Invalid (Belum dikalibrasi).");
  } else {
    bno.setSensorOffsets(calibData);
    Serial.println("[BNO] Kalibrasi Dimuat dari EEPROM.");
  }
}

// --- INITIALIZATION ---
void imu_hardware_init() {
  Serial.println("[BNO] Menunggu sensor boot...");
  delay(1000); 
  Wire.begin();
  Wire.setClock(400000); 
  if (!bno.begin(OPERATION_MODE_NDOF)) { 
    Serial.println("BNO Error! Cek kabel (SDA=18, SCL=19)."); 
    while(1) {
       delay(100); // Stuck loop blink
    }
  }
  
  bno.setExtCrystalUse(true);

  if (CALIBRATION_MODE) {
    Serial.println("=== MODE KALIBRASI: Gerakkan Sensor ===");
    while (true) {
      uint8_t sys, g, a, m;
      bno.getCalibration(&sys, &g, &a, &m);
      imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

       
      // Print status kalibrasi (0=Uncalibrated, 3=Fully Calibrated)
      Serial.println("[MODE KALIB]");
      Serial.printf("accX: "); Serial.print(accel.x());
      Serial.printf("| accY: "); Serial.print(accel.y());
      Serial.printf("| accZ: "); Serial.print(accel.z());
      Serial.printf("| sys:"); Serial.print(sys);
      Serial.printf("| gyro:"); Serial.print(g);
      Serial.printf("| accel:"); Serial.print(a);
      Serial.printf("| magn:"); Serial.println(m);
      
      if (sys == 3 && g == 3 && a == 3 && m == 3) {
        _save_calib();
        Serial.println("KALIBRASI SELESAI! Ubah CALIBRATION_MODE ke false & Upload ulang.");
        while(1) delay(100); // Stop here
      }
      delay(100);
    }
  } else {
    _load_calib();
  }
}

void imu_hardware_read() {
  // Ambil Data Vector
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // KONVERSI PENTING UNTUK EKF:
  // // Gyro BNO055 defaultnya Degrees/s -> Harus diubah ke Radians/s
  // gyro_x = -gyro.y() * DEG_TO_RAD;
  // gyro_y = -gyro.x() * DEG_TO_RAD;
  // gyro_z = -gyro.z() * DEG_TO_RAD;

  gyro_x = -gyro.y();
  gyro_y = -gyro.x();
  gyro_z = -gyro.z();

  // Accel m/s^2 (Sudah ok)
  accel_x = accel.y();
  accel_y = accel.x();
  accel_z = accel.z();

  const float MAG_ALPHA = 0.05; 

  static float filt_mag_x = 0;
  static float filt_mag_y = 0;
  static float filt_mag_z = 0;
  static bool first_run_mag = true;

  float raw_mx = mag.y();
  float raw_my = mag.x();
  float raw_mz = mag.z();

  if (first_run_mag) {
    filt_mag_x = raw_mx;
    filt_mag_y = raw_my;
    filt_mag_z = raw_mz;
    first_run_mag = false;
  }

  // 3. Rumus Low Pass Filter: Output = (Alpha * Raw) + ((1-Alpha) * Previous)
  filt_mag_x = (MAG_ALPHA * raw_mx) + ((1.0 - MAG_ALPHA) * filt_mag_x);
  filt_mag_y = (MAG_ALPHA * raw_my) + ((1.0 - MAG_ALPHA) * filt_mag_y);
  filt_mag_z = (MAG_ALPHA * raw_mz) + ((1.0 - MAG_ALPHA) * filt_mag_z);

  // 4. Masukkan ke variabel global
  mag_x = filt_mag_x;
  mag_y = filt_mag_y;
  mag_z = filt_mag_z;

  roll = 0.8*gyro_x + 0.2*accel_x;
  pitch = 0.8*gyro_y + 0.2*accel_y;
  yaw = 0.8*gyro_z + 0.2*accel_z;
}

#endif
