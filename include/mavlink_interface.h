#pragma once

#include <Arduino.h>
#include <common/mavlink.h>
#include <imu.h>
#include <numbers>

#define GCS_SERIAL Serial2
#define GCS_BAUD 57600

// Identitas MAVLink 
static const uint8_t MAVLINK_SYSTEM_ID    = 1;
static const uint8_t MAVLINK_COMPONENT_ID = MAV_COMP_ID_AUTOPILOT1;

static const uint8_t VEHICLE_TYPE = MAV_TYPE_QUADROTOR; 

static const unsigned long HEARTBEAT_INTERVAL_MS = 1000; // wajib ~1 Hz
static const unsigned long ATTITUDE_INTERVAL_MS  = 100;  // 10 Hz
static const unsigned long ALTITUDE_INTERVAL_MS = 100;
unsigned long last_altitude_ms = 0;

static const unsigned long RANGEFINDER_INTERVAL_MS = 50; // 20 Hz
unsigned long last_rangefinder_ms = 0;

extern bool arming;
extern float roll, pitch, yaw;
extern int16_t alt;


// STATE DATA (isi ini dari kode flight-control-mu yang sebenarnya)
struct FlightData {
  bool  armed = false;
  float roll  = 0.0f;   // radian
  float pitch = 0.0f;   // radian
  float yaw   = 0.0f;   // radian
  float rollspeed  = 0.0f; // rad/s (opsional, boleh 0 kalau belum ada)
  float pitchspeed = 0.0f;
  float yawspeed   = 0.0f;
  float rangefinder_distance_m = 0.0f;
};

FlightData flight_data;


// TIMER
unsigned long last_heartbeat_ms = 0;
unsigned long last_attitude_ms  = 0;


// HELPER: kirim satu mavlink_message_t ke radio
void mavlink_send(mavlink_message_t *msg) {
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
  GCS_SERIAL.write(buf, len);
}

// heartbeat
void send_heartbeat() {
  mavlink_message_t msg;

  uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  if (flight_data.armed) {
    base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
  }

  uint8_t system_status = flight_data.armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY;

  mavlink_msg_heartbeat_pack(
    MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
    VEHICLE_TYPE,
    MAV_AUTOPILOT_GENERIC,   // bukan ArduPilot/PX4 -> pakai GENERIC
    base_mode,
    0,                        // custom_mode, belum dipakai
    system_status
  );

  mavlink_send(&msg);
}

// send attitude
void send_attitude() {
  mavlink_message_t msg;

  mavlink_msg_attitude_pack(
    MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
    millis(),                  // time_boot_ms
    flight_data.roll,
    flight_data.pitch,
    flight_data.yaw,
    flight_data.rollspeed,
    flight_data.pitchspeed,
    flight_data.yawspeed
  );

  mavlink_send(&msg);
}

void send_rangefinder() {
  mavlink_message_t msg;

  uint16_t current_distance_cm = (uint16_t)(flight_data.rangefinder_distance_m * 100.0f);

  const float quaternion[4] = {0, 0, 0, 0}; // 0 = tidak dipakai (orientasi bukan CUSTOM)

  mavlink_msg_distance_sensor_pack(
    MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
    millis(),                        // time_boot_ms
    5,                                // min_distance (cm) - sesuaikan datasheet sensormu
    4000,                             // max_distance (cm) - sesuaikan datasheet sensormu
    current_distance_cm,              // current_distance (cm)
    MAV_DISTANCE_SENSOR_LASER,        // type: ganti sesuai jenis sensor (lihat catatan di bawah)
    0,                                 // id: 0 kalau cuma satu sensor
    MAV_SENSOR_ROTATION_PITCH_270,    // orientation: 270 = menghadap ke bawah (umum untuk altitude hold)
    255,                               // covariance: 255 = UINT8_MAX = tidak diketahui
    0.0f,                              // horizontal_fov: 0 kalau tidak diketahui
    0.0f,                              // vertical_fov: 0 kalau tidak diketahui
    quaternion,
    0                                  // signal_quality: 0 = unknown/unset
  );

  mavlink_send(&msg);
}

// // send altitude
// void send_altitude() {
//   mavlink_message_t msg;

//   mavlink_msg_altitude_pack(
//     MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
//     (uint64_t)millis() * 1000ULL,   // time_usec — boleh pakai waktu sejak boot
//     NAN,                             // altitude_monotonic — nggak dipakai
//     NAN,                             // altitude_amsl — isi kalau ada data GPS MSL
//     NAN,                             // altitude_local
//     flight_data.altitude_relative,   // altitude_relative — ini yang biasanya ditampilkan QGC
//     NAN,                             // altitude_terrain
//     NAN                              // bottom_clearance
//   );

//   mavlink_send(&msg);
// }

// ---------------------------------------------------------------------
// ==== FUTURE: PARAMETER PROTOCOL (PID gain) ====
// Nanti tinggal:
//   1. Definisikan array/struct parameter (nama max 16 char, contoh "PID_ROLL_P").
//   2. Di process_mavlink_input(), tangani case MAVLINK_MSG_ID_PARAM_REQUEST_LIST
//      -> loop kirim mavlink_msg_param_value_pack() untuk tiap parameter.
//   3. Tangani case MAVLINK_MSG_ID_PARAM_REQUEST_READ -> kirim satu PARAM_VALUE
//      sesuai param_id atau param_index yang diminta.
//   4. Tangani case MAVLINK_MSG_ID_PARAM_SET -> update nilai gain di RAM,
//      lalu balas dengan PARAM_VALUE supaya QGC tahu berhasil.
// Struktur di bawah ini (process_mavlink_input) sudah siap nampung logic itu.
// ---------------------------------------------------------------------

void process_mavlink_input() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (GCS_SERIAL.available() > 0) {
    uint8_t byte = GCS_SERIAL.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
      switch (msg.msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT:
          // heartbeat dari GCS, tidak perlu action khusus untuk saat ini
          break;

        // TODO (tahap PID):
        // case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        // case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        // case MAVLINK_MSG_ID_PARAM_SET:

        default:
          break;
      }
    }
  }
}

float degtoRad(float deg){
    return deg * (M_PI / 180.0);
}

void send_mavlink_msg(){
    unsigned long now  = millis();
    // --- TODO: update flight_data dari state flight-controller-mu di sini ---
    flight_data.armed = arming;
    flight_data.roll  = degtoRad(roll);
    flight_data.pitch = degtoRad(pitch);
    flight_data.yaw   = degtoRad(yaw);
    flight_data.rangefinder_distance_m = alt;
    if (now - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
      last_heartbeat_ms = now;
      send_heartbeat();
    }   
    if (now - last_attitude_ms >= ATTITUDE_INTERVAL_MS) {
      last_attitude_ms = now;
      send_attitude();

      if (now - last_rangefinder_ms >= RANGEFINDER_INTERVAL_MS) {
        send_rangefinder();
        last_rangefinder_ms = now;
      }
    }   

    // // di dalam send_mavlink_msg():
    // if (now - last_altitude_ms >= ALTITUDE_INTERVAL_MS) {
    //   last_altitude_ms = now;
    //   send_altitude();
    // }

    
    process_mavlink_input(); // baca pesan masuk dari QGC (penting untuk tahap PID nanti)
}