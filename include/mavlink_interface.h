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

extern bool arming;
extern float roll, pitch, yaw;


// STATE DATA (isi ini dari kode flight-control-mu yang sebenarnya)
struct FlightData {
  bool  armed = false;
  float roll  = 0.0f;   // radian
  float pitch = 0.0f;   // radian
  float yaw   = 0.0f;   // radian
  float rollspeed  = 0.0f; // rad/s (opsional, boleh 0 kalau belum ada)
  float pitchspeed = 0.0f;
  float yawspeed   = 0.0f;
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
    if (now - last_heartbeat_ms >= HEARTBEAT_INTERVAL_MS) {
      last_heartbeat_ms = now;
      send_heartbeat();
    }   
    if (now - last_attitude_ms >= ATTITUDE_INTERVAL_MS) {
      last_attitude_ms = now;
      send_attitude();
    }   
    process_mavlink_input(); // baca pesan masuk dari QGC (penting untuk tahap PID nanti)
}