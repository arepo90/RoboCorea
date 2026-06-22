#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"

// ─── Comms ────────────────────────────────────────────────────────────────────
// Binary framed UART protocol between the ESP32 and the Jetson.
//   Frame: [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]
//   CRC  : XOR of TYPE, LEN_H, LEN_L and every payload byte.
//
// tick() must be called continuously from the comms task; incoming-frame
// callbacks are dispatched from within it.

using ArmJointsCallback    = void(*)(const ArmJointsPayload&);
using SensorEnableCallback = void(*)(uint8_t mask);
using EstopCallback        = void(*)(bool active);
using PpmCalibCallback     = void(*)(const PpmCalibPayload&);
using ArmLifecycleCallback = void(*)(bool arm);   // true = arm/init, false = disarm
using ArmModeCallback      = void(*)(uint8_t mode);
using TractionCmdCallback  = void(*)(const TractionCmdPayload&);  // external /cmd_vel drive
using GripperCallback      = void(*)(float rate); // gripper open/close rate (+open/−close)

class Comms {
public:
    static void begin();
    static void tick();

    // ── Outgoing ────────────────────────────────────────────────────────────
    static void sendTelemetry(const TelemetryPayload& p);
    static void sendMagData(const MagData& mag);
    static void sendEncoderExt(float fl, float fr, float rl, float rr);
    static void sendVescStatus(const VescStatusPayload& v);
    static void sendOdriveStatus(const OdriveStatusPayload& o);
    static void sendLktechStatus(const LktechStatusPayload& l);
    static void sendZe300Status(const Ze300StatusPayload& z);
    static void sendOdriveError(const OdriveErrorPayload& e);
    static void sendStatus(const SystemStatus& status);
    static void sendArmLifecycle(const ArmLifecyclePayload& p);
    static void sendBoardIdentity();

    // ── Callback registration ─────────────────────────────────────────────────
    static void onArmJoints(ArmJointsCallback cb)      { s_cb_arm = cb; }
    static void onSensorEnable(SensorEnableCallback cb) { s_cb_sensor = cb; }
    static void onEstop(EstopCallback cb)               { s_cb_estop = cb; }
    static void onPpmCalib(PpmCalibCallback cb)         { s_cb_ppm_calib = cb; }
    static void onArmLifecycle(ArmLifecycleCallback cb) { s_cb_arm_life = cb; }
    static void onArmMode(ArmModeCallback cb)           { s_cb_arm_mode = cb; }
    static void onTraction(TractionCmdCallback cb)      { s_cb_traction = cb; }
    static void onGripper(GripperCallback cb)           { s_cb_gripper = cb; }

    static bool isConnected();

private:
    static void sendFrame(uint8_t type, const uint8_t* payload, uint16_t len);
    static uint8_t computeCRC(uint8_t type, uint16_t len, const uint8_t* payload);
    static void processFrame(uint8_t type, const uint8_t* buf, uint16_t len);

    enum class RxState : uint8_t { SOF0, SOF1, TYPE, LEN_H, LEN_L, PAYLOAD, CRC };

    static RxState  s_rx_state;
    static uint8_t  s_rx_type;
    static uint16_t s_rx_len;
    static uint16_t s_rx_idx;
    static uint8_t  s_rx_buf[PROTO_MAX_PAYLOAD];
    static uint8_t  s_rx_crc;
    static uint32_t s_last_rx_ms;

    static ArmJointsCallback    s_cb_arm;
    static SensorEnableCallback s_cb_sensor;
    static EstopCallback        s_cb_estop;
    static PpmCalibCallback     s_cb_ppm_calib;
    static ArmLifecycleCallback s_cb_arm_life;
    static ArmModeCallback      s_cb_arm_mode;
    static TractionCmdCallback  s_cb_traction;
    static GripperCallback      s_cb_gripper;
};
