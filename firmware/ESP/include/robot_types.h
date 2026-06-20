#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ─── Operating mode ───────────────────────────────────────────────────────────
enum class RobotMode : uint8_t {
    INIT    = 0,  // startup / hardware init
    STANDBY = 1,  // idle, waiting for the RC link
    NORMAL  = 2,  // RC drives the tracks (and any flippers bound in the row)
    ARM     = 3,  // tracks stopped; arm follows joint commands from the PC
    ESTOP   = 4,  // all outputs neutralised; cleared by PC or hardware
    FLIPPER = 5,  // RC drives flippers only (position control)
};

// ─── Channel function (keybind system) ───────────────────────────────────────
// What a PPM channel can be bound to. Values match the GUI enum.
enum class ChannelFunction : uint8_t {
    NONE          = 0,
    TRACTION_FWD  = 1,   // forward / back
    TRACTION_TURN = 2,   // left / right differential
    FLIPPER_ALL   = 3,   // all four flippers together
    FLIPPER_FL    = 4,   // front-left
    FLIPPER_FR    = 5,   // front-right
    FLIPPER_RL    = 6,   // rear-left
    FLIPPER_RR    = 7,   // rear-right
    ARM_FWD       = 8,   // generic arm (legacy)
    ESTOP         = 9,   // virtual e-stop
    ARM_X         = 10,  // arm Cartesian +X
    ARM_Y         = 11,  // arm Cartesian +Y
    ARM_Z         = 12,  // arm Cartesian +Z
    ARM_PITCH     = 13,
    ARM_YAW       = 14,
    ARM_ROLL      = 15,
    GRIPPER       = 16,  // gripper open/close (forwarded to the PC)
};

inline bool isArmFunction(ChannelFunction fn) {
    return fn == ChannelFunction::ARM_FWD   ||
           fn == ChannelFunction::ARM_X     ||
           fn == ChannelFunction::ARM_Y     ||
           fn == ChannelFunction::ARM_Z     ||
           fn == ChannelFunction::ARM_PITCH ||
           fn == ChannelFunction::ARM_YAW   ||
           fn == ChannelFunction::ARM_ROLL  ||
           fn == ChannelFunction::GRIPPER;
}

inline bool isFlipperFunction(ChannelFunction fn) {
    return fn >= ChannelFunction::FLIPPER_ALL && fn <= ChannelFunction::FLIPPER_RR;
}

// Keybind table: 3 Ch5 lever positions × 5 channel slots (Ch1,Ch2,Ch3,Ch4,Ch6).
struct KeybindTable {
    ChannelFunction map[3][5];
};

// ─── PPM / RC ────────────────────────────────────────────────────────────────
struct PPMFrame {
    uint16_t ch[PPM_CHANNELS];  // raw µs, index 0 = channel 1
    uint32_t timestamp_ms;
    bool     valid;
};

// ─── Arm ─────────────────────────────────────────────────────────────────────
struct ArmJoints {
    float angle_deg[6];   // one per DOF, degrees
    bool  valid;
};

// ─── Magnetometer ─────────────────────────────────────────────────────────────
struct MagData {
    int  x_uT, y_uT, z_uT;
    bool valid;
};

// ─── System status ─────────────────────────────────────────────────────────────
struct SystemStatus {
    RobotMode mode;
    bool      ppm_connected;
    bool      minipc_connected;
    bool      can_ok;
    uint8_t   sensor_mask;
    bool      estop;
    uint32_t  uptime_ms;
};

// ─── Binary protocol payloads (packed, sent/received verbatim over UART) ──────
#pragma pack(push, 1)

struct TelemetryPayload {
    uint8_t  mode;                 // RobotMode value
    uint8_t  flags;                // bit0=ppm_ok bit1=sensors bit2=can_ok bit3=estop
    uint16_t ppm[PPM_CHANNELS];    // raw µs per channel
    int16_t  speed_left;           // track RPM × 10
    int16_t  speed_right;          // track RPM × 10
    int16_t  flipper_angle;        // representative flipper angle (FL) × 10 deg
    uint32_t uptime_ms;
};

struct ArmJointsPayload {
    int16_t joint[6];              // degrees × 100
};

struct SensorEnablePayload {
    uint8_t mask;
};

struct MagPayload {
    int16_t x_uT100;               // µT × 100 (raw counts as sent by legacy)
    int16_t y_uT100;
    int16_t z_uT100;
};

// Four flipper output angles × 10 deg (FL, FR, RL, RR).
struct EncoderExtPayload {
    int16_t flipper_fl_deg10;
    int16_t flipper_fr_deg10;
    int16_t flipper_rl_deg10;
    int16_t flipper_rr_deg10;
};

// 15 bytes: 3 modes (Ch5 positions) × 5 slots (Ch1,Ch2,Ch3,Ch4,Ch6).
struct KeybindPayload {
    uint8_t map[3][5];
};

// Per-VESC status (one frame per controller that broadcasts).
struct VescStatusPayload {
    uint8_t vesc_id;
    int32_t erpm;
    int16_t current_10;            // A × 10
    int16_t duty_1000;             // × 1000
    int16_t temp_fet_10;           // °C × 10
    int16_t temp_motor_10;         // °C × 10
    int16_t v_in_10;               // V × 10
    int32_t tachometer;            // VESC STATUS_5 tachometer (commutation steps,
                                   // signed cumulative). Track wheel odometry is
                                   // derived from the two traction VESCs' counts on
                                   // the Jetson bridge (→ /odom/wheel).
};

// Per-channel RC calibration + global deadband.
struct PpmChannelCalibEntry {
    uint16_t min_us;
    uint16_t neutral_us;
    uint16_t max_us;
};
struct PpmCalibPayload {
    PpmChannelCalibEntry ch[PPM_CHANNELS];
    uint16_t deadband_1000;        // normalised deadband × 1000
};

// ODrive arm joint telemetry. (No temperature over CAN on ODrive v3.6.)
struct OdriveStatusPayload {
    uint8_t joint_idx;             // 0=J1, 1=J2, 2=J3
    int16_t pos_turns_100;         // motor turns × 100
    int16_t vel_turns_s_100;       // turns/s × 100
    int16_t iq_measured_100;       // A × 100
    int16_t bus_voltage_10;        // V × 10
    int16_t bus_current_100;       // A × 100
};

struct OdriveErrorPayload {
    uint8_t  node_id;
    uint64_t motor_error;          // v3.6 motor_error bitfield
};

// LKTech J5/J6 telemetry (parsed from the A4 command acknowledge).
struct LktechStatusPayload {
    uint8_t joint_idx;             // 0=J5, 1=J6
    uint8_t motor_id;
    int8_t  temp_c;
    int16_t iq_100;                // A × 100
    int16_t speed_dps;             // °/s
    int16_t angle_deg;             // raw single-turn motor degrees
    int16_t output_deg_10;         // gear-compensated output × 10
};

// ZE300 J4 telemetry.
struct Ze300StatusPayload {
    uint8_t device_id;
    int8_t  temp_c;
    int16_t iq_1000;               // A × 1000
    int16_t speed_rpm_100;         // rpm × 100
    int16_t single_turn_counts;
    int32_t position_counts;       // multi-turn (from C2 replies)
    int16_t output_deg_10;         // software-zeroed output × 10
};

// Gripper command forwarded to the PC.
struct GripperPayload {
    int16_t value_1000;            // normalised × 1000 (−1000..+1000)
};

// Arm safety lifecycle + fault diagnostics (ESP → PC).
struct ArmLifecyclePayload {
    uint8_t  state;                // 0 UNINIT, 1 INITIALIZING, 2 READY, 3 FAULT
    uint8_t  fault_code;           // 0 none, 1 can_send, 2 motor_cmd
    uint16_t can_fail_count;       // failed arm-send cycles in the current window
    uint16_t motor_fail_count;
    uint8_t  eflg;                 // last MCP2515 EFLG snapshot (0 if N/A)
    uint16_t init_presence_mask;    // bits: 0..2 ODrive J1..J3, 3 ZE300 J4, 4..5 LKTech J5..J6
    uint8_t  operating_mode;        // 0 DEXTERITY, 1 CHASSIS
    uint8_t  active_joint_mask;     // bits 0..5 = J1..J6 actively position-controlled
};

#pragma pack(pop)

// ─── Wire-format guards ───────────────────────────────────────────────────────
// These sizes are part of the protocol contract with the Jetson bridge
// (software/ros2_ws/src/esp32_bridge/esp32_bridge/main_bridge.py). Do not change
// a payload without updating the matching struct.unpack/pack format there.
static_assert(sizeof(TelemetryPayload)   == 24, "TelemetryPayload size");
static_assert(sizeof(MagPayload)         ==  6, "MagPayload size");
static_assert(sizeof(EncoderExtPayload)  ==  8, "EncoderExtPayload size");
static_assert(sizeof(VescStatusPayload)  == 19, "VescStatusPayload size");
static_assert(sizeof(OdriveStatusPayload) == 11, "OdriveStatusPayload size");
static_assert(sizeof(OdriveErrorPayload) ==  9, "OdriveErrorPayload size");
static_assert(sizeof(LktechStatusPayload) == 11, "LktechStatusPayload size");
static_assert(sizeof(Ze300StatusPayload) == 14, "Ze300StatusPayload size");
static_assert(sizeof(GripperPayload)     ==  2, "GripperPayload size");
static_assert(sizeof(ArmLifecyclePayload) == 11, "ArmLifecyclePayload size");
static_assert(sizeof(ArmJointsPayload)   == 12, "ArmJointsPayload size");
static_assert(sizeof(KeybindPayload)     == 15, "KeybindPayload size");
static_assert(sizeof(PpmCalibPayload)    == 38, "PpmCalibPayload size");
static_assert(sizeof(SensorEnablePayload) == 1, "SensorEnablePayload size");
