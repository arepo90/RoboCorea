#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ─── Operating mode ───────────────────────────────────────────────────────────
enum class RobotMode : uint8_t {
    INIT    = 0,  // startup / hardware init
    STANDBY = 1,  // idle, waiting for the RC link
    NORMAL  = 2,  // RC drives the tracks + flippers (fixed scheme); arm relayed
    ARM     = 3,  // (reserved — RC no longer has a dedicated arm mode)
    ESTOP   = 4,  // all outputs neutralised; cleared by PC or hardware
    FLIPPER = 5,  // (reserved — drive + flippers are always active together now)
};

// The per-channel keybind table (ChannelFunction enum / KeybindTable) was removed
// when the RC moved to a single FIXED control scheme — see config.h "Channel
// roles" and Control::applyControl(). MSG_KEYBIND (0x14) is now reserved-unused.

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
    bool      virtual_flip;     // Ch6-up "drive from the other end" active
    uint32_t  uptime_ms;
};

// ─── Binary protocol payloads (packed, sent/received verbatim over UART) ──────
#pragma pack(push, 1)

struct TelemetryPayload {
    uint8_t  mode;                 // RobotMode value
    uint8_t  flags;                // bit0=ppm_ok bit1=sensors bit2=can_ok bit3=estop bit4=vflip
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

// (KeybindPayload removed — MSG_KEYBIND 0x14 is reserved-unused; the RC uses a
// fixed control scheme, see config.h "Channel roles" + Control::applyControl().)

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

// Gripper open/close rate from the PC (PC→ESP, arm PCB): int16 ×1000, +open/−close.
struct GripperPayload {
    int16_t value_1000;            // normalised × 1000 (−1000..+1000)
};

// External traction command (PC → ESP32). Normalised left/right track speed
// × 1000 (−1000..+1000); the Jetson bridge derives these from Nav2 /cmd_vel and
// the wheel/track geometry. enable=0 releases the tracks back to RC control.
struct TractionCmdPayload {
    int16_t left_1000;
    int16_t right_1000;
    uint8_t enable;
};

// Board identity. Sent at startup and periodically so the Jetson can discover
// which USB serial port belongs to the chassis PCB versus the arm PCB.
struct BoardIdentityPayload {
    uint8_t  role;                 // ROBOCOREA_BOARD_ROLE_CHASSIS / _ARM
    uint8_t  protocol_version;     // ROBOCOREA_PROTOCOL_VERSION
    uint16_t capabilities;         // BOARD_CAP_* bitmask
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
static_assert(sizeof(TractionCmdPayload) ==  5, "TractionCmdPayload size");
static_assert(sizeof(BoardIdentityPayload) == 4, "BoardIdentityPayload size");
static_assert(sizeof(ArmLifecyclePayload) == 11, "ArmLifecyclePayload size");
static_assert(sizeof(ArmJointsPayload)   == 12, "ArmJointsPayload size");
static_assert(sizeof(PpmCalibPayload)    == 38, "PpmCalibPayload size");
static_assert(sizeof(SensorEnablePayload) == 1, "SensorEnablePayload size");
