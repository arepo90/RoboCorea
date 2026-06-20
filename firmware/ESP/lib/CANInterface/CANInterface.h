#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "robot_types.h"

// ─── CANInterface ─────────────────────────────────────────────────────────────
// One 500 kbps CAN 2.0 bus (on-board SMD MCP2515; TWAI/SN65HVD230 optional)
// carries every actuator:
//   • 6 VESC — traction (velocity, SET_RPM) + flippers (position loop runs ON
//     the VESC in LispBM; the ESP sends the target angle over a custom frame)
//   • 3 ODrive (J1–J3) + ZE300 (J4) + 2 LKTech (J5–J6) for the arm
//
// Track feedback comes from the VESC CAN status frames (no separate encoders):
// eRPM gives live track speed and the STATUS_5 tachometer is forwarded for wheel
// odometry (integrated on the Jetson bridge). Flipper angle comes from the lisp's
// 0x7F report. Enable VESC status frames 1/4/5 in VESC Tool for eRPM / temp /
// input-voltage (+ tachometer) telemetry.

// Arm safety lifecycle state (see config.h "Arm safety lifecycle").
enum class ArmState : uint8_t { UNINIT = 0, INITIALIZING = 1, READY = 2, FAULT = 3 };
enum class ArmOperatingMode : uint8_t { DEXTERITY = 0, CHASSIS = 1 };

class CANInterface {
public:
    // Brings up the CAN HAL. With ARM_PASSIVE_BOOT the arm boots DISARMED (no
    // motor enable / zero capture / motion) — arm it explicitly via requestArm().
    static bool begin();
    static bool isOk();

    // ── Arm safety lifecycle ────────────────────────────────────────────────
    // requestArm()/requestDisarm() set a flag serviced inside poll() (the arm
    // bring-up blocks for ~seconds, so it must not run in the control loop).
    static void requestArm();
    static void requestDisarm();
    static void requestOperatingMode(ArmOperatingMode mode);
    static uint8_t armState();                          // ArmState value
    static uint8_t armOperatingMode();                  // ArmOperatingMode value
    static void getArmLifecycle(ArmLifecyclePayload& out);

    // ── Base drivetrain (VESC) ──────────────────────────────────────────────
    // Traction inputs are normalised [-1,+1] → eRPM (TRACTION_ERPM_MAX).
    // Flippers are position-controlled ON the VESC (LispBM): we send an absolute
    // wrapped target angle [0,360) per flipper plus an enable flag (false = coast).
    static bool sendTrackSpeeds(float left_norm, float right_norm);
    static bool sendFlipperAngles(const float target_deg[4], bool enabled);  // FL,FR,RL,RR

    // ── Arm ─────────────────────────────────────────────────────────────────
    // angles_deg[0..5] = J1..J6 output degrees. ODrive J1–J3, ZE300 J4, LKTech J5–J6.
    static bool sendArmJoints(const float angles_deg[6]);

    // Drain RX + emit ODrive telemetry RTRs. Call from the CAN task (~200 Hz).
    static void poll();

    // ── Drivetrain feedback (derived from VESC status) ──────────────────────
    static void getTractionSpeeds(float& left_rpm, float& right_rpm);   // output RPM
    static void getFlipperAngles(float out_deg[4]);                     // [0,360) FL,FR,RL,RR

    // ── Raw status accessors (clear the per-source "fresh" flag) ────────────
    static uint8_t vescIdByIndex(uint8_t idx);   // 0..5 → CAN id, else 0
    static bool getVescStatus(uint8_t vesc_id, VescStatusPayload& out);
    static bool getOdriveStatus(uint8_t joint_idx, OdriveStatusPayload& out);
    static bool getLktechStatus(uint8_t joint_idx, LktechStatusPayload& out);
    static bool getZe300Status(Ze300StatusPayload& out);
    static bool getOdriveError(uint8_t node_idx, OdriveErrorPayload& out);
    static uint8_t odriveNodeCount();

    // ── Arm e-stop / recovery ───────────────────────────────────────────────
    static void estopArm();
    static void clearEstopArm();
};
