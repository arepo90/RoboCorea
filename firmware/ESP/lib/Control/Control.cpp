#include "Control.h"
#include "config.h"
#include "RC.h"
#include "Locomotion.h"
#include "Sensors.h"
#include "CANInterface.h"
#include "Comms.h"
#include "FlipperCollision.h"
#include <Arduino.h>
#include <cmath>

RobotMode    Control::s_mode          = RobotMode::INIT;
ArmJoints    Control::s_arm_joints    = {};
uint8_t      Control::s_sensor_mask   = 0;
bool         Control::s_hw_estop      = false;
bool         Control::s_virtual_estop = false;
float        Control::s_deadband      = 0.05f;
float        Control::s_flip_target[4] = { 0, 0, 0, 0 };
bool         Control::s_flip_seeded    = false;
static bool s_bench_neutral_sent       = false;

// Default keybind table (Ch6/slot 4 is the hardware e-stop → always NONE here).
//   mode0 (Ch5 low):  drive  — Ch2 fwd, Ch4 turn, Ch1 all-flippers
//   mode1 (Ch5 mid):  flippers individually — Ch1..Ch4 = FL/FR/RL/RR
//   mode2 (Ch5 high): arm    — Ch1..Ch4 = X/Y/Z/PITCH
KeybindTable Control::s_keybind = {{
    {ChannelFunction::FLIPPER_ALL, ChannelFunction::TRACTION_FWD, ChannelFunction::NONE,
     ChannelFunction::TRACTION_TURN, ChannelFunction::NONE},
    {ChannelFunction::FLIPPER_FL, ChannelFunction::FLIPPER_FR, ChannelFunction::FLIPPER_RL,
     ChannelFunction::FLIPPER_RR, ChannelFunction::NONE},
    {ChannelFunction::ARM_X, ChannelFunction::ARM_Y, ChannelFunction::ARM_Z,
     ChannelFunction::ARM_PITCH, ChannelFunction::NONE},
}};

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static inline float wrap360f(float d) {
    d = fmodf(d, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d;
}
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void Control::begin() {
    Comms::onArmJoints(   [](const ArmJointsPayload& p) { Control::setArmJoints(p); });
    Comms::onSensorEnable([](uint8_t mask)              { Control::setSensorMask(mask); });
    Comms::onEstop(       [](bool active) {
        if (active) Control::triggerEstop(); else Control::clearEstop();
    });
    Comms::onArmLifecycle([](bool arm) {
        if (arm) CANInterface::requestArm(); else CANInterface::requestDisarm();
    });
    Comms::onKeybind([](const KeybindPayload& p) { Control::setKeybind(p); });
    Comms::onPpmCalib([](const PpmCalibPayload& p) {
        RC::setCalib(p);
        portENTER_CRITICAL(&s_mux);
        s_deadband = p.deadband_1000 / 1000.0f;
        portEXIT_CRITICAL(&s_mux);
    });

#if ARM_BENCH_MODE_NO_PPM
    s_mode = RobotMode::ARM;
#else
    s_mode = RobotMode::STANDBY;
#endif
}

void Control::tick() {
    PPMFrame ppm;
    bool have_ppm = RC::getFrame(ppm);

    // ── Ch6 hardware e-stop (highest priority) ──────────────────────────────
    if (have_ppm) {
        portENTER_CRITICAL(&s_mux);
        bool was_hw = s_hw_estop;
        s_hw_estop = (ppm.ch[PPM_CH_ESTOP - 1] > 1800);
        if (s_hw_estop && !was_hw) {
            s_mode = RobotMode::ESTOP;
            portEXIT_CRITICAL(&s_mux);
            Locomotion::estopOutputs();
            CANInterface::estopArm();
        } else if (!s_hw_estop && was_hw) {
            if (!s_virtual_estop) s_mode = RobotMode::STANDBY;
            bool clear = !s_virtual_estop;
            portEXIT_CRITICAL(&s_mux);
            if (clear) CANInterface::clearEstopArm();
        } else {
            portEXIT_CRITICAL(&s_mux);
        }
    }

    portENTER_CRITICAL(&s_mux);
    RobotMode current = s_mode;
    portEXIT_CRITICAL(&s_mux);

    if (current == RobotMode::ESTOP) {
        Locomotion::estopOutputs();
        return;
    }

    // ── Arm-only bench mode: allow keyboard/laptop tests without an RC link. ─
#if ARM_BENCH_MODE_NO_PPM
    if (!RC::isConnected()) {
        // Send the unused base/flipper neutral frames once. Re-sending them at
        // 50 Hz during arm-only bench tests competes with the 6 arm command
        // frames and can fill the MCP2515's three TX buffers.
        if (!s_bench_neutral_sent) {
            Locomotion::neutralise();  // tracks 0, flippers hold/coast per locomotion
            s_bench_neutral_sent = true;
        }
        s_flip_seeded = false;
        portENTER_CRITICAL(&s_mux);
        s_mode = RobotMode::ARM;
        ArmJoints joints = s_arm_joints;
        portEXIT_CRITICAL(&s_mux);
        if (joints.valid) CANInterface::sendArmJoints(joints.angle_deg);
        return;
    }
    s_bench_neutral_sent = false;
#endif

    // ── PPM link watchdog → failsafe: hold flippers, stop tracks, STANDBY ────
    if (!RC::isConnected()) {
        Locomotion::neutralise();      // tracks 0, flippers HOLD their angle
        s_flip_seeded = false;         // re-seed from measured when RC returns
        portENTER_CRITICAL(&s_mux);
        s_mode = RobotMode::STANDBY;
        portEXIT_CRITICAL(&s_mux);
        return;
    }

    if (!have_ppm) return;

    int mode_idx = decodeModeIndex(ppm);

    portENTER_CRITICAL(&s_mux);
    KeybindTable kb = s_keybind;
    portEXIT_CRITICAL(&s_mux);

    bool has_arm = false, has_traction = false, has_flipper = false;
    for (int c = 0; c < 5; ++c) {
        ChannelFunction fn = kb.map[mode_idx][c];
        if (isArmFunction(fn))     has_arm = true;
        if (fn == ChannelFunction::TRACTION_FWD || fn == ChannelFunction::TRACTION_TURN)
            has_traction = true;
        if (isFlipperFunction(fn)) has_flipper = true;
    }

    RobotMode new_mode;
    if (has_arm && !has_traction && !has_flipper) new_mode = RobotMode::ARM;
    else if (has_flipper && !has_traction)        new_mode = RobotMode::FLIPPER;
    else                                          new_mode = RobotMode::NORMAL;

    portENTER_CRITICAL(&s_mux);
    RobotMode old_mode = s_mode;
    if (s_mode != RobotMode::ESTOP) s_mode = new_mode;
    // On entering ARM mode, discard any stale buffered joint command so the arm
    // only follows joint_states received from this point on (no jump to an old
    // pose). setArmJoints() also drops commands while not in ARM mode.
    if (s_mode == RobotMode::ARM && old_mode != RobotMode::ARM)
        s_arm_joints.valid = false;
    portEXIT_CRITICAL(&s_mux);

    applyKeybindRow(mode_idx, ppm);
}

void Control::applyKeybindRow(int mode_idx, const PPMFrame& ppm) {
    constexpr float dt = 1.0f / CONTROL_LOOP_HZ;

    portENTER_CRITICAL(&s_mux);
    KeybindTable kb = s_keybind;
    float kDeadband = s_deadband;
    portEXIT_CRITICAL(&s_mux);

    // Slot → raw PPM channel index: slot 0=Ch1 … 3=Ch4, 4=Ch6.
    static const int slot_to_ppm[5] = {0, 1, 2, 3, 5};

    float forward = 0.0f, turn = 0.0f;
    float flip_all = 0.0f, flip[4] = {0, 0, 0, 0};
    float gripper_val = 0.0f;
    bool  has_traction = false, has_flipper = false, has_arm = false, has_gripper = false;

    for (int c = 0; c < 5; ++c) {
        ChannelFunction fn = kb.map[mode_idx][c];
        if (fn == ChannelFunction::NONE) continue;
        int ppm_idx = slot_to_ppm[c];
        float val = RC::normalise((uint8_t)ppm_idx, ppm.ch[ppm_idx]);
        if (fabsf(val) < kDeadband) val = 0.0f;

        switch (fn) {
            case ChannelFunction::TRACTION_FWD:  forward = val; has_traction = true; break;
            case ChannelFunction::TRACTION_TURN: turn = val;    has_traction = true; break;
            case ChannelFunction::FLIPPER_ALL:   flip_all = val; has_flipper = true; break;
            case ChannelFunction::FLIPPER_FL:    flip[0] = val;  has_flipper = true; break;
            case ChannelFunction::FLIPPER_FR:    flip[1] = val;  has_flipper = true; break;
            case ChannelFunction::FLIPPER_RL:    flip[2] = val;  has_flipper = true; break;
            case ChannelFunction::FLIPPER_RR:    flip[3] = val;  has_flipper = true; break;
            case ChannelFunction::GRIPPER:       gripper_val = val; has_gripper = true; has_arm = true; break;
            default:
                if (isArmFunction(fn)) has_arm = true;
                break;
        }
    }

    // ── Traction ────────────────────────────────────────────────────────────
    if (has_traction) {
        Locomotion::setDriveCommand(forward, turn);
    } else if (!has_arm) {
        Locomotion::setTrackSpeeds(0.0f, 0.0f);
    }

    // ── Flippers (integrate stick → target angle; loop closed on the VESC) ───
    // Stick deflection = rate: the target moves while deflected and HOLDS where
    // released. The VESC lisp closes the position loop and reports the angle.
    static const float kFlipDir[4] = { FLIPPER_DIR_FL, FLIPPER_DIR_FR,
                                       FLIPPER_DIR_RL, FLIPPER_DIR_RR };
    if (has_flipper) {
        // Bumpless transfer: on (re)entering flipper control, seed targets from
        // the measured angle so the flippers don't jump to a stale setpoint.
        if (!s_flip_seeded) {
            float measured[4];
            CANInterface::getFlipperAngles(measured);
            for (int i = 0; i < 4; i++) s_flip_target[i] = measured[i];
            s_flip_seeded = true;
        }

        float norms[4];
        if (flip_all != 0.0f) { norms[0] = norms[1] = norms[2] = norms[3] = flip_all; }
        else                  { for (int i = 0; i < 4; i++) norms[i] = flip[i]; }

        // Integrate each stick into a candidate target first; commit afterwards
        // so the collision clamp sees this cycle's intent for every flipper.
        float candidate[4];
        for (int i = 0; i < 4; i++) {
            float delta = norms[i] * kFlipDir[i] * FLIPPER_RATE_DPS * dt;
#if FLIPPER_SOFT_LIMIT_ENABLE
            candidate[i] = clampf(s_flip_target[i] + delta,
                                  FLIPPER_ANGLE_MIN, FLIPPER_ANGLE_MAX);
#else
            candidate[i] = wrap360f(s_flip_target[i] + delta);
#endif
        }

#if FLIPPER_COLLISION_AVOID_ENABLE
        // Dynamic joint limits: refuse any step that would close a front/rear
        // pair to within FLIPPER_COLLISION_MARGIN_M, measured against the paired
        // flipper's actual reported angle (see FlipperCollision.h).
        float measured[4];
        CANInterface::getFlipperAngles(measured);
        for (int i = 0; i < 4; i++)
            candidate[i] = FlipperCollision::clampTarget(i, candidate[i],
                                                         s_flip_target[i], measured);
#endif

        for (int i = 0; i < 4; i++) s_flip_target[i] = candidate[i];
        Locomotion::setFlipperAngles(s_flip_target, /*enabled=*/true);
    } else {
        // No flipper bound in this row → HOLD the last target (keep the loop
        // closed on the VESC); re-seed next time flippers are actively bound.
        s_flip_seeded = false;
        Locomotion::setFlipperAngles(s_flip_target, /*enabled=*/true);
    }

    // ── Arm (commanded from the PC; tracks stopped for safety) ──────────────
    if (has_arm) {
        Locomotion::setTrackSpeeds(0.0f, 0.0f);
        portENTER_CRITICAL(&s_mux);
        ArmJoints joints = s_arm_joints;
        portEXIT_CRITICAL(&s_mux);
        if (joints.valid) CANInterface::sendArmJoints(joints.angle_deg);
        if (has_gripper)  Comms::sendGripper(gripper_val);
    }
}

void Control::triggerEstop() {
    portENTER_CRITICAL(&s_mux);
    s_virtual_estop = true;
    s_mode = RobotMode::ESTOP;
    s_flip_seeded = false;          // re-seed flippers on resume
    portEXIT_CRITICAL(&s_mux);
    Locomotion::estopOutputs();
    CANInterface::estopArm();
}

void Control::clearEstop() {
    portENTER_CRITICAL(&s_mux);
    s_virtual_estop = false;
    bool can_clear = (s_mode == RobotMode::ESTOP) && !s_hw_estop;
    if (can_clear) s_mode = RobotMode::STANDBY;
    portEXIT_CRITICAL(&s_mux);
    if (can_clear) CANInterface::clearEstopArm();
}

void Control::setArmJoints(const ArmJointsPayload& payload) {
    portENTER_CRITICAL(&s_mux);
    // Only capture joint commands while in ARM mode; otherwise drop them.
    if (s_mode == RobotMode::ARM) {
        for (int i = 0; i < 6; i++) s_arm_joints.angle_deg[i] = payload.joint[i] * 0.01f;
        s_arm_joints.valid = true;
    }
    portEXIT_CRITICAL(&s_mux);
}

void Control::setSensorMask(uint8_t mask) {
    portENTER_CRITICAL(&s_mux);
    s_sensor_mask = mask;
    portEXIT_CRITICAL(&s_mux);
    Sensors::setEnabledMask(mask);
}

void Control::setKeybind(const KeybindPayload& payload) {
    portENTER_CRITICAL(&s_mux);
    for (int m = 0; m < 3; ++m)
        for (int c = 0; c < 5; ++c)
            s_keybind.map[m][c] = (ChannelFunction)payload.map[m][c];
    portEXIT_CRITICAL(&s_mux);
}

RobotMode Control::getMode() {
    portENTER_CRITICAL(&s_mux);
    RobotMode m = s_mode;
    portEXIT_CRITICAL(&s_mux);
    return m;
}

void Control::getSystemStatus(SystemStatus& out) {
    portENTER_CRITICAL(&s_mux);
    out.mode        = s_mode;
    out.sensor_mask = s_sensor_mask;
    out.estop       = (s_mode == RobotMode::ESTOP);
    portEXIT_CRITICAL(&s_mux);
    out.ppm_connected    = RC::isConnected();
    out.minipc_connected = Comms::isConnected();
    out.can_ok           = CANInterface::isOk();
    out.uptime_ms        = millis();
}

int Control::decodeModeIndex(const PPMFrame& ppm) {
    constexpr uint16_t kLow  = PPM_MIN_US + (PPM_MAX_US - PPM_MIN_US) / 4;  // ~1250
    constexpr uint16_t kHigh = PPM_MAX_US - (PPM_MAX_US - PPM_MIN_US) / 4;  // ~1750
    uint16_t ch5 = ppm.ch[PPM_CH_MODE - 1];
    if (ch5 < kLow)  return 0;
    if (ch5 > kHigh) return 2;
    return 1;
}
