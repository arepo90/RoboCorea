#include "Control.h"
#include "config.h"
#include "RC.h"
#include "Locomotion.h"
#include "CANInterface.h"
#include "Comms.h"
#include "Gripper.h"
#include "FlipperCollision.h"
#include <Arduino.h>
#include <cmath>

RobotMode    Control::s_mode          = RobotMode::INIT;
ArmJoints    Control::s_arm_joints    = {};
bool         Control::s_hw_estop      = false;
bool         Control::s_virtual_estop = false;
bool         Control::s_virtual_flip  = false;
float        Control::s_deadband      = 0.05f;
float        Control::s_flip_target[4] = { 0, 0, 0, 0 };
bool         Control::s_flip_seeded    = false;
float        Control::s_ext_left       = 0.0f;
float        Control::s_ext_right      = 0.0f;
bool         Control::s_ext_enable     = false;
uint32_t     Control::s_ext_ms         = 0;
static bool s_bench_neutral_sent       = false;

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
    Comms::onArmJoints([](const ArmJointsPayload& p) {
#if ROBOCOREA_ROLE_IS_ARM
        Control::setArmJoints(p);
#else
        (void)p;
#endif
    });
    Comms::onEstop(       [](bool active) {
        if (active) Control::triggerEstop(); else Control::clearEstop();
    });
    Comms::onArmLifecycle([](bool arm) {
#if ROBOCOREA_ROLE_IS_ARM
        if (arm) CANInterface::requestArm(); else CANInterface::requestDisarm();
#else
        (void)arm;
#endif
    });
    Comms::onArmMode([](uint8_t mode) {
#if ROBOCOREA_ROLE_IS_ARM
        if (mode <= (uint8_t)ArmOperatingMode::CHASSIS)
            CANInterface::requestOperatingMode((ArmOperatingMode)mode);
#else
        (void)mode;
#endif
    });
    Comms::onPpmCalib([](const PpmCalibPayload& p) {
#if ROBOCOREA_ROLE_IS_CHASSIS
        RC::setCalib(p);
        portENTER_CRITICAL(&s_mux);
        s_deadband = p.deadband_1000 / 1000.0f;
        portEXIT_CRITICAL(&s_mux);
#else
        (void)p;
#endif
    });
    Comms::onTraction([](const TractionCmdPayload& p) {
#if ROBOCOREA_ROLE_IS_CHASSIS
        Control::setExternalTraction(p.left_1000 / 1000.0f, p.right_1000 / 1000.0f,
                                     p.enable != 0);
#else
        (void)p;
#endif
    });
    Comms::onGripper([](float rate) {
#if ROBOCOREA_ROLE_IS_ARM
        // Gripper open/close rate from the workstation gamepad (RT/LT → MSG_GRIPPER).
        Gripper::setCommand(rate);
#else
        (void)rate;
#endif
    });

    s_mode = RobotMode::STANDBY;
}

void Control::tick() {
#if ROBOCOREA_ROLE_IS_ARM
    portENTER_CRITICAL(&s_mux);
    RobotMode current = s_mode;
    ArmJoints joints = s_arm_joints;
    portEXIT_CRITICAL(&s_mux);

    if (current == RobotMode::ESTOP) {
        CANInterface::estopArm();
        Gripper::hold();   // stop stepping; the servo holds its current angle
        return;
    }
    // Step the end-effector gripper toward its open/close limit. The command's
    // own timeout halts it if the gamepad link drops; independent of the arm.
    Gripper::update(1.0f / CONTROL_LOOP_HZ);
    if (joints.valid) CANInterface::sendArmJoints(joints.angle_deg);
    return;
#else
    PPMFrame ppm;
    bool have_ppm = RC::getFrame(ppm);

    // ── Ch6-down RC e-stop (highest priority) ───────────────────────────────
    // Ch6 is a 3-position lever: down third = E-STOP, centre = normal, up third
    // = virtual-flip. Level-based, like the old hw e-stop: releasing the lever
    // back to centre resumes (unless a software e-stop is also active).
    if (have_ppm) {
        float n6 = RC::normalise(PPM_CH_MODE2 - 1, ppm.ch[PPM_CH_MODE2 - 1]);
        portENTER_CRITICAL(&s_mux);
        bool was_hw = s_hw_estop;
        s_hw_estop = (n6 <= LEVER_LO_THRESH);
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

    // ── Bench mode: relay workstation arm commands without an RC link. ──────
#if ARM_BENCH_MODE_NO_PPM
    if (!RC::isConnected()) {
        // Neutralise the base once (re-sending at 50 Hz competes with the 6 arm
        // command frames and can fill the MCP2515's three TX buffers).
        if (!s_bench_neutral_sent) {
            Locomotion::neutralise();  // tracks 0, flippers hold
            s_bench_neutral_sent = true;
        }
        s_flip_seeded = false;
        portENTER_CRITICAL(&s_mux);
        s_mode = RobotMode::STANDBY;
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

    // Fixed control scheme: drive + flippers from the RC every loop.
    portENTER_CRITICAL(&s_mux);
    if (s_mode != RobotMode::ESTOP) s_mode = RobotMode::NORMAL;
    portEXIT_CRITICAL(&s_mux);

    applyControl(ppm);

    // Relay the workstation's arm-joint stream. sendArmJoints() ignores it unless
    // the arm lifecycle is READY (armed), so drive + arm can run concurrently.
    portENTER_CRITICAL(&s_mux);
    ArmJoints joints = s_arm_joints;
    portEXIT_CRITICAL(&s_mux);
    if (joints.valid) CANInterface::sendArmJoints(joints.angle_deg);
#endif
}

void Control::applyControl(const PPMFrame& ppm) {
    constexpr float dt = 1.0f / CONTROL_LOOP_HZ;

    portENTER_CRITICAL(&s_mux);
    float kDeadband = s_deadband;
    portEXIT_CRITICAL(&s_mux);

    // Calibrated, normalised channel values in [-1,1].
    auto nrm = [&](int ch1) { return RC::normalise((uint8_t)(ch1 - 1), ppm.ch[ch1 - 1]); };
    float n_sel  = nrm(PPM_CH_FLIP_SELECT);    // Ch1 flipper L/R selector
    float n_rate = nrm(PPM_CH_FLIP_RATE);      // Ch2 flipper rate
    float n_fwd  = nrm(PPM_CH_TRACTION_FWD);   // Ch3 traction forward
    float n_turn = nrm(PPM_CH_TRACTION_TURN);  // Ch4 traction turn
    float n_pair = nrm(PPM_CH_FLIP_PAIR);      // Ch5 front/rear pair select
    float n_mode = nrm(PPM_CH_MODE2);          // Ch6 3-pos lever

    // Ch6 up third = virtual flip ("drive from the other end"). Publish for the GUI.
    bool vflip = (n_mode >= LEVER_HI_THRESH);
    portENTER_CRITICAL(&s_mux);
    s_virtual_flip = vflip;
    portEXIT_CRITICAL(&s_mux);

    // Deadband the analog sticks (Ch1 keeps its raw value — it's a 3-way selector).
    if (fabsf(n_rate) < kDeadband) n_rate = 0.0f;
    if (fabsf(n_fwd)  < kDeadband) n_fwd  = 0.0f;
    if (fabsf(n_turn) < kDeadband) n_turn = 0.0f;

    // ── Traction (Ch3 forward, Ch4 turn), or autonomy (Nav2 /cmd_vel) ───────
    // The external command drives the tracks ONLY while the RC drive sticks are
    // neutral, virtual-flip is off, and the command is fresh. Touching a stick,
    // engaging virtual-flip, or letting the command go stale instantly returns
    // control to the operator (RC loss is already handled by the failsafe above).
    bool rc_traction_neutral = (n_fwd == 0.0f && n_turn == 0.0f);   // already deadbanded
    bool ext_active = false;
    float ext_l = 0.0f, ext_r = 0.0f;
    portENTER_CRITICAL(&s_mux);
    if (s_ext_enable && (millis() - s_ext_ms) < EXT_DRIVE_TIMEOUT_MS) {
        ext_active = true; ext_l = s_ext_left; ext_r = s_ext_right;
    }
    portEXIT_CRITICAL(&s_mux);

    if (ext_active && rc_traction_neutral && !vflip) {
        Locomotion::setTrackSpeeds(ext_l, ext_r);   // autonomy drives the tracks
    } else {
        float fwd = n_fwd, turn = n_turn;
#if VFLIP_INVERT_FORWARD
        if (vflip) fwd = -fwd;
#endif
#if VFLIP_INVERT_TURN
        if (vflip) turn = -turn;
#endif
        Locomotion::setDriveCommand(fwd, turn);
    }

    // ── Flipper selection: Ch5 picks the pair, Ch1 picks left / right / both ─
    // Pair indices into s_flip_target[] (FL=0, FR=1, RL=2, RR=3). Ch5 min → FRONT.
    bool front_pair = (n_pair < 0.0f);
#if VFLIP_SWAP_PAIR
    if (vflip) front_pair = !front_pair;       // flipped: Ch5 "front" → physical rear
#endif
    int leftIdx  = front_pair ? 0 : 2;         // FL or RL
    int rightIdx = front_pair ? 1 : 3;         // FR or RR
#if VFLIP_SWAP_LEFTRIGHT
    if (vflip) { int t = leftIdx; leftIdx = rightIdx; rightIdx = t; }  // op-left = robot-right
#endif

    // Ch1: below −deadband → left only, above +deadband → right only, else both.
    bool drive_left, drive_right;
    if (n_sel < -kDeadband)      { drive_left = true;  drive_right = false; }
    else if (n_sel >  kDeadband) { drive_left = false; drive_right = true;  }
    else                         { drive_left = true;  drive_right = true;  }

    float rate_cmd = n_rate;
#if VFLIP_INVERT_FLIP_RATE
    if (vflip) rate_cmd = -rate_cmd;
#endif

    // ── Integrate the selected flippers' targets (loop closed on the VESC) ──
    // Stick deflection = rate; the per-flipper target integrates it and HOLDS
    // where released (rate 0 → candidate == target). FLIPPER_DIR_* applies the
    // physical orientation sign. Unselected flippers get rate 0 → they hold.
    static const float kFlipDir[4] = { FLIPPER_DIR_FL, FLIPPER_DIR_FR,
                                       FLIPPER_DIR_RL, FLIPPER_DIR_RR };
    float norms[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    if (drive_left)  norms[leftIdx]  = rate_cmd;
    if (drive_right) norms[rightIdx] = rate_cmd;

#if !FLIPPER_USE_LEGACY_RPM_LISP
    // Custom-frame path only: seed the targets from the VESC-measured angle once
    // (bumpless). The fake-RPM lisp path free-runs the accumulator (no seeding).
    if (!s_flip_seeded) {
        float measured[4];
        CANInterface::getFlipperAngles(measured);
        for (int i = 0; i < 4; i++) s_flip_target[i] = measured[i];
        s_flip_seeded = true;
    }
#endif

    float candidate[4];
    for (int i = 0; i < 4; i++) {
        float next = s_flip_target[i] + norms[i] * kFlipDir[i] * FLIPPER_RATE_DPS * dt;
#if FLIPPER_SOFT_LIMIT_ENABLE
        next = clampf(next, FLIPPER_ANGLE_MIN, FLIPPER_ANGLE_MAX);
#elif !FLIPPER_USE_LEGACY_RPM_LISP
        next = wrap360f(next);   // custom-frame target wraps; legacy lisp wants continuous
#endif
        candidate[i] = next;
    }

#if FLIPPER_COLLISION_ENABLE
    // Dynamic joint limits: a front/rear pair on a side may not both sit in their
    // danger arcs at once. Left = FL,RL  Right = FR,RR.
    FlipperCollision::applySide(0, 2, s_flip_target, candidate);  // left
    FlipperCollision::applySide(1, 3, s_flip_target, candidate);  // right
#endif

    for (int i = 0; i < 4; i++) s_flip_target[i] = candidate[i];
    Locomotion::setFlipperAngles(s_flip_target, /*enabled=*/true);
}

void Control::triggerEstop() {
    portENTER_CRITICAL(&s_mux);
    s_virtual_estop = true;
    s_mode = RobotMode::ESTOP;
    s_flip_seeded = false;          // re-seed flippers on resume
    portEXIT_CRITICAL(&s_mux);
#if ROBOCOREA_ROLE_IS_CHASSIS
    Locomotion::estopOutputs();
#endif
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
#if ROBOCOREA_ROLE_IS_ARM
    portENTER_CRITICAL(&s_mux);
    // Capture the workstation's joint stream whenever we're not e-stopped (the
    // RC no longer has a dedicated arm mode; the arm lifecycle gate in
    // CANInterface::sendArmJoints() decides whether it actually moves).
    if (s_mode != RobotMode::ESTOP) {
        for (int i = 0; i < 6; i++) s_arm_joints.angle_deg[i] = payload.joint[i] * 0.01f;
        s_arm_joints.valid = true;
    }
    portEXIT_CRITICAL(&s_mux);
#else
    (void)payload;
#endif
}

void Control::setExternalTraction(float left, float right, bool enable) {
#if ROBOCOREA_ROLE_IS_CHASSIS
    // Stored here; applyControl() arbitrates against the RC sticks every loop and
    // the freshness check (EXT_DRIVE_TIMEOUT_MS) makes a dropped link fail safe.
    portENTER_CRITICAL(&s_mux);
    s_ext_left  = (left  < -1.0f) ? -1.0f : (left  > 1.0f) ? 1.0f : left;
    s_ext_right = (right < -1.0f) ? -1.0f : (right > 1.0f) ? 1.0f : right;
    s_ext_enable = enable;
    s_ext_ms = millis();
    portEXIT_CRITICAL(&s_mux);
#else
    (void)left; (void)right; (void)enable;
#endif
}

RobotMode Control::getMode() {
    portENTER_CRITICAL(&s_mux);
    RobotMode m = s_mode;
    portEXIT_CRITICAL(&s_mux);
    return m;
}

void Control::getSystemStatus(SystemStatus& out) {
    portENTER_CRITICAL(&s_mux);
    out.mode         = s_mode;
    // Sensors moved to the dedicated SENSOR ESP32 (always-on); field kept so the
    // status payload layout is stable.
    out.sensor_mask  = 0;
    out.estop        = (s_mode == RobotMode::ESTOP);
    out.virtual_flip = s_virtual_flip;
    portEXIT_CRITICAL(&s_mux);
    out.ppm_connected    = RC::isConnected();
    out.minipc_connected = Comms::isConnected();
    out.can_ok           = CANInterface::isOk();
    out.uptime_ms        = millis();
}
