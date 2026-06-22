#pragma once
//
// RoboCorea — ESP32 firmware configuration
// =========================================
// Two identical PCBs run this same firmware. The only intended per-board
// difference is ROBOCOREA_BOARD_ROLE below:
//   • CHASSIS owns RC/PPM, traction VESCs, flipper VESCs, wheel-odom VESC
//     telemetry.
//   • ARM owns the mixed-CAN arm bus: ODrive J1–J3, ZE300 J4, LKTech J5–J6.
//
// Pin numbers below were inherited from the legacy Dicerox bring-up and MUST be
// reconciled against the actual RoboCorea PCB before flashing.
//
// See reference/architecture.md for the full system picture.

// ─── Board role / identity ───────────────────────────────────────────────────
// Leave the committed default as CHASSIS. For the arm PCB, change only this macro
// to ROBOCOREA_BOARD_ROLE_ARM before building/flashing.
#define ROBOCOREA_BOARD_ROLE_CHASSIS  1
#define ROBOCOREA_BOARD_ROLE_ARM      2
#ifndef ROBOCOREA_BOARD_ROLE
  #define ROBOCOREA_BOARD_ROLE ROBOCOREA_BOARD_ROLE_CHASSIS
#endif

#if ROBOCOREA_BOARD_ROLE == ROBOCOREA_BOARD_ROLE_CHASSIS
  #define ROBOCOREA_ROLE_IS_CHASSIS 1
  #define ROBOCOREA_ROLE_IS_ARM     0
#elif ROBOCOREA_BOARD_ROLE == ROBOCOREA_BOARD_ROLE_ARM
  #define ROBOCOREA_ROLE_IS_CHASSIS 0
  #define ROBOCOREA_ROLE_IS_ARM     1
#else
  #error "ROBOCOREA_BOARD_ROLE must be ROBOCOREA_BOARD_ROLE_CHASSIS or ROBOCOREA_BOARD_ROLE_ARM"
#endif

#define ROBOCOREA_PROTOCOL_VERSION       1
#define BOARD_CAP_CHASSIS_IO       (1 << 0)
#define BOARD_CAP_ARM_IO           (1 << 1)
#define BOARD_CAP_RC_PPM           (1 << 2)
#define BOARD_CAP_MAG              (1 << 3)  // reserved; mag is on Jetson I2C
#define BOARD_CAP_VESC_BASE        (1 << 4)
#define BOARD_CAP_ARM_CAN          (1 << 5)

#if ROBOCOREA_ROLE_IS_CHASSIS
  #define ROBOCOREA_BOARD_CAPABILITIES \
    (BOARD_CAP_CHASSIS_IO | BOARD_CAP_RC_PPM | BOARD_CAP_VESC_BASE)
#else
  #define ROBOCOREA_BOARD_CAPABILITIES \
    (BOARD_CAP_ARM_IO | BOARD_CAP_ARM_CAN)
#endif

// ─── I2C (reserved on ESP32) ─────────────────────────────────────────────────
// The MLX90640 and LIS3MDL both live on the Jetson I2C bus. These legacy pin
// constants are kept for bench experiments only; normal firmware does not start
// an ESP32 I2C sensor task.
#define PIN_I2C_SDA          21
#define PIN_I2C_SCL          22

// ─── CAN backend selection ───────────────────────────────────────────────────
// Pick exactly ONE transceiver backend. The rest of the CAN code is
// backend-agnostic (see lib/CANInterface — it talks through a small HAL).
//
// This PCB has an MCP2515 soldered on (SMD, permanent) on the SPI pins below, so
// MCP2515 is the active backend. The TWAI backend is kept compilable for a
// future board; its pins are PLACEHOLDERS — pick real free GPIOs then.
#define CAN_BACKEND_MCP2515            // on-board SMD MCP2515 over SPI (default/active)
// #define CAN_BACKEND_TWAI            // future ESP32 TWAI + SN65HVD230

#define CAN_BITRATE_BPS   500000       // 500 kbps — matches 1.ino / the working VESCs.
                                       // To move to 1 Mbps, set this to 1000000 AND set
                                       // every VESC's CAN baud in VESC Tool (App → General).

#if defined(CAN_BACKEND_MCP2515)
// MCP2515: SPI bus + chip-select. Pins per 1.ino — confirmed on the PCB.
  #define PIN_CAN_CS          5
  #define PIN_CAN_SCK        18
  #define PIN_CAN_MISO       19
  #define PIN_CAN_MOSI       23
  #define MCP2515_OSC_MHZ     8        // crystal on the on-board MCP2515 (8 MHz per 1.ino)
#elif defined(CAN_BACKEND_TWAI)
// TWAI: route TX → CTX/D and RX ← CRX/R on the SN65HVD230. PLACEHOLDER PINS.
  #define PIN_CAN_TX         17        // TODO: pick a real free GPIO on the TWAI board
  #define PIN_CAN_RX         16        // TODO: pick a real free GPIO on the TWAI board
#else
  #error "Select a CAN backend: define CAN_BACKEND_MCP2515 or CAN_BACKEND_TWAI"
#endif

// ─── CAN robustness ──────────────────────────────────────────────────────────
#define CAN_RX_QUEUE_LEN     32        // TWAI RX depth (default 5 overflows w/ 6 VESC status @100Hz)
#define CAN_TX_QUEUE_LEN     16
#define CAN_TX_TIMEOUT_MS     1        // bounded TX: drop-on-full, don't stall the control loop
#define CAN_HEALTH_PERIOD_MS 200       // re-check bus health / attempt recovery this often
#define CAN_MCP_FAIL_LIMIT    20       // consecutive MCP2515 send failures → re-init the chip

// ─── UART to the Jetson ──────────────────────────────────────────────────────
#define MINIPC_BAUD      921600
#define ENABLE_COMMS                 // binary protocol active (disables text Serial debug)
// #define DEBUG_ARM                 // verbose arm/CAN debug over Serial (needs ENABLE_COMMS off-ish)

// ─── PPM input (FlySky FS-iA6B, 6 ch on one wire) ────────────────────────────
#define PIN_PPM               4      // PPM input (GPIO4 per 1.ino/2.ino)
#define PPM_CHANNELS          6
#define PPM_SYNC_US        3000      // gap > this µs → frame sync
#define PPM_MIN_US         1000      // nominal min pulse
#define PPM_MAX_US         2000      // nominal max pulse
#define PPM_TIMEOUT_MS      500      // failsafe: no valid frame within this → neutral

// Channel roles (1-indexed to match the physical FlySky labels). FIXED control
// scheme (no GUI keybind table): Ch1-4 are joysticks, Ch5/Ch6 are levers.
//   Ch1  flipper L/R selector  (min=left only, center=both, max=right only)
//   Ch2  flipper rate          (drives whichever flipper(s) Ch1+Ch5 select)
//   Ch3  traction forward/back
//   Ch4  traction turn
//   Ch5  2-state pair select   (min=FRONT pair FL/FR, max=REAR pair RL/RR)
//   Ch6  3-position lever      (down=E-STOP, center=normal, up=virtual-flip)
// Drive + flippers are always active together; the arm is driven from the
// workstation (MSG_ARM_JOINTS), not the RC.
#define PPM_CH_FLIP_SELECT    1
#define PPM_CH_FLIP_RATE      2
#define PPM_CH_TRACTION_FWD   3
#define PPM_CH_TRACTION_TURN  4
#define PPM_CH_FLIP_PAIR      5
#define PPM_CH_MODE2          6      // 3-position: e-stop / normal / virtual-flip

// Lever decision thresholds on the calibrated normalised value (RC::normalise →
// [-1,1]). Ch5 (2-state) splits at 0; Ch6 (3-state) uses the outer thirds.
#define LEVER_HI_THRESH       0.5f    // Ch6 up  → virtual-flip
#define LEVER_LO_THRESH     (-0.5f)   // Ch6 down → E-STOP

// Virtual-flip ("drive from the other end") sign conventions. With Ch6 up the
// symmetric robot is driven as if its rear were the front, so it can back out of
// a dead-end hallway without turning around. Each part is a bench-tunable sign —
// set to 0 to disable that piece of the 180° remap.
#define VFLIP_INVERT_FORWARD     1   // negate traction forward
#define VFLIP_INVERT_TURN        0   // keep turn as-is: negating forward already
                                     // mirrors steering, so inverting turn too
                                     // double-flips it (left/right swapped)
#define VFLIP_SWAP_PAIR          1   // Ch5 front<->rear pair selection swaps
#define VFLIP_SWAP_LEFTRIGHT     1   // flipper L/R selection mirrors (op-left = robot-right)
#define VFLIP_INVERT_FLIP_RATE   1   // negate the flipper rate stick: VFLIP_SWAP_PAIR +
                                     // VFLIP_SWAP_LEFTRIGHT together flip which physical
                                     // side (FL/RL dir=-1 vs FR/RR dir=+1) Ch1-left/right
                                     // lands on, so the rate sign must flip too or the
                                     // same stick input spins the flipper backwards

// ─── Drivetrain: all six base motors are VESCs ───────────────────────────────
// VESC CAN controller IDs — values from 1.ino/2.ino (the working board).
// They need NOT be contiguous; the firmware maps id → array index by lookup.
#define VESC_ID_TRACTION_LEFT   60
#define VESC_ID_TRACTION_RIGHT  50
#define VESC_ID_FLIPPER_FL      20
#define VESC_ID_FLIPPER_FR      10
#define VESC_ID_FLIPPER_RL      40
#define VESC_ID_FLIPPER_RR      30

// Direction correction per motor (+1 normal, -1 reversed wiring) — verify on bench.
#define TRACTION_DIR_LEFT     (1.0f)
#define TRACTION_DIR_RIGHT    (1.0f)
// Flippers: the left and right motors are mirror-mounted, so a single "raise"
// stick must drive the two sides in opposite directions. This matches the proven
// 5.ino, which integrates the left flippers (FL/RL) with a negative sign and the
// right flippers (FR/RR) with a positive sign. Re-confirm against the wiring on
// the bench (flip a sign here if a corner moves the wrong way).
#define FLIPPER_DIR_FL        (-1.0f)
#define FLIPPER_DIR_FR        (1.0f)
#define FLIPPER_DIR_RL        (-1.0f)
#define FLIPPER_DIR_RR        (1.0f)

// Mechanical reduction (motor → output shaft).
#define TRACTION_GEAR_RATIO     23.333f
#define FLIPPER_GEAR_RATIO     100.0f

// VESC SET_RPM commands eRPM = mechanical_rpm × pole_pairs.
// Used to scale traction stick → eRPM and to convert eRPM feedback → output RPM.
#define VESC_POLE_PAIRS          7      // TODO: confirm motor pole-pair count

// Traction: full-stick output speed (eRPM sent to the traction VESCs).
#define TRACTION_ERPM_MAX     14000      // TODO: tune for the actual motors

// ── Flippers: position loop runs ON the VESC (firmware/VESC/flipper_position.lisp)
// The ESP32 integrates the stick into a target angle and sends it to the flipper
// VESC. The most reliable transport on this robot has been the legacy fake-RPM
// contract: SET_RPM carries target_degrees × 1000 and the VESC Lisp reads it
// with get-rpm-set. Keep this enabled unless the custom 0x7E/0x7F Lisp CAN path
// is proven on the actual VESC firmware build.
#define FLIPPER_USE_LEGACY_RPM_LISP  1

// Telemetry source for the flipper angle reported on /encoders/flipper.
//
// This selects ONLY where the *reported* angle comes from — it no longer touches
// the control loop. The flipper target is a free-running accumulator (Control.cpp)
// that the lisp tracks; the ESP never feeds a measured angle back into the target.
// (That feedback path, with the sign-inverted tach below, is what used to make the
// flippers run away to a ~180° attractor — that coupling is now gone, so enabling
// this is safe.)
//
//   1 = report the STATUS_5 tachometer angle. Shows real motion the command does
//       not capture: moving a flipper by hand, or a stall/under-load error the
//       VESC loop can't fully correct. Requires STATUS_5 enabled on the flipper
//       VESCs in VESC Tool, and the FLIPPER_TACH_* scale/sign/zero below
//       calibrated. The angle is INCREMENTAL from the first STATUS_5 frame after
//       boot (boot pose ≈ 0 + FLIPPER_TACH_ZERO_DEG_*), not absolute.
//   0 = report the commanded target instead (no measurement; the lisp tracks it,
//       so it equals the real angle only while the loop is actually holding).
#define FLIPPER_USE_TACH_FEEDBACK    1

// Tachometer conversion. Bench calibration: the old scale reported 535 deg for
// a measured 90 deg motion, and its sign was inverted (downward was positive).
// Keep the negative scale so downward motion maps to 270 deg after positive
// modulo 360.
#define FLIPPER_TACH_SCALE_CAL   (90.0f / 535.0f)
#define FLIPPER_TACH_BASE_DEG_PER_COUNT  (360.0f / (VESC_POLE_PAIRS * FLIPPER_GEAR_RATIO))
#define FLIPPER_TACH_DEG_PER_COUNT_FL  (-FLIPPER_TACH_BASE_DEG_PER_COUNT * FLIPPER_TACH_SCALE_CAL)
#define FLIPPER_TACH_DEG_PER_COUNT_FR  (-FLIPPER_TACH_BASE_DEG_PER_COUNT * FLIPPER_TACH_SCALE_CAL)
#define FLIPPER_TACH_DEG_PER_COUNT_RL  (-FLIPPER_TACH_BASE_DEG_PER_COUNT * FLIPPER_TACH_SCALE_CAL)
#define FLIPPER_TACH_DEG_PER_COUNT_RR  (-FLIPPER_TACH_BASE_DEG_PER_COUNT * FLIPPER_TACH_SCALE_CAL)

#define FLIPPER_TACH_ZERO_DEG_FL   0.0f
#define FLIPPER_TACH_ZERO_DEG_FR   0.0f
#define FLIPPER_TACH_ZERO_DEG_RL   0.0f
#define FLIPPER_TACH_ZERO_DEG_RR   0.0f

// If fake-RPM mode is disabled, normal mode uses custom CAN frames: the VESC
// closes the loop and reports its measured angle back. The PD/feedforward gains
// and angle scale live in the Lisp, not here.
//
// Custom CAN command bytes carried in the VESC extended-ID cmd field
// (id = (cmd<<8)|vesc_id). 0x7E/0x7F are outside the VESC 6.06 CAN_PACKET set, so
// the firmware ignores them and hands them to the lisp's event-can-eid handler.
// CONFIRM they don't collide with a CAN_PACKET_* in your VESC 6.06 build.
#define VESC_CMD_FLIPPER_TARGET  0x7E   // ESP → VESC: [int32 BE millideg][u8 enable]
#define VESC_CMD_FLIPPER_REPORT  0x7F   // VESC → ESP: [int16 BE deci-deg measured]

// Stick → setpoint integration: full stick slews the target at this rate.
#define FLIPPER_RATE_DPS      90.0f     // TODO: tune feel (deg/s at full deflection)

// Optional soft limits. 0 = free 360° spin (wrapped). Enabling limits switches
// the ESP target from wrapped to clamped continuous — only do this if the
// flippers are mechanically range-limited (then the lisp wrap is harmless).
#define FLIPPER_SOFT_LIMIT_ENABLE 0
#define FLIPPER_ANGLE_MIN       0.0f
#define FLIPPER_ANGLE_MAX     360.0f

// ── Flipper anti-collision (dynamic joint limits, front vs rear, per side) ────
// The front and rear flipper on a side collide when both point toward the middle
// of the robot — the front leaning back toward ~180° and the rear leaning forward
// toward ~0°. Forbid that joint state: a flipper may not enter its danger arc
// while its same-side partner is already in its own danger arc; otherwise both
// spin freely. Left (FL/RL) and right (FR/RR) are checked independently.
//
// Each arc is CENTER ± HALFWIDTH degrees on the wrapped [0,360) angle:
//   FRONT 180 ± 60 → forbidden when front ∈ [120,240]   (the arc through 180°)
//   REAR    0 ± 60 → forbidden when rear  ∈ [300,60]     (the arc through 0°)
// (For an arc [a,b], set CENTER=(a+b)/2 and HALFWIDTH=(b−a)/2.)
//
// This limits the COMMANDED targets, so the firmware never drives the two
// flippers into each other. It does not react to a flipper shoved into its arc
// by pure external load (that motion is not commanded).
#define FLIPPER_COLLISION_ENABLE               1
#define FLIPPER_COLLISION_FRONT_CENTER_DEG     180.0f
#define FLIPPER_COLLISION_FRONT_HALFWIDTH_DEG   80.0f
#define FLIPPER_COLLISION_REAR_CENTER_DEG        0.0f
#define FLIPPER_COLLISION_REAR_HALFWIDTH_DEG    80.0f

// On e-stop (hardware OR software): 1 = HOLD the flippers where they are (the
// VESC keeps the position loop closed at the last commanded target, exactly like
// the RC-loss failsafe in Locomotion::neutralise()), 0 = "coast".
//
// IMPORTANT — keep this 1 in fake-RPM mode (FLIPPER_USE_LEGACY_RPM_LISP). That
// lisp path has no enable/coast bit, so enable=0 does NOT free-wheel: it maps to
// target 0 (CANInterface::sendFlipperAngles → command_deg = 0), which drives
// EVERY flipper to its zero angle on e-stop — they appear to jump to "random"
// positions. Holding (=1) freezes them in place with no jump. Only the
// custom-frame (non-fake-RPM) path actually coasts when this is 0.
#define FLIPPER_ESTOP_HOLD        1

// ─── Gripper (end-effector servo, local hobby PWM — ARM board only) ──────────
// A single JX CLS-12V7346 coreless servo on a direct ESP32 PWM pin drives the
// gripper (one servo moves both jaws via a linkage). It lives on the ARM PCB
// (the arm's end-effector) and is the only PWM actuator — everything else is
// CAN. The open/close COMMAND comes from the workstation gamepad's RT/LT
// triggers: the joystick node sends a signed rate on /gripper (+1 open … −1
// close) → bridge → MSG_GRIPPER (PC→ESP, routed to the ARM link). The firmware
// integrates that rate into a clamped target angle at GRIPPER_RATE_DPS and holds
// at the limits. Power the servo from the 11.1–15 V rail (NOT the ESP32) and
// common-ground the signal. Bench sketch: firmware/servo_test.
// The Gripper module only runs when ROBOCOREA_ROLE_IS_ARM (see main.cpp/Control).
#define PIN_GRIPPER_SERVO       26     // ARM-board GPIO for the servo signal
#define GRIPPER_LEDC_CHANNEL     0     // LEDC channel (no other PWM on the arm board)
#define GRIPPER_PWM_HZ          50     // 50 Hz standard; the JX servo also accepts up to 330
#define GRIPPER_PWM_BITS        16     // LEDC duty resolution
#define GRIPPER_PULSE_MIN_US   500     // 0.5 ms → 0 deg  (JX control-board full range)
#define GRIPPER_PULSE_MAX_US  2500     // 2.5 ms → 180 deg
#define GRIPPER_CLOSED_DEG     40.0f    // fully-closed servo angle  (TODO: set mechanically)
#define GRIPPER_OPEN_DEG     150.0f    // fully-open servo angle    (TODO: set mechanically)
#define GRIPPER_RATE_DPS      90.0f    // open/close speed at full trigger (deg/s)
#define GRIPPER_CMD_TIMEOUT_MS 500     // no fresh command within this → stop stepping (hold)

// ─── Arm: ODrive (J1–J3) ─────────────────────────────────────────────────────
// CANSimple, COB-ID = (node_id << 5) | cmd_id. Node IDs from the legacy bridge.
#define ODRIVE_NUM_JOINTS         3
#define ODRIVE_NODE_J1         0x10
#define ODRIVE_NODE_J2         0x11
#define ODRIVE_NODE_J3         0x12
#define ODRIVE_GEAR_J1        48.0f
#define ODRIVE_GEAR_J2        48.0f
#define ODRIVE_GEAR_J3        48.0f
#define ODRIVE_DIR_J1        (-1.0f)
#define ODRIVE_DIR_J2        (-1.0f)
#define ODRIVE_DIR_J3        (-1.0f)
#define ODRIVE_ZERO_TIMEOUT_MS  400    // blocking timeout for startup encoder-zero RTR (per try)
#define ODRIVE_INIT_MAX_RETRIES   3    // retry closed-loop + encoder-zero capture this many times
#define ODRIVE_INIT_RETRY_DELAY_MS 150
#define ODRIVE_ENABLE_ERROR_POLL  1    // include Get_Error in the telemetry round-robin
// ODrive e-stop: 1 = native hard stop (cmd 0x02, can latch until clear/reboot),
//                0 = request AXIS_STATE_IDLE like the working Dicerox firmware.
#define ODRIVE_ESTOP_USE_NATIVE   0

// ─── Arm: ZE300 (J4) ─────────────────────────────────────────────────────────
// Standard 11-bit. Tagged request ID = ZE300_REQ_ID_BASE | device_id; reply ID = device_id.
// Command in output degrees; driver handles its 1:8 gearbox (16384 counts/output-rev).
#define ZE300_ID_J4              13
#define ZE300_REQ_ID_BASE     0x100
#define ZE300_COUNTS_PER_REV  16384
// J4 direction: -1 matches the reference Dicerox arm's net working direction
// (its PC bridge applies -1 for J4 and the ZE300 firmware applies no sign; we
// apply the whole sign here). Still re-verify on bench against the final
// workstation joint-command sign convention.
#define ZE300_DIR_J4          (1.0f)
#define ZE300_MAX_SPEED_CRPM    417    // 25 output deg/s -> 4.17 RPM -> 417 centi-RPM
#define ZE300_ZERO_TIMEOUT_MS   250    // blocking timeout for startup RTRs (per try)
#define ZE300_INIT_MAX_RETRIES    5    // B2 (set-speed) only ACKs after a power-on warmup — retry
#define ZE300_INIT_RETRY_DELAY_MS 100
#define ZE300_TELEM_INTERVAL_MS 200    // realtime-state poll period (~5 Hz)

// ─── Arm: LKTech / MyActuator (J5–J6) ────────────────────────────────────────
// Standard 11-bit, ID = LKTECH_ID_BASE + motor_id (request and reply share it).
#define LKTECH_NUM_JOINTS         2
#define LKTECH_ID_BASE        0x140
#define LKTECH_ID_J5             14
#define LKTECH_ID_J6             15
#define LKTECH_GEAR_J5        10.0f
#define LKTECH_GEAR_J6        10.0f
// J5 direction: -1 matches the reference Dicerox arm's net working direction
// (its PC bridge applies -1 for J5, LKTech firmware applies no sign). J6 is +1
// in the reference. Re-verify both on bench against the workstation sign.
#define LKTECH_DIR_J5         (1.0f)
#define LKTECH_DIR_J6         (1.0f)
#define LKTECH_DEFAULT_SPEED_DPS  250  // 25 output deg/s × 10:1 gear = 250 motor deg/s
#define LKTECH_ZERO_TIMEOUT_MS    250
#define LKTECH_INIT_MAX_RETRIES    5   // motor-on / read-angle intermittent on first txns — retry
#define LKTECH_INIT_RETRY_DELAY_MS 100

// ─── Arm joint soft limits ───────────────────────────────────────────────────
// Firmware-side travel clamps applied in sendArmJoints() as a safety backstop
// behind MoveIt's own joint_limits (which own the real envelope on the
// workstation). Values are the reference Dicerox arm's working range
// (dicerox moveit_arm_bridge JOINT_MAPPINGS). They are applied to the joint
// command as received (J1..J6 output degrees) BEFORE the per-joint dir/gear —
// confirm the frame matches the final workstation sign convention on the bench.
// Set ARM_SOFT_LIMIT_ENABLE 0 to disable.
#define ARM_SOFT_LIMIT_ENABLE     1
#define ARM_LIMIT_J1_MIN_DEG   (-90.0f)
#define ARM_LIMIT_J1_MAX_DEG    (90.0f)
#define ARM_LIMIT_J2_MIN_DEG     (0.0f)
#define ARM_LIMIT_J2_MAX_DEG   (180.0f)
#define ARM_LIMIT_J3_MIN_DEG  (-200.0f)
#define ARM_LIMIT_J3_MAX_DEG     (0.0f)
#define ARM_LIMIT_J4_MIN_DEG   (-90.0f)
#define ARM_LIMIT_J4_MAX_DEG    (90.0f)
#define ARM_LIMIT_J5_MIN_DEG   (-90.0f)
#define ARM_LIMIT_J5_MAX_DEG    (90.0f)
#define ARM_LIMIT_J6_MIN_DEG   (-90.0f)
#define ARM_LIMIT_J6_MAX_DEG    (90.0f)

// ─── Arm safety lifecycle ────────────────────────────────────────────────────
// Ported from the reference Dicerox arm's safety redesign: the arm boots PASSIVE
// (CAN HAL only — no motor enable, no zero capture, no motion) and must be armed
// EXPLICITLY (MSG_ARM_INIT) before it will accept joint commands. A latched CAN
// fault disarms the arm and rejects motion until a fresh arm. States:
//   0 UNINIT (passive/disarmed) · 1 INITIALIZING · 2 READY · 3 FAULT
#define ARM_PASSIVE_BOOT          1    // 1 = boot disarmed (safe); 0 = legacy auto-arm at boot

// On e-stop (RC Ch6-down, software, or mirrored chassis stop): 1 = HOLD the arm
// at its last commanded pose with the motors still energized, so the (heavy,
// non-backdrivable-enough) arm freezes in place instead of going limp and
// falling under gravity. 0 = legacy behaviour: de-energize every joint (ODrive
// IDLE / LKTech off / ZE300 disable → arm droops) and drop to UNINIT (a fresh
// /arm/arm is then required). Mirrors FLIPPER_ESTOP_HOLD. Disarm and latched
// faults still de-energize regardless of this flag — only e-stop holds.
#define ARM_ESTOP_HOLD            1

// Bench-only arm test mode. Set to 1 only when the PCB has no RC receiver/PPM
// input during an arm-only desk test: the control task stays in ARM mode even
// without RC, neutralizes base outputs, and accepts MSG_ARM_JOINTS from the
// laptop. The arm lifecycle gate still applies, so you must call /arm/arm before
// motion. Keep this 0 for the robot.
#define ARM_BENCH_MODE_NO_PPM     0

// Latched fault: count failures inside a sliding window; exceed → FAULT + disarm.
#define ARM_FAULT_WINDOW_MS    1000
#define ARM_CAN_FAIL_THRESHOLD    5    // failed arm-send cycles in the window → CAN fault
#define ARM_MOTOR_FAIL_THRESHOLD  3    // (reserved) motor-command failures → fault

// First ZE300 (J4) command after arming is clamped to ±this from captured zero,
// to catch a bad startup pose before a large move.
#define ARM_JOINT4_FIRST_CMD_MAX_DEG  30.0f

// Firmware-side velocity/accel ramp on the arm target (deg/s, deg/s²), J1..J6 —
// a backstop behind the workstation servo's own limiting. Values from the
// reference's coordinated-motion defaults.
#define ARM_COMMAND_PERIOD_MS    50    // match working Dicerox 6-DOF stream interval
#define ARM_RAMP_ENABLE           1
#define ARM_RAMP_MAX_VEL_DPS   { 12.5f, 10.0f, 10.0f, 25.0f, 25.0f, 25.0f }
#define ARM_RAMP_MAX_ACC_DPS2  { 37.5f, 30.0f, 30.0f, 75.0f, 75.0f, 75.0f }

// ─── Sensors ─────────────────────────────────────────────────────────────────
// Passive victim-detection sensors are Jetson-hosted now: LIS3MDL magnetometer
// and MLX90640 thermal over Linux I2C, with orientation from the ZED2. These
// mask bits remain reserved so the wire protocol numbering stays stable; the
// Jetson sensor nodes consume /sensors/enable_mask directly.
#define SENSOR_BIT_MAG       (1 << 0)  // reserved (handled on the Jetson)
#define SENSOR_BIT_THERMAL   (1 << 1)  // reserved (handled on the Jetson)
#define SENSOR_BIT_GAS       (1 << 2)  // reserved (sensor removed)
#define SENSOR_BIT_IMU       (1 << 3)  // reserved (no IMU on the ESP32; orientation from ZED2)

#define SENSOR_MAG_HZ           50

// ─── Jetson binary protocol ──────────────────────────────────────────────────
// Frame: [0xAA][0x55][TYPE:1][LEN_H:1][LEN_L:1][PAYLOAD:LEN][CRC:1]
// CRC = XOR of TYPE, LEN_H, LEN_L, and all payload bytes.
#define PROTO_SOF_0          0xAA
#define PROTO_SOF_1          0x55
#define PROTO_MAX_PAYLOAD    1600

// ESP32 → PC. (Type numbers kept identical to the legacy protocol so the
// existing GUI/bridge stay compatible; 0x02 thermal / 0x03 mag / 0x04 gas /
// 0x09 main-PWM
// are reserved-unused on this robot.)
#define MSG_TELEMETRY        0x01      // PPM + state + track speed + flipper angle, 50 Hz
#define MSG_SENSOR_THERMAL   0x02      // reserved (thermal is published by the Jetson)
#define MSG_SENSOR_MAG       0x03      // reserved (magnetometer is published by the Jetson)
#define MSG_SENSOR_GAS       0x04      // reserved (sensor removed)
#define MSG_STATUS           0x05      // system status / heartbeat
#define MSG_SENSOR_IMU       0x06      // reserved (no IMU on the ESP32; orientation from ZED2)
#define MSG_ENCODER_EXT      0x07      // 4 flipper output angles (FL,FR,RL,RR)
#define MSG_VESC_STATUS      0x08      // per-VESC feedback (incl. tachometer for track odometry)
#define MSG_MOTOR_MAIN       0x09      // reserved (was ROBOT_MAIN PWM duties)
#define MSG_ODRIVE_STATUS    0x0A      // arm J1–J3 telemetry
#define MSG_LKTECH_STATUS    0x0B      // arm J5–J6 telemetry
#define MSG_ZE300_STATUS     0x0C      // arm J4 telemetry
#define MSG_ODRIVE_ERROR     0x0D      // arm ODrive error snapshot (optional)
#define MSG_ARM_LIFECYCLE    0x0E      // arm safety state + fault + diagnostics
#define MSG_BOARD_IDENTITY   0x0F      // role + protocol version + capability bits

// PC → ESP32
#define MSG_ARM_JOINTS       0x10      // 6 × int16 joint angles (deg × 100)
#define MSG_SENSOR_ENABLE    0x11      // reserved legacy 1-byte bitmask
#define MSG_ESTOP            0x12      // 0-byte payload — immediate stop
#define MSG_ESTOP_CLEAR      0x13      // 0-byte payload — resume
#define MSG_KEYBIND          0x14      // RESERVED — the per-channel keybind table was
                                       // replaced by the fixed RC control scheme
                                       // (firmware Control.cpp); id kept reserved.
#define MSG_PPM_CALIB        0x15      // 6ch × (min,neutral,max) u16 + deadband
#define MSG_GRIPPER          0x16      // PC→ESP(ARM): gripper open/close rate, int16 ×1000 (+open/−close)
#define MSG_ARM_INIT         0x17      // 0-byte — explicit arm/init (passive boot)
#define MSG_ARM_DISARM       0x18      // 0-byte — disarm the arm motors
#define MSG_ARM_MODE         0x19      // 1 byte — 0 dexterity, 1 chassis/transport
#define MSG_TRACTION_CMD     0x1A      // 2×int16 normalised L/R track speed ×1000 + u8 enable
                                       // (autonomy: the Jetson bridge derives this from Nav2 /cmd_vel)

// External (ROS/Nav2 /cmd_vel) traction command. Autonomy only drives the tracks
// while the RC link is up, the drive sticks are within deadband, and a fresh
// command has arrived — touching a stick, engaging virtual-flip, losing the RC
// link, or letting the command go stale all instantly reclaim manual control.
#define EXT_DRIVE_TIMEOUT_MS 300       // stale external command → fall back to RC

// ─── FreeRTOS task layout ────────────────────────────────────────────────────
// Core 0: protocol (comms + CAN).  Core 1: control.
#define TASK_CORE_COMMS      0
#define TASK_CORE_CAN        0
#define TASK_CORE_CONTROL    1

#define STACK_CONTROL     5120
#define STACK_COMMS       4096
#define STACK_CAN         4096

#define PRIO_CONTROL         5      // highest — real-time loop
#define PRIO_COMMS           4
#define PRIO_CAN             4

#define CONTROL_LOOP_HZ     50
#define CAN_POLL_HZ        200
