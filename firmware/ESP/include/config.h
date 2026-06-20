#pragma once
//
// RoboCorea — ESP32 firmware configuration
// =========================================
// Single robot. Every actuator hangs off ONE 500 kbps CAN 2.0 bus through the
// on-board SMD MCP2515 (TWAI + SN65HVD230 kept compilable for a future board):
//   • 6 VESC mini 6.7 Pro  — 2 traction (velocity) + 4 flippers (position loop
//                            runs ON the VESC in LispBM; the ESP owns the target)
//   • 3 ODrive (CANSimple)  — arm J1–J3
//   • 1 ZE300              — arm J4
//   • 2 LKTech/MyActuator  — arm J5–J6
//
// Pin numbers below were inherited from the legacy Dicerox bring-up and MUST be
// reconciled against the actual RoboCorea PCB before flashing.
//
// See reference/architecture.md for the full system picture.

// ─── I2C (LIS3MDL magnetometer) ──────────────────────────────────────────────
// Pins per 1.ino/2.ino (the working board). The on-board magnetometer is a
// LIS3MDL and the Sensors lib uses the Adafruit LIS3MDL driver. There is no IMU
// on the ESP32 — orientation comes from the ZED2 camera on the Jetson.
#define PIN_I2C_SDA          21
#define PIN_I2C_SCL          22

// ─── CAN backend selection ───────────────────────────────────────────────────
// Pick exactly ONE transceiver backend. The rest of the CAN code is
// backend-agnostic (see lib/CANInterface — it talks through a small HAL).
//
// This PCB has an MCP2515 soldered on (SMD, permanent) on the SPI pins below, so
// MCP2515 is the active backend and GPIO21/22 stay dedicated to I2C. The TWAI
// backend is kept compilable for a future board; its pins are PLACEHOLDERS —
// pick real free GPIOs then (21/22 are NOT available, the MCP owns nothing there
// but the mag does).
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

// Channel roles (1-indexed to match the physical FlySky labels)
#define PPM_CH_MODE           5      // Ch5 — 3-position lever selects the keybind row
#define PPM_CH_ESTOP          6      // Ch6 — dedicated hardware e-stop

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
#define FLIPPER_DIR_FL        (1.0f)
#define FLIPPER_DIR_FR        (1.0f)
#define FLIPPER_DIR_RL        (1.0f)
#define FLIPPER_DIR_RR        (1.0f)

// Mechanical reduction (motor → output shaft). Same gearbox on all six motors.
#define DRIVE_GEAR_RATIO       100.0f

// VESC SET_RPM commands eRPM = mechanical_rpm × pole_pairs.
// Used to scale traction stick → eRPM and to convert eRPM feedback → output RPM.
#define VESC_POLE_PAIRS          7      // TODO: confirm motor pole-pair count

// Traction: full-stick output speed (eRPM sent to the traction VESCs).
#define TRACTION_ERPM_MAX     8000      // TODO: tune for the actual motors

// ── Flippers: position loop runs ON the VESC (firmware/VESC/flipper_position.lisp)
// The ESP32 integrates the stick into a wrapped target angle and sends it to the
// flipper VESC over a custom CAN frame; the VESC closes the loop and reports its
// measured angle back. The PD/feedforward gains and the angle scale live in the
// LISP, not here — see that file.
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

// On hard e-stop: 1 = hold position (VESC keeps the loop closed), 0 = coast
// (ESP sends enable=0 → lisp set-current 0). RC-loss always HOLDS (never homes).
#define FLIPPER_ESTOP_HOLD        0

// ── Flipper collision avoidance (dynamic joint limits) ──────────────────────
// The front and rear flipper on the SAME side share a side-view plane and can
// collide when they lean toward each other (rear up-forward + front up-back
// meeting over the chassis, and the mirror image below). Geometry, per side:
//   • both pivots lie on the chassis FLIPPER_PIVOT_SPACING_M apart (same height);
//   • each flipper is a FLIPPER_LENGTH_M segment from its pivot to the tip;
//   • 0° = pointing straight forward (+x), angle increases toward "up".
// A collision zone exists only when 2·length > spacing > length (long enough to
// reach each other, but pivots not so close they always overlap). The ESP clamps
// each flipper's integrated TARGET so its segment never closes to within
// FLIPPER_COLLISION_MARGIN_M of the paired flipper's MEASURED segment; a step
// that would close the gap below the margin is refused (the flipper holds at the
// boundary and frees the instant the stick reverses or the other flipper backs
// off). The VESC still owns the actual position loop. Math + the 2y>x>y guard
// live in lib/Control/FlipperCollision.h.
#define FLIPPER_COLLISION_AVOID_ENABLE  1

// Side-view geometry. PLACEHOLDERS — measure on the real chassis. Must satisfy
// 2·FLIPPER_LENGTH_M > FLIPPER_PIVOT_SPACING_M > FLIPPER_LENGTH_M (a static_assert
// in FlipperCollision.h enforces this whenever the feature is enabled).
#define FLIPPER_PIVOT_SPACING_M   0.52f   // x: front↔rear pivot center distance (TODO measure)
#define FLIPPER_LENGTH_M          0.38f   // y: pivot to tip (TODO measure)

// Clearance kept between the two flipper centerlines: this flipper's half-width
// + the other's half-width + a safety buffer. The buffer MUST exceed the tip
// travel of both flippers in one control tick
//   2 · FLIPPER_LENGTH_M · FLIPPER_RATE_DPS · (π/180) / CONTROL_LOOP_HZ
// so a single step can never jump from "clear" to "touching". TODO: set from the
// real flipper width plus a few mm of safety.
#define FLIPPER_COLLISION_MARGIN_M  0.1f

// Per-flipper mapping from the VESC-reported angle to the shared geometric frame
// (0° = forward, + = up/CCW):  geo = SIGN · (reported_deg − OFFSET_DEG). Use
// these if a flipper's zero isn't "forward" or it rotates the opposite way (e.g.
// a mirrored mount). Default: identity (reported angle is already in-frame).
#define FLIPPER_GEO_OFFSET_FL   0.0f
#define FLIPPER_GEO_OFFSET_FR   0.0f
#define FLIPPER_GEO_OFFSET_RL   0.0f
#define FLIPPER_GEO_OFFSET_RR   0.0f
#define FLIPPER_GEO_SIGN_FL   (1.0f)
#define FLIPPER_GEO_SIGN_FR   (1.0f)
#define FLIPPER_GEO_SIGN_RL   (1.0f)
#define FLIPPER_GEO_SIGN_RR   (1.0f)

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

// Bench-only arm test mode. Set to 1 only when the PCB has no RC receiver/PPM
// input during an arm-only desk test: the control task stays in ARM mode even
// without RC, neutralizes base outputs, and accepts MSG_ARM_JOINTS from the
// laptop. The arm lifecycle gate still applies, so you must call /arm/arm before
// motion. Keep this 0 for the robot.
#define ARM_BENCH_MODE_NO_PPM     1

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
// Only the magnetometer (LIS3MDL) lives on the ESP32. The MLX90640 thermal
// camera is on the Jetson; the legacy MQ2 gas sensor is removed; orientation
// comes from the ZED2 camera on the Jetson (no IMU on the ESP32).
// The LIS3MDL uses the Adafruit driver's default I2C address.

// Sensor-enable bitmask bits (PC → ESP32). Thermal/gas/IMU bits are reserved but
// unused on this robot; only MAG does anything.
#define SENSOR_BIT_MAG       (1 << 0)
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
// existing GUI/bridge stay compatible; 0x02 thermal / 0x04 gas / 0x09 main-PWM
// are reserved-unused on this robot.)
#define MSG_TELEMETRY        0x01      // PPM + state + track speed + flipper angle, 50 Hz
#define MSG_SENSOR_THERMAL   0x02      // reserved (thermal is published by the Jetson)
#define MSG_SENSOR_MAG       0x03      // LIS3MDL XYZ
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

// PC → ESP32
#define MSG_ARM_JOINTS       0x10      // 6 × int16 joint angles (deg × 100)
#define MSG_SENSOR_ENABLE    0x11      // 1-byte bitmask
#define MSG_ESTOP            0x12      // 0-byte payload — immediate stop
#define MSG_ESTOP_CLEAR      0x13      // 0-byte payload — resume
#define MSG_KEYBIND          0x14      // 15 bytes: 3 modes × 5 channel slots
#define MSG_PPM_CALIB        0x15      // 6ch × (min,neutral,max) u16 + deadband
#define MSG_GRIPPER          0x16      // int16 normalised × 1000
#define MSG_ARM_INIT         0x17      // 0-byte — explicit arm/init (passive boot)
#define MSG_ARM_DISARM       0x18      // 0-byte — disarm the arm motors

// ─── FreeRTOS task layout ────────────────────────────────────────────────────
// Core 0: protocol (comms + CAN).  Core 1: control + sensors.
#define TASK_CORE_COMMS      0
#define TASK_CORE_CAN        0
#define TASK_CORE_CONTROL    1
#define TASK_CORE_SENSORS    1

#define STACK_CONTROL     5120
#define STACK_COMMS       4096
#define STACK_CAN         4096
#define STACK_SENSORS     4096

#define PRIO_CONTROL         5      // highest — real-time loop
#define PRIO_COMMS           4
#define PRIO_CAN             4
#define PRIO_SENSORS         2      // background

#define CONTROL_LOOP_HZ     50
#define CAN_POLL_HZ        200
