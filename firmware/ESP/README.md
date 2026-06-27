# RoboCorea ESP32 firmware

PlatformIO project (DOIT ESP32 DevKit V1, custom PCB) for the robot's real-time
I/O. Two identical PCBs run this same firmware: one built as **chassis** and one
built as **arm**. The only intended per-board difference is
`ROBOCOREA_BOARD_ROLE` in `include/config.h`.

For the whole-system picture see [`../../reference/architecture.md`](../../reference/architecture.md).

---

## What it does

| Subsystem | Description |
|-----------|-------------|
| **RC input** | Decodes a 6-channel PPM stream from the FlySky FS-iA6B on `GPIO4` (ISR). |
| **Board role** | `ROBOCOREA_BOARD_ROLE_CHASSIS` owns RC, traction, flippers, and wheel-odom VESC telemetry. `ROBOCOREA_BOARD_ROLE_ARM` owns ODrive/ZE300/LKTech arm CAN. |
| **Control scheme (fixed)** | No keybind table. Ch3=traction fwd, Ch4=turn, Ch2=flipper rate, Ch1=flipper L/R selector (min/center/max = left/both/right), Ch5=2-state pair select (min=front FL·FR, max=rear RL·RR), Ch6=3-position lever (down=E-STOP, center=normal, up=virtual-flip). Drive + flippers are always active together. See `config.h` "Channel roles". |
| **Virtual flip (Ch6 up)** | "Drive from the other end" — a 180° remap of the control frame (negate forward, swap front/rear pair, mirror flipper L/R; turn is left as-is) so the symmetric robot can back out of a dead end without turning around. Signs are `VFLIP_*` macros in `config.h`. |
| **Traction** | 2 VESCs, differential drive, `SET_RPM` velocity commands. |
| **Flippers** | 4 VESCs. The **position loop runs on the VESC** (LispBM — see [`../VESC/flipper_position.lisp`](../VESC/flipper_position.lisp)). The ESP integrates the stick into a target angle, sends it through the fake-RPM carrier, and reports measured `[FL, FR, RL, RR]` angles from STATUS_5 tachometer feedback. Center stick = hold; no separate encoders. |
| **Arm relay** | Arm-role firmware relays workstation joint commands (gamepad → IK) to CAN whenever armed & not e-stopped: ODrive J1–J3, ZE300 J4, LKTech J5–J6. |
| **Arm operating mode** | Dexterity controls J1–J6. Chassis/transport keeps J1–J4 controlled and sends LKTech J5/J6 to motor-stop (torque-off). |
| **Sensors** | The **arm PCB** hosts the LIS3MDL magnetometer + MLX90640 thermal camera on its I2C bus (low-priority tasks, forwarded over UART, republished by `esp32_bridge`). The chassis has no passive sensors. Orientation comes from the ZED2; there is no gas sensor. |
| **Protocol** | Binary UART at 921600 baud to the Jetson. Each board periodically sends `MSG_BOARD_IDENTITY`, then only publishes the telemetry owned by its role. |

---

## Architecture (FreeRTOS)

| Core | Task | Rate | Prio | Purpose |
|------|------|------|------|---------|
| 1 | `controlTask` | 50 Hz | 5 | Chassis: RC/base FSM. Arm: relay latest joint command while armed. |
| 0 | `commsTask`   | 50 Hz | 4 | UART RX parse + identity + role-owned telemetry TX |
| 0 | `canTask`     | 200 Hz | 4 | Chassis: VESC CAN. Arm: ODrive/ZE300/LKTech CAN. |

All UART transmission is funnelled through a mutex in `Comms` so frames from
`commsTask` and the occasional gripper frame from `controlTask` never interleave.

---

## CAN bus

Each PCB has one **500 kbps** bus (`config.h: CAN_BITRATE_BPS`). The chassis bus
has the 6 VESCs only; the arm bus has the 3 ODrives, ZE300, and 2 LKTech motors.
(To move to 1 Mbps, change that constant **and** set every controller's CAN baud
in its tool.)

**Transceiver backend** is a compile-time switch in `config.h` — the rest of the
CAN code goes through a small HAL and doesn't care which is used:

```c
#define CAN_BACKEND_MCP2515     // on-board SMD MCP2515 over SPI (active; CS=5, SCK=18, MISO=19, MOSI=23, 8 MHz)
// #define CAN_BACKEND_TWAI     // future ESP32 TWAI + SN65HVD230 (PLACEHOLDER pins)
```

This PCB has an **MCP2515 soldered on** (permanent) on the SPI pins above, so
GPIO21/22 stay dedicated to I2C. The TWAI backend stays compilable for a future
board but its TX/RX pins are placeholders. The HAL also adds **bus-off / fault
recovery** and a cross-core SPI mutex for the MCP2515 path.

- **VESC traction (L/R)** — extended frames, ID = `(cmd<<8)|id`, `SET_RPM`
  (cmd 3). Status frames 1/4/5 give speed/temp/voltage telemetry.
- **VESC flippers (×4)** — position loop runs **on the VESC** (LispBM). The
  reliable command path is the legacy fake-RPM carrier:
  `SET_RPM = target_degrees * 1000`, read by the Lisp with `get-rpm-set`.
  Measured flipper angle comes from each VESC's STATUS_5 tachometer when
  `FLIPPER_USE_TACH_FEEDBACK=1`, so manual movement can show up in
  `/encoders/flipper` while the VESC is powered and broadcasting STATUS_5.
- **ODrive** (J1–J3) — arm role only; CANSimple, `SET_INPUT_POS`; encoder zero captured at boot.
- **ZE300** (J4) — arm role only; output-degree position; boot pose captured as zero.
- **LKTech** (J5–J6) — arm role only; multi-loop angle control; boot pose captured as zero.

VESC IDs (`60/50` traction, `20/10/40/30` flippers), gear ratios
(`TRACTION_GEAR_RATIO=23.333`, `FLIPPER_GEAR_RATIO=100`), and direction signs
live in `config.h` and **must be confirmed on the bench** (see the TODO list
there and in the architecture doc). Enable VESC status frames 1/4/5 in VESC
Tool for traction/voltage/temperature telemetry.

---

## Flippers: position loop on the VESC

The flipper position loop runs **on each flipper VESC** in LispBM
([`../VESC/flipper_position.lisp`](../VESC/flipper_position.lisp)) — PD + stiction
feedforward with shortest-path error on a wrapped `[0,360)` angle, so crossing
0/360 is smooth. The ESP32's jobs are:

1. **Integrate the stick into a target angle** (`controlTask`, 50 Hz): stick
   deflection = rate (`FLIPPER_RATE_DPS`), so the target moves while deflected and
   **holds** where released. Direction sign is applied here (`FLIPPER_DIR_*`).
2. **Send the target** through fake RPM (`SET_RPM = degrees * 1000`). The Lisp
   treats it as a position setpoint, not a speed command. On (re)entering flipper
   control the ESP seeds the target from measured angle for a **bumpless** start.
3. **Derive measured angle** from the VESC STATUS_5 tachometer for telemetry.
   Enable STATUS_5 broadcasts on all four flipper VESCs.

Failsafe: on RC-loss the ESP keeps re-sending the frozen target (**hold**, never
"go home"). In fake-RPM mode, hard e-stop target behavior follows
`FLIPPER_ESTOP_HOLD`, but there is no separate Lisp enable/coast bit. The Lisp's
internal angle scale (`deg-per-dist`) and the ESP tachometer scale
(`FLIPPER_TACH_DEG_PER_COUNT_*`) both need bench calibration.

### Fake-RPM + Tach Hybrid

Default bring-up mode:

```c
#define FLIPPER_USE_LEGACY_RPM_LISP  1
#define FLIPPER_USE_TACH_FEEDBACK    1
```

This avoids the custom `0x7E/0x7F` Lisp CAN path. The VESC still closes position
locally, while the ESP uses tachometer feedback for `/encoders/flipper` and
bumpless target seeding. The tachometer angle is relative to the first
STATUS_5 frame after ESP boot plus
`FLIPPER_TACH_ZERO_DEG_*`, then folded into `[0,360)` so full turns are dropped.
With the current sign calibration, a 90 degree downward move reports as `270`.
Power up in a known flipper pose or set offsets.

---

## Binary protocol

Frame: `[0xAA][0x55][TYPE][LEN_H][LEN_L][PAYLOAD][CRC]`, CRC = XOR of TYPE, the
two length bytes, and the payload. Payload layouts are the packed structs in
[`include/robot_types.h`](include/robot_types.h) (guarded by `static_assert`s),
and must stay in sync with the Jetson bridge `struct` formats.

| Dir | Type | Name |
|-----|------|------|
| → PC | 0x01 | Telemetry (mode, flags, PPM[6], track speeds, flipper angle, uptime) |
| → PC | 0x02 | Thermal frame (arm PCB: seq, min/max °C×100, cols/rows, 768 quantised px) |
| → PC | 0x03 | Magnetometer (arm PCB: LIS3MDL XYZ, 3× int16 µT) |
| → PC | 0x05 | Status |
| → PC | 0x07 | Flipper angles (FL,FR,RL,RR) |
| → PC | 0x08 | VESC status (incl. tachometer → track odometry) |
| → PC | 0x0A / 0x0B / 0x0C / 0x0D | ODrive / LKTech / ZE300 status, ODrive error |
| → PC | 0x0E / 0x0F | Arm lifecycle / board identity |
| ← PC | 0x10 | Arm joints (6 × int16 deg×100) |
| ← PC | 0x11 | Sensor enable mask (arm PCB: bit0 mag, bit1 thermal) |
| ← PC | 0x12 / 0x13 | E-stop / clear |
| ← PC | 0x19 | Arm operating mode: `0` dexterity, `1` chassis |
| ← PC | 0x14 | *(reserved — was the keybind table; RC scheme is fixed now)* |
| ← PC | 0x15 | PPM calibration |
| ← PC | 0x16 | Gripper (→ PC originates; reserved) |

(0x02 thermal + 0x03 magnetometer are sent by the **arm PCB** and republished by
`esp32_bridge`. 0x04 gas, 0x06 IMU, 0x09 main-PWM stay reserved-unused but the
numbering is kept stable for GUI compatibility. Orientation comes from the ZED2
camera on the Jetson, not the ESP32.)

The Jetson bridge routes outbound frames by role. Chassis ignores arm-only frames;
arm ignores chassis-only frames; both accept software e-stop frames.

---

## Modes & arm relay

Chassis role: `INIT → STANDBY → NORMAL`, plus `ESTOP` (the legacy
`FLIPPER`/`ARM` enum values are reserved-unused). Once the RC link is up, it
drives the tracks **and** flippers from the fixed scheme every loop.

Arm role: starts without PPM, accepts arm lifecycle commands, and relays the
latest workstation arm-joint stream whenever not e-stopped and the arm lifecycle
is `READY`. Chassis RC e-stop is mirrored to the arm by the Jetson bridge.

The arm operating mode is independent of the robot's high-level `ARM` mode.
`DEXTERITY` is the boot default. `CHASSIS` gates all J5/J6 position frames before
sending LKTech motor-stop, and the gate remains in force across disarm/re-arm.
J5/J6 are enabled again only by an explicit switch back to `DEXTERITY`.

---

## Project layout

```
include/  config.h        board role, pins, CAN/VESC IDs, gear ratios, flipper params, protocol IDs
          robot_types.h   enums, structs, packed payloads (+ static_asserts)
lib/      RC/             PPM decode (ISR) + calibration
          Control/        state machine, fixed RC control mapping, flipper setpoint integration
          Locomotion/     drivetrain output (track mix + flipper angle/hold)
          CANInterface/   CAN HAL (MCP2515/TWAI) + VESC/ODrive/ZE300/LKTech
          Comms/          binary UART protocol
          Sensors/        arm-PCB I2C: LIS3MDL mag + MLX90640 thermal (ARM role)
          Gripper/        end-effector servo (LEDC PWM, ARM role)
          PID/            reusable PID (linear + shortest-angle) — spare
src/      main.cpp        setup() + FreeRTOS tasks (arm adds sensor + thermal tasks)

../VESC/  flipper_position.lisp   position loop that runs ON the flipper VESCs
```

---

## Build & flash

```bash
pio run            # build
pio run -t upload  # flash
pio device monitor # only shows text if ENABLE_COMMS is disabled in config.h
```

Leave `ROBOCOREA_BOARD_ROLE` as `ROBOCOREA_BOARD_ROLE_CHASSIS` for the chassis
PCB. Change only that macro to `ROBOCOREA_BOARD_ROLE_ARM` before building and
flashing the arm PCB. Confirm pins, CAN IDs, gear ratios, directions, and PID
gains against the real hardware before commanding motors. Bring up motors one at
a time with small commands.
