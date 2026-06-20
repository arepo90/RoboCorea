# RoboCorea ESP32 firmware

PlatformIO project (DOIT ESP32 DevKit V1, custom PCB) that runs the robot's
real-time I/O: it decodes the RC receiver, drives every motor over a single CAN
bus, reads the magnetometer, and talks to the Jetson over USB serial.

For the whole-system picture see [`../../reference/architecture.md`](../../reference/architecture.md).

---

## What it does

| Subsystem | Description |
|-----------|-------------|
| **RC input** | Decodes a 6-channel PPM stream from the FlySky FS-iA6B on `GPIO4` (ISR). |
| **Keybind / modes** | Ch5 (3-position lever) selects one of three keybind rows. Each row maps Ch1–Ch4 + Ch6 to a function (traction, flippers, arm axes, e-stop). The GUI can replace the table at runtime. |
| **Traction** | 2 VESCs, differential drive, `SET_RPM` velocity commands. |
| **Flippers** | 4 VESCs. The **position loop runs on the VESC** (LispBM — see [`../VESC/flipper_position.lisp`](../VESC/flipper_position.lisp)). The ESP integrates the stick into a target angle and sends it over a custom CAN frame; the VESC closes the loop and reports its angle back. Center stick = hold; no separate encoders. |
| **Arm relay** | In ARM mode the tracks stop and the latest joint command from the PC is sent to the arm motors over CAN: ODrive J1–J3, ZE300 J4, LKTech J5–J6. |
| **Arm operating mode** | Dexterity controls J1–J6. Chassis/transport keeps J1–J4 controlled and sends LKTech J5/J6 to motor-stop (torque-off). |
| **Sensors** | LIS3MDL magnetometer over I2C. (No IMU on the ESP32 — orientation comes from the ZED2 camera on the Jetson; the thermal camera is on the Jetson; there is no gas sensor.) |
| **Protocol** | Binary UART at 921600 baud to the Jetson. Telemetry at 50 Hz; the PC sends arm joints, keybinds, PPM calibration, sensor-enable, and e-stop. |

---

## Architecture (FreeRTOS)

| Core | Task | Rate | Prio | Purpose |
|------|------|------|------|---------|
| 1 | `controlTask` | 50 Hz | 5 | State machine, keybinds, flipper setpoint integration, motor output |
| 1 | `sensorTask`  | ~50 Hz | 2 | LIS3MDL magnetometer sampling |
| 0 | `commsTask`   | 50 Hz | 4 | UART RX parse + telemetry/sensor/motor-status TX |
| 0 | `canTask`     | 200 Hz | 4 | CAN drain + ODrive telemetry RTRs + bus-health recovery |

All UART transmission is funnelled through a mutex in `Comms` so frames from
`commsTask` and the occasional gripper frame from `controlTask` never interleave.

---

## CAN bus

One **500 kbps** bus (`config.h: CAN_BITRATE_BPS`) shared by all actuators —
matches the working board / VESC config. (To move to 1 Mbps, change that constant
**and** set every controller's CAN baud in its tool.)

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
- **VESC flippers (×4)** — position loop runs **on the VESC** (LispBM). The ESP
  sends a target angle and receives the measured angle via **custom CAN frames**
  (`VESC_CMD_FLIPPER_TARGET 0x7E` / `VESC_CMD_FLIPPER_REPORT 0x7F`, see
  [`../VESC/`](../VESC/)). No tachometer parsing on the ESP for flippers.
- **ODrive** (J1–J3) — CANSimple, `SET_INPUT_POS`; encoder zero captured at boot.
- **ZE300** (J4) — output-degree position; boot pose captured as zero.
- **LKTech** (J5–J6) — multi-loop angle control; boot pose captured as zero.

VESC IDs (`60/50` traction, `20/10/40/30` flippers) and all gear ratios /
direction signs live in `config.h` and **must be confirmed on the bench** (see
the TODO list there and in the architecture doc). Enable VESC status frames 1/4/5
in VESC Tool for traction/voltage/temperature telemetry.

---

## Flippers: position loop on the VESC

The flipper position loop runs **on each flipper VESC** in LispBM
([`../VESC/flipper_position.lisp`](../VESC/flipper_position.lisp)) — PD + stiction
feedforward with shortest-path error on a wrapped `[0,360)` angle, so a flipper
can spin continuously past 360°. The ESP32's only jobs are:

1. **Integrate the stick into a target angle** (`controlTask`, 50 Hz): stick
   deflection = rate (`FLIPPER_RATE_DPS`), so the target moves while deflected and
   **holds** where released. Direction sign is applied here (`FLIPPER_DIR_*`).
2. **Send the target** as an absolute wrapped angle (`SET_RPM` is *not* used —
   that would fight the lisp's `set-current`). On (re)entering flipper control the
   ESP seeds the target from the VESC-reported angle for a **bumpless** start.
3. **Receive the measured angle** the VESC reports back, for GUI telemetry.

Failsafe: on RC-loss the ESP keeps re-sending the frozen target (**hold**, never
"go home"); on hard e-stop it sends `enable=0` to coast (or hold, per
`FLIPPER_ESTOP_HOLD`). The angle scale (`deg-per-dist`) and the PD gains live in
the **lisp**, not `config.h` — **verify the reported angle tracks physical
rotation 1:1 on the bench**.

---

## Binary protocol

Frame: `[0xAA][0x55][TYPE][LEN_H][LEN_L][PAYLOAD][CRC]`, CRC = XOR of TYPE, the
two length bytes, and the payload. Payload layouts are the packed structs in
[`include/robot_types.h`](include/robot_types.h) (guarded by `static_assert`s),
and must stay in sync with the Jetson bridge `struct` formats.

| Dir | Type | Name |
|-----|------|------|
| → PC | 0x01 | Telemetry (mode, flags, PPM[6], track speeds, flipper angle, uptime) |
| → PC | 0x03 | Magnetometer |
| → PC | 0x05 | Status |
| → PC | 0x07 | Flipper angles (FL,FR,RL,RR) |
| → PC | 0x08 | VESC status (incl. tachometer → track odometry) |
| → PC | 0x0A / 0x0B / 0x0C / 0x0D | ODrive / LKTech / ZE300 status, ODrive error |
| ← PC | 0x10 | Arm joints (6 × int16 deg×100) |
| ← PC | 0x11 | Sensor enable mask |
| ← PC | 0x12 / 0x13 | E-stop / clear |
| ← PC | 0x19 | Arm operating mode: `0` dexterity, `1` chassis |
| ← PC | 0x14 | Keybind table (15 bytes) |
| ← PC | 0x15 | PPM calibration |
| ← PC | 0x16 | Gripper (→ PC originates; reserved) |

(0x02 thermal, 0x04 gas, 0x06 IMU, 0x09 main-PWM are reserved-unused on this robot
but the numbering is kept stable for GUI compatibility. Orientation now comes from
the ZED2 camera on the Jetson, not the ESP32.)

---

## Modes & arm relay

`INIT → STANDBY → NORMAL / FLIPPER / ARM`, plus `ESTOP`. The high-level mode is
derived from what the active Ch5 keybind row binds. The arm is relayed to CAN
**only in ARM mode** (tracks stopped). Joint commands received over UART while
**not** in ARM mode are **dropped**; on entering ARM mode the buffered target is
invalidated, so the arm only follows `/joint_states` received from that point on
(no jump to a stale pose). Ch6 HIGH (or an MSG_ESTOP frame) forces ESTOP, which
neutralises the drivetrain and stops the arm.

The arm operating mode is independent of the robot's high-level `ARM` mode.
`DEXTERITY` is the boot default. `CHASSIS` gates all J5/J6 position frames before
sending LKTech motor-stop, and the gate remains in force across disarm/re-arm.
J5/J6 are enabled again only by an explicit switch back to `DEXTERITY`.

---

## Project layout

```
include/  config.h        pins, CAN/VESC IDs, gear ratios, flipper params, protocol IDs
          robot_types.h   enums, structs, packed payloads (+ static_asserts)
lib/      RC/             PPM decode (ISR) + calibration
          Control/        state machine, keybinds, flipper setpoint integration
          Locomotion/     drivetrain output (track mix + flipper angle/hold)
          CANInterface/   CAN HAL (MCP2515/TWAI) + VESC/ODrive/ZE300/LKTech
          Comms/          binary UART protocol
          Sensors/        LIS3MDL magnetometer
          PID/            reusable PID (linear + shortest-angle) — spare
src/      main.cpp        setup() + FreeRTOS tasks

../VESC/  flipper_position.lisp   position loop that runs ON the flipper VESCs
```

---

## Build & flash

```bash
pio run            # build
pio run -t upload  # flash
pio device monitor # only shows text if ENABLE_COMMS is disabled in config.h
```

Confirm `config.h` (pins, VESC IDs, gear ratios, directions, PID gains) against
the real PCB before commanding motors. Bring up motors one at a time with small
commands.
