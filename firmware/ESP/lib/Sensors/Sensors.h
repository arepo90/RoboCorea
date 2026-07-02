#pragma once
#include <stdint.h>
#include "robot_types.h"

// ─── Sensors (SENSOR ESP32 I2C) ───────────────────────────────────────────────
// QMC5883L magnetometer (GY-271/HW-246 breakout, heading reference) + MLX90640
// thermal camera, both on the dedicated SENSOR ESP32's I2C bus. They moved off
// the arm PCB onto this bare DevKit so the arm firmware carries no sensor work
// at all; this board reads them and forwards them to the Jetson over its own
// binary UART link; the bridge republishes the same /sensors/mag +
// /sensors/thermal topics. (RoboCorea has no IMU on the ESP32 — orientation
// comes from the ZED2.)
//
// Both sensors are ALWAYS ON: acquisition starts at boot and never stops (the
// old MSG_SENSOR_ENABLE mask is gone — the GUI toggles only gate display).
//
// Threading: a fast task calls runOnce() (mag) and a separate low-priority task
// calls runThermalOnce() (the MLX90640 getFrame() blocks ~250 ms). An internal
// I2C mutex serialises the shared Wire bus between the two — the mag read backs
// off and skips a sample rather than wait out a thermal frame.
class Sensors {
public:
    static bool begin();

    // Fast-sensor task (magnetometer). Rate-limited internally; safe to spin with
    // a short delay. Returns the bitmask of sensors actually read this tick.
    static uint8_t runOnce();

    // Dedicated thermal task. Blocks on MLX90640 getFrame(); returns true when a
    // fresh frame was captured (so the caller only forwards new frames).
    static bool runThermalOnce();

    static void getMag(MagData& out);
    static void getThermal(ThermalData& out);

private:
    static void readMag();
    static bool readThermal();

    static MagData     s_mag;
    static ThermalData s_thermal;
    static bool        s_mag_ok;
    static bool        s_mlx_ok;
};
