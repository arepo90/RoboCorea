#pragma once
#include <stdint.h>
#include "robot_types.h"

// ─── Sensors (ARM PCB I2C) ────────────────────────────────────────────────────
// LIS3MDL magnetometer (heading reference) + MLX90640 thermal camera, both on the
// ARM PCB's I2C bus. They moved off the Jetson I2C (which proved unreliable) onto
// the arm ESP32, which reads them and forwards them to the Jetson over the binary
// UART link; the bridge republishes the same /sensors/mag + /sensors/thermal
// topics. (RoboCorea has no IMU on the ESP32 — orientation comes from the ZED2.)
//
// Each sensor stays idle until its bit is set via setEnabledMask() (driven by the
// GUI's /sensors/enable_mask → bridge → MSG_SENSOR_ENABLE), so a disabled sensor
// never touches the I2C bus.
//
// Threading: a fast task calls runOnce() (mag) and a separate low-priority task
// calls runThermalOnce() (the MLX90640 getFrame() blocks ~250 ms). An internal
// I2C mutex serialises the shared Wire bus between the two — the mag read backs
// off and skips a sample rather than wait out a thermal frame. Both tasks sit
// below the control/CAN tasks so they can never delay the arm (see config.h).
class Sensors {
public:
    static bool begin();

    // Fast-sensor task (magnetometer). Rate-limited internally; safe to spin with
    // a short delay. Returns the bitmask of sensors actually read this tick.
    static uint8_t runOnce();

    // Dedicated thermal task. Blocks on MLX90640 getFrame(); returns true when a
    // fresh frame was captured (so the caller only forwards new frames).
    static bool runThermalOnce();

    static void    setEnabledMask(uint8_t mask);
    static uint8_t getEnabledMask();

    static void getMag(MagData& out);
    static void getThermal(ThermalData& out);

private:
    static void readMag();
    static bool readThermal();

    static uint8_t     s_mask;
    static MagData     s_mag;
    static ThermalData s_thermal;
    static bool        s_mag_ok;
    static bool        s_mlx_ok;
};
