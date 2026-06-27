#include "Sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MLX90640.h>

uint8_t     Sensors::s_mask    = 0;
MagData     Sensors::s_mag      = {};
ThermalData Sensors::s_thermal  = {};
bool        Sensors::s_mag_ok   = false;
bool        Sensors::s_mlx_ok   = false;

// Short critical sections for the published data fields.
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// I2C bus mutex — serialises Wire access across the mag task and the thermal
// task. Must be a FreeRTOS mutex (not a portMUX spinlock) because the MLX90640
// frame read blocks for ~250 ms and we must yield the CPU while it waits.
static SemaphoreHandle_t s_i2c_mutex = nullptr;

static Adafruit_LIS3MDL  s_lis;
static Adafruit_MLX90640 s_mlx;

// getFrame() scratch buffer (kept off the task stack — 768 floats = 3 KB).
static float s_mlx_pixels[THERMAL_PIXELS];

bool Sensors::begin() {
    s_i2c_mutex = xSemaphoreCreateMutex();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(SENSOR_I2C_HZ);

    // LIS3MDL magnetometer (Adafruit driver probes its default I2C address).
    if (s_lis.begin_I2C()) {
        s_lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
        s_lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        s_lis.setDataRate(LIS3MDL_DATARATE_155_HZ);
        s_lis.setRange(LIS3MDL_RANGE_16_GAUSS);  // 16 G avoids saturation near the BLDC motors
        s_mag_ok = true;
    }

    // MLX90640 thermal camera. CHESS pattern + 18-bit ADC at the configured
    // refresh; getFrame() then does the two-sub-page read + °C calibration.
    if (s_mlx.begin(MLX90640_I2C_ADDR, &Wire)) {
        s_mlx.setMode(MLX90640_CHESS);
        s_mlx.setResolution((mlx90640_resolution_t)THERMAL_RESOLUTION);
        s_mlx.setRefreshRate((mlx90640_refreshrate_t)THERMAL_REFRESH_RATE);
        s_mlx_ok = true;
    }

    return true;  // non-fatal: check s_mag_ok / s_mlx_ok
}

static uint32_t s_last_mag_ms = 0;

uint8_t Sensors::runOnce() {
    uint32_t now  = millis();
    uint8_t  read = 0;

    if ((getEnabledMask() & SENSOR_BIT_MAG) && (now - s_last_mag_ms >= 1000 / SENSOR_MAG_HZ)) {
        s_last_mag_ms = now;
        readMag();
        read |= SENSOR_BIT_MAG;
    }
    return read;
}

bool Sensors::runThermalOnce() {
    if (!(getEnabledMask() & SENSOR_BIT_THERMAL)) return false;
    return readThermal();
}

void Sensors::setEnabledMask(uint8_t mask) {
    portENTER_CRITICAL(&s_mux);
    s_mask = mask;
    portEXIT_CRITICAL(&s_mux);
}

uint8_t Sensors::getEnabledMask() {
    portENTER_CRITICAL(&s_mux);
    uint8_t m = s_mask;
    portEXIT_CRITICAL(&s_mux);
    return m;
}

void Sensors::getMag(MagData& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_mag;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::getThermal(ThermalData& out) {
    portENTER_CRITICAL(&s_mux);
    out = s_thermal;
    portEXIT_CRITICAL(&s_mux);
}

void Sensors::readMag() {
    if (!s_mag_ok) return;
    // Short timeout: if the thermal task is mid-frame, skip this sample rather
    // than block ~250 ms waiting for the bus.
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    sensors_event_t e;
    s_lis.getEvent(&e);          // magnetic field in µT (1 G = 100 µT)
    xSemaphoreGive(s_i2c_mutex);

    MagData d;
    d.x_uT  = (int)e.magnetic.x;
    d.y_uT  = (int)e.magnetic.y;
    d.z_uT  = (int)e.magnetic.z;
    d.valid = true;
    portENTER_CRITICAL(&s_mux);
    s_mag = d;
    portEXIT_CRITICAL(&s_mux);
}

bool Sensors::readThermal() {
    if (!s_mlx_ok) return false;
    if (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(500)) != pdTRUE) return false;
    // getFrame() handles the two-sub-page protocol + calibration and blocks until
    // a complete frame is ready (≈1/(refresh/2) s).
    int rc = s_mlx.getFrame(s_mlx_pixels);
    xSemaphoreGive(s_i2c_mutex);
    if (rc != 0) return false;

    portENTER_CRITICAL(&s_mux);
    for (int i = 0; i < THERMAL_PIXELS; i++) s_thermal.pixels[i] = s_mlx_pixels[i];
    s_thermal.valid = true;
    portEXIT_CRITICAL(&s_mux);
    return true;
}
