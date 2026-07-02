#include "Sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>

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

static Adafruit_MLX90640 s_mlx;

// getFrame() scratch buffer (kept off the task stack — 768 floats = 3 KB).
static float s_mlx_pixels[THERMAL_PIXELS];

// ─── QMC5883L raw driver ──────────────────────────────────────────────────────
// The GY-271/HW-246 breakout carries the QMC5883L clone at 0x0D (NOT the genuine
// HMC5883L register map at 0x1E). Tiny enough that a library isn't worth it:
// one config write, then poll DRDY and read six data bytes.
static constexpr uint8_t QMC_REG_DATA    = 0x00;  // X LSB..Z MSB, little-endian int16 ×3
static constexpr uint8_t QMC_REG_STATUS  = 0x06;  // bit0 DRDY, bit1 OVL (overflow)
static constexpr uint8_t QMC_REG_CTRL1   = 0x09;
static constexpr uint8_t QMC_REG_SRST    = 0x0B;  // SET/RESET period, datasheet says write 0x01
// CTRL1: OSR[7:6]=00 (512), RNG[5:4]=01 (±8 G), ODR[3:2]=10 (100 Hz), MODE[1:0]=01 (continuous)
static constexpr uint8_t QMC_CTRL1_VALUE = 0x19;

static bool qmcWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(QMC5883L_I2C_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

// Read the XYZ registers if a fresh sample is ready. Returns false on a bus
// error, no-DRDY, or field overflow (OVL) — the caller keeps the last sample.
static bool qmcRead(int16_t& x, int16_t& y, int16_t& z) {
    Wire.beginTransmission(QMC5883L_I2C_ADDR);
    Wire.write(QMC_REG_STATUS);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)QMC5883L_I2C_ADDR, (uint8_t)1) != 1) return false;
    uint8_t status = Wire.read();
    if (!(status & 0x01) || (status & 0x02)) return false;  // no DRDY, or overflow

    Wire.beginTransmission(QMC5883L_I2C_ADDR);
    Wire.write(QMC_REG_DATA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)QMC5883L_I2C_ADDR, (uint8_t)6) != 6) return false;
    uint8_t b[6];
    for (int i = 0; i < 6; i++) b[i] = Wire.read();
    x = (int16_t)(b[0] | (b[1] << 8));
    y = (int16_t)(b[2] | (b[3] << 8));
    z = (int16_t)(b[4] | (b[5] << 8));
    return true;
}

bool Sensors::begin() {
    s_i2c_mutex = xSemaphoreCreateMutex();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(SENSOR_I2C_HZ);

    // QMC5883L magnetometer: soft-set the SET/RESET period then start continuous
    // conversions. Both writes ACKing is the presence check.
    if (qmcWrite(QMC_REG_SRST, 0x01) && qmcWrite(QMC_REG_CTRL1, QMC_CTRL1_VALUE)) {
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

    if (now - s_last_mag_ms >= 1000 / SENSOR_MAG_HZ) {
        s_last_mag_ms = now;
        readMag();
        read |= SENSOR_BIT_MAG;
    }
    return read;
}

bool Sensors::runThermalOnce() {
    return readThermal();
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
    int16_t x = 0, y = 0, z = 0;
    bool ok = qmcRead(x, y, z);
    xSemaphoreGive(s_i2c_mutex);
    if (!ok) return;

    MagData d;
    d.x_uT  = x / QMC5883L_LSB_PER_UT;
    d.y_uT  = y / QMC5883L_LSB_PER_UT;
    d.z_uT  = z / QMC5883L_LSB_PER_UT;
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
