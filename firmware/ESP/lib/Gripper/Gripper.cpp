#include "Gripper.h"
#include "config.h"
#include <Arduino.h>

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static float    s_target_deg = GRIPPER_CLOSED_DEG;  // commanded angle (guarded)
static float    s_cmd        = 0.0f;                // latest signed rate (−1..+1)
static uint32_t s_cmd_ms     = 0;                   // arrival time (stale-cmd failsafe)

// Open/closed need not be ordered; derive the travel bounds and the "open"
// direction from the two configured endpoints so the mechanic can set either
// angle as the open end.
static constexpr float kLo      = (GRIPPER_OPEN_DEG >= GRIPPER_CLOSED_DEG)
                                      ? GRIPPER_CLOSED_DEG : GRIPPER_OPEN_DEG;
static constexpr float kHi      = (GRIPPER_OPEN_DEG >= GRIPPER_CLOSED_DEG)
                                      ? GRIPPER_OPEN_DEG : GRIPPER_CLOSED_DEG;
static constexpr float kOpenDir = (GRIPPER_OPEN_DEG >= GRIPPER_CLOSED_DEG) ? 1.0f : -1.0f;

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void Gripper::writeAngle(float deg) {
    // Map a 0..180° servo angle to its pulse width, then to LEDC duty counts.
    // The full 500–2500 µs span covers 0–180° on the JX control board.
    float pulse_us = GRIPPER_PULSE_MIN_US +
        (deg / 180.0f) * (float)(GRIPPER_PULSE_MAX_US - GRIPPER_PULSE_MIN_US);
    const float    period_us = 1000000.0f / (float)GRIPPER_PWM_HZ;
    const uint32_t max_duty  = (1u << GRIPPER_PWM_BITS) - 1u;
    uint32_t duty = (uint32_t)((pulse_us / period_us) * (float)max_duty);
    ledcWrite(GRIPPER_LEDC_CHANNEL, duty);
}

void Gripper::begin() {
    ledcSetup(GRIPPER_LEDC_CHANNEL, GRIPPER_PWM_HZ, GRIPPER_PWM_BITS);
    ledcAttachPin(PIN_GRIPPER_SERVO, GRIPPER_LEDC_CHANNEL);
    s_target_deg = clampf(GRIPPER_CLOSED_DEG, kLo, kHi);
    writeAngle(s_target_deg);
}

void Gripper::setCommand(float rate) {
    rate = clampf(rate, -1.0f, 1.0f);
    uint32_t now = millis();
    portENTER_CRITICAL(&s_mux);
    s_cmd    = rate;
    s_cmd_ms = now;
    portEXIT_CRITICAL(&s_mux);
}

void Gripper::update(float dt) {
    portENTER_CRITICAL(&s_mux);
    float    cmd    = s_cmd;
    uint32_t cmd_ms = s_cmd_ms;
    float    target = s_target_deg;
    portEXIT_CRITICAL(&s_mux);

    // Failsafe: ignore a stale command (lost link) → stop stepping, hold angle.
    if ((millis() - cmd_ms) > GRIPPER_CMD_TIMEOUT_MS) cmd = 0.0f;

    // Integrate the rate into the target and clamp to the mechanical travel.
    target = clampf(target + cmd * kOpenDir * GRIPPER_RATE_DPS * dt, kLo, kHi);

    portENTER_CRITICAL(&s_mux);
    s_target_deg = target;
    portEXIT_CRITICAL(&s_mux);

    writeAngle(target);
}

void Gripper::hold() {
    portENTER_CRITICAL(&s_mux);
    s_cmd = 0.0f;
    portEXIT_CRITICAL(&s_mux);
}

float Gripper::targetDeg() {
    portENTER_CRITICAL(&s_mux);
    float t = s_target_deg;
    portEXIT_CRITICAL(&s_mux);
    return t;
}
