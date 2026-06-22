// RoboCorea — standalone servo bench test
// =======================================
// Sweeps a JX CLS-12V7346 46kg coreless servo from 0 to 180 deg and back on
// GPIO26. This is a bring-up sketch ONLY — it is deliberately kept out of the
// firmware/ESP project (the robot's drivetrain is all VESC/CAN, no PWM servos).
//
// Wiring:
//   • Signal (white/yellow) -> ESP32 GPIO26
//   • Power  (red)          -> separate 11.1-15.0 V supply  (NOT the ESP32!)
//   • Ground (black)        -> servo supply GND *and* ESP32 GND (common ground)
//
// Servo datasheet (JX CLS-12V7346):
//   • Pulse 0.5 ms -> 0 deg, 2.5 ms -> 180 deg  (full control-board range)
//   • Center 1520 us, 2 us deadband, accepts up to 330 Hz refresh
//   • Speed ~0.10-0.12 s / 60 deg; stall torque 38-47 kg.cm (size the supply!)

#include <Arduino.h>
#include <ESP32Servo.h>

static const int SERVO_PIN    = 26;
static const int PULSE_MIN_US = 500;     // 0.5 ms -> 0   deg
static const int PULSE_MAX_US = 2500;    // 2.5 ms -> 180 deg

// Smoothness knobs:
//   • REFRESH_HZ — this is a DIGITAL servo rated to 330 Hz. A high refresh rate
//     lets it correct position far more often, so motion looks continuous
//     instead of stepping at 50 Hz.
//   • SWEEP_MS — time for one full 0->180 pass. Bigger = slower & smoother.
// The sweep is interpolated in MICROSECONDS (not whole degrees), so each update
// nudges the target by a fraction of a degree -> no visible micro-steps.
static const int   REFRESH_HZ = 330;     // max the JX CLS-12V7346 accepts
static const float SWEEP_MS   = 2500.0f; // 2.5 s end-to-end (~0.83 s / 60 deg)

Servo servo;

// Glide the pulse width from -> to over duration_ms, writing once per refresh
// tick. writeMicroseconds gives ~1 us resolution (well under the 11 us/deg),
// so the steps are too fine to see or hear.
void sweep(int from_us, int to_us, float duration_ms) {
    const int step_ms = 1000 / REFRESH_HZ;   // ~3 ms at 330 Hz
    const uint32_t t0 = millis();
    for (;;) {
        float t = (float)(millis() - t0);
        if (t >= duration_ms) break;
        float frac = t / duration_ms;                       // 0.0 -> 1.0
        int   us   = from_us + (int)((to_us - from_us) * frac);
        servo.writeMicroseconds(us);
        delay(step_ms);
    }
    servo.writeMicroseconds(to_us);          // land exactly on the endpoint
}

void setup() {
    Serial.begin(115200);
    delay(200);

    // The ESP32Servo lib needs a couple of LEDC timers reserved for PWM.
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    servo.setPeriodHertz(REFRESH_HZ);
    // Attach with the FULL datasheet pulse range so 500/2500 us hit the real
    // mechanical endpoints (a default 1000-2000 us would clip the travel).
    servo.attach(SERVO_PIN, PULSE_MIN_US, PULSE_MAX_US);

    Serial.println("JX CLS-12V7346 servo test on GPIO26: smooth sweep 0 <-> 180");
}

void loop() {
    sweep(PULSE_MIN_US, PULSE_MAX_US, SWEEP_MS);   // 0 -> 180
    Serial.println("at 180 deg");
    delay(500);

    sweep(PULSE_MAX_US, PULSE_MIN_US, SWEEP_MS);   // 180 -> 0
    Serial.println("at 0 deg");
    delay(500);
}
