// RoboCorea — ESP32 firmware entry point
// ======================================
// Brings up the hardware and spawns the FreeRTOS tasks:
//
//   Core 1                              Core 0
//   ──────────────────────────         ──────────────────────────
//   controlTask  50 Hz  prio 5         commsTask  50 Hz  prio 4
//                                      canTask   200 Hz  prio 4
//
// controlTask runs the state machine (RC → mix / flipper-PID / arm-relay).
// commsTask owns the UART: it parses inbound frames and emits telemetry and
// motor-status frames. canTask drains the TWAI bus and emits ODrive telemetry
// RTRs. I2C victim-detection sensors live on the Jetson.
//
// See reference/architecture.md and the firmware README for details.

#include <Arduino.h>
#include "config.h"
#include "robot_types.h"

#include "RC.h"
#include "CANInterface.h"
#include "Locomotion.h"
#include "Comms.h"
#include "Control.h"
#include "Gripper.h"

// ─── Core 1: control state machine ────────────────────────────────────────────
static void controlTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        Control::tick();
        vTaskDelayUntil(&last, period);
    }
}

// ─── Core 0: CAN bus servicing ────────────────────────────────────────────────
static void canTask(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000 / CAN_POLL_HZ);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        CANInterface::poll();
        vTaskDelayUntil(&last, period);
    }
}

// ─── Core 0: UART comms + telemetry ───────────────────────────────────────────
// All UART transmission happens here (plus the occasional gripper frame from the
// control task, which is mutex-guarded inside Comms).
static void commsTask(void*) {
    const TickType_t telem_period = pdMS_TO_TICKS(1000 / CONTROL_LOOP_HZ);  // 50 Hz
    TickType_t last = xTaskGetTickCount();
    uint32_t last_identity_ms = 0;
    bool identity_sent = false;
    uint8_t status_div = 0;

    for (;;) {
        Comms::tick();   // drain RX, dispatch callbacks

        uint32_t now_ms = millis();
        if (!identity_sent || now_ms - last_identity_ms >= 1000) {
            identity_sent = true;
            last_identity_ms = now_ms;
            Comms::sendBoardIdentity();
        }

        if (xTaskGetTickCount() - last >= telem_period) {
            last = xTaskGetTickCount();

#if ROBOCOREA_ROLE_IS_CHASSIS
            SystemStatus status;
            Control::getSystemStatus(status);

            float spd_l = 0, spd_r = 0, flip[4] = {0, 0, 0, 0};
            CANInterface::getTractionSpeeds(spd_l, spd_r);
            CANInterface::getFlipperAngles(flip);

            PPMFrame ppm;
            RC::peekFrame(ppm);

            // ── Telemetry ──────────────────────────────────────────────────
            TelemetryPayload t;
            t.mode  = (uint8_t)status.mode;
            t.flags = (status.ppm_connected ? 0x01 : 0)
                    | (status.sensor_mask   ? 0x02 : 0)
                    | (status.can_ok        ? 0x04 : 0)
                    | (status.estop         ? 0x08 : 0)
                    | (status.virtual_flip  ? 0x10 : 0);
            for (int i = 0; i < PPM_CHANNELS; i++) t.ppm[i] = ppm.valid ? ppm.ch[i] : 0;
            t.speed_left    = (int16_t)(spd_l * 10.0f);
            t.speed_right   = (int16_t)(spd_r * 10.0f);
            t.flipper_angle = (int16_t)(flip[0] * 10.0f);
            t.uptime_ms     = status.uptime_ms;
            Comms::sendTelemetry(t);

            Comms::sendEncoderExt(flip[0], flip[1], flip[2], flip[3]);

            // ── System status (lower rate, 10 Hz) ──────────────────────────
            if (++status_div >= 5) {
                status_div = 0;
                Comms::sendStatus(status);
            }

            // ── Motor status (send only fresh frames) ──────────────────────
            for (uint8_t idx = 0; idx < 6; idx++) {
                uint8_t id = CANInterface::vescIdByIndex(idx);
                VescStatusPayload v;
                if (id && CANInterface::getVescStatus(id, v)) Comms::sendVescStatus(v);
            }
#endif

#if ROBOCOREA_ROLE_IS_ARM
            if (++status_div >= 5) {
                status_div = 0;
                ArmLifecyclePayload al;
                CANInterface::getArmLifecycle(al);
                Comms::sendArmLifecycle(al);
            }
            for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
                OdriveStatusPayload o;
                if (CANInterface::getOdriveStatus(j, o)) Comms::sendOdriveStatus(o);
            }
            for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
                LktechStatusPayload l;
                if (CANInterface::getLktechStatus(j, l)) Comms::sendLktechStatus(l);
            }
            { Ze300StatusPayload z; if (CANInterface::getZe300Status(z)) Comms::sendZe300Status(z); }
            for (uint8_t n = 0; n < CANInterface::odriveNodeCount(); n++) {
                OdriveErrorPayload e;
                if (CANInterface::getOdriveError(n, e)) Comms::sendOdriveError(e);
            }
#endif
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void setup() {
    Comms::begin();          // UART + TX mutex
    CANInterface::begin();   // TWAI + arm controller bring-up (blocking)
#if ROBOCOREA_ROLE_IS_CHASSIS
    Locomotion::begin();     // zero the drivetrain VESCs
#endif
#if ROBOCOREA_ROLE_IS_ARM
    Gripper::begin();        // LEDC end-effector servo on GPIO26; park at closed
#endif
    Control::begin();        // register callbacks, configure flipper PIDs
#if ROBOCOREA_ROLE_IS_CHASSIS
    RC::begin(PIN_PPM);      // attach the PPM ISR
#endif

    xTaskCreatePinnedToCore(controlTask, "control", STACK_CONTROL, nullptr,
                            PRIO_CONTROL, nullptr, TASK_CORE_CONTROL);
    xTaskCreatePinnedToCore(commsTask,   "comms",   STACK_COMMS,   nullptr,
                            PRIO_COMMS,   nullptr, TASK_CORE_COMMS);
    xTaskCreatePinnedToCore(canTask,     "can",     STACK_CAN,     nullptr,
                            PRIO_CAN,     nullptr, TASK_CORE_CAN);
}

void loop() {
    // Everything runs in FreeRTOS tasks; nothing to do here.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
