#pragma once

// ─── Gripper ──────────────────────────────────────────────────────────────────
// Local end-effector gripper: ONE hobby servo on PIN_GRIPPER_SERVO (GPIO26),
// driven by the ESP32's LEDC PWM (the only PWM actuator on this board — every
// other motor is CAN). A single servo moves both jaws via a linkage.
//
// Control model (matches the flippers' rate-integration): the workstation
// gamepad's RT/LT triggers produce a signed rate command (+1 fully open …
// −1 fully close), delivered as MSG_GRIPPER (PC→ESP) and pushed in via
// setCommand(). Each control tick, update() integrates that rate into a target
// angle and CLAMPS it to [GRIPPER_CLOSED_DEG, GRIPPER_OPEN_DEG] — so holding a
// trigger steps the gripper until it reaches a limit, then it holds. A stale
// command (link loss) decays to "stop" via GRIPPER_CMD_TIMEOUT_MS.
//
// Thread-safety: setCommand() runs on the comms task (core 0); update() runs on
// the control task (core 1). Shared state is guarded by an internal spinlock.

class Gripper {
public:
    static void  begin();                 // configure LEDC; park at the closed limit
    static void  setCommand(float rate);  // signed −1..+1 (+open / −close); thread-safe
    static void  update(float dt);        // integrate → clamp → write; call each control tick
    static void  hold();                  // freeze stepping (e.g. on e-stop); servo holds angle
    static float targetDeg();             // current commanded target angle (for telemetry)

private:
    static void writeAngle(float deg);    // angle → pulse width → LEDC duty
};
