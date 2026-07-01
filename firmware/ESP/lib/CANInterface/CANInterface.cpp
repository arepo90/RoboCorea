#include "CANInterface.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>
#include <cmath>

// ══════════════════════════════════════════════════════════════════════════════
//  CAN HAL — one tiny frame type + send/receive, with two interchangeable
//  backends selected in config.h (CAN_BACKEND_TWAI or CAN_BACKEND_MCP2515).
//  Everything below the HAL is backend-agnostic.
// ══════════════════════════════════════════════════════════════════════════════
struct CanFrame {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
    bool     extd;
    bool     rtr;
};

static bool canBackendBegin();
static bool canSend(const CanFrame& f);
static bool canReceive(CanFrame& f);    // non-blocking; true if a frame was read
static void canDrainRx();

#if defined(CAN_BACKEND_TWAI)
// ─── Backend: ESP32 built-in TWAI + SN65HVD230 ───────────────────────────────
#include "driver/twai.h"

static bool canBackendBegin() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
    g.rx_queue_len = CAN_RX_QUEUE_LEN;   // default 5 overflows at 6 VESC × STATUS_1/4/5 @100Hz
    g.tx_queue_len = CAN_TX_QUEUE_LEN;
  #if   CAN_BITRATE_BPS == 1000000
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
  #elif CAN_BITRATE_BPS == 500000
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  #elif CAN_BITRATE_BPS == 250000
    twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS();
  #else
    #error "Unsupported CAN_BITRATE_BPS for TWAI"
  #endif
    twai_filter_config_t fl = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &fl) != ESP_OK) return false;
    if (twai_start() != ESP_OK) { twai_driver_uninstall(); return false; }
    return true;
}
static bool canSend(const CanFrame& f) {
    twai_message_t m = {};
    m.extd = f.extd; m.rtr = f.rtr; m.identifier = f.id; m.data_length_code = f.dlc;
    memcpy(m.data, f.data, 8);
    // Bounded TX: never block the control loop on a wedged bus; drop on full.
    return twai_transmit(&m, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS)) == ESP_OK;
}
static bool canReceive(CanFrame& f) {
    twai_message_t m;
    if (twai_receive(&m, 0) != ESP_OK) return false;
    f.extd = m.extd; f.rtr = m.rtr; f.id = m.identifier; f.dlc = m.data_length_code;
    memcpy(f.data, m.data, 8);
    return true;
}
static void canDrainRx() {
    CanFrame f;
    while (canReceive(f)) {}
}
// Returns true if the bus is RUNNING. Recovers from BUS_OFF (e.g. wrong bitrate
// or a single-node bring-up with no ACKs) instead of staying silently dead.
static bool canBackendHealthy() {
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return false;
    if (st.state == TWAI_STATE_BUS_OFF) { twai_initiate_recovery(); return false; }
    if (st.state == TWAI_STATE_STOPPED) { twai_start();            return false; }
    return st.state == TWAI_STATE_RUNNING;
}
static uint8_t canBackendEflg() { return 0; }   // EFLG is MCP2515-specific

#elif defined(CAN_BACKEND_MCP2515)
// ─── Backend: legacy MCP2515 over SPI ────────────────────────────────────────
#include <SPI.h>
#include <mcp2515.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static MCP2515 s_mcp(PIN_CAN_CS);
// TX runs on the control task (core 1), RX on the CAN task (core 0); the MCP2515
// shares one SPI transaction engine, so every access must be serialised.
static SemaphoreHandle_t s_mcp_mtx = nullptr;
static volatile uint32_t s_mcp_fail = 0;   // consecutive send failures

static bool mcpConfigure() {
  #if   CAN_BITRATE_BPS == 1000000
    const CAN_SPEED sp = CAN_1000KBPS;
  #elif CAN_BITRATE_BPS == 500000
    const CAN_SPEED sp = CAN_500KBPS;
  #elif CAN_BITRATE_BPS == 250000
    const CAN_SPEED sp = CAN_250KBPS;
  #else
    #error "Unsupported CAN_BITRATE_BPS for MCP2515"
  #endif
  #if   MCP2515_OSC_MHZ == 8
    const CAN_CLOCK cl = MCP_8MHZ;
  #elif MCP2515_OSC_MHZ == 16
    const CAN_CLOCK cl = MCP_16MHZ;
  #else
    #error "Unsupported MCP2515_OSC_MHZ (use 8 or 16)"
  #endif
    if (s_mcp.reset()            != MCP2515::ERROR_OK) return false;
    if (s_mcp.setBitrate(sp, cl) != MCP2515::ERROR_OK) return false;
    if (s_mcp.setNormalMode()    != MCP2515::ERROR_OK) return false;
    return true;
}
static bool canBackendBegin() {
    if (!s_mcp_mtx) s_mcp_mtx = xSemaphoreCreateMutex();
    SPI.begin(PIN_CAN_SCK, PIN_CAN_MISO, PIN_CAN_MOSI, PIN_CAN_CS);
    return mcpConfigure();
}
static bool canSend(const CanFrame& f) {
    struct can_frame fr;
    fr.can_id  = f.id | (f.extd ? CAN_EFF_FLAG : 0) | (f.rtr ? CAN_RTR_FLAG : 0);
    fr.can_dlc = f.dlc;
    memcpy(fr.data, f.data, 8);
    bool ok = false;
    const uint32_t deadline_us = micros() + (uint32_t)CAN_TX_TIMEOUT_MS * 1000U;
    do {
        xSemaphoreTake(s_mcp_mtx, portMAX_DELAY);
        ok = (s_mcp.sendMessage(&fr) == MCP2515::ERROR_OK);
        xSemaphoreGive(s_mcp_mtx);
        if (ok) break;
        delayMicroseconds(100);
    } while ((int32_t)(micros() - deadline_us) < 0);
    s_mcp_fail = ok ? 0 : (s_mcp_fail + 1);
    return ok;
}
static bool canReceive(CanFrame& f) {
    struct can_frame fr;
    xSemaphoreTake(s_mcp_mtx, portMAX_DELAY);
    bool ok = (s_mcp.readMessage(&fr) == MCP2515::ERROR_OK);
    xSemaphoreGive(s_mcp_mtx);
    if (!ok) return false;
    f.extd = (fr.can_id & CAN_EFF_FLAG) != 0;
    f.rtr  = (fr.can_id & CAN_RTR_FLAG) != 0;
    f.id   = fr.can_id & (f.extd ? CAN_EFF_MASK : CAN_SFF_MASK);
    f.dlc  = fr.can_dlc;
    memcpy(f.data, fr.data, 8);
    return true;
}
static void canDrainRx() {
    CanFrame f;
    while (canReceive(f)) {}
}
// Re-init the chip after a run of failed sends (TX error / bus fault).
static bool canBackendHealthy() {
    if (s_mcp_fail < CAN_MCP_FAIL_LIMIT) return true;
    xSemaphoreTake(s_mcp_mtx, portMAX_DELAY);
    bool ok = mcpConfigure();
    xSemaphoreGive(s_mcp_mtx);
    if (ok) s_mcp_fail = 0;
    return ok;
}
// MCP2515 EFLG register snapshot (RX overflow / TX bus-off / error-warning bits)
// for the arm lifecycle diagnostics.
static uint8_t canBackendEflg() {
    xSemaphoreTake(s_mcp_mtx, portMAX_DELAY);
    uint8_t e = s_mcp.getErrorFlags();
    xSemaphoreGive(s_mcp_mtx);
    return e;
}
#else
  #error "Select a CAN backend in config.h"
#endif

// ─── State + small helpers ────────────────────────────────────────────────────
static bool s_ok = false;
static uint16_t s_arm_presence_mask = 0;
// Last time (ms) each arm joint J1..J6 was heard on the CAN bus. Used to downgrade
// the presence mask live so a joint that drops off mid-session stops reading as
// present (see getArmLifecycle + ARM_JOINT_STALE_MS). 0 = never heard this arm.
static uint32_t s_arm_joint_heard_ms[6] = {0};

static inline void markArmJoint(uint8_t j) {
    if (j >= 6) return;
    s_arm_presence_mask |= (uint16_t)(1U << j);
    s_arm_joint_heard_ms[j] = millis();
}

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline float wrap360(float d) {
    d = fmodf(d, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d;
}

static inline void putInt32BE(uint8_t* d, int32_t v) {
    d[0] = (v >> 24) & 0xFF; d[1] = (v >> 16) & 0xFF;
    d[2] = (v >>  8) & 0xFF; d[3] = (v >>  0) & 0xFF;
}
static inline int32_t getInt32BE(const uint8_t* d) {
    return ((int32_t)d[0] << 24) | ((int32_t)d[1] << 16) |
           ((int32_t)d[2] << 8)  |  (int32_t)d[3];
}
static inline int16_t getInt16BE(const uint8_t* d) {
    return (int16_t)(((uint16_t)d[0] << 8) | d[1]);
}
static inline void putFloat32LE(uint8_t* b, float v) {
    uint32_t raw; memcpy(&raw, &v, 4);
    b[0] = raw & 0xFF; b[1] = (raw >> 8) & 0xFF;
    b[2] = (raw >> 16) & 0xFF; b[3] = (raw >> 24) & 0xFF;
}
static inline void putUint32LE(uint8_t* b, uint32_t v) {
    b[0] = v & 0xFF; b[1] = (v >> 8) & 0xFF;
    b[2] = (v >> 16) & 0xFF; b[3] = (v >> 24) & 0xFF;
}
static inline float getFloat32LE(const uint8_t* b) {
    uint32_t raw = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                   ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    float v; memcpy(&v, &raw, 4); return v;
}
static inline void putInt32LE(uint8_t* d, int32_t v) {
    d[0] = (uint8_t)(v >> 0);  d[1] = (uint8_t)(v >> 8);
    d[2] = (uint8_t)(v >> 16); d[3] = (uint8_t)(v >> 24);
}
static inline int32_t getInt32LE(const uint8_t* d) {
    return (int32_t)d[0] | ((int32_t)d[1] << 8) |
           ((int32_t)d[2] << 16) | ((int32_t)d[3] << 24);
}
static inline int16_t getInt16LE(const uint8_t* d) {
    return (int16_t)((uint16_t)d[0] | ((uint16_t)d[1] << 8));
}
static inline uint16_t getUint16LE(const uint8_t* d) {
    return (uint16_t)d[0] | ((uint16_t)d[1] << 8);
}

static void noteArmPresenceFromRxFrame(const CanFrame& f) {
    if (f.extd || f.rtr) return;

    const uint32_t id = f.id;
    const uint8_t odrv_node = (uint8_t)((id >> 5) & 0x3F);
    if (odrv_node == ODRIVE_NODE_J1) markArmJoint(0);
    if (odrv_node == ODRIVE_NODE_J2) markArmJoint(1);
    if (odrv_node == ODRIVE_NODE_J3) markArmJoint(2);

    if (id == (uint32_t)ZE300_ID_J4)
        markArmJoint(3);
    if (id == (uint32_t)(LKTECH_ID_BASE + LKTECH_ID_J5))
        markArmJoint(4);
    if (id == (uint32_t)(LKTECH_ID_BASE + LKTECH_ID_J6))
        markArmJoint(5);
}

// Blocking receive of one frame matching a predicate, until timeout_ms.
template <typename Pred>
static bool canReceiveUntil(uint32_t timeout_ms, CanFrame& out, Pred match) {
    uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        CanFrame f;
        if (canReceive(f)) {
            noteArmPresenceFromRxFrame(f);
            if (match(f)) { out = f; return true; }
        } else {
            delay(1);
        }
    }
    return false;
}

// ══════════════════════════════════════════════════════════════════════════════
//  VESC (extended frames, ID = (cmd << 8) | controller_id)
// ══════════════════════════════════════════════════════════════════════════════
[[maybe_unused]] static constexpr uint8_t VESC_CMD_SET_CURRENT = 1;  // bench fallback
static constexpr uint8_t VESC_CMD_SET_RPM     = 3;

static bool vescSendRpm(uint8_t id, int32_t erpm) {
    CanFrame f = {}; f.extd = true; f.id = ((uint32_t)VESC_CMD_SET_RPM << 8) | id; f.dlc = 4;
    putInt32BE(f.data, erpm);
    return canSend(f);
}
// Custom frame to a flipper VESC running flipper_position.lisp: target angle
// (int32 BE millideg) + enable flag. cmd byte = VESC_CMD_FLIPPER_TARGET so the
// VESC firmware ignores it and the lisp's event-can-eid handler picks it up.
static bool vescSendFlipperTarget(uint8_t id, float wrapped_deg, bool enabled) {
    CanFrame f = {}; f.extd = true;
    f.id = ((uint32_t)VESC_CMD_FLIPPER_TARGET << 8) | id; f.dlc = 5;
    putInt32BE(f.data, (int32_t)lroundf(wrapped_deg * 1000.0f));
    f.data[4] = enabled ? 1 : 0;
    return canSend(f);
}

struct VescFeedback {
    int32_t  erpm;
    int16_t  current_10, duty_1000, temp_fet_10, temp_motor_10, v_in_10;
    int32_t  tacho;          // STATUS_5 tachometer (forwarded for track odometry)
    bool     fresh;
};
static VescFeedback s_vesc[6] = {};
static portMUX_TYPE s_vesc_mux = portMUX_INITIALIZER_UNLOCKED;

// CAN id of each array slot. IDs need not be contiguous (id → slot by lookup).
static const uint8_t s_vesc_id[6] = { VESC_ID_TRACTION_LEFT, VESC_ID_TRACTION_RIGHT,
                                      VESC_ID_FLIPPER_FL, VESC_ID_FLIPPER_FR,
                                      VESC_ID_FLIPPER_RL, VESC_ID_FLIPPER_RR };
static inline int vescIdToIndex(uint8_t id) {
    for (int i = 0; i < 6; i++) if (s_vesc_id[i] == id) return i;
    return -1;
}

static const uint8_t s_traction_id[2] = { VESC_ID_TRACTION_LEFT, VESC_ID_TRACTION_RIGHT };
static const float   s_traction_dir[2]= { TRACTION_DIR_LEFT, TRACTION_DIR_RIGHT };
static const uint8_t s_flipper_id[4]  = { VESC_ID_FLIPPER_FL, VESC_ID_FLIPPER_FR,
                                          VESC_ID_FLIPPER_RL, VESC_ID_FLIPPER_RR };
// Flipper angle used by telemetry (deg). Index = FL,FR,RL,RR.
// Usually derived from STATUS_5 tachometer; custom Lisp report is still parsed
// when that experimental path is enabled.
static float s_flipper_deg[4]  = { 0, 0, 0, 0 };
static bool  s_flipper_deg_ok[4] = { false, false, false, false };
#if FLIPPER_USE_TACH_FEEDBACK
static const float s_flipper_tach_deg_per_count[4] = {
    FLIPPER_TACH_DEG_PER_COUNT_FL, FLIPPER_TACH_DEG_PER_COUNT_FR,
    FLIPPER_TACH_DEG_PER_COUNT_RL, FLIPPER_TACH_DEG_PER_COUNT_RR
};
static const float s_flipper_tach_zero_deg[4] = {
    FLIPPER_TACH_ZERO_DEG_FL, FLIPPER_TACH_ZERO_DEG_FR,
    FLIPPER_TACH_ZERO_DEG_RL, FLIPPER_TACH_ZERO_DEG_RR
};
static int32_t s_flipper_tach_zero_count[4] = { 0, 0, 0, 0 };
static bool    s_flipper_tach_zero_ok[4] = { false, false, false, false };
#endif

// ══════════════════════════════════════════════════════════════════════════════
//  ODrive arm (standard frames, COB-ID = (node_id << 5) | cmd_id)
// ══════════════════════════════════════════════════════════════════════════════
static constexpr uint8_t ODRV_HEARTBEAT      = 0x01;
static constexpr uint8_t ODRV_ESTOP          = 0x02;
static constexpr uint8_t ODRV_GET_ERROR      = 0x03;
static constexpr uint8_t ODRV_SET_AXIS_STATE = 0x07;
static constexpr uint8_t ODRV_GET_ENCODER    = 0x09;
static constexpr uint8_t ODRV_SET_INPUT_POS  = 0x0C;
static constexpr uint8_t ODRV_GET_IQ         = 0x14;
static constexpr uint8_t ODRV_GET_BUS_V      = 0x17;
static constexpr uint8_t ODRV_CLEAR_ERRORS   = 0x18;

static inline uint32_t odrvCOB(uint8_t node, uint8_t cmd) {
    return ((uint32_t)node << 5) | cmd;
}
static void odrvAxisState(uint8_t node, uint32_t state) {
    CanFrame f = {}; f.id = odrvCOB(node, ODRV_SET_AXIS_STATE); f.dlc = 4;
    putUint32LE(f.data, state); canSend(f);
}
static bool odrvInputPos(uint8_t node, float turns) {
    CanFrame f = {}; f.id = odrvCOB(node, ODRV_SET_INPUT_POS); f.dlc = 8;
    putFloat32LE(f.data, turns);   // vel_ff = torque_ff = 0
    return canSend(f);
}
static void odrvClearErrors(uint8_t node) {
    CanFrame f = {}; f.id = odrvCOB(node, ODRV_CLEAR_ERRORS); f.dlc = 0; canSend(f);
}
static void odrvEstop(uint8_t node) {
    CanFrame f = {}; f.id = odrvCOB(node, ODRV_ESTOP); f.dlc = 0; canSend(f);
}
static bool odrvReadEncoderZero(uint8_t node, float& out_turns) {
    canDrainRx();
    CanFrame rtr = {}; rtr.rtr = true; rtr.id = odrvCOB(node, ODRV_GET_ENCODER); rtr.dlc = 8;
    canSend(rtr);
    CanFrame rx;
    if (canReceiveUntil(ODRIVE_ZERO_TIMEOUT_MS, rx, [&](const CanFrame& f) {
            return !f.rtr && !f.extd && f.id == odrvCOB(node, ODRV_GET_ENCODER) && f.dlc >= 4;
        })) {
        out_turns = getFloat32LE(rx.data);
        return true;
    }
    return false;
}
static bool odrvInitAndReadZero(uint8_t node, float& zero) {
    for (uint8_t a = 0; a < ODRIVE_INIT_MAX_RETRIES; a++) {
        if (a > 0) delay(ODRIVE_INIT_RETRY_DELAY_MS);
        odrvAxisState(node, 8 /*CLOSED_LOOP*/);
        delay(50);
        if (odrvReadEncoderZero(node, zero)) return true;
    }
    return false;
}

struct OdriveFeedback {
    float pos_turns, vel_turns_s, iq_measured_a, bus_voltage_v, bus_current_a;
    bool  fresh;
};
static OdriveFeedback s_odrv_fb[ODRIVE_NUM_JOINTS] = {};
static portMUX_TYPE   s_odrv_mux = portMUX_INITIALIZER_UNLOCKED;

static const uint8_t s_odrv_node[ODRIVE_NUM_JOINTS] = { ODRIVE_NODE_J1, ODRIVE_NODE_J2, ODRIVE_NODE_J3 };
static const float   s_odrv_gear[ODRIVE_NUM_JOINTS] = { ODRIVE_GEAR_J1, ODRIVE_GEAR_J2, ODRIVE_GEAR_J3 };
static const float   s_odrv_dir[ODRIVE_NUM_JOINTS]  = { ODRIVE_DIR_J1, ODRIVE_DIR_J2, ODRIVE_DIR_J3 };
static float         s_odrv_zero[ODRIVE_NUM_JOINTS] = {};

static const uint8_t s_odrv_telem_cmds[] = {
    ODRV_GET_ENCODER, ODRV_GET_IQ, ODRV_GET_BUS_V,
#if ODRIVE_ENABLE_ERROR_POLL
    ODRV_GET_ERROR,
#endif
};
static constexpr uint8_t ODRV_TELEM_COUNT = sizeof(s_odrv_telem_cmds) / sizeof(s_odrv_telem_cmds[0]);
static uint8_t s_odrv_telem_slot = 0;

#if ODRIVE_ENABLE_ERROR_POLL
struct OdriveErrorFb { uint8_t node_id; uint64_t motor_error; bool fresh; };
static OdriveErrorFb s_odrv_err[ODRIVE_NUM_JOINTS] = {};
static portMUX_TYPE  s_odrv_err_mux = portMUX_INITIALIZER_UNLOCKED;
#endif

// ══════════════════════════════════════════════════════════════════════════════
//  LKTech / MyActuator arm J5–J6 (standard frames, ID = 0x140 + motor_id)
// ══════════════════════════════════════════════════════════════════════════════
static constexpr uint8_t LK_MOTOR_ON         = 0x88;
static constexpr uint8_t LK_MOTOR_STOP       = 0x81;
static constexpr uint8_t LK_READ_MULTI_ANGLE = 0x92;
static constexpr uint8_t LK_MULTI_CONTROL_2  = 0xA4;

static const uint8_t s_lk_id[LKTECH_NUM_JOINTS]   = { LKTECH_ID_J5, LKTECH_ID_J6 };
static const float   s_lk_gear[LKTECH_NUM_JOINTS] = { LKTECH_GEAR_J5, LKTECH_GEAR_J6 };
static const float   s_lk_dir[LKTECH_NUM_JOINTS]  = { LKTECH_DIR_J5, LKTECH_DIR_J6 };
static int64_t       s_lk_zero_cdeg[LKTECH_NUM_JOINTS] = {};

struct LktechFeedback {
    int8_t temp_c; int16_t iq_100, speed_dps, angle_deg; float output_deg; bool fresh;
};
static LktechFeedback s_lk_fb[LKTECH_NUM_JOINTS] = {};
static portMUX_TYPE   s_lk_mux = portMUX_INITIALIZER_UNLOCKED;

static bool lkSendRaw(uint8_t motor_id, const uint8_t p[8]) {
    CanFrame f = {}; f.id = LKTECH_ID_BASE + motor_id; f.dlc = 8;
    memcpy(f.data, p, 8);
    return canSend(f);
}
static bool lkMotorOn(uint8_t id)   { uint8_t p[8] = { LK_MOTOR_ON };   return lkSendRaw(id, p); }
static bool lkMotorStop(uint8_t id) { uint8_t p[8] = { LK_MOTOR_STOP }; return lkSendRaw(id, p); }
static bool lkSendPosition(uint8_t id, int32_t cdeg, uint16_t max_dps) {
    uint8_t p[8] = {};
    p[0] = LK_MULTI_CONTROL_2; p[1] = 0;
    p[2] = max_dps & 0xFF; p[3] = (max_dps >> 8) & 0xFF;
    p[4] = cdeg & 0xFF; p[5] = (cdeg >> 8) & 0xFF;
    p[6] = (cdeg >> 16) & 0xFF; p[7] = (cdeg >> 24) & 0xFF;
    return lkSendRaw(id, p);
}
static bool lkReadMultiAngle(uint8_t id, int64_t& out_cdeg) {
    canDrainRx();
    uint8_t p[8] = { LK_READ_MULTI_ANGLE };
    if (!lkSendRaw(id, p)) return false;
    CanFrame rx;
    if (!canReceiveUntil(LKTECH_ZERO_TIMEOUT_MS, rx, [&](const CanFrame& f) {
            return !f.rtr && !f.extd && f.id == (uint32_t)(LKTECH_ID_BASE + id) &&
                   f.dlc >= 8 && f.data[0] == LK_READ_MULTI_ANGLE;
        })) return false;
    uint64_t raw = 0;
    for (int i = 0; i < 7; ++i) raw |= ((uint64_t)rx.data[1 + i]) << (8 * i);
    out_cdeg = (int64_t)(raw << 8) >> 8;   // sign-extend 56-bit
    return true;
}

// ══════════════════════════════════════════════════════════════════════════════
//  ZE300 arm J4 (req ID = 0x100|device_id, reply ID = device_id, variable DLC)
// ══════════════════════════════════════════════════════════════════════════════
static constexpr uint8_t ZE_READ_ABS_ANGLES = 0xA3;
static constexpr uint8_t ZE_READ_RT_STATE   = 0xA4;
static constexpr uint8_t ZE_SET_MAX_SPEED   = 0xB2;
static constexpr uint8_t ZE_ABS_POSITION    = 0xC2;
static constexpr uint8_t ZE_DISABLE_OUTPUT  = 0xCF;

static inline uint32_t zeReqId(uint8_t dev)   { return ZE300_REQ_ID_BASE | dev; }
static inline uint32_t zeReplyId(uint8_t dev) { return dev; }

static int32_t s_ze_zero_counts = 0;
struct Ze300Feedback {
    int8_t temp_c; int16_t iq_1000, speed_rpm_100; uint16_t single_turn;
    int32_t position_counts; bool fresh;
};
static Ze300Feedback s_ze_fb = {};
static portMUX_TYPE  s_ze_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t      s_ze_last_telem_ms = 0;

static bool zeSetMaxSpeed(uint8_t dev, int32_t centi_rpm) {
    canDrainRx();
    CanFrame f = {}; f.id = zeReqId(dev); f.dlc = 5;
    f.data[0] = ZE_SET_MAX_SPEED; putInt32LE(f.data + 1, centi_rpm);
    if (!canSend(f)) return false;
    CanFrame rx;
    return canReceiveUntil(ZE300_ZERO_TIMEOUT_MS, rx, [&](const CanFrame& g) {
        return !g.rtr && !g.extd && g.id == zeReplyId(dev) && g.dlc >= 1 &&
               g.data[0] == ZE_SET_MAX_SPEED;
    });
}
static bool zeReadAbsAngles(uint8_t dev, int32_t& out_counts) {
    canDrainRx();
    CanFrame f = {}; f.id = zeReqId(dev); f.dlc = 1; f.data[0] = ZE_READ_ABS_ANGLES;
    if (!canSend(f)) return false;
    CanFrame rx;
    if (!canReceiveUntil(ZE300_ZERO_TIMEOUT_MS, rx, [&](const CanFrame& g) {
            return !g.rtr && !g.extd && g.id == zeReplyId(dev) && g.dlc >= 7 &&
                   g.data[0] == ZE_READ_ABS_ANGLES;
        })) return false;
    out_counts = getInt32LE(rx.data + 3);
    return true;
}
static bool zeSendAbsPosition(uint8_t dev, int32_t counts) {
    CanFrame f = {}; f.id = zeReqId(dev); f.dlc = 5;
    f.data[0] = ZE_ABS_POSITION; putInt32LE(f.data + 1, counts);
    return canSend(f);
}
static bool zeDisableOutput(uint8_t dev) {
    CanFrame f = {}; f.id = zeReqId(dev); f.dlc = 1; f.data[0] = ZE_DISABLE_OUTPUT;
    return canSend(f);
}
static bool zeReadRealtimeState(uint8_t dev) {
    CanFrame f = {}; f.id = zeReqId(dev); f.dlc = 1; f.data[0] = ZE_READ_RT_STATE;
    return canSend(f);
}

// ══════════════════════════════════════════════════════════════════════════════
//  Arm safety lifecycle: passive boot, explicit arm, latched CAN fault, ramp
// ══════════════════════════════════════════════════════════════════════════════
static volatile ArmState s_arm_state     = ArmState::UNINIT;
static volatile ArmOperatingMode s_arm_mode = ArmOperatingMode::DEXTERITY;
static volatile bool     s_arm_req_arm    = false;
static volatile bool     s_arm_req_disarm = false;
static volatile bool     s_arm_req_mode   = false;
static volatile ArmOperatingMode s_arm_requested_mode = ArmOperatingMode::DEXTERITY;
static uint8_t  s_arm_fault_code = 0;
static uint16_t s_arm_can_fail   = 0;
static uint16_t s_arm_motor_fail = 0;
static uint32_t s_arm_fail_win   = 0;       // window start (millis); 0 = inactive
static bool     s_arm_first_cmd  = true;    // first cmd after arm → ±J4 clamp
#if ARM_RAMP_ENABLE
static const float s_arm_vmax[6] = ARM_RAMP_MAX_VEL_DPS;
static const float s_arm_amax[6] = ARM_RAMP_MAX_ACC_DPS2;
static float    s_arm_ramp_pos[6] = {0, 0, 0, 0, 0, 0};
static float    s_arm_ramp_vel[6] = {0, 0, 0, 0, 0, 0};
static bool     s_arm_ramp_init   = false;
static uint32_t s_arm_ramp_last   = 0;
#endif
static uint32_t s_arm_last_cmd_ms = 0;
static float    s_arm_hold_pos[6] = {0, 0, 0, 0, 0, 0};  // last per-joint output (deg) sent to the motors
static bool     s_arm_estop_hold  = false;               // true while an e-stop is holding the arm energized

// Best-effort disable of every arm motor (used by disarm and latched faults —
// and e-stop only when ARM_ESTOP_HOLD is 0).
static void disableArmMotors() {
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
#if ODRIVE_ESTOP_USE_NATIVE
        odrvEstop(s_odrv_node[j]);
#else
        odrvAxisState(s_odrv_node[j], 1 /*IDLE*/);
#endif
    }
    for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) lkMotorStop(s_lk_id[j]);
    zeDisableOutput(ZE300_ID_J4);
}

// E-stop HOLD: freeze every arm joint at its last commanded pose with the motors
// still ENERGIZED, so the arm stays put instead of dropping under gravity. Just
// re-issues the last output to each controller's position loop (never IDLE /
// motor-off / output-disable). The controllers hold autonomously between calls;
// because the arm control tick re-invokes this every loop while in ESTOP, the
// hold is also continuously re-affirmed (throttled to the normal command cadence
// so it does not flood the MCP2515 TX buffers). The wrist is only re-commanded in
// DEXTERITY mode — in CHASSIS mode J5/J6 are intentionally torque-off (parked),
// so e-stop leaves them off rather than re-energizing them.
static void holdArmMotors() {
    uint32_t now = millis();
    if (s_arm_last_cmd_ms != 0 && now - s_arm_last_cmd_ms < ARM_COMMAND_PERIOD_MS)
        return;                       // controllers already hold between re-sends
    s_arm_last_cmd_ms = now;

    static constexpr float DEG2RAD  = 3.14159265359f / 180.0f;
    static constexpr float TWO_PI_F = 6.28318530718f;
    const float* out = s_arm_hold_pos;

    // J1–J3: ODrive SET_INPUT_POS (turns), offset by boot-pose zero.
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float turns = s_odrv_zero[j] + (out[j] * DEG2RAD / TWO_PI_F) * s_odrv_gear[j] * s_odrv_dir[j];
        odrvInputPos(s_odrv_node[j], turns);
    }
    // J4: ZE300 ABSOLUTE_POSITION (output counts).
    {
        float out_deg = out[3] * ZE300_DIR_J4;
        int32_t counts = s_ze_zero_counts +
            (int32_t)(out_deg * (float)ZE300_COUNTS_PER_REV / 360.0f);
        zeSendAbsPosition(ZE300_ID_J4, counts);
    }
    // J5–J6: LKTech MULTI_LOOP_CONTROL_2 (motor centidegrees), DEXTERITY only.
    if (s_arm_mode == ArmOperatingMode::DEXTERITY) {
        for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
            float motor_deg = out[4 + j] * s_lk_gear[j] * s_lk_dir[j];
            int64_t cdeg = s_lk_zero_cdeg[j] + (int64_t)(motor_deg * 100.0f);
            lkSendPosition(s_lk_id[j], (int32_t)cdeg, LKTECH_DEFAULT_SPEED_DPS);
        }
    }
}

// Latch a fault: disable motors and reject motion until an explicit re-arm.
static void enterArmFault(uint8_t code) {
    if (s_arm_state == ArmState::FAULT) return;
    s_arm_fault_code = code;
    s_arm_motor_fail = 1;
    disableArmMotors();
    s_arm_state = ArmState::FAULT;
}

static bool applyArmOperatingMode(ArmOperatingMode mode) {
    if (mode == s_arm_mode) return true;

    if (mode == ArmOperatingMode::CHASSIS) {
        // Gate position sends before torque-off so a concurrent control tick
        // cannot restore position hold during the transition.
        s_arm_mode = mode;
        if (s_arm_state != ArmState::READY) return true;
        bool ok = true;
        for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) ok &= lkMotorStop(s_lk_id[j]);
        if (!ok) enterArmFault(2 /*motor_cmd*/);
        return ok;
    }

    if (s_arm_state == ArmState::READY) {
        bool ok = true;
        int64_t current_cdeg[LKTECH_NUM_JOINTS] = {};
        for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
            ok &= lkMotorOn(s_lk_id[j]);
            delay(20);
            ok &= lkReadMultiAngle(s_lk_id[j], current_cdeg[j]);
        }
        for (uint8_t j = 0; j < LKTECH_NUM_JOINTS && ok; j++)
            ok &= lkSendPosition(s_lk_id[j], (int32_t)current_cdeg[j], LKTECH_DEFAULT_SPEED_DPS);
        if (!ok) {
            enterArmFault(2 /*motor_cmd*/);
            return false;
        }
#if ARM_RAMP_ENABLE
        for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
            float motor_deg = (current_cdeg[j] - s_lk_zero_cdeg[j]) / 100.0f;
            s_arm_ramp_pos[4 + j] = motor_deg / (s_lk_gear[j] * s_lk_dir[j]);
            s_arm_ramp_vel[4 + j] = 0.0f;
        }
#endif
    }
    s_arm_mode = mode;
    return true;
}

// Sliding-window failure counter on the arm send path → escalate to FAULT.
static void noteArmSendResult(bool ok_send) {
    uint32_t now = millis();
    if (s_arm_fail_win == 0 || now - s_arm_fail_win > ARM_FAULT_WINDOW_MS) {
        s_arm_fail_win = now;
        s_arm_can_fail = 0;
    }
    if (!ok_send && ++s_arm_can_fail >= ARM_CAN_FAIL_THRESHOLD) {
        enterArmFault(1 /*can_send*/);
    }
}

// Blocking arm bring-up (runs from poll(), never the control loop): enable each
// controller and capture its software zero. Strict — any stage failing leaves
// the arm DISARMED rather than READY.
static bool armArmInternal() {
    if (!s_ok) return false;
    s_arm_state      = ArmState::INITIALIZING;
    s_arm_fault_code = 0;
    s_arm_can_fail   = 0;
    s_arm_motor_fail = 0;
    s_arm_presence_mask = 0;
    for (uint8_t j = 0; j < 6; j++) s_arm_joint_heard_ms[j] = 0;
    s_arm_fail_win   = 0;
    s_arm_estop_hold = false;   // a fresh arm clears any lingering e-stop hold
    bool ok = true;

    // ODrive: clear errors → closed loop → capture zero (with retries).
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) odrvClearErrors(s_odrv_node[j]);
    delay(10);
    uint8_t failed_stage = 0;

    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float zero = 0.0f; bool zeroed = false;
        zeroed = odrvInitAndReadZero(s_odrv_node[j], zero);
        s_odrv_zero[j] = zeroed ? zero : 0.0f;
        if (zeroed) markArmJoint(j);
        if (!zeroed && failed_stage == 0) failed_stage = 10 + j;  // 10..12 = ODrive J1..J3
        ok &= zeroed;
    }

    // LKTech: motor on → capture boot pose as zero (with retries).
    for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
        int64_t zero = 0; bool zeroed = false;
        for (uint8_t a = 0; a < LKTECH_INIT_MAX_RETRIES && !zeroed; a++) {
            if (a > 0) delay(LKTECH_INIT_RETRY_DELAY_MS);
            lkMotorOn(s_lk_id[j]);
            delay(20);
            zeroed = lkReadMultiAngle(s_lk_id[j], zero);
        }
        s_lk_zero_cdeg[j] = zeroed ? zero : 0;
        if (zeroed) markArmJoint(4 + j);
        if (!zeroed && failed_stage == 0) failed_stage = 20 + j;  // 20..21 = LKTech J5..J6
        ok &= zeroed;
        if (zeroed && s_arm_mode == ArmOperatingMode::DEXTERITY) {
            bool held = lkSendPosition(s_lk_id[j], (int32_t)zero, LKTECH_DEFAULT_SPEED_DPS);
            if (!held && failed_stage == 0) failed_stage = 22 + j; // 22..23 = LKTech hold J5..J6
            ok &= held;
        }
    }

    // ZE300: set max speed → capture absolute zero (with retries).
    {
        int32_t counts = 0; bool ready = false;
        for (uint8_t a = 0; a < ZE300_INIT_MAX_RETRIES && !ready; a++) {
            if (a > 0) delay(ZE300_INIT_RETRY_DELAY_MS);
            ready = zeSetMaxSpeed(ZE300_ID_J4, ZE300_MAX_SPEED_CRPM) &&
                    zeReadAbsAngles(ZE300_ID_J4, counts);
        }
        if (ready) s_ze_zero_counts = counts;
        if (ready) markArmJoint(3);
        if (!ready && failed_stage == 0) failed_stage = 30;       // ZE300 J4
        ok &= ready;
    }

    // Initialization needs J5/J6 awake to capture their zero. If transport mode
    // was selected before arming, return them to torque-off before READY.
    if (s_arm_mode == ArmOperatingMode::CHASSIS) {
        bool stopped = true;
        for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) stopped &= lkMotorStop(s_lk_id[j]);
        if (!stopped && failed_stage == 0) failed_stage = 40;
        ok &= stopped;
    }

    if (!ok) {
        s_arm_fault_code = failed_stage;
        s_arm_motor_fail = 1;
        disableArmMotors();
        s_arm_state = ArmState::UNINIT;
        return false;
    }

    s_arm_first_cmd = true;
#if ARM_RAMP_ENABLE
    s_arm_ramp_init = false;
#endif
    s_arm_last_cmd_ms = 0;
    s_arm_state = ArmState::READY;
    return true;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Public API
// ══════════════════════════════════════════════════════════════════════════════
bool CANInterface::begin() {
    if (!canBackendBegin()) { s_ok = false; return false; }
    s_ok = true;
    s_arm_state = ArmState::UNINIT;        // passive boot — the arm is DISARMED
#if ROBOCOREA_ROLE_IS_ARM && !ARM_PASSIVE_BOOT
    armArmInternal();                      // legacy: auto-arm the arm at boot
#endif
    return true;
}

void CANInterface::requestArm() {
#if ROBOCOREA_ROLE_IS_ARM
    s_arm_req_arm = true;
#endif
}
void CANInterface::requestDisarm() {
#if ROBOCOREA_ROLE_IS_ARM
    s_arm_req_disarm = true;
#endif
}
void CANInterface::requestOperatingMode(ArmOperatingMode mode) {
#if ROBOCOREA_ROLE_IS_ARM
    s_arm_requested_mode = mode;
    s_arm_req_mode = true;
#else
    (void)mode;
#endif
}
uint8_t CANInterface::armState()   { return (uint8_t)s_arm_state; }
uint8_t CANInterface::armOperatingMode() { return (uint8_t)s_arm_mode; }

void CANInterface::getArmLifecycle(ArmLifecyclePayload& out) {
    out.state            = (uint8_t)s_arm_state;
    out.fault_code       = s_arm_fault_code;
    out.can_fail_count   = s_arm_can_fail;
    out.motor_fail_count = s_arm_motor_fail;
    out.eflg             = s_ok ? canBackendEflg() : 0;
    out.operating_mode     = (uint8_t)s_arm_mode;
    if (s_arm_state != ArmState::READY) out.active_joint_mask = 0;
    else out.active_joint_mask = (s_arm_mode == ArmOperatingMode::DEXTERITY) ? 0x3F : 0x0F;

    // Live presence: start from the arm-time snapshot, then clear any ACTIVE joint
    // that has gone silent longer than ARM_JOINT_STALE_MS, so a joint that drops
    // off the bus mid-session reads as absent instead of frozen-green. Idle joints
    // (not in active_joint_mask — e.g. J5/J6 in CHASSIS) are left as captured: they
    // are intentionally uncommanded and legitimately quiet.
    uint16_t presence = s_arm_presence_mask;
#if ARM_JOINT_STALE_MS > 0
    if (s_arm_state == ArmState::READY) {
        const uint32_t now = millis();
        for (uint8_t j = 0; j < 6; j++) {
            const uint16_t bit = (uint16_t)(1U << j);
            if (!(presence & bit) || !(out.active_joint_mask & bit)) continue;
            if (now - s_arm_joint_heard_ms[j] > (uint32_t)ARM_JOINT_STALE_MS)
                presence &= (uint16_t)~bit;
        }
    }
#endif
    out.init_presence_mask = presence;
}

bool CANInterface::isOk() { return s_ok; }

bool CANInterface::sendTrackSpeeds(float left_norm, float right_norm) {
#if !ROBOCOREA_ROLE_IS_CHASSIS
    (void)left_norm; (void)right_norm;
    return true;
#else
    if (!s_ok) return false;
    left_norm  = clampf(left_norm,  -1.0f, 1.0f);
    right_norm = clampf(right_norm, -1.0f, 1.0f);
    bool ok = vescSendRpm(s_traction_id[0], (int32_t)(left_norm  * TRACTION_ERPM_MAX * s_traction_dir[0]));
    ok    &= vescSendRpm(s_traction_id[1], (int32_t)(right_norm * TRACTION_ERPM_MAX * s_traction_dir[1]));
    return ok;
#endif
}

bool CANInterface::sendFlipperAngles(const float target_deg[4], bool enabled) {
#if !ROBOCOREA_ROLE_IS_CHASSIS
    (void)target_deg; (void)enabled;
    return true;
#else
    if (!s_ok) return false;
    bool ok = true;
#if FLIPPER_USE_LEGACY_RPM_LISP
    // Fake-RPM flipper Lisp reads the VESC RPM setpoint as:
    //   target_degrees = get-rpm-set / 1000
    // It has no measured-angle report and no enable/coast bit. When disabled,
    // command target 0 to match the old 5.ino failsafe/e-stop behavior.
    for (uint8_t i = 0; i < 4; i++) {
        float command_deg = enabled ? target_deg[i] : 0.0f;
        ok &= vescSendRpm(s_flipper_id[i], (int32_t)lroundf(command_deg * 1000.0f));

#if !FLIPPER_USE_TACH_FEEDBACK
        // No measured-angle report in fake-RPM mode: the lisp faithfully tracks
        // this commanded target, so it IS the reported flipper angle. Wrap to
        // [0,360) for clean /encoders/flipper telemetry (and so the int16×10 wire
        // field never overflows). The control accumulator in Control stays
        // continuous, so the lisp setpoint sent above is still continuous.
        portENTER_CRITICAL(&s_vesc_mux);
        s_flipper_deg[i]    = wrap360(command_deg);
        s_flipper_deg_ok[i] = true;
        portEXIT_CRITICAL(&s_vesc_mux);
#endif
    }
#else
    // Targets are absolute angles in the VESC's own (lisp-measured) frame; no
    // direction sign here — operator direction is applied to the stick rate in
    // Control. The lisp wraps internally; we wrap too for a clean wire value.
    for (uint8_t i = 0; i < 4; i++)
        ok &= vescSendFlipperTarget(s_flipper_id[i], wrap360(target_deg[i]), enabled);
#endif
    return ok;
#endif
}

bool CANInterface::sendArmJoints(const float angles_deg_in[6]) {
#if !ROBOCOREA_ROLE_IS_ARM
    (void)angles_deg_in;
    return true;
#else
    if (!s_ok) return false;
    if (s_arm_state != ArmState::READY) return false;   // gated: arm must be armed
    if (s_arm_estop_hold) return true;                  // e-stop hold: keep the freeze, ignore new commands
    uint32_t now = millis();
    if (s_arm_last_cmd_ms != 0 && now - s_arm_last_cmd_ms < ARM_COMMAND_PERIOD_MS)
        return true;
    s_arm_last_cmd_ms = now;

    static constexpr float DEG2RAD  = 3.14159265359f / 180.0f;
    static constexpr float TWO_PI_F = 6.28318530718f;
    bool ok = true;

    // ── goal = command, clamped by soft limits + the ±30° first-J4 guard ─────
    float goal[6];
    for (uint8_t j = 0; j < 6; j++) goal[j] = angles_deg_in[j];
#if ARM_SOFT_LIMIT_ENABLE
    static const float s_arm_min[6] = {
        ARM_LIMIT_J1_MIN_DEG, ARM_LIMIT_J2_MIN_DEG, ARM_LIMIT_J3_MIN_DEG,
        ARM_LIMIT_J4_MIN_DEG, ARM_LIMIT_J5_MIN_DEG, ARM_LIMIT_J6_MIN_DEG };
    static const float s_arm_max[6] = {
        ARM_LIMIT_J1_MAX_DEG, ARM_LIMIT_J2_MAX_DEG, ARM_LIMIT_J3_MAX_DEG,
        ARM_LIMIT_J4_MAX_DEG, ARM_LIMIT_J5_MAX_DEG, ARM_LIMIT_J6_MAX_DEG };
    for (uint8_t j = 0; j < 6; j++) goal[j] = clampf(goal[j], s_arm_min[j], s_arm_max[j]);
#endif
    // First J4 command after arming is clamped near the captured zero to catch a
    // bad startup pose before a large ZE300 move.
    if (s_arm_first_cmd)
        goal[3] = clampf(goal[3], -ARM_JOINT4_FIRST_CMD_MAX_DEG, ARM_JOINT4_FIRST_CMD_MAX_DEG);

    // ── velocity/accel-limited ramp → the actual per-joint output ────────────
#if ARM_RAMP_ENABLE
    if (!s_arm_ramp_init) {
        for (uint8_t j = 0; j < 6; j++) { s_arm_ramp_pos[j] = goal[j]; s_arm_ramp_vel[j] = 0.0f; }
        s_arm_ramp_init = true;
        s_arm_ramp_last = now;
    }
    float dt = (now - s_arm_ramp_last) * 0.001f;
    s_arm_ramp_last = now;
    if (dt > 0.0f && dt < 0.5f) {
        uint8_t ramp_count = (s_arm_mode == ArmOperatingMode::DEXTERITY) ? 6 : 4;
        for (uint8_t j = 0; j < ramp_count; j++) {
            float v_des = clampf((goal[j] - s_arm_ramp_pos[j]) / dt, -s_arm_vmax[j], s_arm_vmax[j]);
            float dv    = clampf(v_des - s_arm_ramp_vel[j], -s_arm_amax[j] * dt, s_arm_amax[j] * dt);
            s_arm_ramp_vel[j] = clampf(s_arm_ramp_vel[j] + dv, -s_arm_vmax[j], s_arm_vmax[j]);
            s_arm_ramp_pos[j] += s_arm_ramp_vel[j] * dt;
        }
    }
    const float* out = s_arm_ramp_pos;
#else
    const float* out = goal;
#endif

    // J1–J3: ODrive SET_INPUT_POS (turns), offset by boot-pose zero.
    for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
        float turns = s_odrv_zero[j] + (out[j] * DEG2RAD / TWO_PI_F) * s_odrv_gear[j] * s_odrv_dir[j];
        ok &= odrvInputPos(s_odrv_node[j], turns);
    }

    // J4: ZE300 ABSOLUTE_POSITION (output counts; driver handles its gearbox).
    {
        float out_deg = out[3] * ZE300_DIR_J4;
        int32_t counts = s_ze_zero_counts +
            (int32_t)(out_deg * (float)ZE300_COUNTS_PER_REV / 360.0f);
        ok &= zeSendAbsPosition(ZE300_ID_J4, counts);
    }

    // J5–J6: LKTech MULTI_LOOP_CONTROL_2 (motor centidegrees), offset by zero.
    if (s_arm_mode == ArmOperatingMode::DEXTERITY) {
        for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
            float motor_deg = out[4 + j] * s_lk_gear[j] * s_lk_dir[j];
            int64_t cdeg = s_lk_zero_cdeg[j] + (int64_t)(motor_deg * 100.0f);
            ok &= lkSendPosition(s_lk_id[j], (int32_t)cdeg, LKTECH_DEFAULT_SPEED_DPS);
        }
    }

    // Remember the actual output so an e-stop can freeze the arm right here.
    for (uint8_t j = 0; j < 6; j++) s_arm_hold_pos[j] = out[j];

    if (ok && s_arm_first_cmd) s_arm_first_cmd = false;
    noteArmSendResult(ok);
    return ok;
#endif
}

void CANInterface::poll() {
#if ROBOCOREA_ROLE_IS_ARM
    // Service arm lifecycle requests here — the bring-up blocks for ~seconds, so
    // it must run in the CAN task, never the 50 Hz control loop.
    if (s_arm_req_disarm) {
        s_arm_req_disarm = false;
        s_arm_estop_hold = false;   // explicit disarm overrides an e-stop hold
        disableArmMotors();
        // Clear the presence snapshot so a disarmed arm doesn't keep reporting the
        // last session's joints as present (the GUI dots go blank until re-arm).
        s_arm_presence_mask = 0;
        for (uint8_t j = 0; j < 6; j++) s_arm_joint_heard_ms[j] = 0;
        if (s_arm_state != ArmState::FAULT) s_arm_state = ArmState::UNINIT;
    }
    if (s_arm_req_arm) {
        s_arm_req_arm = false;
        if (s_arm_state == ArmState::UNINIT || s_arm_state == ArmState::FAULT)
            armArmInternal();
    }
    if (s_arm_req_mode) {
        s_arm_req_mode = false;
        applyArmOperatingMode(s_arm_requested_mode);
    }
#endif

    // Bus health / fault recovery first, so a BUS_OFF (or MCP fault) can recover
    // even though the rest of poll() and the senders bail out while !s_ok.
    {
        static uint32_t s_last_health_ms = 0;
        uint32_t now = millis();
        if (now - s_last_health_ms >= CAN_HEALTH_PERIOD_MS) {
            s_last_health_ms = now;
            s_ok = canBackendHealthy();
        }
    }
    if (!s_ok) return;

    CanFrame f;
    while (canReceive(f)) {
        noteArmPresenceFromRxFrame(f);
        if (f.rtr) continue;

        // ── Extended frames: VESC status + flipper-lisp reports ─────────────
        if (f.extd) {
            uint8_t id  = f.id & 0xFF;
            uint8_t cmd = (f.id >> 8) & 0xFF;

            // Flipper angle report from flipper_position.lisp (int16 BE deci-deg).
            if (cmd == VESC_CMD_FLIPPER_REPORT) {
                if (f.dlc >= 2) {
                    float deg = wrap360(getInt16BE(f.data) / 10.0f);
                    for (int k = 0; k < 4; k++) if (s_flipper_id[k] == id) {
                        portENTER_CRITICAL(&s_vesc_mux);
                        s_flipper_deg[k]    = deg;
                        s_flipper_deg_ok[k] = true;
                        portEXIT_CRITICAL(&s_vesc_mux);
                        break;
                    }
                }
                continue;
            }

            // Broadcast status frames. Gate DLC per command (some VESC builds
            // emit STATUS_5 as 6 bytes, not padded to 8).
            int i = vescIdToIndex(id);
            if (i < 0) continue;
            portENTER_CRITICAL(&s_vesc_mux);
            switch (cmd) {
                case 9:   // STATUS_1: eRPM, current, duty
                    if (f.dlc >= 8) {
                        s_vesc[i].erpm       = getInt32BE(f.data);
                        s_vesc[i].current_10 = getInt16BE(f.data + 4);
                        s_vesc[i].duty_1000  = getInt16BE(f.data + 6);
                        s_vesc[i].fresh = true;
                    }
                    break;
                case 16:  // STATUS_4: temp_fet, temp_motor, current_in, pid_pos
                    if (f.dlc >= 4) {
                        s_vesc[i].temp_fet_10   = getInt16BE(f.data);
                        s_vesc[i].temp_motor_10 = getInt16BE(f.data + 2);
                        s_vesc[i].fresh = true;
                    }
                    break;
                case 27:  // STATUS_5: tachometer (i32 BE), v_in (i16 BE)
                    if (f.dlc >= 6) {
                        int32_t tach      = getInt32BE(f.data);
                        s_vesc[i].tacho   = tach;
                        s_vesc[i].v_in_10 = getInt16BE(f.data + 4);
                        s_vesc[i].fresh = true;
#if FLIPPER_USE_TACH_FEEDBACK
                        for (uint8_t k = 0; k < 4; k++) {
                            if (s_flipper_id[k] != id) continue;
                            if (!s_flipper_tach_zero_ok[k]) {
                                s_flipper_tach_zero_count[k] = tach;
                                s_flipper_tach_zero_ok[k] = true;
                            }
                            int64_t delta = (int64_t)tach - (int64_t)s_flipper_tach_zero_count[k];
                            float deg = s_flipper_tach_zero_deg[k] +
                                        (float)delta * s_flipper_tach_deg_per_count[k];
                            s_flipper_deg[k] = wrap360(deg);
                            s_flipper_deg_ok[k] = true;
                            break;
                        }
#endif
                    }
                    break;
                default: break;
            }
            portEXIT_CRITICAL(&s_vesc_mux);
            continue;
        }

#if !ROBOCOREA_ROLE_IS_ARM
        continue;
#else
        uint8_t node = (f.id >> 5) & 0x3F;
        uint8_t cmd  = f.id & 0x1F;

        // ── ODrive telemetry (DLC ≥ 8) ──────────────────────────────────────
        if (f.dlc >= 8) {
            bool matched = false;
            for (uint8_t j = 0; j < ODRIVE_NUM_JOINTS; j++) {
                if (s_odrv_node[j] != node) continue;
                matched = true;
                portENTER_CRITICAL(&s_odrv_mux);
                switch (cmd) {
                    case ODRV_GET_ENCODER:
                        s_odrv_fb[j].pos_turns   = getFloat32LE(f.data);
                        s_odrv_fb[j].vel_turns_s = getFloat32LE(f.data + 4);
                        s_odrv_fb[j].fresh = true; break;
                    case ODRV_GET_IQ:
                        s_odrv_fb[j].iq_measured_a = getFloat32LE(f.data + 4);
                        s_odrv_fb[j].fresh = true; break;
                    case ODRV_GET_BUS_V:
                        s_odrv_fb[j].bus_voltage_v = getFloat32LE(f.data);
                        s_odrv_fb[j].bus_current_a = getFloat32LE(f.data + 4);
                        s_odrv_fb[j].fresh = true; break;
#if ODRIVE_ENABLE_ERROR_POLL
                    case ODRV_GET_ERROR: {
                        uint64_t e = 0;
                        for (int b = 0; b < 8; b++) e |= (uint64_t)f.data[b] << (8 * b);
                        portENTER_CRITICAL(&s_odrv_err_mux);
                        s_odrv_err[j].node_id = node;
                        s_odrv_err[j].motor_error = e;
                        s_odrv_err[j].fresh = true;
                        portEXIT_CRITICAL(&s_odrv_err_mux);
                        break;
                    }
#endif
                    default: break;
                }
                portEXIT_CRITICAL(&s_odrv_mux);
                break;
            }
            if (matched) continue;

            // ── LKTech reply (ID = 0x140 + motor_id) ────────────────────────
            if (f.id >= (uint32_t)LKTECH_ID_BASE && f.id < (uint32_t)(LKTECH_ID_BASE + 256)) {
                uint8_t mid = (uint8_t)(f.id - LKTECH_ID_BASE);
                for (uint8_t j = 0; j < LKTECH_NUM_JOINTS; j++) {
                    if (s_lk_id[j] != mid) continue;
                    uint8_t c = f.data[0];
                    if (c != LK_MULTI_CONTROL_2 && c != 0x9C /*READ_STATE_2*/) break;
                    portENTER_CRITICAL(&s_lk_mux);
                    s_lk_fb[j].temp_c    = (int8_t)f.data[1];
                    s_lk_fb[j].iq_100    = getInt16LE(f.data + 2);
                    s_lk_fb[j].speed_dps = getInt16LE(f.data + 4);
                    s_lk_fb[j].angle_deg = getInt16LE(f.data + 6);
                    s_lk_fb[j].output_deg = ((float)s_lk_fb[j].angle_deg / s_lk_gear[j]) * s_lk_dir[j];
                    s_lk_fb[j].fresh = true;
                    portEXIT_CRITICAL(&s_lk_mux);
                    break;
                }
                continue;
            }
        }

        // ── ZE300 reply (reply ID = device_id, variable DLC) ────────────────
        if (f.id == (uint32_t)zeReplyId(ZE300_ID_J4) && f.dlc >= 1) {
            uint8_t c = f.data[0];
            if (c == ZE_ABS_POSITION && f.dlc >= 5) {
                portENTER_CRITICAL(&s_ze_mux);
                s_ze_fb.position_counts = getInt32LE(f.data + 1);
                s_ze_fb.fresh = true;
                portEXIT_CRITICAL(&s_ze_mux);
            } else if (c == ZE_READ_RT_STATE && f.dlc >= 8) {
                portENTER_CRITICAL(&s_ze_mux);
                s_ze_fb.temp_c        = (int8_t)f.data[1];
                s_ze_fb.iq_1000       = getInt16LE(f.data + 2);
                s_ze_fb.speed_rpm_100 = getInt16LE(f.data + 4);
                s_ze_fb.single_turn   = getUint16LE(f.data + 6);
                s_ze_fb.fresh = true;
                portEXIT_CRITICAL(&s_ze_mux);
            }
            continue;
        }
#endif
    }

#if ROBOCOREA_ROLE_IS_ARM
    // ── One ODrive telemetry RTR per poll (round-robin) ─────────────────────
    {
        uint8_t j = s_odrv_telem_slot / ODRV_TELEM_COUNT;
        uint8_t c = s_odrv_telem_slot % ODRV_TELEM_COUNT;
        CanFrame rtr = {}; rtr.rtr = true;
        rtr.id = odrvCOB(s_odrv_node[j], s_odrv_telem_cmds[c]); rtr.dlc = 8;
        canSend(rtr);
        s_odrv_telem_slot = (s_odrv_telem_slot + 1) % (ODRIVE_NUM_JOINTS * ODRV_TELEM_COUNT);
    }

    // ── ZE300 realtime-state poll (~5 Hz) ───────────────────────────────────
    {
        uint32_t now = millis();
        if (now - s_ze_last_telem_ms >= ZE300_TELEM_INTERVAL_MS) {
            s_ze_last_telem_ms = now;
            zeReadRealtimeState(ZE300_ID_J4);
        }
    }
#endif

}

// ─── Derived drivetrain feedback ──────────────────────────────────────────────
void CANInterface::getTractionSpeeds(float& left_rpm, float& right_rpm) {
#if !ROBOCOREA_ROLE_IS_CHASSIS
    left_rpm = 0.0f;
    right_rpm = 0.0f;
#else
    int il = vescIdToIndex(s_traction_id[0]);
    int ir = vescIdToIndex(s_traction_id[1]);
    portENTER_CRITICAL(&s_vesc_mux);
    int32_t el = (il >= 0) ? s_vesc[il].erpm : 0;
    int32_t er = (ir >= 0) ? s_vesc[ir].erpm : 0;
    portEXIT_CRITICAL(&s_vesc_mux);
    // eRPM → mechanical motor RPM → output RPM, with direction correction.
    const float k = 1.0f / ((float)VESC_POLE_PAIRS * TRACTION_GEAR_RATIO);
    left_rpm  = el * k * s_traction_dir[0];
    right_rpm = er * k * s_traction_dir[1];
#endif
}

// Flipper angle comes from STATUS_5 tachometer when FLIPPER_USE_TACH_FEEDBACK is
// enabled, otherwise from the custom Lisp report or command estimate.
void CANInterface::getFlipperAngles(float out_deg[4]) {
#if !ROBOCOREA_ROLE_IS_CHASSIS
    for (uint8_t i = 0; i < 4; i++) out_deg[i] = 0.0f;
#else
    portENTER_CRITICAL(&s_vesc_mux);
    for (uint8_t i = 0; i < 4; i++)
        out_deg[i] = s_flipper_deg_ok[i] ? s_flipper_deg[i] : 0.0f;
    portEXIT_CRITICAL(&s_vesc_mux);
#endif
}

// ─── Raw status accessors ─────────────────────────────────────────────────────
uint8_t CANInterface::vescIdByIndex(uint8_t idx) {
#if !ROBOCOREA_ROLE_IS_CHASSIS
    (void)idx;
    return 0;
#else
    return (idx < 6) ? s_vesc_id[idx] : 0;
#endif
}

bool CANInterface::getVescStatus(uint8_t vesc_id, VescStatusPayload& out) {
#if !ROBOCOREA_ROLE_IS_CHASSIS
    (void)vesc_id; (void)out;
    return false;
#else
    int idx = vescIdToIndex(vesc_id);
    if (idx < 0) return false;
    uint8_t i = (uint8_t)idx;
    portENTER_CRITICAL(&s_vesc_mux);
    if (!s_vesc[i].fresh) { portEXIT_CRITICAL(&s_vesc_mux); return false; }
    out.vesc_id       = vesc_id;
    out.erpm          = s_vesc[i].erpm;
    out.current_10    = s_vesc[i].current_10;
    out.duty_1000     = s_vesc[i].duty_1000;
    out.temp_fet_10   = s_vesc[i].temp_fet_10;
    out.temp_motor_10 = s_vesc[i].temp_motor_10;
    out.v_in_10       = s_vesc[i].v_in_10;
    out.tachometer    = s_vesc[i].tacho;
    s_vesc[i].fresh   = false;
    portEXIT_CRITICAL(&s_vesc_mux);
    return true;
#endif
}

bool CANInterface::getOdriveStatus(uint8_t joint_idx, OdriveStatusPayload& out) {
#if !ROBOCOREA_ROLE_IS_ARM
    (void)joint_idx; (void)out;
    return false;
#else
    if (joint_idx >= ODRIVE_NUM_JOINTS) return false;
    portENTER_CRITICAL(&s_odrv_mux);
    if (!s_odrv_fb[joint_idx].fresh) { portEXIT_CRITICAL(&s_odrv_mux); return false; }
    out.joint_idx       = joint_idx;
    out.pos_turns_100   = (int16_t)(s_odrv_fb[joint_idx].pos_turns     * 100.0f);
    out.vel_turns_s_100 = (int16_t)(s_odrv_fb[joint_idx].vel_turns_s   * 100.0f);
    out.iq_measured_100 = (int16_t)(s_odrv_fb[joint_idx].iq_measured_a * 100.0f);
    out.bus_voltage_10  = (int16_t)(s_odrv_fb[joint_idx].bus_voltage_v * 10.0f);
    out.bus_current_100 = (int16_t)(s_odrv_fb[joint_idx].bus_current_a * 100.0f);
    s_odrv_fb[joint_idx].fresh = false;
    portEXIT_CRITICAL(&s_odrv_mux);
    return true;
#endif
}

bool CANInterface::getLktechStatus(uint8_t joint_idx, LktechStatusPayload& out) {
#if !ROBOCOREA_ROLE_IS_ARM
    (void)joint_idx; (void)out;
    return false;
#else
    if (joint_idx >= LKTECH_NUM_JOINTS) return false;
    portENTER_CRITICAL(&s_lk_mux);
    if (!s_lk_fb[joint_idx].fresh) { portEXIT_CRITICAL(&s_lk_mux); return false; }
    out.joint_idx     = joint_idx;
    out.motor_id      = s_lk_id[joint_idx];
    out.temp_c        = s_lk_fb[joint_idx].temp_c;
    out.iq_100        = s_lk_fb[joint_idx].iq_100;
    out.speed_dps     = s_lk_fb[joint_idx].speed_dps;
    out.angle_deg     = s_lk_fb[joint_idx].angle_deg;
    out.output_deg_10 = (int16_t)(s_lk_fb[joint_idx].output_deg * 10.0f);
    s_lk_fb[joint_idx].fresh = false;
    portEXIT_CRITICAL(&s_lk_mux);
    return true;
#endif
}

bool CANInterface::getZe300Status(Ze300StatusPayload& out) {
#if !ROBOCOREA_ROLE_IS_ARM
    (void)out;
    return false;
#else
    portENTER_CRITICAL(&s_ze_mux);
    if (!s_ze_fb.fresh) { portEXIT_CRITICAL(&s_ze_mux); return false; }
    out.device_id          = ZE300_ID_J4;
    out.temp_c             = s_ze_fb.temp_c;
    out.iq_1000            = s_ze_fb.iq_1000;
    out.speed_rpm_100      = s_ze_fb.speed_rpm_100;
    out.single_turn_counts = (int16_t)s_ze_fb.single_turn;
    out.position_counts    = s_ze_fb.position_counts;
    {
        float out_deg = (float)(s_ze_fb.position_counts - s_ze_zero_counts)
                        * 360.0f / (float)ZE300_COUNTS_PER_REV * ZE300_DIR_J4;
        out.output_deg_10 = (int16_t)(out_deg * 10.0f);
    }
    s_ze_fb.fresh = false;
    portEXIT_CRITICAL(&s_ze_mux);
    return true;
#endif
}

uint8_t CANInterface::odriveNodeCount() {
#if ROBOCOREA_ROLE_IS_ARM
    return ODRIVE_NUM_JOINTS;
#else
    return 0;
#endif
}

bool CANInterface::getOdriveError(uint8_t node_idx, OdriveErrorPayload& out) {
#if ODRIVE_ENABLE_ERROR_POLL && ROBOCOREA_ROLE_IS_ARM
    if (node_idx >= ODRIVE_NUM_JOINTS) return false;
    portENTER_CRITICAL(&s_odrv_err_mux);
    if (!s_odrv_err[node_idx].fresh) { portEXIT_CRITICAL(&s_odrv_err_mux); return false; }
    out.node_id     = s_odrv_err[node_idx].node_id;
    out.motor_error = s_odrv_err[node_idx].motor_error;
    s_odrv_err[node_idx].fresh = false;
    portEXIT_CRITICAL(&s_odrv_err_mux);
    return true;
#else
    (void)node_idx; (void)out; return false;
#endif
}

// ─── Arm e-stop / recovery ────────────────────────────────────────────────────
// With ARM_ESTOP_HOLD (default): e-stop FREEZES the armed arm at its last pose
// with the motors energized (so it cannot fall), and clearing the e-stop resumes
// motion in place — no re-arm needed. With ARM_ESTOP_HOLD=0 (legacy): e-stop
// DISARMS the arm (de-energise + drop to UNINIT) and, under the passive-safety
// model, it stays disarmed until an explicit re-arm (MSG_ARM_INIT). Either way an
// un-armed arm has nothing to hold, and a latched FAULT always de-energises.
void CANInterface::estopArm() {
#if ROBOCOREA_ROLE_IS_ARM
    if (!s_ok) return;
#if ARM_ESTOP_HOLD
    // Hold the arm where it is, motors energized, so it does not fall. The arm
    // control tick calls this every loop while in ESTOP, so the hold is
    // continuously re-affirmed. Keep the lifecycle state untouched (the motors
    // stay armed/energized); sendArmJoints() is blocked while s_arm_estop_hold is
    // set, and clearEstopArm() resumes seamlessly without a re-arm. A FAULT can
    // still escalate to disableArmMotors() independently. If the arm was never
    // armed (UNINIT/INITIALIZING/FAULT) there is nothing energized to hold.
    if (s_arm_state == ArmState::READY) {
        s_arm_estop_hold = true;
        holdArmMotors();
    } else {
        disableArmMotors();
    }
#else
    // Legacy behaviour: de-energize every joint (the arm will droop) and disarm.
    disableArmMotors();
    s_arm_state = ArmState::UNINIT;
#endif
#endif
}

void CANInterface::clearEstopArm() {
#if ROBOCOREA_ROLE_IS_ARM
    // With ARM_ESTOP_HOLD the motors stayed energized at the hold pose, so simply
    // lift the hold and joint commands flow again (the firmware ramp eases the arm
    // from the hold pose toward the live target). When holding was not engaged
    // this is a no-op: re-arm explicitly via requestArm() (MSG_ARM_INIT). (Set
    // ARM_PASSIVE_BOOT 0 + call requestArm here for the old auto-recover.)
    s_arm_estop_hold = false;
#endif
}
