#include "Comms.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>

Comms::RxState Comms::s_rx_state = Comms::RxState::SOF0;
uint8_t  Comms::s_rx_type    = 0;
uint16_t Comms::s_rx_len     = 0;
uint16_t Comms::s_rx_idx     = 0;
uint8_t  Comms::s_rx_buf[PROTO_MAX_PAYLOAD];
uint8_t  Comms::s_rx_crc     = 0;
uint32_t Comms::s_last_rx_ms = 0;

ArmJointsCallback    Comms::s_cb_arm        = nullptr;
SensorEnableCallback Comms::s_cb_sensor     = nullptr;
EstopCallback        Comms::s_cb_estop      = nullptr;
PpmCalibCallback     Comms::s_cb_ppm_calib  = nullptr;
ArmLifecycleCallback Comms::s_cb_arm_life   = nullptr;
ArmModeCallback      Comms::s_cb_arm_mode   = nullptr;
TractionCmdCallback  Comms::s_cb_traction   = nullptr;
GripperCallback      Comms::s_cb_gripper    = nullptr;

static HardwareSerial& s_uart = Serial;   // UART0, shared with the USB cable

// Serializes whole-frame TX so writes from the comms task and the control task
// (gripper) can never interleave and corrupt a frame.
static SemaphoreHandle_t s_tx_mutex = nullptr;

void Comms::begin() {
    s_tx_mutex = xSemaphoreCreateMutex();
#ifdef ENABLE_COMMS
    s_uart.begin(MINIPC_BAUD);
#else
    Serial.begin(115200);
    Serial.println("Debug mode");
#endif
}

void Comms::tick() {
    while (s_uart.available()) {
        uint8_t b = s_uart.read();
        switch (s_rx_state) {
            case RxState::SOF0:
                if (b == PROTO_SOF_0) s_rx_state = RxState::SOF1;
                break;
            case RxState::SOF1:
                s_rx_state = (b == PROTO_SOF_1) ? RxState::TYPE : RxState::SOF0;
                break;
            case RxState::TYPE:
                s_rx_type = b; s_rx_crc = b; s_rx_state = RxState::LEN_H;
                break;
            case RxState::LEN_H:
                s_rx_len = (uint16_t)b << 8; s_rx_crc ^= b; s_rx_state = RxState::LEN_L;
                break;
            case RxState::LEN_L:
                s_rx_len |= b; s_rx_crc ^= b; s_rx_idx = 0;
                if (s_rx_len == 0)                    s_rx_state = RxState::CRC;
                else if (s_rx_len > PROTO_MAX_PAYLOAD) s_rx_state = RxState::SOF0;
                else                                  s_rx_state = RxState::PAYLOAD;
                break;
            case RxState::PAYLOAD:
                s_rx_buf[s_rx_idx++] = b; s_rx_crc ^= b;
                if (s_rx_idx >= s_rx_len) s_rx_state = RxState::CRC;
                break;
            case RxState::CRC:
                if (b == s_rx_crc) {
                    s_last_rx_ms = millis();
                    processFrame(s_rx_type, s_rx_buf, s_rx_len);
                }
                s_rx_state = RxState::SOF0;
                break;
        }
    }
}

void Comms::processFrame(uint8_t type, const uint8_t* buf, uint16_t len) {
    switch (type) {
        case MSG_ARM_JOINTS:
            if (len == sizeof(ArmJointsPayload) && s_cb_arm) {
                ArmJointsPayload p; memcpy(&p, buf, sizeof(p)); s_cb_arm(p);
            }
            break;
        case MSG_SENSOR_ENABLE:
            if (len == sizeof(SensorEnablePayload) && s_cb_sensor) s_cb_sensor(buf[0]);
            break;
        case MSG_ESTOP:
            if (s_cb_estop) s_cb_estop(true);
            break;
        case MSG_ESTOP_CLEAR:
            if (s_cb_estop) s_cb_estop(false);
            break;
        // MSG_KEYBIND (0x14) is reserved-unused — the RC uses a fixed control
        // scheme now, so there is no keybind table to receive.
        case MSG_PPM_CALIB:
            if (len == sizeof(PpmCalibPayload) && s_cb_ppm_calib) {
                PpmCalibPayload p; memcpy(&p, buf, sizeof(p)); s_cb_ppm_calib(p);
            }
            break;
        case MSG_ARM_INIT:
            if (s_cb_arm_life) s_cb_arm_life(true);
            break;
        case MSG_ARM_DISARM:
            if (s_cb_arm_life) s_cb_arm_life(false);
            break;
        case MSG_ARM_MODE:
            if (len == 1 && s_cb_arm_mode) s_cb_arm_mode(buf[0]);
            break;
        case MSG_TRACTION_CMD:
            if (len == sizeof(TractionCmdPayload) && s_cb_traction) {
                TractionCmdPayload p; memcpy(&p, buf, sizeof(p)); s_cb_traction(p);
            }
            break;
        case MSG_GRIPPER:
            if (len == sizeof(GripperPayload) && s_cb_gripper) {
                GripperPayload p; memcpy(&p, buf, sizeof(p));
                s_cb_gripper(p.value_1000 / 1000.0f);
            }
            break;
        default: break;
    }
}

uint8_t Comms::computeCRC(uint8_t type, uint16_t len, const uint8_t* payload) {
    uint8_t crc = type ^ (uint8_t)(len >> 8) ^ (uint8_t)(len & 0xFF);
    for (uint16_t i = 0; i < len; i++) crc ^= payload[i];
    return crc;
}

void Comms::sendFrame(uint8_t type, const uint8_t* payload, uint16_t len) {
    uint8_t header[5] = { PROTO_SOF_0, PROTO_SOF_1, type,
                          (uint8_t)(len >> 8), (uint8_t)(len & 0xFF) };
    uint8_t crc = computeCRC(type, len, payload);
    if (s_tx_mutex) xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    s_uart.write(header, 5);
    if (len > 0) s_uart.write(payload, len);
    s_uart.write(&crc, 1);
    if (s_tx_mutex) xSemaphoreGive(s_tx_mutex);
}

// ─── Outgoing builders ─────────────────────────────────────────────────────────
void Comms::sendTelemetry(const TelemetryPayload& p) {
    sendFrame(MSG_TELEMETRY, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void Comms::sendMagData(const MagData& mag) {
    MagPayload p;
    p.x_uT100 = (int16_t)mag.x_uT;
    p.y_uT100 = (int16_t)mag.y_uT;
    p.z_uT100 = (int16_t)mag.z_uT;
    sendFrame(MSG_SENSOR_MAG, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void Comms::sendEncoderExt(float fl, float fr, float rl, float rr) {
    EncoderExtPayload p;
    p.flipper_fl_deg10 = (int16_t)(fl * 10.0f);
    p.flipper_fr_deg10 = (int16_t)(fr * 10.0f);
    p.flipper_rl_deg10 = (int16_t)(rl * 10.0f);
    p.flipper_rr_deg10 = (int16_t)(rr * 10.0f);
    sendFrame(MSG_ENCODER_EXT, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void Comms::sendVescStatus(const VescStatusPayload& v) {
    sendFrame(MSG_VESC_STATUS, reinterpret_cast<const uint8_t*>(&v), sizeof(v));
}
void Comms::sendOdriveStatus(const OdriveStatusPayload& o) {
    sendFrame(MSG_ODRIVE_STATUS, reinterpret_cast<const uint8_t*>(&o), sizeof(o));
}
void Comms::sendLktechStatus(const LktechStatusPayload& l) {
    sendFrame(MSG_LKTECH_STATUS, reinterpret_cast<const uint8_t*>(&l), sizeof(l));
}
void Comms::sendZe300Status(const Ze300StatusPayload& z) {
    sendFrame(MSG_ZE300_STATUS, reinterpret_cast<const uint8_t*>(&z), sizeof(z));
}
void Comms::sendOdriveError(const OdriveErrorPayload& e) {
    sendFrame(MSG_ODRIVE_ERROR, reinterpret_cast<const uint8_t*>(&e), sizeof(e));
}

void Comms::sendStatus(const SystemStatus& s) {
    uint8_t buf[4];
    buf[0] = (uint8_t)s.mode;
    buf[1] = (s.ppm_connected    ? 0x01 : 0)
           | (s.minipc_connected ? 0x02 : 0)
           | (s.can_ok           ? 0x04 : 0)
           | (s.estop            ? 0x08 : 0)
           | (s.virtual_flip     ? 0x10 : 0);
    buf[2] = s.sensor_mask;
    buf[3] = 0;
    sendFrame(MSG_STATUS, buf, sizeof(buf));
}

void Comms::sendArmLifecycle(const ArmLifecyclePayload& p) {
    sendFrame(MSG_ARM_LIFECYCLE, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

void Comms::sendBoardIdentity() {
    BoardIdentityPayload p;
    p.role = (uint8_t)ROBOCOREA_BOARD_ROLE;
    p.protocol_version = (uint8_t)ROBOCOREA_PROTOCOL_VERSION;
    p.capabilities = (uint16_t)ROBOCOREA_BOARD_CAPABILITIES;
    sendFrame(MSG_BOARD_IDENTITY, reinterpret_cast<const uint8_t*>(&p), sizeof(p));
}

bool Comms::isConnected() {
    return (millis() - s_last_rx_ms) < 1000;
}
