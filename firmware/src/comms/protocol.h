#pragma once

// ============================================================================
// SERIAL COMMUNICATION PROTOCOL
// ============================================================================
// COBS-encoded binary protocol with CRC16 for reliable communication
// ============================================================================

#include <Arduino.h>
#include "config.h"

// Message IDs
namespace MsgId {
    // Telemetry (Teensy → Host)
    constexpr uint8_t TELEMETRY_STATE = 0x01;
    constexpr uint8_t TELEMETRY_RAW = 0x02;
    constexpr uint8_t TELEMETRY_STATUS = 0x03;
    constexpr uint8_t TELEMETRY_GPS = 0x04;
    constexpr uint8_t TELEMETRY_LOG = 0x05;
    
    // Commands (Host → Teensy)
    constexpr uint8_t CMD_ARM = 0x10;
    constexpr uint8_t CMD_DISARM = 0x11;
    constexpr uint8_t CMD_ABORT = 0x12;
    constexpr uint8_t CMD_CALIBRATE = 0x13;
    constexpr uint8_t CMD_SET_TARGET = 0x14;
    constexpr uint8_t CMD_SET_PARAM = 0x15;
    constexpr uint8_t CMD_GET_PARAM = 0x16;
    
    // Responses
    constexpr uint8_t RESP_ACK = 0x20;
    constexpr uint8_t RESP_NACK = 0x21;
    constexpr uint8_t RESP_PARAM = 0x22;
}

// Message structures (packed for transmission)
#pragma pack(push, 1)

struct TelemetryState {
    uint32_t timestampMs;
    float posN, posE, posD;
    float velN, velE, velD;
    float qw, qx, qy, qz;
    float gyroX, gyroY, gyroZ;
};

struct TelemetryRaw {
    uint32_t timestampMs;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float pressure;
    float temperature;
};

struct TelemetryStatus {
    uint32_t timestampMs;
    uint8_t flightState;
    uint8_t armStatus;
    uint8_t sensorStatus;
    uint8_t errorCode;
    float batteryVoltage;
    float cpuTemp;
};

struct TelemetryGps {
    uint32_t timestampMs;
    double latitude;
    double longitude;
    float altitudeMsl;
    float velN, velE, velD;
    uint8_t fixType;
    uint8_t satellites;
    uint16_t hdop;
};

struct CmdArm {
    uint32_t armKey;
};

struct CmdSetTarget {
    float roll;
    float pitch;
    float yaw;
    float thrust;
};

struct CmdSetParam {
    uint16_t paramId;
    uint16_t reserved;
    float value;
};

#pragma pack(pop)

class SerialProtocol {
public:
    SerialProtocol(Stream& serial);
    
    void init();
    void update();
    
    // Send telemetry
    void sendState(const TelemetryState& state);
    void sendRaw(const TelemetryRaw& raw);
    void sendStatus(const TelemetryStatus& status);
    void sendGps(const TelemetryGps& gps);
    void sendLog(const char* message);
    
    // Send response
    void sendAck(uint8_t cmdId, uint8_t status = 0);
    void sendNack(uint8_t cmdId, uint8_t errorCode);
    
    // Command callbacks
    void setArmCallback(void (*cb)(uint32_t key)) { _armCallback = cb; }
    void setDisarmCallback(void (*cb)()) { _disarmCallback = cb; }
    void setAbortCallback(void (*cb)(uint8_t reason)) { _abortCallback = cb; }
    void setCalibrateCallback(void (*cb)()) { _calibrateCallback = cb; }
    void setTargetCallback(void (*cb)(float r, float p, float y, float t)) { _targetCallback = cb; }
    void setParamCallback(void (*cb)(uint16_t id, float val)) { _paramCallback = cb; }
    
private:
    void sendPacket(uint8_t msgId, const void* data, size_t len);
    void processPacket(const uint8_t* data, size_t len);
    
    size_t cobsEncode(const uint8_t* input, size_t len, uint8_t* output);
    size_t cobsDecode(const uint8_t* input, size_t len, uint8_t* output);
    uint16_t crc16(const uint8_t* data, size_t len);
    
    Stream& _serial;
    
    uint8_t _rxBuffer[PROTOCOL_MAX_PACKET_SIZE];
    size_t _rxIndex;
    
    uint8_t _txBuffer[PROTOCOL_MAX_PACKET_SIZE];
    uint8_t _cobsBuffer[PROTOCOL_MAX_PACKET_SIZE + 10];
    
    void (*_armCallback)(uint32_t key);
    void (*_disarmCallback)();
    void (*_abortCallback)(uint8_t reason);
    void (*_calibrateCallback)();
    void (*_targetCallback)(float r, float p, float y, float t);
    void (*_paramCallback)(uint16_t id, float val);
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline SerialProtocol::SerialProtocol(Stream& serial)
    : _serial(serial)
    , _rxIndex(0)
    , _armCallback(nullptr)
    , _disarmCallback(nullptr)
    , _abortCallback(nullptr)
    , _calibrateCallback(nullptr)
    , _targetCallback(nullptr)
    , _paramCallback(nullptr)
{
}

inline void SerialProtocol::init() {
    _rxIndex = 0;
}

inline void SerialProtocol::update() {
    while (_serial.available()) {
        uint8_t byte = _serial.read();
        
        if (byte == 0x00) {
            // End of COBS packet
            if (_rxIndex > 0) {
                uint8_t decoded[PROTOCOL_MAX_PACKET_SIZE];
                size_t decodedLen = cobsDecode(_rxBuffer, _rxIndex, decoded);
                if (decodedLen >= 4) {  // Min: sync + msgId + crc16
                    processPacket(decoded, decodedLen);
                }
            }
            _rxIndex = 0;
        } else if (_rxIndex < PROTOCOL_MAX_PACKET_SIZE) {
            _rxBuffer[_rxIndex++] = byte;
        } else {
            _rxIndex = 0;  // Buffer overflow, reset
        }
    }
}

inline void SerialProtocol::sendState(const TelemetryState& state) {
    sendPacket(MsgId::TELEMETRY_STATE, &state, sizeof(state));
}

inline void SerialProtocol::sendRaw(const TelemetryRaw& raw) {
    sendPacket(MsgId::TELEMETRY_RAW, &raw, sizeof(raw));
}

inline void SerialProtocol::sendStatus(const TelemetryStatus& status) {
    sendPacket(MsgId::TELEMETRY_STATUS, &status, sizeof(status));
}

inline void SerialProtocol::sendGps(const TelemetryGps& gps) {
    sendPacket(MsgId::TELEMETRY_GPS, &gps, sizeof(gps));
}

inline void SerialProtocol::sendLog(const char* message) {
    size_t len = strlen(message);
    if (len > 200) len = 200;
    sendPacket(MsgId::TELEMETRY_LOG, message, len);
}

inline void SerialProtocol::sendAck(uint8_t cmdId, uint8_t status) {
    uint8_t data[2] = { cmdId, status };
    sendPacket(MsgId::RESP_ACK, data, 2);
}

inline void SerialProtocol::sendNack(uint8_t cmdId, uint8_t errorCode) {
    uint8_t data[2] = { cmdId, errorCode };
    sendPacket(MsgId::RESP_NACK, data, 2);
}

inline void SerialProtocol::sendPacket(uint8_t msgId, const void* data, size_t len) {
    size_t idx = 0;
    _txBuffer[idx++] = PROTOCOL_SYNC_BYTE;
    _txBuffer[idx++] = msgId;
    
    if (data && len > 0) {
        memcpy(&_txBuffer[idx], data, len);
        idx += len;
    }
    
    // Calculate CRC over sync + msgId + payload
    uint16_t crc = crc16(_txBuffer, idx);
    _txBuffer[idx++] = crc & 0xFF;
    _txBuffer[idx++] = (crc >> 8) & 0xFF;
    
    // COBS encode
    size_t cobsLen = cobsEncode(_txBuffer, idx, _cobsBuffer);
    _cobsBuffer[cobsLen++] = 0x00;  // Delimiter
    
    _serial.write(_cobsBuffer, cobsLen);
}

inline void SerialProtocol::processPacket(const uint8_t* data, size_t len) {
    if (len < 4) return;
    if (data[0] != PROTOCOL_SYNC_BYTE) return;
    
    // Verify CRC
    uint16_t rxCrc = data[len-2] | (data[len-1] << 8);
    uint16_t calcCrc = crc16(data, len - 2);
    if (rxCrc != calcCrc) return;
    
    uint8_t msgId = data[1];
    const uint8_t* payload = &data[2];
    size_t payloadLen = len - 4;
    
    switch (msgId) {
        case MsgId::CMD_ARM:
            if (payloadLen >= sizeof(CmdArm) && _armCallback) {
                CmdArm cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                _armCallback(cmd.armKey);
            }
            break;
            
        case MsgId::CMD_DISARM:
            if (_disarmCallback) _disarmCallback();
            break;
            
        case MsgId::CMD_ABORT:
            if (payloadLen >= 1 && _abortCallback) {
                _abortCallback(payload[0]);
            }
            break;
            
        case MsgId::CMD_CALIBRATE:
            if (_calibrateCallback) _calibrateCallback();
            break;
            
        case MsgId::CMD_SET_TARGET:
            if (payloadLen >= sizeof(CmdSetTarget) && _targetCallback) {
                CmdSetTarget cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                _targetCallback(cmd.roll, cmd.pitch, cmd.yaw, cmd.thrust);
            }
            break;
            
        case MsgId::CMD_SET_PARAM:
            if (payloadLen >= sizeof(CmdSetParam) && _paramCallback) {
                CmdSetParam cmd;
                memcpy(&cmd, payload, sizeof(cmd));
                _paramCallback(cmd.paramId, cmd.value);
            }
            break;
    }
}

inline size_t SerialProtocol::cobsEncode(const uint8_t* input, size_t len, uint8_t* output) {
    size_t readIdx = 0;
    size_t writeIdx = 1;
    size_t codeIdx = 0;
    uint8_t code = 1;
    
    while (readIdx < len) {
        if (input[readIdx] == 0) {
            output[codeIdx] = code;
            code = 1;
            codeIdx = writeIdx++;
        } else {
            output[writeIdx++] = input[readIdx];
            code++;
            if (code == 0xFF) {
                output[codeIdx] = code;
                code = 1;
                codeIdx = writeIdx++;
            }
        }
        readIdx++;
    }
    output[codeIdx] = code;
    return writeIdx;
}

inline size_t SerialProtocol::cobsDecode(const uint8_t* input, size_t len, uint8_t* output) {
    size_t readIdx = 0;
    size_t writeIdx = 0;
    
    while (readIdx < len) {
        uint8_t code = input[readIdx++];
        for (uint8_t i = 1; i < code && readIdx < len; i++) {
            output[writeIdx++] = input[readIdx++];
        }
        if (code < 0xFF && readIdx < len) {
            output[writeIdx++] = 0;
        }
    }
    
    // Remove trailing zero if present
    if (writeIdx > 0 && output[writeIdx-1] == 0) {
        writeIdx--;
    }
    
    return writeIdx;
}

inline uint16_t SerialProtocol::crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
