#pragma once

// ============================================================================
// SD CARD DATA LOGGER
// ============================================================================
// High-speed binary logging to SD card
// ============================================================================

#include <Arduino.h>
#include <SdFat.h>
#include "config.h"

// Log entry types
enum class LogType : uint8_t {
    IMU = 0x01,
    BARO = 0x02,
    GPS = 0x03,
    STATE = 0x04,
    EVENT = 0x05,
    DEBUG = 0x06
};

#pragma pack(push, 1)
struct LogHeader {
    uint8_t type;
    uint32_t timestamp;
};

struct LogImu {
    LogHeader header;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
};

struct LogBaro {
    LogHeader header;
    float pressure;
    float altitude;
    float temperature;
};

struct LogState {
    LogHeader header;
    float posN, posE, posD;
    float velN, velE, velD;
    float roll, pitch, yaw;
    uint8_t flightState;
};

struct LogEvent {
    LogHeader header;
    uint8_t eventType;
    uint8_t data[8];
};
#pragma pack(pop)

class DataLogger {
public:
    DataLogger();
    
    bool begin();
    void createNewFlight();
    void close();
    
    void logImu(float ax, float ay, float az, float gx, float gy, float gz);
    void logBaro(float pressure, float altitude, float temperature);
    void logState(float pn, float pe, float pd, float vn, float ve, float vd,
                  float roll, float pitch, float yaw, uint8_t state);
    void logEvent(uint8_t eventType, const uint8_t* data = nullptr, uint8_t len = 0);
    void logDebug(const char* message);
    
    void flush();
    
    bool isReady() const { return _ready; }
    uint32_t getBytesWritten() const { return _bytesWritten; }
    uint16_t getFlightNumber() const { return _flightNumber; }

private:
    bool findNextFlightNumber();
    void writeLogEntry(const void* data, size_t len);
    
    SdFat _sd;
    File _file;
    
    bool _ready;
    uint16_t _flightNumber;
    uint32_t _bytesWritten;
    uint32_t _lastFlush;
    
    uint8_t _buffer[512];
    size_t _bufferIndex;
    
    static constexpr uint32_t FLUSH_INTERVAL_MS = 1000;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline DataLogger::DataLogger()
    : _ready(false)
    , _flightNumber(0)
    , _bytesWritten(0)
    , _lastFlush(0)
    , _bufferIndex(0)
{
}

inline bool DataLogger::begin() {
    if (!_sd.begin(SdioConfig(FIFO_SDIO))) {
        return false;
    }
    
    // Find next flight number
    if (!findNextFlightNumber()) {
        return false;
    }
    
    _ready = true;
    return true;
}

inline void DataLogger::createNewFlight() {
    if (!_ready) return;
    
    // Close any existing file
    if (_file.isOpen()) {
        flush();
        _file.close();
    }
    
    // Create new file
    char filename[32];
    snprintf(filename, sizeof(filename), "flight_%04d.bin", _flightNumber);
    
    if (_file.open(filename, O_RDWR | O_CREAT | O_TRUNC)) {
        _bytesWritten = 0;
        _bufferIndex = 0;
        _flightNumber++;
    }
}

inline void DataLogger::close() {
    if (_file.isOpen()) {
        flush();
        _file.close();
    }
}

inline void DataLogger::logImu(float ax, float ay, float az, 
                                float gx, float gy, float gz) {
    LogImu entry;
    entry.header.type = (uint8_t)LogType::IMU;
    entry.header.timestamp = micros();
    entry.accelX = ax;
    entry.accelY = ay;
    entry.accelZ = az;
    entry.gyroX = gx;
    entry.gyroY = gy;
    entry.gyroZ = gz;
    writeLogEntry(&entry, sizeof(entry));
}

inline void DataLogger::logBaro(float pressure, float altitude, float temperature) {
    LogBaro entry;
    entry.header.type = (uint8_t)LogType::BARO;
    entry.header.timestamp = micros();
    entry.pressure = pressure;
    entry.altitude = altitude;
    entry.temperature = temperature;
    writeLogEntry(&entry, sizeof(entry));
}

inline void DataLogger::logState(float pn, float pe, float pd,
                                  float vn, float ve, float vd,
                                  float roll, float pitch, float yaw,
                                  uint8_t state) {
    LogState entry;
    entry.header.type = (uint8_t)LogType::STATE;
    entry.header.timestamp = micros();
    entry.posN = pn;
    entry.posE = pe;
    entry.posD = pd;
    entry.velN = vn;
    entry.velE = ve;
    entry.velD = vd;
    entry.roll = roll;
    entry.pitch = pitch;
    entry.yaw = yaw;
    entry.flightState = state;
    writeLogEntry(&entry, sizeof(entry));
}

inline void DataLogger::logEvent(uint8_t eventType, const uint8_t* data, uint8_t len) {
    LogEvent entry;
    entry.header.type = (uint8_t)LogType::EVENT;
    entry.header.timestamp = micros();
    entry.eventType = eventType;
    memset(entry.data, 0, sizeof(entry.data));
    if (data && len > 0) {
        memcpy(entry.data, data, min((size_t)len, sizeof(entry.data)));
    }
    writeLogEntry(&entry, sizeof(entry));
}

inline void DataLogger::logDebug(const char* message) {
    size_t len = strlen(message);
    if (len > 200) len = 200;
    
    uint8_t entry[206];
    entry[0] = (uint8_t)LogType::DEBUG;
    uint32_t ts = micros();
    memcpy(&entry[1], &ts, 4);
    entry[5] = (uint8_t)len;
    memcpy(&entry[6], message, len);
    
    writeLogEntry(entry, 6 + len);
}

inline void DataLogger::flush() {
    if (!_file.isOpen() || _bufferIndex == 0) return;
    
    _file.write(_buffer, _bufferIndex);
    _file.sync();
    _bufferIndex = 0;
    _lastFlush = millis();
}

inline bool DataLogger::findNextFlightNumber() {
    _flightNumber = 1;
    
    char filename[32];
    while (_flightNumber < 9999) {
        snprintf(filename, sizeof(filename), "flight_%04d.bin", _flightNumber);
        if (!_sd.exists(filename)) {
            return true;
        }
        _flightNumber++;
    }
    return false;
}

inline void DataLogger::writeLogEntry(const void* data, size_t len) {
    if (!_file.isOpen()) return;
    
    // Check if buffer has space
    if (_bufferIndex + len > sizeof(_buffer)) {
        flush();
    }
    
    memcpy(&_buffer[_bufferIndex], data, len);
    _bufferIndex += len;
    _bytesWritten += len;
    
    // Periodic flush
    if (millis() - _lastFlush > FLUSH_INTERVAL_MS) {
        flush();
    }
}
