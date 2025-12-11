#pragma once

// ============================================================================
// WATCHDOG AND SAFETY SYSTEMS
// ============================================================================
// Hardware and software watchdog, fault detection, and safety monitors
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "pins.h"

// Fault types
enum class FaultType : uint16_t {
    NONE = 0,
    IMU_FAIL = 1 << 0,
    BARO_FAIL = 1 << 1,
    GPS_FAIL = 1 << 2,
    SERVO_FAIL = 1 << 3,
    ESC_FAIL = 1 << 4,
    BATTERY_LOW = 1 << 5,
    BATTERY_CRITICAL = 1 << 6,
    OVERHEAT = 1 << 7,
    COMM_LOST = 1 << 8,
    SENSOR_TIMEOUT = 1 << 9,
    ALTITUDE_LIMIT = 1 << 10,
    GEOFENCE = 1 << 11,
    TILT_LIMIT = 1 << 12,
    LOOP_OVERRUN = 1 << 13,
    SD_FAIL = 1 << 14
};

// Fault action
enum class FaultAction : uint8_t {
    NONE = 0,
    WARN = 1,           // Log warning
    ABORT = 2,          // Trigger abort sequence
    SHUTDOWN = 3        // Immediate shutdown
};

struct FaultConfig {
    FaultType type;
    FaultAction action;
    uint32_t debounceMs;
    bool requiresClear;
};

class Watchdog {
public:
    Watchdog();
    
    void init();
    void feed();                    // Hardware watchdog
    void update();                  // Software checks
    
    // Fault management
    void setFault(FaultType fault);
    void clearFault(FaultType fault);
    bool hasFault(FaultType fault) const;
    uint16_t getFaults() const { return _faults; }
    
    // Sensor timeouts
    void updateImu() { _lastImuTime = millis(); }
    void updateBaro() { _lastBaroTime = millis(); }
    void updateGps() { _lastGpsTime = millis(); }
    void updateComm() { _lastCommTime = millis(); }
    
    // Safety monitors
    void setBatteryVoltage(float voltage);
    void setCpuTemperature(float temp);
    void setAltitude(float altitude);
    void setTiltAngle(float angle);
    void setPosition(float n, float e);
    
    // Loop timing monitor
    void loopStart();
    void loopEnd();
    float getLoopTime() const { return _avgLoopTime; }
    bool isLoopOverrun() const { return _loopOverruns > 10; }
    
    // Callbacks
    void setFaultCallback(void (*cb)(FaultType, FaultAction)) { _faultCallback = cb; }
    
    // Status
    bool isHealthy() const { return _faults == 0; }

private:
    void checkTimeouts();
    void checkSafetyLimits();
    void triggerAction(FaultType fault, FaultAction action);
    
    uint16_t _faults;
    
    // Sensor timestamps
    uint32_t _lastImuTime;
    uint32_t _lastBaroTime;
    uint32_t _lastGpsTime;
    uint32_t _lastCommTime;
    
    // Safety values
    float _batteryVoltage;
    float _cpuTemp;
    float _altitude;
    float _tiltAngle;
    float _posN, _posE;
    
    // Loop timing
    uint32_t _loopStartTime;
    float _avgLoopTime;
    uint16_t _loopOverruns;
    
    // Fault callback
    void (*_faultCallback)(FaultType, FaultAction);
    
    // Timeout thresholds (ms)
    static constexpr uint32_t IMU_TIMEOUT = 100;
    static constexpr uint32_t BARO_TIMEOUT = 500;
    static constexpr uint32_t GPS_TIMEOUT = 5000;
    static constexpr uint32_t COMM_TIMEOUT = 5000;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline Watchdog::Watchdog()
    : _faults(0)
    , _lastImuTime(0)
    , _lastBaroTime(0)
    , _lastGpsTime(0)
    , _lastCommTime(0)
    , _batteryVoltage(8.4f)
    , _cpuTemp(25.0f)
    , _altitude(0.0f)
    , _tiltAngle(0.0f)
    , _posN(0.0f), _posE(0.0f)
    , _loopStartTime(0)
    , _avgLoopTime(0.001f)
    , _loopOverruns(0)
    , _faultCallback(nullptr)
{
}

inline void Watchdog::init() {
    // Configure hardware watchdog on Teensy 4.1
    // Note: Teensy 4.x has a built-in watchdog
    // WDT_timings_t config;
    // config.trigger = 1.0f;  // 1 second
    // config.timeout = 2.0f;  // 2 second reset
    // wdt.begin(config);
    
    _lastImuTime = _lastBaroTime = _lastGpsTime = _lastCommTime = millis();
}

inline void Watchdog::feed() {
    // Feed hardware watchdog
    // wdt.feed();
}

inline void Watchdog::update() {
    checkTimeouts();
    checkSafetyLimits();
}

inline void Watchdog::setFault(FaultType fault) {
    uint16_t bit = static_cast<uint16_t>(fault);
    if (!(_faults & bit)) {
        _faults |= bit;
        
        // Determine action
        FaultAction action = FaultAction::WARN;
        
        if (fault == FaultType::BATTERY_CRITICAL ||
            fault == FaultType::TILT_LIMIT ||
            fault == FaultType::ALTITUDE_LIMIT ||
            fault == FaultType::GEOFENCE) {
            action = FaultAction::ABORT;
        }
        
        if (fault == FaultType::IMU_FAIL) {
            action = FaultAction::ABORT;
        }
        
        triggerAction(fault, action);
    }
}

inline void Watchdog::clearFault(FaultType fault) {
    _faults &= ~static_cast<uint16_t>(fault);
}

inline bool Watchdog::hasFault(FaultType fault) const {
    return _faults & static_cast<uint16_t>(fault);
}

inline void Watchdog::setBatteryVoltage(float voltage) {
    _batteryVoltage = voltage;
}

inline void Watchdog::setCpuTemperature(float temp) {
    _cpuTemp = temp;
}

inline void Watchdog::setAltitude(float altitude) {
    _altitude = altitude;
}

inline void Watchdog::setTiltAngle(float angle) {
    _tiltAngle = angle;
}

inline void Watchdog::setPosition(float n, float e) {
    _posN = n;
    _posE = e;
}

inline void Watchdog::loopStart() {
    _loopStartTime = micros();
}

inline void Watchdog::loopEnd() {
    uint32_t elapsed = micros() - _loopStartTime;
    float loopTime = elapsed * 1e-6f;
    
    // Exponential moving average
    _avgLoopTime = 0.95f * _avgLoopTime + 0.05f * loopTime;
    
    // Check for overrun (> 2ms for 1kHz loop)
    if (loopTime > 0.002f) {
        _loopOverruns++;
        if (_loopOverruns > 100) {
            setFault(FaultType::LOOP_OVERRUN);
        }
    } else {
        if (_loopOverruns > 0) _loopOverruns--;
    }
}

inline void Watchdog::checkTimeouts() {
    uint32_t now = millis();
    
    if (now - _lastImuTime > IMU_TIMEOUT) {
        setFault(FaultType::IMU_FAIL);
    } else {
        clearFault(FaultType::IMU_FAIL);
    }
    
    if (now - _lastBaroTime > BARO_TIMEOUT) {
        setFault(FaultType::BARO_FAIL);
    } else {
        clearFault(FaultType::BARO_FAIL);
    }
    
    if (now - _lastGpsTime > GPS_TIMEOUT) {
        setFault(FaultType::GPS_FAIL);
    } else {
        clearFault(FaultType::GPS_FAIL);
    }
    
    if (now - _lastCommTime > COMM_TIMEOUT) {
        setFault(FaultType::COMM_LOST);
    } else {
        clearFault(FaultType::COMM_LOST);
    }
}

inline void Watchdog::checkSafetyLimits() {
    // Battery
    if (_batteryVoltage < BATTERY_CRITICAL_VOLTAGE) {
        setFault(FaultType::BATTERY_CRITICAL);
    } else if (_batteryVoltage < BATTERY_LOW_VOLTAGE) {
        setFault(FaultType::BATTERY_LOW);
    } else {
        clearFault(FaultType::BATTERY_LOW);
        clearFault(FaultType::BATTERY_CRITICAL);
    }
    
    // Temperature
    if (_cpuTemp > 85.0f) {
        setFault(FaultType::OVERHEAT);
    } else {
        clearFault(FaultType::OVERHEAT);
    }
    
    // Altitude limit
    if (_altitude > MAX_ALTITUDE_M) {
        setFault(FaultType::ALTITUDE_LIMIT);
    }
    
    // Tilt limit
    if (_tiltAngle > MAX_TILT_ANGLE_DEG * constants::DEG_TO_RAD) {
        setFault(FaultType::TILT_LIMIT);
    } else {
        clearFault(FaultType::TILT_LIMIT);
    }
    
    // Geofence
    float distFromHome = sqrtf(_posN * _posN + _posE * _posE);
    if (distFromHome > GEOFENCE_RADIUS_M) {
        setFault(FaultType::GEOFENCE);
    }
}

inline void Watchdog::triggerAction(FaultType fault, FaultAction action) {
    if (_faultCallback) {
        _faultCallback(fault, action);
    }
    
    // Built-in actions
    if (action == FaultAction::SHUTDOWN) {
        // Emergency actions
        digitalWrite(PIN_IGNITER, LOW);  // Ensure igniter is off
    }
}


// ============================================================================
// HEALTH MONITOR
// ============================================================================
// Tracks system health metrics over time

class HealthMonitor {
public:
    void update(float batteryV, float cpuTemp, float loopTime, uint16_t faults);
    
    float getAvgBattery() const { return _avgBattery; }
    float getMinBattery() const { return _minBattery; }
    float getMaxTemp() const { return _maxTemp; }
    float getMaxLoopTime() const { return _maxLoopTime; }
    uint32_t getFaultCount() const { return _faultCount; }
    float getUptime() const { return (millis() - _startTime) / 1000.0f; }

private:
    float _avgBattery = 8.4f;
    float _minBattery = 8.4f;
    float _maxTemp = 25.0f;
    float _maxLoopTime = 0.001f;
    uint32_t _faultCount = 0;
    uint16_t _lastFaults = 0;
    uint32_t _startTime = 0;
    bool _initialized = false;
};

inline void HealthMonitor::update(float batteryV, float cpuTemp, float loopTime, uint16_t faults) {
    if (!_initialized) {
        _startTime = millis();
        _initialized = true;
    }
    
    // Battery
    _avgBattery = 0.99f * _avgBattery + 0.01f * batteryV;
    if (batteryV < _minBattery) _minBattery = batteryV;
    
    // Temperature
    if (cpuTemp > _maxTemp) _maxTemp = cpuTemp;
    
    // Loop time
    if (loopTime > _maxLoopTime) _maxLoopTime = loopTime;
    
    // Fault count
    uint16_t newFaults = faults & ~_lastFaults;
    for (int i = 0; i < 16; i++) {
        if (newFaults & (1 << i)) _faultCount++;
    }
    _lastFaults = faults;
}
