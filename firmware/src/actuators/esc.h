#pragma once

// ============================================================================
// ESC (Electronic Speed Controller) DRIVER
// ============================================================================
// Controls brushless motors via PWM/OneShot/DShot protocols
// Supports motor arming, throttle control, and telemetry
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "pins.h"
#include "constants.h"

// ESC Protocol types
enum class EscProtocol : uint8_t {
    PWM_STANDARD = 0,   // 1000-2000us @ 50Hz
    PWM_FAST = 1,       // 1000-2000us @ 400Hz
    ONESHOT125 = 2,     // 125-250us
    ONESHOT42 = 3,      // 42-84us
    DSHOT150 = 4,       // Digital
    DSHOT300 = 5,
    DSHOT600 = 6
};

// ESC state
enum class EscState : uint8_t {
    DISARMED = 0,
    ARMING = 1,
    ARMED = 2,
    ERROR = 3
};

// Motor telemetry (if supported)
struct MotorTelemetry {
    float rpm;
    float current;       // Amps
    float temperature;   // Celsius
    float voltage;       // Volts
    bool valid;
};

class ESC {
public:
    ESC(uint8_t pin, EscProtocol protocol = EscProtocol::PWM_FAST);
    
    void init();
    void arm();
    void disarm();
    
    // Set throttle (0.0 - 1.0)
    void setThrottle(float throttle);
    
    // Set raw microseconds (for calibration)
    void setMicroseconds(uint16_t us);
    
    // Emergency stop
    void emergencyStop();
    
    // Getters
    EscState getState() const { return _state; }
    float getThrottle() const { return _throttle; }
    bool isArmed() const { return _state == EscState::ARMED; }
    
    // Telemetry (if available)
    const MotorTelemetry& getTelemetry() const { return _telemetry; }
    
    // Calibration
    void calibrate();

private:
    void writePwm(uint16_t us);
    void writeDshot(uint16_t value);
    
    uint8_t _pin;
    EscProtocol _protocol;
    EscState _state;
    
    float _throttle;
    uint16_t _minUs, _maxUs;
    uint32_t _armStartTime;
    
    MotorTelemetry _telemetry;
    
    static constexpr uint16_t PWM_MIN_US = 1000;
    static constexpr uint16_t PWM_MAX_US = 2000;
    static constexpr uint16_t PWM_ARM_US = 1000;
    static constexpr uint32_t ARM_DURATION_MS = 2000;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline ESC::ESC(uint8_t pin, EscProtocol protocol)
    : _pin(pin)
    , _protocol(protocol)
    , _state(EscState::DISARMED)
    , _throttle(0.0f)
    , _minUs(PWM_MIN_US)
    , _maxUs(PWM_MAX_US)
    , _armStartTime(0)
{
    _telemetry = {0, 0, 0, 0, false};
}

inline void ESC::init() {
    pinMode(_pin, OUTPUT);
    
    // Configure PWM frequency based on protocol
    switch (_protocol) {
        case EscProtocol::PWM_STANDARD:
            analogWriteFrequency(_pin, 50);
            break;
        case EscProtocol::PWM_FAST:
            analogWriteFrequency(_pin, 400);
            break;
        case EscProtocol::ONESHOT125:
            analogWriteFrequency(_pin, 4000);
            _minUs = 125;
            _maxUs = 250;
            break;
        default:
            analogWriteFrequency(_pin, 400);
            break;
    }
    
    // Set resolution for Teensy
    analogWriteResolution(16);
    
    // Send minimum throttle
    writePwm(PWM_MIN_US);
    
    _state = EscState::DISARMED;
}

inline void ESC::arm() {
    if (_state == EscState::ARMED) return;
    
    _state = EscState::ARMING;
    _armStartTime = millis();
    
    // Send arming signal (minimum throttle for ARM_DURATION_MS)
    writePwm(PWM_ARM_US);
    
    // Note: Arming completes in update loop after ARM_DURATION_MS
    // For immediate arming, use:
    delay(ARM_DURATION_MS);
    _state = EscState::ARMED;
}

inline void ESC::disarm() {
    _throttle = 0.0f;
    writePwm(PWM_MIN_US);
    _state = EscState::DISARMED;
}

inline void ESC::setThrottle(float throttle) {
    if (_state != EscState::ARMED) {
        _throttle = 0.0f;
        return;
    }
    
    _throttle = constants::clamp(throttle, 0.0f, 1.0f);
    
    uint16_t us = (uint16_t)(_minUs + _throttle * (_maxUs - _minUs));
    writePwm(us);
}

inline void ESC::setMicroseconds(uint16_t us) {
    us = constants::clamp(us, (uint16_t)800, (uint16_t)2200);
    writePwm(us);
}

inline void ESC::emergencyStop() {
    _throttle = 0.0f;
    writePwm(PWM_MIN_US);
    _state = EscState::DISARMED;
}

inline void ESC::calibrate() {
    // ESC calibration sequence
    // 1. Send max throttle
    writePwm(PWM_MAX_US);
    delay(3000);
    
    // 2. Send min throttle
    writePwm(PWM_MIN_US);
    delay(3000);
    
    _state = EscState::DISARMED;
}

inline void ESC::writePwm(uint16_t us) {
    // Convert microseconds to PWM duty cycle
    // For 400Hz: period = 2500us, so duty = us / 2500 * 65535
    float period;
    switch (_protocol) {
        case EscProtocol::PWM_STANDARD:
            period = 20000.0f;  // 50Hz
            break;
        case EscProtocol::PWM_FAST:
            period = 2500.0f;   // 400Hz
            break;
        case EscProtocol::ONESHOT125:
            period = 250.0f;    // 4kHz
            break;
        default:
            period = 2500.0f;
            break;
    }
    
    uint16_t duty = (uint16_t)(us / period * 65535.0f);
    analogWrite(_pin, duty);
}


// ============================================================================
// MULTI-MOTOR CONTROLLER
// ============================================================================
// For rockets with multiple motors (e.g., landing legs with gimbal motors)

class MotorController {
public:
    static constexpr int MAX_MOTORS = 4;
    
    MotorController();
    
    void init();
    void addMotor(uint8_t index, uint8_t pin, EscProtocol protocol = EscProtocol::PWM_FAST);
    
    void armAll();
    void disarmAll();
    void emergencyStopAll();
    
    void setThrottle(uint8_t index, float throttle);
    void setAllThrottles(float throttle);
    
    // Differential thrust for attitude control
    void setDifferentialThrust(float thrust, float rollMix, float pitchMix, float yawMix);
    
    bool allArmed() const;
    
    ESC* getMotor(uint8_t index);

private:
    ESC* _motors[MAX_MOTORS];
    uint8_t _numMotors;
};

inline MotorController::MotorController() : _numMotors(0) {
    for (int i = 0; i < MAX_MOTORS; i++) {
        _motors[i] = nullptr;
    }
}

inline void MotorController::init() {
    for (uint8_t i = 0; i < _numMotors; i++) {
        if (_motors[i]) {
            _motors[i]->init();
        }
    }
}

inline void MotorController::addMotor(uint8_t index, uint8_t pin, EscProtocol protocol) {
    if (index >= MAX_MOTORS) return;
    
    _motors[index] = new ESC(pin, protocol);
    if (index >= _numMotors) {
        _numMotors = index + 1;
    }
}

inline void MotorController::armAll() {
    for (uint8_t i = 0; i < _numMotors; i++) {
        if (_motors[i]) {
            _motors[i]->arm();
        }
    }
}

inline void MotorController::disarmAll() {
    for (uint8_t i = 0; i < _numMotors; i++) {
        if (_motors[i]) {
            _motors[i]->disarm();
        }
    }
}

inline void MotorController::emergencyStopAll() {
    for (uint8_t i = 0; i < _numMotors; i++) {
        if (_motors[i]) {
            _motors[i]->emergencyStop();
        }
    }
}

inline void MotorController::setThrottle(uint8_t index, float throttle) {
    if (index < _numMotors && _motors[index]) {
        _motors[index]->setThrottle(throttle);
    }
}

inline void MotorController::setAllThrottles(float throttle) {
    for (uint8_t i = 0; i < _numMotors; i++) {
        if (_motors[i]) {
            _motors[i]->setThrottle(throttle);
        }
    }
}

inline void MotorController::setDifferentialThrust(float thrust, float rollMix, 
                                                     float pitchMix, float yawMix) {
    // For a quad configuration:
    // Motor 0: Front-Right (+pitch, -roll, -yaw)
    // Motor 1: Back-Right  (-pitch, -roll, +yaw)
    // Motor 2: Back-Left   (-pitch, +roll, -yaw)
    // Motor 3: Front-Left  (+pitch, +roll, +yaw)
    
    if (_numMotors >= 4) {
        float m0 = thrust + pitchMix - rollMix - yawMix;
        float m1 = thrust - pitchMix - rollMix + yawMix;
        float m2 = thrust - pitchMix + rollMix - yawMix;
        float m3 = thrust + pitchMix + rollMix + yawMix;
        
        setThrottle(0, m0);
        setThrottle(1, m1);
        setThrottle(2, m2);
        setThrottle(3, m3);
    } else if (_numMotors == 1) {
        // Single motor - just set thrust
        setThrottle(0, thrust);
    }
}

inline bool MotorController::allArmed() const {
    for (uint8_t i = 0; i < _numMotors; i++) {
        if (_motors[i] && !_motors[i]->isArmed()) {
            return false;
        }
    }
    return true;
}

inline ESC* MotorController::getMotor(uint8_t index) {
    if (index < MAX_MOTORS) {
        return _motors[index];
    }
    return nullptr;
}
