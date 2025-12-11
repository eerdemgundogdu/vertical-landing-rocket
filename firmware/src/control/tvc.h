#pragma once

// ============================================================================
// THRUST VECTOR CONTROL (TVC)
// ============================================================================
// Servo control for gimbaled motor mount
// ============================================================================

#include <Arduino.h>
#include <PWMServo.h>
#include "config.h"
#include "pins.h"
#include "constants.h"

class TVC {
public:
    TVC();
    
    void init();
    void update(float cmdX, float cmdY);  // Commands in degrees
    void center();
    void detach();
    
    void setTrim(float trimX, float trimY);
    void setLimits(float limitX, float limitY);
    void setReverse(bool reverseX, bool reverseY);
    
    float getServoXAngle() const { return _currentX; }
    float getServoYAngle() const { return _currentY; }
    
    bool isAttached() const { return _attached; }

private:
    float angleToMicroseconds(float angle, float center, float limit, bool reverse);
    
    PWMServo _servoX;
    PWMServo _servoY;
    
    float _trimX, _trimY;
    float _limitX, _limitY;
    bool _reverseX, _reverseY;
    
    float _currentX, _currentY;
    bool _attached;
    
    static constexpr int SERVO_MIN_US = 1000;
    static constexpr int SERVO_MAX_US = 2000;
    static constexpr int SERVO_CENTER_US = 1500;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline TVC::TVC()
    : _trimX(TVC_SERVO_X_TRIM)
    , _trimY(TVC_SERVO_Y_TRIM)
    , _limitX(TVC_SERVO_X_LIMIT)
    , _limitY(TVC_SERVO_Y_LIMIT)
    , _reverseX(TVC_SERVO_X_REVERSE)
    , _reverseY(TVC_SERVO_Y_REVERSE)
    , _currentX(0.0f)
    , _currentY(0.0f)
    , _attached(false)
{
}

inline void TVC::init() {
    _servoX.attach(PIN_SERVO_X, SERVO_MIN_US, SERVO_MAX_US);
    _servoY.attach(PIN_SERVO_Y, SERVO_MIN_US, SERVO_MAX_US);
    _attached = true;
    
    center();
    delay(500);  // Allow servos to move to center
}

inline void TVC::update(float cmdX, float cmdY) {
    if (!_attached) return;
    
    // Apply limits
    cmdX = constants::clamp(cmdX, -_limitX, _limitX);
    cmdY = constants::clamp(cmdY, -_limitY, _limitY);
    
    // Apply trim
    cmdX += _trimX;
    cmdY += _trimY;
    
    // Convert to microseconds
    float usX = angleToMicroseconds(cmdX, TVC_SERVO_X_CENTER, _limitX, _reverseX);
    float usY = angleToMicroseconds(cmdY, TVC_SERVO_Y_CENTER, _limitY, _reverseY);
    
    // Write to servos
    _servoX.writeMicroseconds((int)usX);
    _servoY.writeMicroseconds((int)usY);
    
    _currentX = cmdX - _trimX;
    _currentY = cmdY - _trimY;
}

inline void TVC::center() {
    if (!_attached) return;
    
    _servoX.writeMicroseconds(SERVO_CENTER_US + (int)(_trimX * 10));
    _servoY.writeMicroseconds(SERVO_CENTER_US + (int)(_trimY * 10));
    
    _currentX = 0.0f;
    _currentY = 0.0f;
}

inline void TVC::detach() {
    center();
    delay(100);
    _servoX.detach();
    _servoY.detach();
    _attached = false;
}

inline void TVC::setTrim(float trimX, float trimY) {
    _trimX = trimX;
    _trimY = trimY;
}

inline void TVC::setLimits(float limitX, float limitY) {
    _limitX = limitX;
    _limitY = limitY;
}

inline void TVC::setReverse(bool reverseX, bool reverseY) {
    _reverseX = reverseX;
    _reverseY = reverseY;
}

inline float TVC::angleToMicroseconds(float angle, float center, float limit, bool reverse) {
    if (reverse) angle = -angle;
    
    // Map angle (-limit to +limit) to microseconds
    // Assuming 10 microseconds per degree of servo movement
    float us = SERVO_CENTER_US + (angle * 500.0f / 45.0f);  // ±45° = ±500us
    
    return constants::clamp(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
}
