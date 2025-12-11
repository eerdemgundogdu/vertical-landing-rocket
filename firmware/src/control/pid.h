#pragma once

// ============================================================================
// PID CONTROLLER
// ============================================================================
// Robust PID controller with anti-windup and derivative filtering
// ============================================================================

#include <Arduino.h>
#include "constants.h"

class PIDController {
public:
    PIDController(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);
    
    // Configure gains
    void setGains(float kp, float ki, float kd);
    void setKp(float kp) { _kp = kp; }
    void setKi(float ki) { _ki = ki; }
    void setKd(float kd) { _kd = kd; }
    
    // Configure limits
    void setOutputLimits(float min, float max);
    void setIntegratorLimits(float min, float max);
    
    // Configure derivative filter (0-1, higher = more filtering)
    void setDerivativeFilter(float alpha) { _derivAlpha = constants::clamp(alpha, 0.0f, 0.99f); }
    
    // Configure setpoint weighting (0-1)
    void setSetpointWeight(float b) { _setpointWeight = constants::clamp(b, 0.0f, 1.0f); }
    
    // Main computation
    float compute(float setpoint, float measurement, float dt);
    
    // Compute with separate derivative input (e.g., from gyro)
    float compute(float setpoint, float measurement, float derivative, float dt);
    
    // Reset controller state
    void reset();
    
    // Get individual terms (for debugging)
    float getP() const { return _lastP; }
    float getI() const { return _lastI; }
    float getD() const { return _lastD; }
    float getError() const { return _lastError; }
    
private:
    // Gains
    float _kp, _ki, _kd;
    
    // Limits
    float _outputMin, _outputMax;
    float _integMin, _integMax;
    
    // Derivative filter coefficient
    float _derivAlpha;
    
    // Setpoint weighting
    float _setpointWeight;
    
    // State
    float _integral;
    float _lastError;
    float _lastMeasurement;
    float _lastDerivative;
    bool _firstRun;
    
    // Debug output
    float _lastP, _lastI, _lastD;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd)
    , _outputMin(-1e10f), _outputMax(1e10f)
    , _integMin(-1e10f), _integMax(1e10f)
    , _derivAlpha(0.2f)
    , _setpointWeight(1.0f)
    , _integral(0.0f)
    , _lastError(0.0f)
    , _lastMeasurement(0.0f)
    , _lastDerivative(0.0f)
    , _firstRun(true)
    , _lastP(0.0f), _lastI(0.0f), _lastD(0.0f)
{
}

inline void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

inline void PIDController::setOutputLimits(float min, float max) {
    _outputMin = min;
    _outputMax = max;
}

inline void PIDController::setIntegratorLimits(float min, float max) {
    _integMin = min;
    _integMax = max;
}

inline float PIDController::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f || dt > 1.0f) {
        return 0.0f;
    }
    
    // Error calculation with setpoint weighting for P term
    float error = setpoint - measurement;
    float errorP = _setpointWeight * setpoint - measurement;
    
    // Proportional term
    _lastP = _kp * errorP;
    
    // Integral term with anti-windup
    _integral += error * dt;
    _integral = constants::clamp(_integral, _integMin, _integMax);
    _lastI = _ki * _integral;
    
    // Derivative term (on measurement to avoid setpoint kick)
    float derivative;
    if (_firstRun) {
        derivative = 0.0f;
        _firstRun = false;
    } else {
        float rawDerivative = -(measurement - _lastMeasurement) / dt;
        // Low-pass filter on derivative
        derivative = _derivAlpha * _lastDerivative + (1.0f - _derivAlpha) * rawDerivative;
    }
    _lastDerivative = derivative;
    _lastD = _kd * derivative;
    
    // Store for next iteration
    _lastError = error;
    _lastMeasurement = measurement;
    
    // Sum and clamp output
    float output = _lastP + _lastI + _lastD;
    output = constants::clamp(output, _outputMin, _outputMax);
    
    // Anti-windup: back-calculate integrator if saturated
    if (output == _outputMin || output == _outputMax) {
        _integral -= error * dt * 0.5f;  // Reduce integrator
    }
    
    return output;
}

inline float PIDController::compute(float setpoint, float measurement, float derivative, float dt) {
    if (dt <= 0.0f || dt > 1.0f) {
        return 0.0f;
    }
    
    float error = setpoint - measurement;
    float errorP = _setpointWeight * setpoint - measurement;
    
    // Proportional
    _lastP = _kp * errorP;
    
    // Integral with anti-windup
    _integral += error * dt;
    _integral = constants::clamp(_integral, _integMin, _integMax);
    _lastI = _ki * _integral;
    
    // Derivative (using provided derivative, e.g., from gyro)
    float filteredDeriv = _derivAlpha * _lastDerivative + (1.0f - _derivAlpha) * (-derivative);
    _lastDerivative = filteredDeriv;
    _lastD = _kd * filteredDeriv;
    
    _lastError = error;
    _lastMeasurement = measurement;
    
    float output = _lastP + _lastI + _lastD;
    output = constants::clamp(output, _outputMin, _outputMax);
    
    if (output == _outputMin || output == _outputMax) {
        _integral -= error * dt * 0.5f;
    }
    
    return output;
}

inline void PIDController::reset() {
    _integral = 0.0f;
    _lastError = 0.0f;
    _lastMeasurement = 0.0f;
    _lastDerivative = 0.0f;
    _firstRun = true;
    _lastP = _lastI = _lastD = 0.0f;
}
