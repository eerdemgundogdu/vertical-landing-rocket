#pragma once

// ============================================================================
// TRAJECTORY PLANNER
// ============================================================================
// Generates smooth trajectories for powered descent landing
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "constants.h"

// Trajectory types
enum class TrajectoryType : uint8_t {
    NONE = 0,
    VERTICAL_DESCENT = 1,
    POLYNOMIAL = 2,
    GRAVITY_TURN = 3,
    POWERED_DESCENT_GUIDANCE = 4
};

// Trajectory waypoint
struct Waypoint {
    float posN, posE, posD;    // Position (NED)
    float velN, velE, velD;    // Velocity (NED)
    float time;                 // Time from start
};

// Trajectory state at any point in time
struct TrajectoryState {
    float posN, posE, posD;
    float velN, velE, velD;
    float accN, accE, accD;
    bool valid;
};

class TrajectoryPlanner {
public:
    TrajectoryPlanner();
    
    void reset();
    
    // Plan a vertical descent trajectory
    void planVerticalDescent(float currentAlt, float targetAlt, 
                             float descentRate, float time);
    
    // Plan polynomial trajectory (5th order)
    void planPolynomial(const Waypoint& start, const Waypoint& end, float duration);
    
    // Plan gravity turn (for ascent)
    void planGravityTurn(float launchAlt, float targetAlt, float pitchoverAlt);
    
    // Get trajectory state at time t
    TrajectoryState evaluate(float t) const;
    
    // Get reference for controller
    void getReference(float t, float& posN, float& posE, float& posD,
                      float& velN, float& velE, float& velD) const;
    
    // Getters
    TrajectoryType getType() const { return _type; }
    float getDuration() const { return _duration; }
    bool isComplete(float t) const { return t >= _duration; }

private:
    TrajectoryType _type;
    float _duration;
    float _startTime;
    
    // Polynomial coefficients (position = a0 + a1*t + a2*t² + ... + a5*t⁵)
    float _coeffN[6], _coeffE[6], _coeffD[6];
    
    // Simple vertical descent params
    float _startAlt, _targetAlt, _descentRate;
    
    void computePolynomialCoeffs(float p0, float v0, float a0,
                                  float pf, float vf, float af,
                                  float T, float* coeffs);
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline TrajectoryPlanner::TrajectoryPlanner()
    : _type(TrajectoryType::NONE)
    , _duration(0.0f)
    , _startTime(0.0f)
    , _startAlt(0.0f)
    , _targetAlt(0.0f)
    , _descentRate(0.0f)
{
    reset();
}

inline void TrajectoryPlanner::reset() {
    _type = TrajectoryType::NONE;
    _duration = 0.0f;
    
    for (int i = 0; i < 6; i++) {
        _coeffN[i] = _coeffE[i] = _coeffD[i] = 0.0f;
    }
}

inline void TrajectoryPlanner::planVerticalDescent(float currentAlt, float targetAlt,
                                                    float descentRate, float time) {
    _type = TrajectoryType::VERTICAL_DESCENT;
    _startAlt = currentAlt;
    _targetAlt = targetAlt;
    _descentRate = descentRate;
    _startTime = time;
    _duration = (currentAlt - targetAlt) / descentRate;
}

inline void TrajectoryPlanner::planPolynomial(const Waypoint& start, 
                                               const Waypoint& end,
                                               float duration) {
    _type = TrajectoryType::POLYNOMIAL;
    _duration = duration;
    _startTime = start.time;
    
    // Compute 5th order polynomial coefficients for each axis
    // Boundary conditions: position, velocity, and acceleration at endpoints
    computePolynomialCoeffs(start.posN, start.velN, 0, 
                            end.posN, end.velN, 0, duration, _coeffN);
    computePolynomialCoeffs(start.posE, start.velE, 0,
                            end.posE, end.velE, 0, duration, _coeffE);
    computePolynomialCoeffs(start.posD, start.velD, 0,
                            end.posD, end.velD, 0, duration, _coeffD);
}

inline void TrajectoryPlanner::planGravityTurn(float launchAlt, float targetAlt,
                                                float pitchoverAlt) {
    _type = TrajectoryType::GRAVITY_TURN;
    _startAlt = launchAlt;
    _targetAlt = targetAlt;
    // Simplified - actual gravity turn is more complex
    _duration = 30.0f;  // Placeholder
}

inline TrajectoryState TrajectoryPlanner::evaluate(float t) const {
    TrajectoryState state = {0, 0, 0, 0, 0, 0, 0, 0, 0, false};
    
    if (_type == TrajectoryType::NONE) {
        return state;
    }
    
    float tau = t - _startTime;
    if (tau < 0) tau = 0;
    if (tau > _duration) tau = _duration;
    
    if (_type == TrajectoryType::VERTICAL_DESCENT) {
        state.posN = 0;
        state.posE = 0;
        state.posD = -(_startAlt - _descentRate * tau);
        state.velN = 0;
        state.velE = 0;
        state.velD = _descentRate;
        state.accN = state.accE = state.accD = 0;
        state.valid = true;
    }
    else if (_type == TrajectoryType::POLYNOMIAL) {
        float t2 = tau * tau;
        float t3 = t2 * tau;
        float t4 = t3 * tau;
        float t5 = t4 * tau;
        
        // Position
        state.posN = _coeffN[0] + _coeffN[1]*tau + _coeffN[2]*t2 + 
                     _coeffN[3]*t3 + _coeffN[4]*t4 + _coeffN[5]*t5;
        state.posE = _coeffE[0] + _coeffE[1]*tau + _coeffE[2]*t2 +
                     _coeffE[3]*t3 + _coeffE[4]*t4 + _coeffE[5]*t5;
        state.posD = _coeffD[0] + _coeffD[1]*tau + _coeffD[2]*t2 +
                     _coeffD[3]*t3 + _coeffD[4]*t4 + _coeffD[5]*t5;
        
        // Velocity
        state.velN = _coeffN[1] + 2*_coeffN[2]*tau + 3*_coeffN[3]*t2 +
                     4*_coeffN[4]*t3 + 5*_coeffN[5]*t4;
        state.velE = _coeffE[1] + 2*_coeffE[2]*tau + 3*_coeffE[3]*t2 +
                     4*_coeffE[4]*t3 + 5*_coeffE[5]*t4;
        state.velD = _coeffD[1] + 2*_coeffD[2]*tau + 3*_coeffD[3]*t2 +
                     4*_coeffD[4]*t3 + 5*_coeffD[5]*t4;
        
        // Acceleration
        state.accN = 2*_coeffN[2] + 6*_coeffN[3]*tau + 12*_coeffN[4]*t2 +
                     20*_coeffN[5]*t3;
        state.accE = 2*_coeffE[2] + 6*_coeffE[3]*tau + 12*_coeffE[4]*t2 +
                     20*_coeffE[5]*t3;
        state.accD = 2*_coeffD[2] + 6*_coeffD[3]*tau + 12*_coeffD[4]*t2 +
                     20*_coeffD[5]*t3;
        
        state.valid = true;
    }
    
    return state;
}

inline void TrajectoryPlanner::getReference(float t, float& posN, float& posE, float& posD,
                                            float& velN, float& velE, float& velD) const {
    TrajectoryState state = evaluate(t);
    posN = state.posN;
    posE = state.posE;
    posD = state.posD;
    velN = state.velN;
    velE = state.velE;
    velD = state.velD;
}

inline void TrajectoryPlanner::computePolynomialCoeffs(float p0, float v0, float a0,
                                                        float pf, float vf, float af,
                                                        float T, float* coeffs) {
    // 5th order polynomial boundary value problem
    // p(0) = p0, p'(0) = v0, p''(0) = a0
    // p(T) = pf, p'(T) = vf, p''(T) = af
    
    coeffs[0] = p0;
    coeffs[1] = v0;
    coeffs[2] = a0 / 2.0f;
    
    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;
    
    // Solve for a3, a4, a5
    float dp = pf - p0 - v0*T - 0.5f*a0*T2;
    float dv = vf - v0 - a0*T;
    float da = af - a0;
    
    // Matrix solution (simplified)
    coeffs[3] = (20*dp - (8*dv + da*T)*T) / (2*T3);
    coeffs[4] = (-30*dp + (14*dv + 2*da*T)*T) / (2*T4);
    coeffs[5] = (12*dp - 6*dv*T) / (2*T5);
}
