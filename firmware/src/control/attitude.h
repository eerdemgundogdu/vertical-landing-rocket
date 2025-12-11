#pragma once

// ============================================================================
// ATTITUDE CONTROLLER
// ============================================================================
// Cascaded PID controller for attitude stabilization
// Outer loop: attitude error → angular rate setpoint
// Inner loop: angular rate error → TVC command
// ============================================================================

#include <Arduino.h>
#include "pid.h"
#include "config.h"
#include "../estimation/quaternion.h"

struct AttitudeCommand {
    float rollCmd;   // Commanded roll angle (rad)
    float pitchCmd;  // Commanded pitch angle (rad)
    float yawCmd;    // Commanded yaw angle (rad)
};

struct TvcCommand {
    float servoX;    // X-axis gimbal angle (deg)
    float servoY;    // Y-axis gimbal angle (deg)
};

class AttitudeController {
public:
    AttitudeController();
    
    void init();
    void reset();
    
    // Set target attitude (radians)
    void setTarget(float roll, float pitch, float yaw);
    void setTargetQuaternion(const Quaternion& target);
    
    // Main control update
    TvcCommand update(float roll, float pitch, float yaw,
                      float gyroX, float gyroY, float gyroZ,
                      float dt);
    
    // Get rate setpoints for debugging
    float getRollRateSetpoint() const { return _rollRateSp; }
    float getPitchRateSetpoint() const { return _pitchRateSp; }
    float getYawRateSetpoint() const { return _yawRateSp; }
    
    // Access individual PIDs for tuning
    PIDController& getRollOuterPID() { return _rollOuter; }
    PIDController& getPitchOuterPID() { return _pitchOuter; }
    PIDController& getYawOuterPID() { return _yawOuter; }
    PIDController& getRollInnerPID() { return _rollInner; }
    PIDController& getPitchInnerPID() { return _pitchInner; }
    PIDController& getYawInnerPID() { return _yawInner; }

private:
    // Outer loop (attitude → rate)
    PIDController _rollOuter;
    PIDController _pitchOuter;
    PIDController _yawOuter;
    
    // Inner loop (rate → TVC)
    PIDController _rollInner;
    PIDController _pitchInner;
    PIDController _yawInner;
    
    // Target attitude
    float _targetRoll;
    float _targetPitch;
    float _targetYaw;
    
    // Rate setpoints (output of outer loop)
    float _rollRateSp;
    float _pitchRateSp;
    float _yawRateSp;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline AttitudeController::AttitudeController()
    : _rollOuter(PID_ROLL_OUTER_P, PID_ROLL_OUTER_I, PID_ROLL_OUTER_D)
    , _pitchOuter(PID_PITCH_OUTER_P, PID_PITCH_OUTER_I, PID_PITCH_OUTER_D)
    , _yawOuter(PID_YAW_OUTER_P, PID_YAW_OUTER_I, PID_YAW_OUTER_D)
    , _rollInner(PID_ROLL_INNER_P, PID_ROLL_INNER_I, PID_ROLL_INNER_D)
    , _pitchInner(PID_PITCH_INNER_P, PID_PITCH_INNER_I, PID_PITCH_INNER_D)
    , _yawInner(PID_YAW_INNER_P, PID_YAW_INNER_I, PID_YAW_INNER_D)
    , _targetRoll(0.0f)
    , _targetPitch(0.0f)
    , _targetYaw(0.0f)
    , _rollRateSp(0.0f)
    , _pitchRateSp(0.0f)
    , _yawRateSp(0.0f)
{
    init();
}

inline void AttitudeController::init() {
    // Configure outer loop (attitude)
    _rollOuter.setIntegratorLimits(-PID_ROLL_OUTER_IMAX, PID_ROLL_OUTER_IMAX);
    _rollOuter.setOutputLimits(-2.0f, 2.0f);  // Max rate setpoint (rad/s)
    _pitchOuter.setIntegratorLimits(-PID_PITCH_OUTER_IMAX, PID_PITCH_OUTER_IMAX);
    _pitchOuter.setOutputLimits(-2.0f, 2.0f);
    _yawOuter.setIntegratorLimits(-PID_YAW_OUTER_IMAX, PID_YAW_OUTER_IMAX);
    _yawOuter.setOutputLimits(-1.0f, 1.0f);
    
    // Configure inner loop (rate)
    _rollInner.setIntegratorLimits(-PID_ROLL_INNER_IMAX, PID_ROLL_INNER_IMAX);
    _rollInner.setOutputLimits(-1.0f, 1.0f);  // Normalized TVC command
    _pitchInner.setIntegratorLimits(-PID_PITCH_INNER_IMAX, PID_PITCH_INNER_IMAX);
    _pitchInner.setOutputLimits(-1.0f, 1.0f);
    _yawInner.setIntegratorLimits(-PID_YAW_INNER_IMAX, PID_YAW_INNER_IMAX);
    _yawInner.setOutputLimits(-0.5f, 0.5f);  // Less authority on yaw for now
}

inline void AttitudeController::reset() {
    _rollOuter.reset();
    _pitchOuter.reset();
    _yawOuter.reset();
    _rollInner.reset();
    _pitchInner.reset();
    _yawInner.reset();
    
    _targetRoll = _targetPitch = _targetYaw = 0.0f;
    _rollRateSp = _pitchRateSp = _yawRateSp = 0.0f;
}

inline void AttitudeController::setTarget(float roll, float pitch, float yaw) {
    _targetRoll = roll;
    _targetPitch = pitch;
    _targetYaw = yaw;
}

inline void AttitudeController::setTargetQuaternion(const Quaternion& target) {
    target.toEuler(_targetRoll, _targetPitch, _targetYaw);
}

inline TvcCommand AttitudeController::update(float roll, float pitch, float yaw,
                                             float gyroX, float gyroY, float gyroZ,
                                             float dt) {
    // Outer loop: compute rate setpoints from attitude error
    _rollRateSp = _rollOuter.compute(_targetRoll, roll, gyroX, dt);
    _pitchRateSp = _pitchOuter.compute(_targetPitch, pitch, gyroY, dt);
    _yawRateSp = _yawOuter.compute(_targetYaw, yaw, gyroZ, dt);
    
    // Inner loop: compute TVC commands from rate error
    float rollCmd = _rollInner.compute(_rollRateSp, gyroX, dt);
    float pitchCmd = _pitchInner.compute(_pitchRateSp, gyroY, dt);
    float yawCmd = _yawInner.compute(_yawRateSp, gyroZ, dt);
    
    // Mix commands to TVC servos
    // For a typical TVC setup:
    // Servo X controls pitch (Y-axis rotation)
    // Servo Y controls roll (X-axis rotation)
    // Yaw is not directly controlled by TVC (requires fins or differential thrust)
    
    TvcCommand cmd;
    cmd.servoX = pitchCmd * TVC_SERVO_X_LIMIT;  // Convert to degrees
    cmd.servoY = rollCmd * TVC_SERVO_Y_LIMIT;
    
    // Apply limits
    cmd.servoX = constants::clamp(cmd.servoX, -TVC_SERVO_X_LIMIT, TVC_SERVO_X_LIMIT);
    cmd.servoY = constants::clamp(cmd.servoY, -TVC_SERVO_Y_LIMIT, TVC_SERVO_Y_LIMIT);
    
    return cmd;
}
