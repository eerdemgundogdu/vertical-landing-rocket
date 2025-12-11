#pragma once

// ============================================================================
// LANDING CONTROLLER
// ============================================================================
// Specialized controller for powered descent and precision landing
// ============================================================================

#include <Arduino.h>
#include "config.h"
#include "constants.h"
#include "../control/pid.h"
#include "../control/attitude.h"
#include "trajectory.h"

// Landing phases
enum class LandingPhase : uint8_t {
    INACTIVE = 0,
    REORIENTATION = 1,      // Flip to landing attitude
    POWERED_DESCENT = 2,    // Main deceleration burn
    TERMINAL_GUIDANCE = 3,  // Final approach
    TOUCHDOWN = 4           // Ground contact
};

// Landing controller output
struct LandingOutput {
    float throttle;         // 0-1 throttle command
    float targetRoll;       // Target roll angle (rad)
    float targetPitch;      // Target pitch angle (rad)
    float targetYaw;        // Target yaw angle (rad)
    bool igniteMotor;       // Motor ignition command
    bool deployLegs;        // Landing leg deployment
};

class LandingController {
public:
    LandingController();
    
    void init();
    void reset();
    
    // Set landing target (NED position)
    void setTarget(float n, float e, float d);
    
    // Set current altitude for phase transitions
    void setPhaseAltitudes(float reorient, float terminal, float touchdown);
    
    // Main update function
    LandingOutput update(float posN, float posE, float posD,
                         float velN, float velE, float velD,
                         float roll, float pitch, float yaw,
                         float dt);
    
    // Manual phase control
    void startLanding();
    void abort();
    
    // Getters
    LandingPhase getPhase() const { return _phase; }
    float getTimeToLanding() const { return _timeToLanding; }

private:
    void updatePhase(float altitude);
    
    LandingPhase _phase;
    
    // Target landing position
    float _targetN, _targetE, _targetD;
    
    // Phase transition altitudes
    float _reorientAlt;
    float _terminalAlt;
    float _touchdownAlt;
    
    // Controllers
    PIDController _posNPid;
    PIDController _posEPid;
    PIDController _altPid;
    PIDController _velNPid;
    PIDController _velEPid;
    PIDController _velDPid;
    
    // State
    float _timeToLanding;
    float _referenceThrottle;
    bool _motorIgnited;
    bool _legsDeployed;
    
    // Trajectory planner
    TrajectoryPlanner _trajectory;
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline LandingController::LandingController()
    : _phase(LandingPhase::INACTIVE)
    , _targetN(0), _targetE(0), _targetD(0)
    , _reorientAlt(100.0f)
    , _terminalAlt(20.0f)
    , _touchdownAlt(2.0f)
    , _posNPid(0.3f, 0.01f, 0.1f)
    , _posEPid(0.3f, 0.01f, 0.1f)
    , _altPid(0.5f, 0.02f, 0.2f)
    , _velNPid(0.5f, 0.0f, 0.1f)
    , _velEPid(0.5f, 0.0f, 0.1f)
    , _velDPid(1.0f, 0.1f, 0.2f)
    , _timeToLanding(0)
    , _referenceThrottle(0.5f)
    , _motorIgnited(false)
    , _legsDeployed(false)
{
}

inline void LandingController::init() {
    // Configure position PIDs
    _posNPid.setOutputLimits(-0.3f, 0.3f);  // Max tilt command
    _posEPid.setOutputLimits(-0.3f, 0.3f);
    _altPid.setOutputLimits(-0.5f, 0.5f);   // Throttle adjustment
    
    // Configure velocity PIDs
    _velNPid.setOutputLimits(-0.2f, 0.2f);
    _velEPid.setOutputLimits(-0.2f, 0.2f);
    _velDPid.setOutputLimits(0.2f, 1.0f);  // Throttle
    
    reset();
}

inline void LandingController::reset() {
    _phase = LandingPhase::INACTIVE;
    
    _posNPid.reset();
    _posEPid.reset();
    _altPid.reset();
    _velNPid.reset();
    _velEPid.reset();
    _velDPid.reset();
    
    _trajectory.reset();
    
    _motorIgnited = false;
    _legsDeployed = false;
}

inline void LandingController::setTarget(float n, float e, float d) {
    _targetN = n;
    _targetE = e;
    _targetD = d;
}

inline void LandingController::setPhaseAltitudes(float reorient, float terminal, float touchdown) {
    _reorientAlt = reorient;
    _terminalAlt = terminal;
    _touchdownAlt = touchdown;
}

inline void LandingController::startLanding() {
    _phase = LandingPhase::REORIENTATION;
    reset();
}

inline void LandingController::abort() {
    _phase = LandingPhase::INACTIVE;
    reset();
}

inline void LandingController::updatePhase(float altitude) {
    if (_phase == LandingPhase::INACTIVE) return;
    
    if (_phase == LandingPhase::REORIENTATION && altitude < _reorientAlt) {
        _phase = LandingPhase::POWERED_DESCENT;
        _motorIgnited = true;
    }
    else if (_phase == LandingPhase::POWERED_DESCENT && altitude < _terminalAlt) {
        _phase = LandingPhase::TERMINAL_GUIDANCE;
        _legsDeployed = true;
    }
    else if (_phase == LandingPhase::TERMINAL_GUIDANCE && altitude < _touchdownAlt) {
        _phase = LandingPhase::TOUCHDOWN;
    }
}

inline LandingOutput LandingController::update(float posN, float posE, float posD,
                                                float velN, float velE, float velD,
                                                float roll, float pitch, float yaw,
                                                float dt) {
    LandingOutput out = {0, 0, 0, 0, false, false};
    
    float altitude = -posD;
    updatePhase(altitude);
    
    if (_phase == LandingPhase::INACTIVE) {
        return out;
    }
    
    // Position errors
    float errN = _targetN - posN;
    float errE = _targetE - posE;
    
    // Estimate time to landing
    float descentRate = velD > 0.1f ? velD : 0.1f;
    _timeToLanding = (altitude - (-_targetD)) / descentRate;
    
    switch (_phase) {
        case LandingPhase::REORIENTATION:
            // Target vertical orientation
            out.targetRoll = 0;
            out.targetPitch = 0;
            out.targetYaw = yaw;  // Maintain current yaw
            out.throttle = 0;
            break;
            
        case LandingPhase::POWERED_DESCENT: {
            // Horizontal position control -> attitude command
            float velNCmdRaw = _posNPid.compute(0, -errN, dt);
            float velECmdRaw = _posEPid.compute(0, -errE, dt);
            
            // Velocity control -> tilt angle
            out.targetPitch = _velNPid.compute(velNCmdRaw, velN, dt);
            out.targetRoll = -_velEPid.compute(velECmdRaw, velE, dt);
            out.targetYaw = yaw;
            
            // Altitude rate control -> throttle
            float targetVelD = altitude * 0.5f;  // Descent rate proportional to altitude
            targetVelD = constants::clamp(targetVelD, 2.0f, 20.0f);
            
            out.throttle = _velDPid.compute(targetVelD, velD, dt);
            out.igniteMotor = true;
            break;
        }
        
        case LandingPhase::TERMINAL_GUIDANCE: {
            // Very gentle horizontal corrections
            float velNCmdRaw = _posNPid.compute(0, -errN, dt) * 0.3f;
            float velECmdRaw = _posEPid.compute(0, -errE, dt) * 0.3f;
            
            out.targetPitch = constants::clamp(velNCmdRaw, -0.1f, 0.1f);
            out.targetRoll = constants::clamp(-velECmdRaw, -0.1f, 0.1f);
            out.targetYaw = yaw;
            
            // Target constant low descent rate
            float targetVelD = 2.0f;
            out.throttle = _velDPid.compute(targetVelD, velD, dt);
            
            out.igniteMotor = true;
            out.deployLegs = true;
            break;
        }
        
        case LandingPhase::TOUCHDOWN:
            // Minimal throttle, maintain attitude
            out.targetRoll = 0;
            out.targetPitch = 0;
            out.targetYaw = yaw;
            out.throttle = 0.1f;
            out.igniteMotor = false;
            out.deployLegs = true;
            break;
            
        default:
            break;
    }
    
    return out;
}
