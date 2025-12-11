#pragma once

// ============================================================================
// FLIGHT STATE MACHINE
// ============================================================================
// Manages flight phases from arming through landing
// ============================================================================

#include <Arduino.h>
#include "config.h"

// Flight states
enum class FlightState : uint8_t {
    IDLE = 0,       // System powered on, waiting for arm
    ARMED = 1,      // Armed, waiting for launch
    BOOST = 2,      // Motor burning, TVC active
    COAST = 3,      // Ballistic arc after burnout
    DESCENT = 4,    // Controlled descent toward landing
    LANDING = 5,    // Final approach and touchdown
    LANDED = 6,     // Successfully landed
    ABORT = 7       // Emergency abort triggered
};

// Abort reasons
enum class AbortReason : uint8_t {
    NONE = 0,
    MANUAL = 1,
    LOW_BATTERY = 2,
    SENSOR_FAILURE = 3,
    EXCESSIVE_TILT = 4,
    GEOFENCE = 5,
    ALTITUDE_LIMIT = 6,
    COMMUNICATION_LOST = 7,
    TIMEOUT = 8
};

// State machine events
struct FlightEvent {
    FlightState fromState;
    FlightState toState;
    uint32_t timestamp;
};

class FlightStateMachine {
public:
    FlightStateMachine();
    
    void init();
    void update(float accelMagnitude, float altitude, float verticalVel,
                float tiltAngle, float batteryVoltage, uint32_t flightTime);
    
    // State transitions
    bool arm(uint32_t key);
    bool disarm();
    void abort(AbortReason reason);
    
    // Getters
    FlightState getState() const { return _state; }
    AbortReason getAbortReason() const { return _abortReason; }
    const char* getStateName() const;
    
    bool isArmed() const { return _state != FlightState::IDLE; }
    bool isFlying() const { 
        return _state == FlightState::BOOST || 
               _state == FlightState::COAST || 
               _state == FlightState::DESCENT || 
               _state == FlightState::LANDING; 
    }
    bool isTvcActive() const {
        return _state == FlightState::BOOST || 
               _state == FlightState::DESCENT || 
               _state == FlightState::LANDING;
    }
    bool isLanded() const { return _state == FlightState::LANDED; }
    bool isAborted() const { return _state == FlightState::ABORT; }
    
    // Timing
    uint32_t getStateEntryTime() const { return _stateEntryTime; }
    uint32_t getTimeInState() const { return millis() - _stateEntryTime; }
    uint32_t getLaunchTime() const { return _launchTime; }
    
    // Event callback (for logging)
    void setEventCallback(void (*callback)(FlightEvent event)) { _eventCallback = callback; }

private:
    void transition(FlightState newState);
    bool checkSafetyLimits(float tiltAngle, float batteryVoltage, 
                           float altitude, uint32_t flightTime);
    
    FlightState _state;
    AbortReason _abortReason;
    
    uint32_t _stateEntryTime;
    uint32_t _launchTime;
    uint32_t _apogeeTime;
    
    float _maxAltitude;
    float _lastAltitude;
    bool _apogeeDetected;
    
    void (*_eventCallback)(FlightEvent event);
};

// ============================================================================
// IMPLEMENTATION
// ============================================================================

inline FlightStateMachine::FlightStateMachine()
    : _state(FlightState::IDLE)
    , _abortReason(AbortReason::NONE)
    , _stateEntryTime(0)
    , _launchTime(0)
    , _apogeeTime(0)
    , _maxAltitude(0.0f)
    , _lastAltitude(0.0f)
    , _apogeeDetected(false)
    , _eventCallback(nullptr)
{
}

inline void FlightStateMachine::init() {
    _state = FlightState::IDLE;
    _stateEntryTime = millis();
}

inline void FlightStateMachine::update(float accelMagnitude, float altitude, 
                                        float verticalVel, float tiltAngle,
                                        float batteryVoltage, uint32_t flightTime) {
    // Safety checks always active
    if (_state != FlightState::IDLE && _state != FlightState::LANDED && 
        _state != FlightState::ABORT) {
        if (!checkSafetyLimits(tiltAngle, batteryVoltage, altitude, flightTime)) {
            return;  // Abort was triggered
        }
    }
    
    // Track max altitude
    if (altitude > _maxAltitude) {
        _maxAltitude = altitude;
    }
    
    switch (_state) {
        case FlightState::IDLE:
            // Waiting for arm command
            break;
            
        case FlightState::ARMED:
            // Detect launch by high acceleration
            if (accelMagnitude > LAUNCH_ACCEL_THRESHOLD) {
                _launchTime = millis();
                transition(FlightState::BOOST);
            }
            break;
            
        case FlightState::BOOST:
            // Detect burnout by decreasing acceleration
            if (accelMagnitude < BURNOUT_ACCEL_THRESHOLD && 
                getTimeInState() > 200) {  // Debounce
                transition(FlightState::COAST);
            }
            break;
            
        case FlightState::COAST:
            // Detect apogee by velocity near zero or altitude decrease
            if ((verticalVel < APOGEE_VEL_THRESHOLD && verticalVel > -APOGEE_VEL_THRESHOLD) ||
                (altitude < _lastAltitude - 1.0f && _lastAltitude > 10.0f)) {
                _apogeeDetected = true;
                _apogeeTime = millis();
                transition(FlightState::DESCENT);
            }
            break;
            
        case FlightState::DESCENT:
            // Transition to landing when low altitude and low velocity
            if (altitude < 10.0f && fabsf(verticalVel) < 5.0f) {
                transition(FlightState::LANDING);
            }
            break;
            
        case FlightState::LANDING:
            // Detect touchdown
            if (altitude < LANDING_ALT_THRESHOLD && 
                fabsf(verticalVel) < LANDING_VEL_THRESHOLD &&
                accelMagnitude < 15.0f) {  // Not high-g impact
                transition(FlightState::LANDED);
            }
            break;
            
        case FlightState::LANDED:
        case FlightState::ABORT:
            // Terminal states
            break;
    }
    
    _lastAltitude = altitude;
}

inline bool FlightStateMachine::arm(uint32_t key) {
    if (_state != FlightState::IDLE) return false;
    if (key != ARM_KEY) return false;
    
    transition(FlightState::ARMED);
    return true;
}

inline bool FlightStateMachine::disarm() {
    if (_state != FlightState::ARMED) return false;
    
    transition(FlightState::IDLE);
    return true;
}

inline void FlightStateMachine::abort(AbortReason reason) {
    _abortReason = reason;
    transition(FlightState::ABORT);
}

inline const char* FlightStateMachine::getStateName() const {
    switch (_state) {
        case FlightState::IDLE:     return "IDLE";
        case FlightState::ARMED:    return "ARMED";
        case FlightState::BOOST:    return "BOOST";
        case FlightState::COAST:    return "COAST";
        case FlightState::DESCENT:  return "DESCENT";
        case FlightState::LANDING:  return "LANDING";
        case FlightState::LANDED:   return "LANDED";
        case FlightState::ABORT:    return "ABORT";
        default:                    return "UNKNOWN";
    }
}

inline void FlightStateMachine::transition(FlightState newState) {
    FlightEvent event;
    event.fromState = _state;
    event.toState = newState;
    event.timestamp = millis();
    
    _state = newState;
    _stateEntryTime = millis();
    
    if (_eventCallback) {
        _eventCallback(event);
    }
}

inline bool FlightStateMachine::checkSafetyLimits(float tiltAngle, float batteryVoltage,
                                                   float altitude, uint32_t flightTime) {
    if (batteryVoltage < BATTERY_CRITICAL_VOLTAGE) {
        abort(AbortReason::LOW_BATTERY);
        return false;
    }
    
    if (tiltAngle > MAX_TILT_ANGLE_DEG * constants::DEG_TO_RAD) {
        abort(AbortReason::EXCESSIVE_TILT);
        return false;
    }
    
    if (altitude > MAX_ALTITUDE_M) {
        abort(AbortReason::ALTITUDE_LIMIT);
        return false;
    }
    
    if (flightTime > ABORT_TIMEOUT_MS) {
        abort(AbortReason::TIMEOUT);
        return false;
    }
    
    return true;
}
