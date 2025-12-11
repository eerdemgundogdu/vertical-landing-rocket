// ============================================================================
// EXTENDED KALMAN FILTER - IMPLEMENTATION
// ============================================================================

#include "kalman.h"
#include "config.h"
#include "constants.h"

ExtendedKalmanFilter::ExtendedKalmanFilter()
    : _homeLat(0.0), _homeLon(0.0), _homeAlt(0.0f), _homeSet(false)
{
    init();
}

void ExtendedKalmanFilter::init() {
    reset();
    
    // Initialize process noise
    _Q[STATE_POS_N] = 0.01f;
    _Q[STATE_POS_E] = 0.01f;
    _Q[STATE_POS_D] = 0.01f;
    _Q[STATE_VEL_N] = KALMAN_Q_ACCEL;
    _Q[STATE_VEL_E] = KALMAN_Q_ACCEL;
    _Q[STATE_VEL_D] = KALMAN_Q_ACCEL;
    _Q[STATE_QW] = KALMAN_Q_GYRO;
    _Q[STATE_QX] = KALMAN_Q_GYRO;
    _Q[STATE_QY] = KALMAN_Q_GYRO;
    _Q[STATE_QZ] = KALMAN_Q_GYRO;
    _Q[STATE_GBX] = KALMAN_Q_BIAS;
    _Q[STATE_GBY] = KALMAN_Q_BIAS;
    _Q[STATE_GBZ] = KALMAN_Q_BIAS;
    _Q[STATE_ABX] = KALMAN_Q_BIAS;
    _Q[STATE_ABY] = KALMAN_Q_BIAS;
    _Q[STATE_ABZ] = KALMAN_Q_BIAS;
    
    // Initialize measurement noise
    _R_baro = KALMAN_R_BARO;
    _R_gps_pos = KALMAN_R_GPS_POS;
    _R_gps_vel = KALMAN_R_GPS_VEL;
}

void ExtendedKalmanFilter::reset() {
    // Reset state
    _state.posN = _state.posE = _state.posD = 0.0f;
    _state.velN = _state.velE = _state.velD = 0.0f;
    _state.attitude = Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
    _state.roll = _state.pitch = _state.yaw = 0.0f;
    _state.gyroX = _state.gyroY = _state.gyroZ = 0.0f;
    _state.accelX = _state.accelY = _state.accelZ = 0.0f;
    _state.gyroBiasX = _state.gyroBiasY = _state.gyroBiasZ = 0.0f;
    _state.accelBiasX = _state.accelBiasY = _state.accelBiasZ = 0.0f;
    _state.altitudeAGL = 0.0f;
    _state.timestamp = 0;
    
    // Reset covariance (initial uncertainty)
    for (int i = 0; i < STATE_SIZE; i++) {
        _P[i] = 1.0f;
    }
    _P[STATE_QW] = 0.01f;  // Lower initial uncertainty for quaternion
    _P[STATE_QX] = 0.01f;
    _P[STATE_QY] = 0.01f;
    _P[STATE_QZ] = 0.01f;
}

void ExtendedKalmanFilter::predict(float ax, float ay, float az,
                                   float gx, float gy, float gz,
                                   float dt) {
    // Store raw sensor data
    _state.gyroX = gx;
    _state.gyroY = gy;
    _state.gyroZ = gz;
    _state.accelX = ax;
    _state.accelY = ay;
    _state.accelZ = az;
    
    // Remove biases
    float gxc = gx - _state.gyroBiasX;
    float gyc = gy - _state.gyroBiasY;
    float gzc = gz - _state.gyroBiasZ;
    float axc = ax - _state.accelBiasX;
    float ayc = ay - _state.accelBiasY;
    float azc = az - _state.accelBiasZ;
    
    predictState(axc, ayc, azc, gxc, gyc, gzc, dt);
    predictCovariance(dt);
    
    _state.timestamp = micros();
}

void ExtendedKalmanFilter::predictState(float ax, float ay, float az,
                                         float gx, float gy, float gz,
                                         float dt) {
    // Update attitude using gyroscope
    // q_new = q * q_delta
    Quaternion qDelta = Quaternion::fromAngularVelocity(gx, gy, gz, dt);
    _state.attitude = _state.attitude * qDelta;
    normalizeQuaternion();
    
    // Convert body-frame acceleration to NED frame
    float aN = ax, aE = ay, aD = az;
    _state.attitude.rotateVector(aN, aE, aD);
    
    // Remove gravity (NED frame, gravity points down = positive D)
    aD += GRAVITY;
    
    // Update velocity
    _state.velN += aN * dt;
    _state.velE += aE * dt;
    _state.velD += aD * dt;
    
    // Update position
    _state.posN += _state.velN * dt + 0.5f * aN * dt * dt;
    _state.posE += _state.velE * dt + 0.5f * aE * dt * dt;
    _state.posD += _state.velD * dt + 0.5f * aD * dt * dt;
    
    // Update Euler angles for convenience
    _state.attitude.toEuler(_state.roll, _state.pitch, _state.yaw);
    
    // Update altitude AGL (positive up)
    _state.altitudeAGL = -_state.posD;
}

void ExtendedKalmanFilter::predictCovariance(float dt) {
    // Simplified covariance propagation (diagonal approximation)
    for (int i = 0; i < STATE_SIZE; i++) {
        _P[i] += _Q[i] * dt;
    }
}

void ExtendedKalmanFilter::correctBarometer(float altitude) {
    // Measurement: altitude = -posD
    float measured = altitude;
    float predicted = -_state.posD;
    float innovation = measured - predicted;
    
    // Kalman gain (simplified)
    float S = _P[STATE_POS_D] + _R_baro;
    float K = _P[STATE_POS_D] / S;
    
    // Update state
    _state.posD -= K * innovation;
    _state.altitudeAGL = -_state.posD;
    
    // Update covariance
    _P[STATE_POS_D] *= (1.0f - K);
    
    // Also correct vertical velocity slightly
    float Kv = _P[STATE_VEL_D] / (S + _R_baro);
    _state.velD -= Kv * innovation * 0.1f;  // Small correction
}

void ExtendedKalmanFilter::correctGps(float posN, float posE, float posD,
                                      float velN, float velE, float velD) {
    // Position correction
    float innovN = posN - _state.posN;
    float innovE = posE - _state.posE;
    float innovD = posD - _state.posD;
    
    float Spos = _P[STATE_POS_N] + _R_gps_pos;
    float Kpos = _P[STATE_POS_N] / Spos;
    
    _state.posN += Kpos * innovN;
    _state.posE += Kpos * innovE;
    _state.posD += Kpos * innovD * 0.5f;  // Less trust on GPS altitude
    
    _P[STATE_POS_N] *= (1.0f - Kpos);
    _P[STATE_POS_E] *= (1.0f - Kpos);
    
    // Velocity correction
    float innovVN = velN - _state.velN;
    float innovVE = velE - _state.velE;
    float innovVD = velD - _state.velD;
    
    float Svel = _P[STATE_VEL_N] + _R_gps_vel;
    float Kvel = _P[STATE_VEL_N] / Svel;
    
    _state.velN += Kvel * innovVN;
    _state.velE += Kvel * innovVE;
    _state.velD += Kvel * innovVD * 0.5f;
    
    _P[STATE_VEL_N] *= (1.0f - Kvel);
    _P[STATE_VEL_E] *= (1.0f - Kvel);
    
    _state.altitudeAGL = -_state.posD;
}

void ExtendedKalmanFilter::correctMagnetometer(float mx, float my, float mz) {
    // Optional magnetometer correction for yaw
    // Simplified heading calculation
    float roll = _state.roll;
    float pitch = _state.pitch;
    
    // Tilt-compensated heading
    float cosRoll = cosf(roll);
    float sinRoll = sinf(roll);
    float cosPitch = cosf(pitch);
    float sinPitch = sinf(pitch);
    
    float Xh = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;
    float Yh = my * cosRoll - mz * sinRoll;
    
    float magYaw = atan2f(-Yh, Xh);
    
    // Correct yaw through quaternion
    float yawError = constants::wrapAngle(magYaw - _state.yaw);
    
    // Small correction to avoid jumps
    float K = 0.01f;
    float yawCorr = K * yawError;
    
    // Apply rotation around Z axis
    Quaternion correction = Quaternion::fromAxisAngle(0, 0, 1, yawCorr);
    _state.attitude = _state.attitude * correction;
    normalizeQuaternion();
    
    _state.attitude.toEuler(_state.roll, _state.pitch, _state.yaw);
}

void ExtendedKalmanFilter::normalizeQuaternion() {
    _state.attitude.normalize();
}

void ExtendedKalmanFilter::setHomePosition(double lat, double lon, float alt) {
    _homeLat = lat;
    _homeLon = lon;
    _homeAlt = alt;
    _homeSet = true;
}

void ExtendedKalmanFilter::gpsToLocal(double lat, double lon, float alt,
                                      float& n, float& e, float& d) const {
    if (!_homeSet) {
        n = e = d = 0.0f;
        return;
    }
    
    // Convert to local NED using flat earth approximation
    double dLat = lat - _homeLat;
    double dLon = lon - _homeLon;
    
    n = (float)(dLat * constants::DEG_TO_RAD * EARTH_RADIUS);
    e = (float)(dLon * constants::DEG_TO_RAD * EARTH_RADIUS * cos(_homeLat * constants::DEG_TO_RAD));
    d = _homeAlt - alt;  // Positive down
}
