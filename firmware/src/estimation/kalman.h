#pragma once

// ============================================================================
// EXTENDED KALMAN FILTER
// ============================================================================
// State estimation combining IMU, barometer, and GPS measurements
// ============================================================================

#include <Arduino.h>
#include "quaternion.h"

// State vector indices
enum StateIndex {
    STATE_POS_N = 0,    // Position North (m)
    STATE_POS_E = 1,    // Position East (m)
    STATE_POS_D = 2,    // Position Down (m)
    STATE_VEL_N = 3,    // Velocity North (m/s)
    STATE_VEL_E = 4,    // Velocity East (m/s)
    STATE_VEL_D = 5,    // Velocity Down (m/s)
    STATE_QW = 6,       // Quaternion w
    STATE_QX = 7,       // Quaternion x
    STATE_QY = 8,       // Quaternion y
    STATE_QZ = 9,       // Quaternion z
    STATE_GBX = 10,     // Gyro bias x (rad/s)
    STATE_GBY = 11,     // Gyro bias y
    STATE_GBZ = 12,     // Gyro bias z
    STATE_ABX = 13,     // Accel bias x (m/s²)
    STATE_ABY = 14,     // Accel bias y
    STATE_ABZ = 15,     // Accel bias z
    STATE_SIZE = 16
};

struct KalmanState {
    // Position (NED frame, meters relative to launch point)
    float posN, posE, posD;
    
    // Velocity (NED frame, m/s)
    float velN, velE, velD;
    
    // Attitude quaternion
    Quaternion attitude;
    
    // Euler angles (derived, for convenience)
    float roll, pitch, yaw;  // radians
    
    // Angular rates (body frame, rad/s)
    float gyroX, gyroY, gyroZ;
    
    // Acceleration (body frame, m/s²)
    float accelX, accelY, accelZ;
    
    // Biases
    float gyroBiasX, gyroBiasY, gyroBiasZ;
    float accelBiasX, accelBiasY, accelBiasZ;
    
    // Altitude AGL
    float altitudeAGL;
    
    // Timestamp
    uint32_t timestamp;
};

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();
    
    void init();
    void reset();
    
    // Prediction step using IMU data
    void predict(float ax, float ay, float az,
                 float gx, float gy, float gz,
                 float dt);
    
    // Correction steps for different sensors
    void correctBarometer(float altitude);
    void correctGps(float posN, float posE, float posD,
                    float velN, float velE, float velD);
    void correctMagnetometer(float mx, float my, float mz);  // Optional
    
    // Get current state estimate
    const KalmanState& getState() const { return _state; }
    
    // Initialize position reference
    void setHomePosition(double lat, double lon, float alt);
    
    // Convert GPS to local NED
    void gpsToLocal(double lat, double lon, float alt,
                    float& n, float& e, float& d) const;

private:
    void predictState(float ax, float ay, float az,
                      float gx, float gy, float gz,
                      float dt);
    void predictCovariance(float dt);
    void normalizeQuaternion();
    
    KalmanState _state;
    
    // State covariance matrix (simplified - diagonal)
    float _P[STATE_SIZE];
    
    // Process noise
    float _Q[STATE_SIZE];
    
    // Measurement noise
    float _R_baro;
    float _R_gps_pos;
    float _R_gps_vel;
    
    // Home position for NED conversion
    double _homeLat, _homeLon;
    float _homeAlt;
    bool _homeSet;
    
    // Earth radius for local tangent plane
    static constexpr float EARTH_RADIUS = 6371000.0f;
};
