#pragma once

// ============================================================================
// VERTICAL LANDING ROCKET - CONFIGURATION
// ============================================================================
// Central configuration file for all tunable parameters.
// Modify these values to match your hardware and desired behavior.
// ============================================================================

// ----------------------------------------------------------------------------
// CONTROL LOOP TIMING
// ----------------------------------------------------------------------------
#define CONTROL_LOOP_FREQ_HZ     1000    // Main control loop frequency
#define TELEMETRY_FREQ_HZ        50      // State telemetry rate
#define SENSOR_LOG_FREQ_HZ       100     // Raw sensor logging rate
#define STATUS_FREQ_HZ           10      // Status message rate
#define GPS_FREQ_HZ              25      // GPS update rate

// ----------------------------------------------------------------------------
// PID CONTROLLER GAINS - ATTITUDE (OUTER LOOP)
// ----------------------------------------------------------------------------
// These control the response from attitude error to angular rate setpoint

// Roll axis
#define PID_ROLL_OUTER_P         5.0f
#define PID_ROLL_OUTER_I         0.1f
#define PID_ROLL_OUTER_D         0.0f
#define PID_ROLL_OUTER_IMAX      0.5f    // Integrator anti-windup limit (rad/s)

// Pitch axis
#define PID_PITCH_OUTER_P        5.0f
#define PID_PITCH_OUTER_I        0.1f
#define PID_PITCH_OUTER_D        0.0f
#define PID_PITCH_OUTER_IMAX     0.5f

// Yaw axis
#define PID_YAW_OUTER_P          3.0f
#define PID_YAW_OUTER_I          0.05f
#define PID_YAW_OUTER_D          0.0f
#define PID_YAW_OUTER_IMAX       0.3f

// ----------------------------------------------------------------------------
// PID CONTROLLER GAINS - RATE (INNER LOOP)
// ----------------------------------------------------------------------------
// These control the response from angular rate error to TVC command

// Roll rate
#define PID_ROLL_INNER_P         0.8f
#define PID_ROLL_INNER_I         0.05f
#define PID_ROLL_INNER_D         0.02f
#define PID_ROLL_INNER_IMAX      0.3f

// Pitch rate
#define PID_PITCH_INNER_P        0.8f
#define PID_PITCH_INNER_I        0.05f
#define PID_PITCH_INNER_D        0.02f
#define PID_PITCH_INNER_IMAX     0.3f

// Yaw rate
#define PID_YAW_INNER_P          0.5f
#define PID_YAW_INNER_I          0.02f
#define PID_YAW_INNER_D          0.01f
#define PID_YAW_INNER_IMAX       0.2f

// ----------------------------------------------------------------------------
// THRUST VECTOR CONTROL (TVC)
// ----------------------------------------------------------------------------
#define TVC_SERVO_X_CENTER       90      // Neutral position (degrees)
#define TVC_SERVO_Y_CENTER       90
#define TVC_SERVO_X_TRIM         0.0f    // Trim offset (degrees)
#define TVC_SERVO_Y_TRIM         0.0f
#define TVC_SERVO_X_LIMIT        15.0f   // Max deflection from center
#define TVC_SERVO_Y_LIMIT        15.0f
#define TVC_SERVO_X_REVERSE      false   // Reverse servo direction
#define TVC_SERVO_Y_REVERSE      false
#define TVC_GIMBAL_LENGTH_MM     20.0f   // Distance from pivot to motor CG

// ----------------------------------------------------------------------------
// KALMAN FILTER TUNING
// ----------------------------------------------------------------------------
// Process noise covariance (how much we trust the model)
#define KALMAN_Q_ACCEL           0.1f    // Acceleration noise (m/s²)²
#define KALMAN_Q_GYRO            0.001f  // Gyro noise (rad/s)²
#define KALMAN_Q_BIAS            0.0001f // Bias random walk

// Measurement noise covariance (how much we trust the sensors)
#define KALMAN_R_ACCEL           0.5f    // Accelerometer noise
#define KALMAN_R_GYRO            0.01f   // Gyroscope noise
#define KALMAN_R_BARO            0.5f    // Barometer altitude noise (m)²
#define KALMAN_R_GPS_POS         2.0f    // GPS position noise (m)²
#define KALMAN_R_GPS_VEL         0.2f    // GPS velocity noise (m/s)²

// ----------------------------------------------------------------------------
// STATE MACHINE THRESHOLDS
// ----------------------------------------------------------------------------
#define LAUNCH_ACCEL_THRESHOLD   20.0f   // m/s², acceleration to detect launch
#define BURNOUT_ACCEL_THRESHOLD  2.0f    // m/s², acceleration to detect burnout
#define APOGEE_VEL_THRESHOLD     1.0f    // m/s, vertical velocity near apogee
#define LANDING_VEL_THRESHOLD    0.5f    // m/s, velocity to detect landing
#define LANDING_ALT_THRESHOLD    0.5f    // m, altitude AGL to detect landing
#define ABORT_TIMEOUT_MS         30000   // Maximum flight time before abort

// ----------------------------------------------------------------------------
// SAFETY LIMITS
// ----------------------------------------------------------------------------
#define BATTERY_LOW_VOLTAGE      6.6f    // 2S LiPo low voltage (V)
#define BATTERY_CRITICAL_VOLTAGE 6.0f    // Trigger abort
#define MAX_ALTITUDE_M           500.0f  // Maximum allowed altitude
#define GEOFENCE_RADIUS_M        100.0f  // Maximum horizontal distance
#define MAX_TILT_ANGLE_DEG       45.0f   // Maximum tilt before abort
#define ARM_KEY                  0xDEADBEEF  // Safety key for arming

// ----------------------------------------------------------------------------
// PHYSICAL CONSTANTS
// ----------------------------------------------------------------------------
#define GRAVITY_MSS              9.80665f // Standard gravity (m/s²)
#define SEA_LEVEL_PRESSURE_PA    101325.0f // Standard sea level pressure

// ----------------------------------------------------------------------------
// ROCKET PHYSICAL PROPERTIES
// ----------------------------------------------------------------------------
#define ROCKET_MASS_KG           1.5f    // Total wet mass
#define ROCKET_DRY_MASS_KG       1.2f    // Mass without propellant
#define ROCKET_LENGTH_M          0.6f    // Total length
#define ROCKET_DIAMETER_M        0.054f  // Body diameter
#define ROCKET_DRAG_COEFF        0.5f    // Drag coefficient
#define MOTOR_THRUST_N           40.0f   // Average thrust (for F-class)
#define MOTOR_BURN_TIME_S        1.5f    // Burn duration

// ----------------------------------------------------------------------------
// SERIAL COMMUNICATION
// ----------------------------------------------------------------------------
#define SERIAL_BAUD_RATE         921600
#define SERIAL_JETSON            Serial1
#define SERIAL_LORA              Serial3
#define SERIAL_GPS               Serial2

// Protocol constants
#define PROTOCOL_SYNC_BYTE       0xAA
#define PROTOCOL_MAX_PACKET_SIZE 256
