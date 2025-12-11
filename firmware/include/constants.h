#pragma once

// ============================================================================
// VERTICAL LANDING ROCKET - PHYSICAL CONSTANTS
// ============================================================================

#include <cmath>

namespace constants {

// Gravitational acceleration
constexpr float GRAVITY = 9.80665f;  // m/s²

// Standard atmosphere
constexpr float SEA_LEVEL_PRESSURE = 101325.0f;  // Pa
constexpr float SEA_LEVEL_TEMP = 288.15f;        // K (15°C)
constexpr float LAPSE_RATE = 0.0065f;            // K/m
constexpr float GAS_CONSTANT = 287.05f;          // J/(kg·K) for dry air
constexpr float PRESSURE_EXPONENT = 5.2561f;     // g/(R·L)

// Mathematical constants  
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;
constexpr float HALF_PI = PI / 2.0f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

// Time conversions
constexpr float MICROS_TO_SEC = 1.0e-6f;
constexpr float MILLIS_TO_SEC = 1.0e-3f;

// Altitude from pressure (barometric formula)
inline float pressureToAltitude(float pressure, float seaLevelPressure = SEA_LEVEL_PRESSURE) {
    return SEA_LEVEL_TEMP / LAPSE_RATE * 
           (1.0f - powf(pressure / seaLevelPressure, 1.0f / PRESSURE_EXPONENT));
}

// Pressure from altitude
inline float altitudeToPressure(float altitude, float seaLevelPressure = SEA_LEVEL_PRESSURE) {
    return seaLevelPressure * powf(1.0f - (LAPSE_RATE * altitude) / SEA_LEVEL_TEMP, PRESSURE_EXPONENT);
}

// Clamp value between min and max
template<typename T>
inline T clamp(T value, T minVal, T maxVal) {
    return (value < minVal) ? minVal : (value > maxVal) ? maxVal : value;
}

// Linear interpolation
inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

// Map value from one range to another
inline float mapRange(float value, float inMin, float inMax, float outMin, float outMax) {
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Sign function
template<typename T>
inline int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// Wrap angle to [-PI, PI]
inline float wrapAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

// Wrap angle to [0, 2*PI]
inline float wrapAngle2Pi(float angle) {
    while (angle >= TWO_PI) angle -= TWO_PI;
    while (angle < 0.0f) angle += TWO_PI;
    return angle;
}

}  // namespace constants
