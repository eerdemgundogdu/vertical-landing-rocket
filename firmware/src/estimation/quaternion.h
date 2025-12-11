#pragma once

// ============================================================================
// QUATERNION MATH LIBRARY
// ============================================================================
// Hamilton convention quaternion operations for attitude representation
// ============================================================================

#include <Arduino.h>
#include <cmath>

struct Quaternion {
    float w, x, y, z;
    
    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
    
    // Normalize quaternion to unit length
    void normalize() {
        float mag = sqrtf(w*w + x*x + y*y + z*z);
        if (mag > 1e-10f) {
            float invMag = 1.0f / mag;
            w *= invMag;
            x *= invMag;
            y *= invMag;
            z *= invMag;
        }
    }
    
    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }
    
    // Conjugate (inverse for unit quaternions)
    Quaternion conjugate() const {
        return Quaternion(w, -x, -y, -z);
    }
    
    // Quaternion multiplication (Hamilton product)
    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        );
    }
    
    Quaternion& operator*=(const Quaternion& q) {
        *this = *this * q;
        return *this;
    }
    
    // Scalar multiplication
    Quaternion operator*(float s) const {
        return Quaternion(w*s, x*s, y*s, z*s);
    }
    
    // Addition
    Quaternion operator+(const Quaternion& q) const {
        return Quaternion(w+q.w, x+q.x, y+q.y, z+q.z);
    }
    
    // Rotate vector by quaternion: v' = q * v * q^-1
    void rotateVector(float& vx, float& vy, float& vz) const {
        // Optimized rotation avoiding full quaternion multiplication
        float qx2 = x * x;
        float qy2 = y * y;
        float qz2 = z * z;
        float qwqx = w * x;
        float qwqy = w * y;
        float qwqz = w * z;
        float qxqy = x * y;
        float qxqz = x * z;
        float qyqz = y * z;
        
        float ox = vx * (1.0f - 2.0f*(qy2 + qz2)) + vy * 2.0f*(qxqy - qwqz) + vz * 2.0f*(qxqz + qwqy);
        float oy = vx * 2.0f*(qxqy + qwqz) + vy * (1.0f - 2.0f*(qx2 + qz2)) + vz * 2.0f*(qyqz - qwqx);
        float oz = vx * 2.0f*(qxqz - qwqy) + vy * 2.0f*(qyqz + qwqx) + vz * (1.0f - 2.0f*(qx2 + qy2));
        
        vx = ox;
        vy = oy;
        vz = oz;
    }
    
    // Convert to Euler angles (roll, pitch, yaw) in radians
    // ZYX convention (yaw-pitch-roll)
    void toEuler(float& roll, float& pitch, float& yaw) const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        roll = atan2f(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w * y - z * x);
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(M_PI / 2.0f, sinp);  // Gimbal lock
        } else {
            pitch = asinf(sinp);
        }
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        yaw = atan2f(siny_cosp, cosy_cosp);
    }
    
    // Create quaternion from Euler angles (roll, pitch, yaw) in radians
    static Quaternion fromEuler(float roll, float pitch, float yaw) {
        float cr = cosf(roll * 0.5f);
        float sr = sinf(roll * 0.5f);
        float cp = cosf(pitch * 0.5f);
        float sp = sinf(pitch * 0.5f);
        float cy = cosf(yaw * 0.5f);
        float sy = sinf(yaw * 0.5f);
        
        return Quaternion(
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        );
    }
    
    // Create quaternion from axis-angle representation
    static Quaternion fromAxisAngle(float ax, float ay, float az, float angle) {
        float halfAngle = angle * 0.5f;
        float s = sinf(halfAngle);
        return Quaternion(cosf(halfAngle), ax * s, ay * s, az * s);
    }
    
    // Create quaternion from angular velocity integration
    // omega: angular velocity (rad/s), dt: time step (s)
    static Quaternion fromAngularVelocity(float wx, float wy, float wz, float dt) {
        float angle = sqrtf(wx*wx + wy*wy + wz*wz) * dt;
        if (angle < 1e-10f) {
            return Quaternion(1.0f, 0.0f, 0.0f, 0.0f);
        }
        float invAngle = 1.0f / (angle / dt);
        return fromAxisAngle(wx * invAngle, wy * invAngle, wz * invAngle, angle);
    }
    
    // Spherical linear interpolation
    static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t) {
        float dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z;
        
        Quaternion q2Adj = q2;
        if (dot < 0.0f) {
            dot = -dot;
            q2Adj = Quaternion(-q2.w, -q2.x, -q2.y, -q2.z);
        }
        
        if (dot > 0.9995f) {
            // Linear interpolation for very close quaternions
            Quaternion result = q1 * (1.0f - t) + q2Adj * t;
            result.normalize();
            return result;
        }
        
        float theta0 = acosf(dot);
        float theta = theta0 * t;
        float sinTheta = sinf(theta);
        float sinTheta0 = sinf(theta0);
        
        float s1 = cosf(theta) - dot * sinTheta / sinTheta0;
        float s2 = sinTheta / sinTheta0;
        
        return q1 * s1 + q2Adj * s2;
    }
    
    // Quaternion representing error between this and target
    Quaternion errorTo(const Quaternion& target) const {
        return conjugate() * target;
    }
    
    // Get rotation axis and angle
    void toAxisAngle(float& ax, float& ay, float& az, float& angle) const {
        angle = 2.0f * acosf(w);
        float s = sqrtf(1.0f - w*w);
        if (s < 1e-10f) {
            ax = 1.0f;
            ay = 0.0f;
            az = 0.0f;
        } else {
            float invS = 1.0f / s;
            ax = x * invS;
            ay = y * invS;
            az = z * invS;
        }
    }
};

// Vector3 helper struct
struct Vector3 {
    float x, y, z;
    
    Vector3() : x(0.0f), y(0.0f), z(0.0f) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    float magnitude() const {
        return sqrtf(x*x + y*y + z*z);
    }
    
    void normalize() {
        float mag = magnitude();
        if (mag > 1e-10f) {
            float inv = 1.0f / mag;
            x *= inv;
            y *= inv;
            z *= inv;
        }
    }
    
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }
    
    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }
    
    Vector3 operator*(float s) const {
        return Vector3(x * s, y * s, z * s);
    }
    
    float dot(const Vector3& v) const {
        return x*v.x + y*v.y + z*v.z;
    }
    
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y*v.z - z*v.y,
            z*v.x - x*v.z,
            x*v.y - y*v.x
        );
    }
};
