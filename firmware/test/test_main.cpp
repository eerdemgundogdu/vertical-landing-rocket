// ============================================================================
// FIRMWARE UNIT TESTS
// ============================================================================
// Native tests for critical firmware components
// Run with: pio test -e native
// ============================================================================

#include <unity.h>
#include <cmath>

// Mock Arduino functions for native testing
#ifndef ARDUINO
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif

// ============================================================================
// QUATERNION TESTS
// ============================================================================

void test_quaternion_identity() {
    // Identity quaternion should represent no rotation
    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, norm);
}

void test_quaternion_normalization() {
    // Non-unit quaternion should normalize
    float qw = 2.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
    float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    
    qw /= norm;
    qx /= norm;
    qy /= norm;
    qz /= norm;
    
    float new_norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, new_norm);
}

void test_quaternion_to_euler_identity() {
    // Identity quaternion should give zero Euler angles
    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
    
    // Roll
    float sinr_cosp = 2 * (qw * qx + qy * qz);
    float cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    float roll = atan2f(sinr_cosp, cosr_cosp);
    
    // Pitch
    float sinp = 2 * (qw * qy - qz * qx);
    float pitch = (fabsf(sinp) >= 1) ? copysignf(M_PI / 2, sinp) : asinf(sinp);
    
    // Yaw
    float siny_cosp = 2 * (qw * qz + qx * qy);
    float cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    float yaw = atan2f(siny_cosp, cosy_cosp);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, yaw);
}

// ============================================================================
// PID CONTROLLER TESTS
// ============================================================================

void test_pid_proportional() {
    // P-only controller
    float kp = 2.0f, ki = 0.0f, kd = 0.0f;
    float setpoint = 100.0f;
    float measurement = 50.0f;
    float error = setpoint - measurement;
    
    float output = kp * error;
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, output);
}

void test_pid_integral_accumulates() {
    float ki = 1.0f;
    float error = 10.0f;
    float dt = 0.01f;
    float integral = 0.0f;
    
    for (int i = 0; i < 100; i++) {
        integral += error * dt;
    }
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, integral);
}

void test_pid_output_limits() {
    float output = 150.0f;
    float min_out = -100.0f;
    float max_out = 100.0f;
    
    output = constrain(output, min_out, max_out);
    
    TEST_ASSERT_EQUAL_FLOAT(100.0f, output);
}

// ============================================================================
// ANGLE MATH TESTS
// ============================================================================

void test_angle_wrap_positive() {
    float angle = 4.0f;  // > PI
    float pi = 3.14159265f;
    
    while (angle > pi) angle -= 2 * pi;
    while (angle < -pi) angle += 2 * pi;
    
    TEST_ASSERT_TRUE(angle >= -pi && angle <= pi);
}

void test_angle_wrap_negative() {
    float angle = -4.0f;  // < -PI
    float pi = 3.14159265f;
    
    while (angle > pi) angle -= 2 * pi;
    while (angle < -pi) angle += 2 * pi;
    
    TEST_ASSERT_TRUE(angle >= -pi && angle <= pi);
}

void test_deg_rad_conversion() {
    float deg = 180.0f;
    float rad = deg * (M_PI / 180.0f);
    
    TEST_ASSERT_FLOAT_WITHIN(0.001f, M_PI, rad);
}

// ============================================================================
// FILTER TESTS
// ============================================================================

void test_low_pass_filter() {
    float alpha = 0.1f;
    float filtered = 0.0f;
    float input = 100.0f;
    
    // Apply filter multiple times
    for (int i = 0; i < 100; i++) {
        filtered = alpha * input + (1 - alpha) * filtered;
    }
    
    // Should approach input value
    TEST_ASSERT_FLOAT_WITHIN(1.0f, input, filtered);
}

void test_complementary_filter() {
    float alpha = 0.98f;
    float gyro_estimate = 10.0f;
    float accel_estimate = 12.0f;
    
    float fused = alpha * gyro_estimate + (1 - alpha) * accel_estimate;
    
    TEST_ASSERT_TRUE(fused > 10.0f && fused < 12.0f);
}

// ============================================================================
// CRC16 TEST
// ============================================================================

void test_crc16_ccitt() {
    uint8_t data[] = {0xAA, 0x01, 0x00, 0x00};
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < sizeof(data); i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    TEST_ASSERT_TRUE(crc != 0xFFFF);  // CRC should change
}

// ============================================================================
// COBS ENCODING TEST
// ============================================================================

void test_cobs_encode() {
    uint8_t input[] = {0x01, 0x00, 0x02, 0x00, 0x03};
    uint8_t output[10] = {0};
    
    size_t readIdx = 0;
    size_t writeIdx = 1;
    size_t codeIdx = 0;
    uint8_t code = 1;
    
    while (readIdx < sizeof(input)) {
        if (input[readIdx] == 0) {
            output[codeIdx] = code;
            code = 1;
            codeIdx = writeIdx++;
        } else {
            output[writeIdx++] = input[readIdx];
            code++;
        }
        readIdx++;
    }
    output[codeIdx] = code;
    
    TEST_ASSERT_EQUAL_UINT8(0x02, output[0]);  // First code
}

// ============================================================================
// ALTITUDE CALCULATION TEST
// ============================================================================

void test_pressure_to_altitude() {
    float pressure = 101325.0f;  // Sea level pressure
    float sea_level_pressure = 101325.0f;
    float temperature = 288.15f;  // Standard temp
    
    // Hypsometric formula
    float altitude = (temperature / 0.0065f) * 
                     (1.0f - powf(pressure / sea_level_pressure, 0.190284f));
    
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, altitude);
}

void test_altitude_increases_with_lower_pressure() {
    float sea_level_pressure = 101325.0f;
    float high_pressure = 100000.0f;
    float low_pressure = 90000.0f;
    float temperature = 288.15f;
    
    float alt_high_p = (temperature / 0.0065f) * 
                       (1.0f - powf(high_pressure / sea_level_pressure, 0.190284f));
    float alt_low_p = (temperature / 0.0065f) * 
                      (1.0f - powf(low_pressure / sea_level_pressure, 0.190284f));
    
    TEST_ASSERT_TRUE(alt_low_p > alt_high_p);
}

// ============================================================================
// SAFETY LIMIT TESTS
// ============================================================================

void test_battery_voltage_limits() {
    float voltage = 6.5f;
    float critical_threshold = 6.8f;
    float low_threshold = 7.2f;
    
    bool is_critical = voltage < critical_threshold;
    bool is_low = voltage < low_threshold;
    
    TEST_ASSERT_TRUE(is_critical);
    TEST_ASSERT_TRUE(is_low);
}

void test_tilt_angle_limit() {
    float roll = 0.3f;
    float pitch = 0.4f;
    float tilt = sqrtf(roll * roll + pitch * pitch);
    float max_tilt = 0.785f;  // 45 degrees
    
    TEST_ASSERT_TRUE(tilt < max_tilt);
}

// ============================================================================
// MAIN
// ============================================================================

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Quaternion tests
    RUN_TEST(test_quaternion_identity);
    RUN_TEST(test_quaternion_normalization);
    RUN_TEST(test_quaternion_to_euler_identity);
    
    // PID tests
    RUN_TEST(test_pid_proportional);
    RUN_TEST(test_pid_integral_accumulates);
    RUN_TEST(test_pid_output_limits);
    
    // Angle math tests
    RUN_TEST(test_angle_wrap_positive);
    RUN_TEST(test_angle_wrap_negative);
    RUN_TEST(test_deg_rad_conversion);
    
    // Filter tests
    RUN_TEST(test_low_pass_filter);
    RUN_TEST(test_complementary_filter);
    
    // Protocol tests
    RUN_TEST(test_crc16_ccitt);
    RUN_TEST(test_cobs_encode);
    
    // Physics tests
    RUN_TEST(test_pressure_to_altitude);
    RUN_TEST(test_altitude_increases_with_lower_pressure);
    
    // Safety tests
    RUN_TEST(test_battery_voltage_limits);
    RUN_TEST(test_tilt_angle_limit);
    
    return UNITY_END();
}
