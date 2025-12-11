#pragma once

// ============================================================================
// BMI088 IMU DRIVER
// ============================================================================
// Driver for Bosch BMI088 6-DOF IMU (Accelerometer + Gyroscope)
// Uses SPI interface for high-speed data acquisition
// ============================================================================

#include <Arduino.h>
#include <SPI.h>

// BMI088 Register definitions
namespace BMI088Reg {
    // Accelerometer registers
    constexpr uint8_t ACC_CHIP_ID = 0x00;
    constexpr uint8_t ACC_ERR_REG = 0x02;
    constexpr uint8_t ACC_STATUS = 0x03;
    constexpr uint8_t ACC_X_LSB = 0x12;
    constexpr uint8_t ACC_X_MSB = 0x13;
    constexpr uint8_t ACC_Y_LSB = 0x14;
    constexpr uint8_t ACC_Y_MSB = 0x15;
    constexpr uint8_t ACC_Z_LSB = 0x16;
    constexpr uint8_t ACC_Z_MSB = 0x17;
    constexpr uint8_t ACC_CONF = 0x40;
    constexpr uint8_t ACC_RANGE = 0x41;
    constexpr uint8_t ACC_PWR_CONF = 0x7C;
    constexpr uint8_t ACC_PWR_CTRL = 0x7D;
    constexpr uint8_t ACC_SOFTRESET = 0x7E;
    
    // Gyroscope registers
    constexpr uint8_t GYRO_CHIP_ID = 0x00;
    constexpr uint8_t GYRO_X_LSB = 0x02;
    constexpr uint8_t GYRO_X_MSB = 0x03;
    constexpr uint8_t GYRO_Y_LSB = 0x04;
    constexpr uint8_t GYRO_Y_MSB = 0x05;
    constexpr uint8_t GYRO_Z_LSB = 0x06;
    constexpr uint8_t GYRO_Z_MSB = 0x07;
    constexpr uint8_t GYRO_RANGE = 0x0F;
    constexpr uint8_t GYRO_BANDWIDTH = 0x10;
    constexpr uint8_t GYRO_LPM1 = 0x11;
    constexpr uint8_t GYRO_SOFTRESET = 0x14;
    
    // Chip IDs
    constexpr uint8_t ACC_CHIP_ID_VAL = 0x1E;
    constexpr uint8_t GYRO_CHIP_ID_VAL = 0x0F;
}

// Accelerometer range settings
enum class AccelRange : uint8_t {
    RANGE_3G = 0x00,   // ±3g
    RANGE_6G = 0x01,   // ±6g
    RANGE_12G = 0x02,  // ±12g
    RANGE_24G = 0x03   // ±24g
};

// Gyroscope range settings
enum class GyroRange : uint8_t {
    RANGE_2000DPS = 0x00,  // ±2000°/s
    RANGE_1000DPS = 0x01,  // ±1000°/s
    RANGE_500DPS = 0x02,   // ±500°/s
    RANGE_250DPS = 0x03,   // ±250°/s
    RANGE_125DPS = 0x04    // ±125°/s
};

// Accelerometer ODR settings
enum class AccelODR : uint8_t {
    ODR_12_5HZ = 0x05,
    ODR_25HZ = 0x06,
    ODR_50HZ = 0x07,
    ODR_100HZ = 0x08,
    ODR_200HZ = 0x09,
    ODR_400HZ = 0x0A,
    ODR_800HZ = 0x0B,
    ODR_1600HZ = 0x0C
};

// Gyroscope ODR settings
enum class GyroODR : uint8_t {
    ODR_2000HZ = 0x00,  // 532Hz bandwidth
    ODR_2000HZ_230HZ = 0x01,  // 230Hz bandwidth
    ODR_1000HZ = 0x02,  // 116Hz bandwidth
    ODR_400HZ = 0x03,   // 47Hz bandwidth
    ODR_200HZ = 0x04,   // 23Hz bandwidth
    ODR_100HZ = 0x05,   // 12Hz bandwidth
    ODR_200HZ_64HZ = 0x06,  // 64Hz bandwidth
    ODR_100HZ_32HZ = 0x07   // 32Hz bandwidth
};

// IMU data structure
struct ImuData {
    float accelX;      // m/s²
    float accelY;
    float accelZ;
    float gyroX;       // rad/s
    float gyroY;
    float gyroZ;
    float temperature; // °C
    uint32_t timestamp;
    bool valid;
};

class BMI088 {
public:
    BMI088(uint8_t csAcc, uint8_t csGyro, SPIClass& spi = SPI);
    
    bool begin();
    bool configure(AccelRange accelRange = AccelRange::RANGE_24G,
                   GyroRange gyroRange = GyroRange::RANGE_2000DPS,
                   AccelODR accelOdr = AccelODR::ODR_1600HZ,
                   GyroODR gyroOdr = GyroODR::ODR_2000HZ);
    
    bool selfTest();
    void calibrate(uint16_t samples = 500);
    
    bool read(ImuData& data);
    bool readAccel(float& x, float& y, float& z);
    bool readGyro(float& x, float& y, float& z);
    
    void setAccelBias(float x, float y, float z);
    void setGyroBias(float x, float y, float z);
    
    bool isConnected() const { return _connected; }

private:
    uint8_t readAccelReg(uint8_t reg);
    void writeAccelReg(uint8_t reg, uint8_t value);
    uint8_t readGyroReg(uint8_t reg);
    void writeGyroReg(uint8_t reg, uint8_t value);
    void readAccelRegs(uint8_t reg, uint8_t* data, uint8_t len);
    void readGyroRegs(uint8_t reg, uint8_t* data, uint8_t len);
    
    uint8_t _csAcc;
    uint8_t _csGyro;
    SPIClass& _spi;
    
    float _accelScale;
    float _gyroScale;
    float _accelBiasX, _accelBiasY, _accelBiasZ;
    float _gyroBiasX, _gyroBiasY, _gyroBiasZ;
    
    bool _connected;
    
    static constexpr uint32_t SPI_CLOCK = 10000000;  // 10 MHz
};
