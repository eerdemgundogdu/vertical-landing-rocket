// ============================================================================
// BMI088 IMU DRIVER - IMPLEMENTATION
// ============================================================================

#include "imu.h"
#include "config.h"
#include "constants.h"

BMI088::BMI088(uint8_t csAcc, uint8_t csGyro, SPIClass& spi)
    : _csAcc(csAcc)
    , _csGyro(csGyro)
    , _spi(spi)
    , _accelScale(0.0f)
    , _gyroScale(0.0f)
    , _accelBiasX(0.0f), _accelBiasY(0.0f), _accelBiasZ(0.0f)
    , _gyroBiasX(0.0f), _gyroBiasY(0.0f), _gyroBiasZ(0.0f)
    , _connected(false)
{
}

bool BMI088::begin() {
    pinMode(_csAcc, OUTPUT);
    pinMode(_csGyro, OUTPUT);
    digitalWrite(_csAcc, HIGH);
    digitalWrite(_csGyro, HIGH);
    
    _spi.begin();
    delay(50);
    
    // Soft reset accelerometer
    writeAccelReg(BMI088Reg::ACC_SOFTRESET, 0xB6);
    delay(50);
    
    // Soft reset gyroscope
    writeGyroReg(BMI088Reg::GYRO_SOFTRESET, 0xB6);
    delay(50);
    
    // Check accelerometer chip ID (requires dummy read on BMI088)
    readAccelReg(BMI088Reg::ACC_CHIP_ID);  // dummy read
    uint8_t accId = readAccelReg(BMI088Reg::ACC_CHIP_ID);
    if (accId != BMI088Reg::ACC_CHIP_ID_VAL) {
        return false;
    }
    
    // Check gyroscope chip ID
    uint8_t gyroId = readGyroReg(BMI088Reg::GYRO_CHIP_ID);
    if (gyroId != BMI088Reg::GYRO_CHIP_ID_VAL) {
        return false;
    }
    
    _connected = true;
    return configure();
}

bool BMI088::configure(AccelRange accelRange, GyroRange gyroRange,
                       AccelODR accelOdr, GyroODR gyroOdr) {
    if (!_connected) return false;
    
    // Configure accelerometer
    // Enable accelerometer (active mode, normal power)
    writeAccelReg(BMI088Reg::ACC_PWR_CONF, 0x00);  // Active mode
    delay(5);
    writeAccelReg(BMI088Reg::ACC_PWR_CTRL, 0x04);  // Enable accelerometer
    delay(50);
    
    // Set accelerometer range
    writeAccelReg(BMI088Reg::ACC_RANGE, static_cast<uint8_t>(accelRange));
    
    // Set accelerometer ODR and bandwidth (normal mode, OSR4)
    writeAccelReg(BMI088Reg::ACC_CONF, 0x80 | static_cast<uint8_t>(accelOdr));
    
    // Calculate accelerometer scale factor (LSB to m/s²)
    switch (accelRange) {
        case AccelRange::RANGE_3G:  _accelScale = 3.0f * GRAVITY / 32768.0f; break;
        case AccelRange::RANGE_6G:  _accelScale = 6.0f * GRAVITY / 32768.0f; break;
        case AccelRange::RANGE_12G: _accelScale = 12.0f * GRAVITY / 32768.0f; break;
        case AccelRange::RANGE_24G: _accelScale = 24.0f * GRAVITY / 32768.0f; break;
    }
    
    // Configure gyroscope
    // Set gyroscope range
    writeGyroReg(BMI088Reg::GYRO_RANGE, static_cast<uint8_t>(gyroRange));
    
    // Set gyroscope bandwidth
    writeGyroReg(BMI088Reg::GYRO_BANDWIDTH, static_cast<uint8_t>(gyroOdr));
    
    // Set gyroscope to normal mode
    writeGyroReg(BMI088Reg::GYRO_LPM1, 0x00);
    
    // Calculate gyroscope scale factor (LSB to rad/s)
    switch (gyroRange) {
        case GyroRange::RANGE_2000DPS: _gyroScale = 2000.0f * constants::DEG_TO_RAD / 32768.0f; break;
        case GyroRange::RANGE_1000DPS: _gyroScale = 1000.0f * constants::DEG_TO_RAD / 32768.0f; break;
        case GyroRange::RANGE_500DPS:  _gyroScale = 500.0f * constants::DEG_TO_RAD / 32768.0f; break;
        case GyroRange::RANGE_250DPS:  _gyroScale = 250.0f * constants::DEG_TO_RAD / 32768.0f; break;
        case GyroRange::RANGE_125DPS:  _gyroScale = 125.0f * constants::DEG_TO_RAD / 32768.0f; break;
    }
    
    delay(10);
    return true;
}

bool BMI088::selfTest() {
    // Simplified self-test: check if readings are reasonable
    ImuData data;
    if (!read(data)) return false;
    
    // Check accelerometer (should see ~1g on one axis at rest)
    float accelMag = sqrtf(data.accelX * data.accelX + 
                           data.accelY * data.accelY + 
                           data.accelZ * data.accelZ);
    if (accelMag < 8.0f || accelMag > 12.0f) return false;
    
    // Check gyroscope (should be near zero at rest)
    float gyroMag = sqrtf(data.gyroX * data.gyroX + 
                          data.gyroY * data.gyroY + 
                          data.gyroZ * data.gyroZ);
    if (gyroMag > 0.5f) return false;
    
    return true;
}

void BMI088::calibrate(uint16_t samples) {
    float accelSumX = 0, accelSumY = 0, accelSumZ = 0;
    float gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
    
    // Reset biases
    _accelBiasX = _accelBiasY = _accelBiasZ = 0;
    _gyroBiasX = _gyroBiasY = _gyroBiasZ = 0;
    
    // Collect samples
    uint16_t validSamples = 0;
    for (uint16_t i = 0; i < samples; i++) {
        ImuData data;
        if (read(data)) {
            accelSumX += data.accelX;
            accelSumY += data.accelY;
            accelSumZ += data.accelZ;
            gyroSumX += data.gyroX;
            gyroSumY += data.gyroY;
            gyroSumZ += data.gyroZ;
            validSamples++;
        }
        delayMicroseconds(1000);  // 1ms between samples
    }
    
    if (validSamples > 0) {
        // Calculate gyro bias (should be zero at rest)
        _gyroBiasX = gyroSumX / validSamples;
        _gyroBiasY = gyroSumY / validSamples;
        _gyroBiasZ = gyroSumZ / validSamples;
        
        // Calculate accel bias (assuming Z-up orientation, remove gravity)
        _accelBiasX = accelSumX / validSamples;
        _accelBiasY = accelSumY / validSamples;
        _accelBiasZ = (accelSumZ / validSamples) - GRAVITY;
    }
}

bool BMI088::read(ImuData& data) {
    uint8_t accBuffer[6];
    uint8_t gyroBuffer[6];
    
    // Read accelerometer (6 bytes starting at ACC_X_LSB)
    readAccelRegs(BMI088Reg::ACC_X_LSB, accBuffer, 6);
    
    // Read gyroscope (6 bytes starting at GYRO_X_LSB)
    readGyroRegs(BMI088Reg::GYRO_X_LSB, gyroBuffer, 6);
    
    // Parse accelerometer data (16-bit signed, LSB first)
    int16_t rawAccX = (int16_t)(accBuffer[1] << 8 | accBuffer[0]);
    int16_t rawAccY = (int16_t)(accBuffer[3] << 8 | accBuffer[2]);
    int16_t rawAccZ = (int16_t)(accBuffer[5] << 8 | accBuffer[4]);
    
    // Parse gyroscope data
    int16_t rawGyroX = (int16_t)(gyroBuffer[1] << 8 | gyroBuffer[0]);
    int16_t rawGyroY = (int16_t)(gyroBuffer[3] << 8 | gyroBuffer[2]);
    int16_t rawGyroZ = (int16_t)(gyroBuffer[5] << 8 | gyroBuffer[4]);
    
    // Convert to physical units and apply calibration
    data.accelX = rawAccX * _accelScale - _accelBiasX;
    data.accelY = rawAccY * _accelScale - _accelBiasY;
    data.accelZ = rawAccZ * _accelScale - _accelBiasZ;
    
    data.gyroX = rawGyroX * _gyroScale - _gyroBiasX;
    data.gyroY = rawGyroY * _gyroScale - _gyroBiasY;
    data.gyroZ = rawGyroZ * _gyroScale - _gyroBiasZ;
    
    data.timestamp = micros();
    data.valid = true;
    
    return true;
}

bool BMI088::readAccel(float& x, float& y, float& z) {
    uint8_t buffer[6];
    readAccelRegs(BMI088Reg::ACC_X_LSB, buffer, 6);
    
    int16_t rawX = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t rawY = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t rawZ = (int16_t)(buffer[5] << 8 | buffer[4]);
    
    x = rawX * _accelScale - _accelBiasX;
    y = rawY * _accelScale - _accelBiasY;
    z = rawZ * _accelScale - _accelBiasZ;
    
    return true;
}

bool BMI088::readGyro(float& x, float& y, float& z) {
    uint8_t buffer[6];
    readGyroRegs(BMI088Reg::GYRO_X_LSB, buffer, 6);
    
    int16_t rawX = (int16_t)(buffer[1] << 8 | buffer[0]);
    int16_t rawY = (int16_t)(buffer[3] << 8 | buffer[2]);
    int16_t rawZ = (int16_t)(buffer[5] << 8 | buffer[4]);
    
    x = rawX * _gyroScale - _gyroBiasX;
    y = rawY * _gyroScale - _gyroBiasY;
    z = rawZ * _gyroScale - _gyroBiasZ;
    
    return true;
}

void BMI088::setAccelBias(float x, float y, float z) {
    _accelBiasX = x;
    _accelBiasY = y;
    _accelBiasZ = z;
}

void BMI088::setGyroBias(float x, float y, float z) {
    _gyroBiasX = x;
    _gyroBiasY = y;
    _gyroBiasZ = z;
}

// Private SPI functions
uint8_t BMI088::readAccelReg(uint8_t reg) {
    uint8_t value;
    _spi.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(_csAcc, LOW);
    _spi.transfer(reg | 0x80);  // Read bit
    _spi.transfer(0x00);        // Dummy byte for BMI088 accelerometer
    value = _spi.transfer(0x00);
    digitalWrite(_csAcc, HIGH);
    _spi.endTransaction();
    return value;
}

void BMI088::writeAccelReg(uint8_t reg, uint8_t value) {
    _spi.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(_csAcc, LOW);
    _spi.transfer(reg);
    _spi.transfer(value);
    digitalWrite(_csAcc, HIGH);
    _spi.endTransaction();
}

uint8_t BMI088::readGyroReg(uint8_t reg) {
    uint8_t value;
    _spi.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(_csGyro, LOW);
    _spi.transfer(reg | 0x80);
    value = _spi.transfer(0x00);
    digitalWrite(_csGyro, HIGH);
    _spi.endTransaction();
    return value;
}

void BMI088::writeGyroReg(uint8_t reg, uint8_t value) {
    _spi.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(_csGyro, LOW);
    _spi.transfer(reg);
    _spi.transfer(value);
    digitalWrite(_csGyro, HIGH);
    _spi.endTransaction();
}

void BMI088::readAccelRegs(uint8_t reg, uint8_t* data, uint8_t len) {
    _spi.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(_csAcc, LOW);
    _spi.transfer(reg | 0x80);
    _spi.transfer(0x00);  // Dummy byte
    for (uint8_t i = 0; i < len; i++) {
        data[i] = _spi.transfer(0x00);
    }
    digitalWrite(_csAcc, HIGH);
    _spi.endTransaction();
}

void BMI088::readGyroRegs(uint8_t reg, uint8_t* data, uint8_t len) {
    _spi.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    digitalWrite(_csGyro, LOW);
    _spi.transfer(reg | 0x80);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = _spi.transfer(0x00);
    }
    digitalWrite(_csGyro, HIGH);
    _spi.endTransaction();
}
