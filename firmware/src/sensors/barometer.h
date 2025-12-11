#pragma once

// ============================================================================
// BMP390 BAROMETER DRIVER
// ============================================================================
// Driver for Bosch BMP390 barometric pressure sensor
// Uses I2C interface for altitude measurement
// ============================================================================

#include <Arduino.h>
#include <Wire.h>

// BMP390 Register definitions
namespace BMP390Reg {
    constexpr uint8_t CHIP_ID = 0x00;
    constexpr uint8_t ERR_REG = 0x02;
    constexpr uint8_t STATUS = 0x03;
    constexpr uint8_t DATA_0 = 0x04;  // Pressure XLSB
    constexpr uint8_t DATA_1 = 0x05;  // Pressure LSB
    constexpr uint8_t DATA_2 = 0x06;  // Pressure MSB
    constexpr uint8_t DATA_3 = 0x07;  // Temperature XLSB
    constexpr uint8_t DATA_4 = 0x08;  // Temperature LSB
    constexpr uint8_t DATA_5 = 0x09;  // Temperature MSB
    constexpr uint8_t EVENT = 0x10;
    constexpr uint8_t INT_STATUS = 0x11;
    constexpr uint8_t INT_CTRL = 0x19;
    constexpr uint8_t IF_CONF = 0x1A;
    constexpr uint8_t PWR_CTRL = 0x1B;
    constexpr uint8_t OSR = 0x1C;
    constexpr uint8_t ODR = 0x1D;
    constexpr uint8_t CONFIG = 0x1F;
    constexpr uint8_t CMD = 0x7E;
    
    // Calibration data registers (NVM)
    constexpr uint8_t NVM_PAR_T1_L = 0x31;
    constexpr uint8_t NVM_PAR_T1_H = 0x32;
    constexpr uint8_t NVM_PAR_T2_L = 0x33;
    constexpr uint8_t NVM_PAR_T2_H = 0x34;
    constexpr uint8_t NVM_PAR_T3 = 0x35;
    constexpr uint8_t NVM_PAR_P1_L = 0x36;
    constexpr uint8_t NVM_PAR_P1_H = 0x37;
    constexpr uint8_t NVM_PAR_P2_L = 0x38;
    constexpr uint8_t NVM_PAR_P2_H = 0x39;
    constexpr uint8_t NVM_PAR_P3 = 0x3A;
    constexpr uint8_t NVM_PAR_P4 = 0x3B;
    constexpr uint8_t NVM_PAR_P5_L = 0x3C;
    constexpr uint8_t NVM_PAR_P5_H = 0x3D;
    constexpr uint8_t NVM_PAR_P6_L = 0x3E;
    constexpr uint8_t NVM_PAR_P6_H = 0x3F;
    constexpr uint8_t NVM_PAR_P7 = 0x40;
    constexpr uint8_t NVM_PAR_P8 = 0x41;
    constexpr uint8_t NVM_PAR_P9_L = 0x42;
    constexpr uint8_t NVM_PAR_P9_H = 0x43;
    constexpr uint8_t NVM_PAR_P10 = 0x44;
    constexpr uint8_t NVM_PAR_P11 = 0x45;
    
    constexpr uint8_t CHIP_ID_VAL = 0x60;
}

// Oversampling settings
enum class BaroOversampling : uint8_t {
    OSR_1X = 0x00,
    OSR_2X = 0x01,
    OSR_4X = 0x02,
    OSR_8X = 0x03,
    OSR_16X = 0x04,
    OSR_32X = 0x05
};

// Output data rate
enum class BaroODR : uint8_t {
    ODR_200HZ = 0x00,
    ODR_100HZ = 0x01,
    ODR_50HZ = 0x02,
    ODR_25HZ = 0x03,
    ODR_12_5HZ = 0x04,
    ODR_6_25HZ = 0x05,
    ODR_3_1HZ = 0x06,
    ODR_1_5HZ = 0x07
};

// IIR filter coefficient
enum class BaroFilter : uint8_t {
    FILTER_OFF = 0x00,
    FILTER_COEF_1 = 0x01,
    FILTER_COEF_3 = 0x02,
    FILTER_COEF_7 = 0x03,
    FILTER_COEF_15 = 0x04,
    FILTER_COEF_31 = 0x05,
    FILTER_COEF_63 = 0x06,
    FILTER_COEF_127 = 0x07
};

// Barometer data structure
struct BaroData {
    float pressure;     // Pa
    float temperature;  // °C
    float altitude;     // m (relative to sea level or reference)
    uint32_t timestamp;
    bool valid;
};

class BMP390 {
public:
    BMP390(TwoWire& wire = Wire, uint8_t address = 0x76);
    
    bool begin();
    bool configure(BaroOversampling pressOsr = BaroOversampling::OSR_8X,
                   BaroOversampling tempOsr = BaroOversampling::OSR_2X,
                   BaroODR odr = BaroODR::ODR_100HZ,
                   BaroFilter filter = BaroFilter::FILTER_COEF_3);
    
    bool read(BaroData& data);
    float readPressure();
    float readTemperature();
    float readAltitude();
    
    void setSeaLevelPressure(float pressure);
    float getSeaLevelPressure() const { return _seaLevelPressure; }
    
    void setReferenceAltitude(float altitude);
    
    bool isConnected() const { return _connected; }

private:
    bool readCalibration();
    float compensateTemperature(uint32_t rawTemp);
    float compensatePressure(uint32_t rawPress);
    
    uint8_t readReg(uint8_t reg);
    void writeReg(uint8_t reg, uint8_t value);
    void readRegs(uint8_t reg, uint8_t* data, uint8_t len);
    
    TwoWire& _wire;
    uint8_t _address;
    bool _connected;
    
    float _seaLevelPressure;
    float _referenceAltitude;
    
    // Calibration coefficients
    struct {
        float T1, T2, T3;
        float P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
    } _calib;
    
    // Temperature for pressure compensation
    float _tLin;
};
