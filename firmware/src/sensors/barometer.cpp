// ============================================================================
// BMP390 BAROMETER DRIVER - IMPLEMENTATION
// ============================================================================

#include "barometer.h"
#include "constants.h"

BMP390::BMP390(TwoWire& wire, uint8_t address)
    : _wire(wire)
    , _address(address)
    , _connected(false)
    , _seaLevelPressure(constants::SEA_LEVEL_PRESSURE)
    , _referenceAltitude(0.0f)
    , _tLin(0.0f)
{
}

bool BMP390::begin() {
    _wire.begin();
    _wire.setClock(400000);  // 400kHz I2C
    delay(10);
    
    // Check chip ID
    uint8_t id = readReg(BMP390Reg::CHIP_ID);
    if (id != BMP390Reg::CHIP_ID_VAL) {
        return false;
    }
    
    // Soft reset
    writeReg(BMP390Reg::CMD, 0xB6);
    delay(10);
    
    // Read calibration data
    if (!readCalibration()) {
        return false;
    }
    
    _connected = true;
    return configure();
}

bool BMP390::configure(BaroOversampling pressOsr, BaroOversampling tempOsr,
                       BaroODR odr, BaroFilter filter) {
    if (!_connected) return false;
    
    // Set oversampling for pressure and temperature
    uint8_t osrVal = (static_cast<uint8_t>(tempOsr) << 3) | static_cast<uint8_t>(pressOsr);
    writeReg(BMP390Reg::OSR, osrVal);
    
    // Set output data rate
    writeReg(BMP390Reg::ODR, static_cast<uint8_t>(odr));
    
    // Set IIR filter
    writeReg(BMP390Reg::CONFIG, static_cast<uint8_t>(filter) << 1);
    
    // Enable pressure and temperature measurement in normal mode
    writeReg(BMP390Reg::PWR_CTRL, 0x33);  // Press_en, Temp_en, Normal mode
    
    delay(10);
    return true;
}

bool BMP390::readCalibration() {
    uint8_t data[21];
    readRegs(BMP390Reg::NVM_PAR_T1_L, data, 21);
    
    // Parse temperature calibration
    uint16_t t1 = (uint16_t)(data[1] << 8 | data[0]);
    uint16_t t2 = (uint16_t)(data[3] << 8 | data[2]);
    int8_t t3 = (int8_t)data[4];
    
    // Parse pressure calibration
    int16_t p1 = (int16_t)(data[6] << 8 | data[5]);
    int16_t p2 = (int16_t)(data[8] << 8 | data[7]);
    int8_t p3 = (int8_t)data[9];
    int8_t p4 = (int8_t)data[10];
    uint16_t p5 = (uint16_t)(data[12] << 8 | data[11]);
    uint16_t p6 = (uint16_t)(data[14] << 8 | data[13]);
    int8_t p7 = (int8_t)data[15];
    int8_t p8 = (int8_t)data[16];
    int16_t p9 = (int16_t)(data[18] << 8 | data[17]);
    int8_t p10 = (int8_t)data[19];
    int8_t p11 = (int8_t)data[20];
    
    // Convert to float coefficients
    _calib.T1 = (float)t1 / powf(2, -8);
    _calib.T2 = (float)t2 / powf(2, 30);
    _calib.T3 = (float)t3 / powf(2, 48);
    
    _calib.P1 = ((float)p1 - powf(2, 14)) / powf(2, 20);
    _calib.P2 = ((float)p2 - powf(2, 14)) / powf(2, 29);
    _calib.P3 = (float)p3 / powf(2, 32);
    _calib.P4 = (float)p4 / powf(2, 37);
    _calib.P5 = (float)p5 / powf(2, -3);
    _calib.P6 = (float)p6 / powf(2, 6);
    _calib.P7 = (float)p7 / powf(2, 8);
    _calib.P8 = (float)p8 / powf(2, 15);
    _calib.P9 = (float)p9 / powf(2, 48);
    _calib.P10 = (float)p10 / powf(2, 48);
    _calib.P11 = (float)p11 / powf(2, 65);
    
    return true;
}

float BMP390::compensateTemperature(uint32_t rawTemp) {
    float partial1 = (float)rawTemp - _calib.T1;
    float partial2 = partial1 * _calib.T2;
    _tLin = partial2 + (partial1 * partial1) * _calib.T3;
    return _tLin;
}

float BMP390::compensatePressure(uint32_t rawPress) {
    float pd1 = _calib.P6 * _tLin;
    float pd2 = _calib.P7 * (_tLin * _tLin);
    float pd3 = _calib.P8 * (_tLin * _tLin * _tLin);
    float po1 = _calib.P5 + pd1 + pd2 + pd3;
    
    pd1 = _calib.P2 * _tLin;
    pd2 = _calib.P3 * (_tLin * _tLin);
    pd3 = _calib.P4 * (_tLin * _tLin * _tLin);
    float po2 = (float)rawPress * (_calib.P1 + pd1 + pd2 + pd3);
    
    pd1 = (float)rawPress * (float)rawPress;
    pd2 = _calib.P9 + _calib.P10 * _tLin;
    pd3 = pd1 * pd2;
    float pd4 = pd3 + ((float)rawPress * (float)rawPress * (float)rawPress) * _calib.P11;
    
    return po1 + po2 + pd4;
}

bool BMP390::read(BaroData& data) {
    // Check if data is ready
    uint8_t status = readReg(BMP390Reg::STATUS);
    if ((status & 0x60) != 0x60) {  // Check drdy_press and drdy_temp
        data.valid = false;
        return false;
    }
    
    // Read pressure and temperature data (6 bytes)
    uint8_t rawData[6];
    readRegs(BMP390Reg::DATA_0, rawData, 6);
    
    uint32_t rawPress = (uint32_t)(rawData[2] << 16 | rawData[1] << 8 | rawData[0]);
    uint32_t rawTemp = (uint32_t)(rawData[5] << 16 | rawData[4] << 8 | rawData[3]);
    
    // Compensate
    data.temperature = compensateTemperature(rawTemp);
    data.pressure = compensatePressure(rawPress);
    
    // Calculate altitude using barometric formula
    data.altitude = constants::pressureToAltitude(data.pressure, _seaLevelPressure) 
                    - _referenceAltitude;
    
    data.timestamp = micros();
    data.valid = true;
    
    return true;
}

float BMP390::readPressure() {
    BaroData data;
    if (read(data)) {
        return data.pressure;
    }
    return 0.0f;
}

float BMP390::readTemperature() {
    BaroData data;
    if (read(data)) {
        return data.temperature;
    }
    return 0.0f;
}

float BMP390::readAltitude() {
    BaroData data;
    if (read(data)) {
        return data.altitude;
    }
    return 0.0f;
}

void BMP390::setSeaLevelPressure(float pressure) {
    _seaLevelPressure = pressure;
}

void BMP390::setReferenceAltitude(float altitude) {
    _referenceAltitude = altitude;
}

uint8_t BMP390::readReg(uint8_t reg) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(_address, (uint8_t)1);
    return _wire.read();
}

void BMP390::writeReg(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(value);
    _wire.endTransmission();
}

void BMP390::readRegs(uint8_t reg, uint8_t* data, uint8_t len) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.endTransmission();
    _wire.requestFrom(_address, len);
    for (uint8_t i = 0; i < len && _wire.available(); i++) {
        data[i] = _wire.read();
    }
}
