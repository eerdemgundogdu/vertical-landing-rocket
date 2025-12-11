// ============================================================================
// GPS DRIVER - IMPLEMENTATION
// ============================================================================

#include "gps.h"
#include "constants.h"

GPS::GPS(HardwareSerial& serial)
    : _serial(serial)
    , _connected(false)
    , _hasNewData(false)
    , _lastUpdate(0)
    , _lastAltitude(0.0f)
    , _lastAltTime(0)
    , _verticalVel(0.0f)
{
}

void GPS::begin(uint32_t baudRate) {
    _serial.begin(baudRate);
    delay(100);
    
    // Configure for airborne mode and higher update rate
    setNavigationMode(8);  // Airborne <4g
    setUpdateRate(25);     // 25 Hz
    
    _lastUpdate = millis();
}

void GPS::update() {
    while (_serial.available() > 0) {
        char c = _serial.read();
        if (_gps.encode(c)) {
            processNMEA();
        }
    }
}

void GPS::processNMEA() {
    if (_gps.location.isUpdated() || _gps.altitude.isUpdated()) {
        _hasNewData = true;
        _lastUpdate = millis();
        _connected = true;
        
        // Update latest data
        if (_gps.location.isValid()) {
            _latestData.latitude = _gps.location.lat();
            _latestData.longitude = _gps.location.lng();
            _latestData.valid = true;
        }
        
        if (_gps.altitude.isValid()) {
            float newAlt = _gps.altitude.meters();
            uint32_t now = millis();
            
            // Calculate vertical velocity
            if (_lastAltTime > 0 && (now - _lastAltTime) > 0) {
                float dt = (now - _lastAltTime) * 0.001f;
                _verticalVel = (newAlt - _lastAltitude) / dt;
            }
            
            _lastAltitude = newAlt;
            _lastAltTime = now;
            _latestData.altitudeMsl = newAlt;
            _latestData.velDown = -_verticalVel;  // NED convention
        }
        
        if (_gps.speed.isValid()) {
            _latestData.groundSpeed = _gps.speed.mps();
        }
        
        if (_gps.course.isValid()) {
            _latestData.course = _gps.course.deg();
            
            // Decompose into North/East components
            float courseRad = _latestData.course * constants::DEG_TO_RAD;
            _latestData.velNorth = _latestData.groundSpeed * cosf(courseRad);
            _latestData.velEast = _latestData.groundSpeed * sinf(courseRad);
        }
        
        _latestData.satellites = _gps.satellites.value();
        _latestData.hdop = _gps.hdop.hdop();
        
        // Determine fix type based on satellites and HDOP
        if (_latestData.satellites >= 4 && _latestData.hdop < 5.0f) {
            _latestData.fixType = 3;  // 3D fix
        } else if (_latestData.satellites >= 3) {
            _latestData.fixType = 2;  // 2D fix
        } else {
            _latestData.fixType = 0;  // No fix
        }
        
        _latestData.timestamp = micros();
    }
}

bool GPS::read(GpsData& data) {
    if (_hasNewData) {
        data = _latestData;
        _hasNewData = false;
        return true;
    }
    return false;
}

void GPS::setUpdateRate(uint16_t rateHz) {
    // UBX-CFG-RATE message
    uint16_t measRate = 1000 / rateHz;  // Measurement period in ms
    uint8_t msg[] = {
        0xB5, 0x62,             // Sync chars
        0x06, 0x08,             // Class, ID (CFG-RATE)
        0x06, 0x00,             // Length
        (uint8_t)(measRate & 0xFF), (uint8_t)(measRate >> 8),  // measRate
        0x01, 0x00,             // navRate (1 = single measurement)
        0x01, 0x00,             // timeRef (1 = GPS time)
        0x00, 0x00              // Checksum (calculated below)
    };
    
    // Calculate checksum
    uint8_t ckA = 0, ckB = 0;
    for (int i = 2; i < 12; i++) {
        ckA += msg[i];
        ckB += ckA;
    }
    msg[12] = ckA;
    msg[13] = ckB;
    
    sendUBX(msg, sizeof(msg));
}

void GPS::setNavigationMode(uint8_t mode) {
    // UBX-CFG-NAV5 message
    uint8_t msg[] = {
        0xB5, 0x62,             // Sync chars
        0x06, 0x24,             // Class, ID (CFG-NAV5)
        0x24, 0x00,             // Length (36 bytes)
        0xFF, 0xFF,             // Mask (apply all)
        mode,                   // dynModel
        0x03,                   // fixMode (auto 2D/3D)
        0x00, 0x00, 0x00, 0x00, // fixedAlt
        0x10, 0x27, 0x00, 0x00, // fixedAltVar
        0x05,                   // minElev
        0x00,                   // drLimit
        0xFA, 0x00,             // pDop
        0xFA, 0x00,             // tDop
        0x64, 0x00,             // pAcc
        0x2C, 0x01,             // tAcc
        0x00,                   // staticHoldThresh
        0x00,                   // dgnssTimeout
        0x00,                   // cnoThreshNumSVs
        0x00,                   // cnoThresh
        0x00, 0x00,             // reserved
        0x00, 0x00,             // staticHoldMaxDist
        0x00,                   // utcStandard
        0x00, 0x00, 0x00, 0x00, 0x00, // reserved
        0x00, 0x00              // Checksum
    };
    
    // Calculate checksum
    uint8_t ckA = 0, ckB = 0;
    for (size_t i = 2; i < sizeof(msg) - 2; i++) {
        ckA += msg[i];
        ckB += ckA;
    }
    msg[sizeof(msg) - 2] = ckA;
    msg[sizeof(msg) - 1] = ckB;
    
    sendUBX(msg, sizeof(msg));
}

void GPS::sendUBX(const uint8_t* msg, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        _serial.write(msg[i]);
    }
}
