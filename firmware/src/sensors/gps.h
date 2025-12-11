#pragma once

// ============================================================================
// GPS DRIVER
// ============================================================================
// Parser for u-blox NEO-M9N GPS module using TinyGPS++ library
// ============================================================================

#include <Arduino.h>
#include <TinyGPS++.h>

struct GpsData {
    double latitude;      // Degrees
    double longitude;     // Degrees
    float altitudeMsl;    // Meters above sea level
    float groundSpeed;    // m/s
    float course;         // Degrees from north
    float velNorth;       // m/s
    float velEast;        // m/s
    float velDown;        // m/s (derived from altitude change)
    uint8_t fixType;      // 0=no fix, 2=2D, 3=3D
    uint8_t satellites;   // Number of satellites
    float hdop;           // Horizontal dilution of precision
    uint32_t timestamp;
    bool valid;
};

class GPS {
public:
    GPS(HardwareSerial& serial);
    
    void begin(uint32_t baudRate = 38400);
    void update();
    
    bool read(GpsData& data);
    bool hasNewData() const { return _hasNewData; }
    
    bool isConnected() const { return _connected; }
    uint32_t getAge() const { return millis() - _lastUpdate; }
    
    // Configure GPS module (u-blox specific)
    void setUpdateRate(uint16_t rateHz);
    void setNavigationMode(uint8_t mode);  // 0=Portable, 6=Airborne<1g, 7=Airborne<2g, 8=Airborne<4g
    
private:
    void processNMEA();
    void sendUBX(const uint8_t* msg, uint8_t len);
    
    HardwareSerial& _serial;
    TinyGPSPlus _gps;
    
    bool _connected;
    bool _hasNewData;
    uint32_t _lastUpdate;
    
    float _lastAltitude;
    uint32_t _lastAltTime;
    float _verticalVel;
    
    GpsData _latestData;
};
