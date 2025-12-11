// ============================================================================
// VERTICAL LANDING ROCKET - MAIN FIRMWARE
// ============================================================================
// Teensy 4.1 Flight Controller
// 1000Hz control loop with sensor fusion and TVC
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "config.h"
#include "pins.h"
#include "constants.h"

#include "sensors/imu.h"
#include "sensors/barometer.h"
#include "sensors/gps.h"
#include "estimation/kalman.h"
#include "control/pid.h"
#include "control/attitude.h"
#include "control/tvc.h"
#include "state/flight_state.h"
#include "comms/protocol.h"
#include "storage/logger.h"

// Global objects
BMI088 imu(PIN_IMU_CS_ACC, PIN_IMU_CS_GYRO);
BMP390 barometer;
GPS gps(SERIAL_GPS);
ExtendedKalmanFilter ekf;
AttitudeController attitudeCtrl;
TVC tvc;
FlightStateMachine stateMachine;
SerialProtocol protocolJetson(SERIAL_JETSON);
DataLogger logger;

// Timing
IntervalTimer controlTimer;
volatile bool controlFlag = false;
uint32_t lastTelemetryTime = 0;
uint32_t lastStatusTime = 0;
uint32_t lastBaroTime = 0;
uint32_t loopCounter = 0;

// Sensor data
ImuData imuData;
BaroData baroData;
GpsData gpsData;

// Battery monitoring
float batteryVoltage = 8.4f;

// Function prototypes
void controlLoopISR();
void initSensors();
void initCommunication();
void runControlLoop();
void sendTelemetry();
void handleCommands();
void updateBatteryVoltage();
void logFlightEvent(FlightEvent event);

// Command callbacks
void onArmCommand(uint32_t key);
void onDisarmCommand();
void onAbortCommand(uint8_t reason);
void onCalibrateCommand();
void onTargetCommand(float r, float p, float y, float t);
void onParamCommand(uint16_t id, float val);

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize serial ports
    Serial.begin(SERIAL_BAUD_RATE);
    SERIAL_JETSON.begin(SERIAL_BAUD_RATE);
    
    // Wait for serial (debug only)
    delay(100);
    Serial.println("=================================");
    Serial.println("VERTICAL LANDING ROCKET v1.0");
    Serial.println("=================================");
    
    // Initialize pins
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_ARM_SWITCH, INPUT_PULLDOWN);
    pinMode(PIN_IGNITER, OUTPUT);
    digitalWrite(PIN_IGNITER, LOW);
    
    // Status LED on during init
    digitalWrite(PIN_LED_STATUS, HIGH);
    
    // Initialize sensors
    initSensors();
    
    // Initialize state estimation
    ekf.init();
    
    // Initialize controllers
    attitudeCtrl.init();
    
    // Initialize TVC servos
    tvc.init();
    
    // Initialize state machine
    stateMachine.init();
    stateMachine.setEventCallback(logFlightEvent);
    
    // Initialize communication
    initCommunication();
    
    // Initialize SD card logger
    if (logger.begin()) {
        Serial.println("[OK] SD Card initialized");
        logger.logDebug("System booted");
    } else {
        Serial.println("[WARN] SD Card not found");
    }
    
    // Calibrate IMU (rocket must be stationary!)
    Serial.println("Calibrating sensors...");
    imu.calibrate(500);
    
    // Set barometer reference altitude
    float refAlt = 0;
    for (int i = 0; i < 100; i++) {
        if (barometer.read(baroData)) {
            refAlt += baroData.altitude;
        }
        delay(10);
    }
    barometer.setReferenceAltitude(refAlt / 100.0f);
    Serial.println("[OK] Sensors calibrated");
    
    // Start control loop timer (1000 Hz)
    controlTimer.begin(controlLoopISR, 1000);  // 1000us = 1ms = 1000Hz
    
    // Ready
    digitalWrite(PIN_LED_STATUS, LOW);
    Serial.println("System ready. Waiting for ARM command.");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
    // Run control loop when flag is set by timer
    if (controlFlag) {
        controlFlag = false;
        runControlLoop();
    }
    
    // Handle incoming commands
    handleCommands();
    
    // Update GPS (non-blocking)
    gps.update();
    
    // Send telemetry at lower rate
    uint32_t now = millis();
    
    if (now - lastTelemetryTime >= 1000 / TELEMETRY_FREQ_HZ) {
        lastTelemetryTime = now;
        sendTelemetry();
    }
    
    if (now - lastStatusTime >= 1000 / STATUS_FREQ_HZ) {
        lastStatusTime = now;
        updateBatteryVoltage();
        
        // Status LED blink pattern based on state
        static bool ledState = false;
        if (stateMachine.isArmed()) {
            ledState = !ledState;
            digitalWrite(PIN_LED_STATUS, ledState);
        } else if (stateMachine.isFlying()) {
            digitalWrite(PIN_LED_STATUS, HIGH);
        } else {
            digitalWrite(PIN_LED_STATUS, LOW);
        }
    }
}

// ============================================================================
// CONTROL LOOP (1000 Hz)
// ============================================================================

void controlLoopISR() {
    controlFlag = true;
}

void runControlLoop() {
    static uint32_t lastLoopTime = 0;
    uint32_t now = micros();
    float dt = (now - lastLoopTime) * constants::MICROS_TO_SEC;
    lastLoopTime = now;
    
    // Sanity check dt
    if (dt <= 0.0f || dt > 0.01f) {
        dt = 0.001f;
    }
    
    // Read IMU
    if (!imu.read(imuData)) {
        return;  // Skip this iteration if IMU read fails
    }
    
    // Read barometer (at lower rate)
    static uint8_t baroCounter = 0;
    if (++baroCounter >= 10) {  // 100 Hz
        baroCounter = 0;
        barometer.read(baroData);
    }
    
    // Process GPS if new data available
    if (gps.read(gpsData)) {
        if (gpsData.valid && gpsData.fixType >= 3) {
            float n, e, d;
            ekf.gpsToLocal(gpsData.latitude, gpsData.longitude, gpsData.altitudeMsl, n, e, d);
            ekf.correctGps(n, e, d, gpsData.velNorth, gpsData.velEast, gpsData.velDown);
        }
    }
    
    // Run state estimation
    ekf.predict(imuData.accelX, imuData.accelY, imuData.accelZ,
                imuData.gyroX, imuData.gyroY, imuData.gyroZ, dt);
    
    // Correct with barometer
    if (baroCounter == 0 && baroData.valid) {
        ekf.correctBarometer(baroData.altitude);
    }
    
    const KalmanState& state = ekf.getState();
    
    // Calculate acceleration magnitude and tilt for state machine
    float accelMag = sqrtf(imuData.accelX * imuData.accelX +
                           imuData.accelY * imuData.accelY +
                           imuData.accelZ * imuData.accelZ);
    float tiltAngle = sqrtf(state.roll * state.roll + state.pitch * state.pitch);
    uint32_t flightTime = stateMachine.isFlying() ? 
                          (millis() - stateMachine.getLaunchTime()) : 0;
    
    // Update state machine
    stateMachine.update(accelMag, state.altitudeAGL, state.velD,
                        tiltAngle, batteryVoltage, flightTime);
    
    // Run attitude controller if TVC should be active
    if (stateMachine.isTvcActive()) {
        TvcCommand cmd = attitudeCtrl.update(state.roll, state.pitch, state.yaw,
                                              state.gyroX, state.gyroY, state.gyroZ, dt);
        tvc.update(cmd.servoX, cmd.servoY);
    } else if (stateMachine.getState() == FlightState::ARMED) {
        // Centering servos when armed but not flying
        tvc.center();
    }
    
    // Log data if flying
    if (stateMachine.isFlying() || stateMachine.isArmed()) {
        if (loopCounter % 10 == 0) {  // 100 Hz logging
            logger.logImu(imuData.accelX, imuData.accelY, imuData.accelZ,
                          imuData.gyroX, imuData.gyroY, imuData.gyroZ);
        }
        if (loopCounter % 20 == 0) {  // 50 Hz state logging
            logger.logState(state.posN, state.posE, state.posD,
                            state.velN, state.velE, state.velD,
                            state.roll, state.pitch, state.yaw,
                            (uint8_t)stateMachine.getState());
        }
        if (loopCounter % 100 == 0 && baroData.valid) {  // 10 Hz baro logging
            logger.logBaro(baroData.pressure, baroData.altitude, baroData.temperature);
        }
    }
    
    loopCounter++;
}

// ============================================================================
// INITIALIZATION
// ============================================================================

void initSensors() {
    // Initialize SPI for IMU
    SPI.begin();
    
    // Initialize IMU
    if (imu.begin()) {
        Serial.println("[OK] IMU (BMI088) initialized");
        if (imu.selfTest()) {
            Serial.println("[OK] IMU self-test passed");
        } else {
            Serial.println("[WARN] IMU self-test failed");
        }
    } else {
        Serial.println("[ERROR] IMU initialization failed!");
    }
    
    // Initialize I2C for barometer
    Wire.begin();
    Wire.setClock(400000);
    
    // Initialize barometer
    if (barometer.begin()) {
        Serial.println("[OK] Barometer (BMP390) initialized");
    } else {
        Serial.println("[ERROR] Barometer initialization failed!");
    }
    
    // Initialize GPS
    gps.begin(38400);
    Serial.println("[OK] GPS (NEO-M9N) initialized");
}

void initCommunication() {
    protocolJetson.init();
    
    // Set command callbacks
    protocolJetson.setArmCallback(onArmCommand);
    protocolJetson.setDisarmCallback(onDisarmCommand);
    protocolJetson.setAbortCallback(onAbortCommand);
    protocolJetson.setCalibrateCallback(onCalibrateCommand);
    protocolJetson.setTargetCallback(onTargetCommand);
    protocolJetson.setParamCallback(onParamCommand);
    
    Serial.println("[OK] Communication initialized");
}

// ============================================================================
// TELEMETRY
// ============================================================================

void sendTelemetry() {
    const KalmanState& state = ekf.getState();
    
    // State telemetry
    TelemetryState telem;
    telem.timestampMs = millis();
    telem.posN = state.posN;
    telem.posE = state.posE;
    telem.posD = state.posD;
    telem.velN = state.velN;
    telem.velE = state.velE;
    telem.velD = state.velD;
    telem.qw = state.attitude.w;
    telem.qx = state.attitude.x;
    telem.qy = state.attitude.y;
    telem.qz = state.attitude.z;
    telem.gyroX = state.gyroX;
    telem.gyroY = state.gyroY;
    telem.gyroZ = state.gyroZ;
    
    protocolJetson.sendState(telem);
    
    // Status telemetry (less frequent)
    static uint8_t statusDiv = 0;
    if (++statusDiv >= 5) {
        statusDiv = 0;
        
        TelemetryStatus status;
        status.timestampMs = millis();
        status.flightState = (uint8_t)stateMachine.getState();
        status.armStatus = stateMachine.isArmed() ? 1 : 0;
        status.sensorStatus = (imu.isConnected() ? 0x01 : 0) |
                               (barometer.isConnected() ? 0x02 : 0) |
                               (gpsData.fixType >= 3 ? 0x04 : 0) |
                               (logger.isReady() ? 0x08 : 0);
        status.errorCode = (uint8_t)stateMachine.getAbortReason();
        status.batteryVoltage = batteryVoltage;
        status.cpuTemp = tempmonGetTemp();
        
        protocolJetson.sendStatus(status);
    }
}

void handleCommands() {
    protocolJetson.update();
}

// ============================================================================
// COMMAND HANDLERS
// ============================================================================

void onArmCommand(uint32_t key) {
    if (stateMachine.arm(key)) {
        logger.createNewFlight();
        logger.logDebug("System armed");
        protocolJetson.sendAck(MsgId::CMD_ARM, 0);
        Serial.println(">>> SYSTEM ARMED <<<");
    } else {
        protocolJetson.sendNack(MsgId::CMD_ARM, 1);
        Serial.println("Arm command rejected");
    }
}

void onDisarmCommand() {
    if (stateMachine.disarm()) {
        logger.close();
        protocolJetson.sendAck(MsgId::CMD_DISARM, 0);
        Serial.println(">>> SYSTEM DISARMED <<<");
    } else {
        protocolJetson.sendNack(MsgId::CMD_DISARM, 1);
    }
}

void onAbortCommand(uint8_t reason) {
    stateMachine.abort((AbortReason)reason);
    tvc.center();
    logger.logEvent(0xFF, &reason, 1);
    protocolJetson.sendAck(MsgId::CMD_ABORT, 0);
    Serial.println(">>> ABORT TRIGGERED <<<");
}

void onCalibrateCommand() {
    if (stateMachine.getState() == FlightState::IDLE) {
        imu.calibrate(500);
        protocolJetson.sendAck(MsgId::CMD_CALIBRATE, 0);
        Serial.println("Calibration complete");
    } else {
        protocolJetson.sendNack(MsgId::CMD_CALIBRATE, 1);
    }
}

void onTargetCommand(float r, float p, float y, float t) {
    attitudeCtrl.setTarget(r, p, y);
    protocolJetson.sendAck(MsgId::CMD_SET_TARGET, 0);
}

void onParamCommand(uint16_t id, float val) {
    // Parameter handling (simplified)
    protocolJetson.sendAck(MsgId::CMD_SET_PARAM, 0);
}

// ============================================================================
// UTILITY
// ============================================================================

void updateBatteryVoltage() {
    int raw = analogRead(PIN_BATTERY_VOLTAGE);
    float voltage = raw * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION * BATTERY_DIVIDER_RATIO;
    
    // Low-pass filter
    batteryVoltage = 0.9f * batteryVoltage + 0.1f * voltage;
}

void logFlightEvent(FlightEvent event) {
    uint8_t data[2] = { (uint8_t)event.fromState, (uint8_t)event.toState };
    logger.logEvent(0x01, data, 2);
    
    Serial.print("State: ");
    Serial.print(stateMachine.getStateName());
    Serial.println();
}
