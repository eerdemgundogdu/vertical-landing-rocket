#pragma once

// ============================================================================
// VERTICAL LANDING ROCKET - PIN DEFINITIONS
// ============================================================================
// Teensy 4.1 pin assignments for all hardware interfaces.
// See docs/hardware/wiring_diagram.md for connection details.
// ============================================================================

// ----------------------------------------------------------------------------
// SPI PINS (Hardware SPI0)
// ----------------------------------------------------------------------------
#define PIN_SPI_MOSI         11
#define PIN_SPI_MISO         12
#define PIN_SPI_SCK          13

// IMU Chip Select pins
#define PIN_IMU_CS_ACC       10    // BMI088 Accelerometer CS
#define PIN_IMU_CS_GYRO      9     // BMI088 Gyroscope CS

// IMU Interrupt pins (optional, for data-ready)
#define PIN_IMU_INT_ACC      6     // Accelerometer interrupt
#define PIN_IMU_INT_GYRO     17    // Gyroscope interrupt

// ----------------------------------------------------------------------------
// I2C PINS (Wire - I2C0)
// ----------------------------------------------------------------------------
#define PIN_I2C_SDA          18
#define PIN_I2C_SCL          19

// I2C Addresses
#define I2C_ADDR_BMP390      0x76  // Barometer (SDO tied to GND)

// ----------------------------------------------------------------------------
// UART PINS
// ----------------------------------------------------------------------------
// Serial1 - Jetson Nano communication
#define PIN_UART1_RX         0     // Receive from Jetson
#define PIN_UART1_TX         1     // Transmit to Jetson

// Serial2 - GPS module
#define PIN_UART2_RX         7     // Receive from GPS
#define PIN_UART2_TX         8     // Transmit to GPS

// Serial3 - LoRa telemetry radio
#define PIN_UART3_RX         14    // Receive from LoRa
#define PIN_UART3_TX         15    // Transmit to LoRa

// ----------------------------------------------------------------------------
// PWM OUTPUT PINS
// ----------------------------------------------------------------------------
#define PIN_SERVO_X          2     // TVC X-axis servo (pitch)
#define PIN_SERVO_Y          3     // TVC Y-axis servo (yaw)
#define PIN_SERVO_PARA       4     // Parachute servo (optional)
#define PIN_SERVO_GEAR       5     // Landing gear servo (optional)

// PWM Configuration
#define PWM_FREQUENCY_HZ     50    // Standard servo frequency
#define PWM_RESOLUTION_BITS  12    // 12-bit resolution

// ----------------------------------------------------------------------------
// DIGITAL OUTPUT PINS
// ----------------------------------------------------------------------------
#define PIN_IGNITER          23    // Motor igniter MOSFET gate
#define PIN_LED_STATUS       20    // Status LED
#define PIN_BUZZER           16    // Piezo buzzer (optional)

// ----------------------------------------------------------------------------
// DIGITAL INPUT PINS
// ----------------------------------------------------------------------------
#define PIN_ARM_SWITCH       22    // Hardware arm switch (active high)

// ----------------------------------------------------------------------------
// ANALOG INPUT PINS
// ----------------------------------------------------------------------------
#define PIN_BATTERY_VOLTAGE  21    // Battery voltage (through divider)

// Battery voltage divider (10k/10k = 1:2 ratio)
#define BATTERY_DIVIDER_RATIO  2.0f
#define ADC_REFERENCE_VOLTAGE  3.3f
#define ADC_RESOLUTION         1023

// ----------------------------------------------------------------------------
// SD CARD (Built-in on Teensy 4.1)
// ----------------------------------------------------------------------------
// Uses BUILTIN_SDCARD, no external pins needed

// ----------------------------------------------------------------------------
// INTERNAL SENSORS
// ----------------------------------------------------------------------------
// Teensy 4.1 has internal temperature sensor accessible via tempmonGetTemp()
