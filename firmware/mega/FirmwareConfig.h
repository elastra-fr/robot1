#pragma once

#include <Arduino.h>

constexpr unsigned long SERIAL_BAUD_RATE = 115200;
constexpr unsigned long TELEMETRY_INTERVAL_MS = 100;
constexpr unsigned long SENSOR_STATUS_INTERVAL_MS = 250;
constexpr unsigned long WATCHDOG_TIMEOUT_MS = 1000;
constexpr unsigned long PIR_WARMUP_MS = 60000;
constexpr unsigned long PIR_DEBOUNCE_MS = 300;
constexpr unsigned long ULTRASONIC_INTERVAL_MS = 60;
constexpr unsigned long ULTRASONIC_ECHO_TIMEOUT_US = 25000;
constexpr unsigned long SCAN_HOME_SETTLE_MS = 500;
constexpr unsigned long SCAN_STEP_SETTLE_MS = 150;
constexpr unsigned long SCAN_SAMPLE_TIMEOUT_MS = 300;
constexpr int16_t MAX_SIMULATED_SPEED_MM_S = 1000;

// Initial Mega wiring for sensor bring-up. Change pins here if the physical
// layout requires it; no sensor or control code should contain pin numbers.
constexpr uint8_t PIR_FRONT_PIN = 22;
constexpr uint8_t PIR_BACK_PIN = 23;
constexpr uint8_t BUMPER_1_PIN = 24;
constexpr uint8_t BUMPER_2_PIN = 25;
constexpr uint8_t BUMPER_3_PIN = 26;
constexpr uint8_t CLIFF_SENSOR_PIN = 27;
constexpr uint8_t ULTRASONIC_A_TRIGGER_PIN = 28;
constexpr uint8_t ULTRASONIC_A_ECHO_PIN = 29;
constexpr uint8_t ULTRASONIC_B_TRIGGER_PIN = 30;
constexpr uint8_t ULTRASONIC_B_ECHO_PIN = 31;
constexpr uint8_t SCAN_SERVO_PIN = 6;

constexpr uint8_t SCAN_MIN_ANGLE_DEG = 0;
constexpr uint8_t SCAN_MAX_ANGLE_DEG = 180;
constexpr uint8_t SCAN_CENTER_ANGLE_DEG = 90;
constexpr uint8_t SCAN_STEP_DEG = 5;

constexpr uint8_t PIR_ACTIVE_LEVEL = HIGH;
constexpr uint8_t PROXIMITY_ACTIVE_LEVEL = LOW;
constexpr uint8_t BUMPER_PROXIMITY_MASK = 0x07;
constexpr uint8_t CLIFF_PROXIMITY_BIT = 0x08;
