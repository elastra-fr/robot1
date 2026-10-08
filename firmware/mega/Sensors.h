#pragma once

#include <Arduino.h>

#include "FirmwareTypes.h"

constexpr uint16_t INVALID_DISTANCE_MM = 0xffff;

class DebouncedDigitalInput {
 public:
  void begin(uint8_t pin, uint8_t activeLevel, unsigned long now);
  void update(unsigned long now);
  bool active() const;

 private:
  uint8_t pin_ = 0;
  uint8_t activeLevel_ = HIGH;
  bool rawActive_ = false;
  bool stableActive_ = false;
  unsigned long rawChangedMs_ = 0;
};

class UltrasonicSensor {
 public:
  void begin(uint8_t triggerPin, uint8_t echoPin, unsigned long firstMeasurementMs);
  void update(unsigned long nowMs, unsigned long nowUs);
  bool due(unsigned long nowMs) const;
  void start(unsigned long nowUs);
  bool busy() const;
  bool valid() const;
  uint16_t distanceMm() const;
  uint16_t measurementSequence() const;

 private:
  enum class Phase : uint8_t {
    Idle,
    TriggerLow,
    TriggerHigh,
    WaitEchoRise,
    WaitEchoFall,
  };

  void finish(unsigned long nowMs, bool valid, unsigned long echoDurationUs);

  uint8_t triggerPin_ = 0;
  uint8_t echoPin_ = 0;
  Phase phase_ = Phase::Idle;
  unsigned long phaseStartedUs_ = 0;
  unsigned long echoStartedUs_ = 0;
  unsigned long nextMeasurementMs_ = 0;
  uint16_t distanceMm_ = INVALID_DISTANCE_MM;
  bool valid_ = false;
  uint16_t measurementSequence_ = 0;
};

class SensorManager {
 public:
  void begin(unsigned long now);
  void update(unsigned long now);

  SensorData snapshot(unsigned long now) const;
  bool pirReady() const;
  uint8_t pirMask() const;
  uint8_t proximityMask() const;
  uint8_t localSafetyMask() const;
  bool localHazardDetected() const;
  uint16_t ultrasonicAMm() const;
  uint16_t ultrasonicBMm() const;
  bool ultrasonicAValid() const;
  bool ultrasonicBValid() const;
  uint16_t ultrasonicASequence() const;
  uint16_t ultrasonicBSequence() const;

 private:
  unsigned long startedMs_ = 0;
  bool pirReady_ = false;
  uint8_t pirMask_ = 0;
  uint8_t proximityMask_ = 0;
  int8_t activeUltrasonic_ = -1;
  DebouncedDigitalInput pirFront_;
  DebouncedDigitalInput pirBack_;
  UltrasonicSensor ultrasonicA_;
  UltrasonicSensor ultrasonicB_;
};
