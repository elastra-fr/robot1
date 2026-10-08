#pragma once

#include <Arduino.h>

#include "FirmwareTypes.h"
#include "Sensors.h"
#include "Servos.h"

class EnvironmentScanner {
 public:
  void begin();
  bool start(unsigned long now, ServoController &servos);
  bool update(
    unsigned long now,
    const SensorManager &sensors,
    ServoController &servos,
    EnvironmentScanSample &sample,
    bool &completed
  );
  bool active() const;

 private:
  enum class Phase : uint8_t {
    Idle,
    Settling,
    WaitingForMeasurements,
  };

  void moveToNextAngle(unsigned long now, ServoController &servos);

  Phase phase_ = Phase::Idle;
  uint8_t angleDeg_ = 0;
  unsigned long deadlineMs_ = 0;
  uint16_t baselineSequenceA_ = 0;
  uint16_t baselineSequenceB_ = 0;
};
