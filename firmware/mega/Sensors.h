#pragma once

#include <Arduino.h>

class SensorManager {
 public:
  void begin(unsigned long now);
  void update(unsigned long now);

  uint16_t distanceMm() const;
  uint16_t batteryMv() const;

 private:
  uint16_t distanceMm_ = 0;
  uint16_t batteryMv_ = 0;
};
