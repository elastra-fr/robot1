#include "Sensors.h"

void SensorManager::begin(unsigned long now) {
  update(now);
}

void SensorManager::update(unsigned long now) {
  // Temporary simulation. Replace these values one sensor at a time while
  // keeping the public interface and telemetry unchanged.
  const uint16_t phase = (now / 10) % 200;
  const uint16_t triangle = phase <= 100 ? phase : 200 - phase;
  distanceMm_ = 200 + triangle * 5;
  batteryMv_ = 12000 - ((now / 1000) % 500);
}

uint16_t SensorManager::distanceMm() const {
  return distanceMm_;
}

uint16_t SensorManager::batteryMv() const {
  return batteryMv_;
}
