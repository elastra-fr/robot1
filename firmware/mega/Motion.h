#pragma once

#include <Arduino.h>

class MotionController {
 public:
  void begin(unsigned long now);
  bool setTargets(int16_t leftMmS, int16_t rightMmS);
  void stop();
  void update(unsigned long now);

  bool isMoving() const;
  int16_t leftTargetMmS() const;
  int16_t rightTargetMmS() const;
  int32_t leftPositionMm() const;
  int32_t rightPositionMm() const;

 private:
  int16_t leftTargetMmS_ = 0;
  int16_t rightTargetMmS_ = 0;
  int32_t leftPositionMm_ = 0;
  int32_t rightPositionMm_ = 0;
  int32_t leftPositionRemainder_ = 0;
  int32_t rightPositionRemainder_ = 0;
  unsigned long lastUpdateMs_ = 0;
};
