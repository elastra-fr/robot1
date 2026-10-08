#pragma once

#include <Arduino.h>
#include <Servo.h>

class ServoController {
 public:
  void begin(unsigned long now);
  void update(unsigned long now);
  void setScanAngle(uint8_t angleDeg);
  uint8_t scanAngle() const;
  void stopAll();

 private:
  Servo scanServo_;
  uint8_t scanAngleDeg_ = 90;
};
