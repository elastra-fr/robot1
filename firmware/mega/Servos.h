#pragma once

#include <Arduino.h>

class ServoController {
 public:
  void begin(unsigned long now);
  void update(unsigned long now);
  void stopAll();
};
