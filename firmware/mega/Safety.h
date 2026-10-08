#pragma once

#include <Arduino.h>

class SafetyController {
 public:
  void begin(unsigned long now);
  void onContact(unsigned long now);
  bool update(unsigned long now, bool sessionActive);
  bool watchdogTriggered() const;

 private:
  unsigned long lastContactMs_ = 0;
  bool watchdogTriggered_ = false;
};
