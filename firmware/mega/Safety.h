#pragma once

#include <Arduino.h>

#include "FirmwareTypes.h"

class SafetyController {
 public:
  void begin(unsigned long now);
  void onContact(unsigned long now);
  bool update(unsigned long now, bool sessionActive, bool localHazard);
  SafetyReason reason() const;
  bool watchdogTriggered() const;
  bool localHazardTriggered() const;

 private:
  unsigned long lastContactMs_ = 0;
  SafetyReason reason_ = SafetyReason::None;
};
