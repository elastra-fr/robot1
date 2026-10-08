#include "Safety.h"

#include "FirmwareConfig.h"

void SafetyController::begin(unsigned long now) {
  lastContactMs_ = now;
  watchdogTriggered_ = false;
}

void SafetyController::onContact(unsigned long now) {
  lastContactMs_ = now;
  watchdogTriggered_ = false;
}

bool SafetyController::update(unsigned long now, bool sessionActive) {
  if (!sessionActive || now - lastContactMs_ <= WATCHDOG_TIMEOUT_MS) {
    return false;
  }

  watchdogTriggered_ = true;
  return true;
}

bool SafetyController::watchdogTriggered() const {
  return watchdogTriggered_;
}
