#include "Safety.h"

#include "FirmwareConfig.h"

void SafetyController::begin(unsigned long now) {
  lastContactMs_ = now;
  reason_ = SafetyReason::None;
}

void SafetyController::onContact(unsigned long now) {
  lastContactMs_ = now;
  if (reason_ == SafetyReason::Watchdog) {
    reason_ = SafetyReason::None;
  }
}

bool SafetyController::update(
  unsigned long now,
  bool sessionActive,
  bool localHazard
) {
  if (localHazard) {
    reason_ = SafetyReason::LocalHazard;
    return true;
  }

  if (sessionActive && now - lastContactMs_ > WATCHDOG_TIMEOUT_MS) {
    reason_ = SafetyReason::Watchdog;
    return true;
  }

  if (reason_ == SafetyReason::LocalHazard) {
    reason_ = SafetyReason::None;
  }
  if (!sessionActive) {
    reason_ = SafetyReason::None;
  }
  return false;
}

SafetyReason SafetyController::reason() const {
  return reason_;
}

bool SafetyController::watchdogTriggered() const {
  return reason_ == SafetyReason::Watchdog;
}

bool SafetyController::localHazardTriggered() const {
  return reason_ == SafetyReason::LocalHazard;
}
