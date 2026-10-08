#include "Motion.h"

#include "FirmwareConfig.h"

void MotionController::begin(unsigned long now) {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  lastUpdateMs_ = now;
}

bool MotionController::setTargets(int16_t leftMmS, int16_t rightMmS) {
  if (leftMmS < -MAX_SIMULATED_SPEED_MM_S
      || leftMmS > MAX_SIMULATED_SPEED_MM_S
      || rightMmS < -MAX_SIMULATED_SPEED_MM_S
      || rightMmS > MAX_SIMULATED_SPEED_MM_S) {
    return false;
  }

  leftTargetMmS_ = leftMmS;
  rightTargetMmS_ = rightMmS;
  digitalWrite(LED_BUILTIN, isMoving() ? HIGH : LOW);
  return true;
}

void MotionController::stop() {
  leftTargetMmS_ = 0;
  rightTargetMmS_ = 0;
  digitalWrite(LED_BUILTIN, LOW);
}

void MotionController::update(unsigned long now) {
  const unsigned long elapsedMs = now - lastUpdateMs_;
  if (elapsedMs == 0) {
    return;
  }

  const int32_t leftScaled = static_cast<int32_t>(leftTargetMmS_) * elapsedMs
    + leftPositionRemainder_;
  const int32_t rightScaled = static_cast<int32_t>(rightTargetMmS_) * elapsedMs
    + rightPositionRemainder_;

  leftPositionMm_ += leftScaled / 1000;
  rightPositionMm_ += rightScaled / 1000;
  leftPositionRemainder_ = leftScaled % 1000;
  rightPositionRemainder_ = rightScaled % 1000;
  lastUpdateMs_ = now;
}

bool MotionController::isMoving() const {
  return leftTargetMmS_ != 0 || rightTargetMmS_ != 0;
}

MotionMode MotionController::mode() const {
  if (!isMoving()) {
    return MotionMode::Stopped;
  }
  if (leftTargetMmS_ >= 0 && rightTargetMmS_ >= 0) {
    return MotionMode::Forward;
  }
  if (leftTargetMmS_ <= 0 && rightTargetMmS_ <= 0) {
    return MotionMode::Backward;
  }
  if (leftTargetMmS_ < 0 && rightTargetMmS_ > 0) {
    return MotionMode::Left;
  }
  if (leftTargetMmS_ > 0 && rightTargetMmS_ < 0) {
    return MotionMode::Right;
  }
  return MotionMode::Mixed;
}

int16_t MotionController::leftTargetMmS() const {
  return leftTargetMmS_;
}

int16_t MotionController::rightTargetMmS() const {
  return rightTargetMmS_;
}

int32_t MotionController::leftPositionMm() const {
  return leftPositionMm_;
}

int32_t MotionController::rightPositionMm() const {
  return rightPositionMm_;
}
