#include "EnvironmentScan.h"

#include "FirmwareConfig.h"

void EnvironmentScanner::begin() {
  phase_ = Phase::Idle;
}

bool EnvironmentScanner::start(
  unsigned long now,
  ServoController &servos
) {
  if (active()) {
    return false;
  }

  angleDeg_ = SCAN_MIN_ANGLE_DEG;
  servos.setScanAngle(angleDeg_);
  deadlineMs_ = now + SCAN_HOME_SETTLE_MS;
  phase_ = Phase::Settling;
  return true;
}

bool EnvironmentScanner::update(
  unsigned long now,
  const SensorManager &sensors,
  ServoController &servos,
  EnvironmentScanSample &sample,
  bool &completed
) {
  completed = false;
  if (phase_ == Phase::Idle) {
    return false;
  }

  if (phase_ == Phase::Settling) {
    if (static_cast<long>(now - deadlineMs_) < 0) {
      return false;
    }
    baselineSequenceA_ = sensors.ultrasonicASequence();
    baselineSequenceB_ = sensors.ultrasonicBSequence();
    deadlineMs_ = now + SCAN_SAMPLE_TIMEOUT_MS;
    phase_ = Phase::WaitingForMeasurements;
    return false;
  }

  const bool freshA = sensors.ultrasonicASequence() != baselineSequenceA_;
  const bool freshB = sensors.ultrasonicBSequence() != baselineSequenceB_;
  const bool timedOut = static_cast<long>(now - deadlineMs_) >= 0;
  if (!(freshA && freshB) && !timedOut) {
    return false;
  }

  sample.angleADeg = angleDeg_;
  sample.distanceAMm = freshA && sensors.ultrasonicAValid()
    ? sensors.ultrasonicAMm()
    : INVALID_DISTANCE_MM;
  sample.angleBDeg = (static_cast<uint16_t>(angleDeg_) + 180U) % 360U;
  sample.distanceBMm = freshB && sensors.ultrasonicBValid()
    ? sensors.ultrasonicBMm()
    : INVALID_DISTANCE_MM;
  sample.validFlags = (sample.distanceAMm != INVALID_DISTANCE_MM ? 1U << 0 : 0)
    | (sample.distanceBMm != INVALID_DISTANCE_MM ? 1U << 1 : 0);

  if (angleDeg_ >= SCAN_MAX_ANGLE_DEG) {
    phase_ = Phase::Idle;
    servos.setScanAngle(SCAN_CENTER_ANGLE_DEG);
    completed = true;
  } else {
    moveToNextAngle(now, servos);
  }
  return true;
}

bool EnvironmentScanner::active() const {
  return phase_ != Phase::Idle;
}

void EnvironmentScanner::moveToNextAngle(
  unsigned long now,
  ServoController &servos
) {
  const uint16_t nextAngle = static_cast<uint16_t>(angleDeg_) + SCAN_STEP_DEG;
  angleDeg_ = nextAngle > SCAN_MAX_ANGLE_DEG
    ? SCAN_MAX_ANGLE_DEG
    : static_cast<uint8_t>(nextAngle);
  servos.setScanAngle(angleDeg_);
  deadlineMs_ = now + SCAN_STEP_SETTLE_MS;
  phase_ = Phase::Settling;
}
