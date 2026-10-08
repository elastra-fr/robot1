#include "Servos.h"

#include "FirmwareConfig.h"

void ServoController::begin(unsigned long now) {
  (void)now;
  scanServo_.attach(SCAN_SERVO_PIN);
  setScanAngle(SCAN_CENTER_ANGLE_DEG);
}

void ServoController::update(unsigned long now) {
  (void)now;
}

void ServoController::setScanAngle(uint8_t angleDeg) {
  scanAngleDeg_ = constrain(
    angleDeg,
    SCAN_MIN_ANGLE_DEG,
    SCAN_MAX_ANGLE_DEG
  );
  scanServo_.write(scanAngleDeg_);
}

uint8_t ServoController::scanAngle() const {
  return scanAngleDeg_;
}

void ServoController::stopAll() {
  // A positional servo is held at its current angle. Cutting its power is a
  // hardware-level operation and must not be emulated by detaching the signal.
}
