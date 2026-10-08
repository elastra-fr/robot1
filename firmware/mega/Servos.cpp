#include "Servos.h"

void ServoController::begin(unsigned long now) {
  (void)now;
  // Servo pins and neutral positions will be configured during hardware tests.
}

void ServoController::update(unsigned long now) {
  (void)now;
}

void ServoController::stopAll() {
  // No physical servo is attached by the firmware yet.
}
