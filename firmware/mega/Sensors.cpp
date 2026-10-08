#include "Sensors.h"

#include "FirmwareConfig.h"

namespace {

uint8_t activeBit(uint8_t pin, uint8_t activeLevel, uint8_t bit) {
  return digitalRead(pin) == activeLevel ? static_cast<uint8_t>(1U << bit) : 0;
}

bool deadlineReached(unsigned long now, unsigned long deadline) {
  return static_cast<long>(now - deadline) >= 0;
}

}  // namespace

void DebouncedDigitalInput::begin(
  uint8_t pin,
  uint8_t activeLevel,
  unsigned long now
) {
  pin_ = pin;
  activeLevel_ = activeLevel;
  pinMode(pin_, INPUT);
  rawActive_ = digitalRead(pin_) == activeLevel_;
  stableActive_ = rawActive_;
  rawChangedMs_ = now;
}

void DebouncedDigitalInput::update(unsigned long now) {
  const bool rawActive = digitalRead(pin_) == activeLevel_;
  if (rawActive != rawActive_) {
    rawActive_ = rawActive;
    rawChangedMs_ = now;
  }

  if (stableActive_ != rawActive_
      && now - rawChangedMs_ >= PIR_DEBOUNCE_MS) {
    stableActive_ = rawActive_;
  }
}

bool DebouncedDigitalInput::active() const {
  return stableActive_;
}

void UltrasonicSensor::begin(
  uint8_t triggerPin,
  uint8_t echoPin,
  unsigned long firstMeasurementMs
) {
  triggerPin_ = triggerPin;
  echoPin_ = echoPin;
  nextMeasurementMs_ = firstMeasurementMs;
  pinMode(triggerPin_, OUTPUT);
  digitalWrite(triggerPin_, LOW);
  pinMode(echoPin_, INPUT);
}

void UltrasonicSensor::update(unsigned long nowMs, unsigned long nowUs) {
  switch (phase_) {
    case Phase::Idle:
      return;

    case Phase::TriggerLow:
      if (nowUs - phaseStartedUs_ >= 2) {
        digitalWrite(triggerPin_, HIGH);
        phase_ = Phase::TriggerHigh;
        phaseStartedUs_ = nowUs;
      }
      return;

    case Phase::TriggerHigh:
      if (nowUs - phaseStartedUs_ >= 10) {
        digitalWrite(triggerPin_, LOW);
        phase_ = Phase::WaitEchoRise;
        phaseStartedUs_ = nowUs;
      }
      return;

    case Phase::WaitEchoRise:
      if (digitalRead(echoPin_) == HIGH) {
        echoStartedUs_ = nowUs;
        phase_ = Phase::WaitEchoFall;
      } else if (nowUs - phaseStartedUs_ >= ULTRASONIC_ECHO_TIMEOUT_US) {
        finish(nowMs, false, 0);
      }
      return;

    case Phase::WaitEchoFall:
      if (digitalRead(echoPin_) == LOW) {
        finish(nowMs, true, nowUs - echoStartedUs_);
      } else if (nowUs - echoStartedUs_ >= ULTRASONIC_ECHO_TIMEOUT_US) {
        finish(nowMs, false, 0);
      }
      return;
  }
}

bool UltrasonicSensor::due(unsigned long nowMs) const {
  return phase_ == Phase::Idle && deadlineReached(nowMs, nextMeasurementMs_);
}

void UltrasonicSensor::start(unsigned long nowUs) {
  if (phase_ != Phase::Idle) {
    return;
  }

  digitalWrite(triggerPin_, LOW);
  phase_ = Phase::TriggerLow;
  phaseStartedUs_ = nowUs;
}

bool UltrasonicSensor::busy() const {
  return phase_ != Phase::Idle;
}

bool UltrasonicSensor::valid() const {
  return valid_;
}

uint16_t UltrasonicSensor::distanceMm() const {
  return distanceMm_;
}

uint16_t UltrasonicSensor::measurementSequence() const {
  return measurementSequence_;
}

void UltrasonicSensor::finish(
  unsigned long nowMs,
  bool valid,
  unsigned long echoDurationUs
) {
  phase_ = Phase::Idle;
  nextMeasurementMs_ = nowMs + ULTRASONIC_INTERVAL_MS;
  ++measurementSequence_;
  valid_ = valid;
  distanceMm_ = valid
    ? static_cast<uint16_t>((echoDurationUs * 343UL) / 2000UL)
    : INVALID_DISTANCE_MM;
}

void SensorManager::begin(unsigned long now) {
  startedMs_ = now;

  pirFront_.begin(PIR_FRONT_PIN, PIR_ACTIVE_LEVEL, now);
  pirBack_.begin(PIR_BACK_PIN, PIR_ACTIVE_LEVEL, now);
  pinMode(BUMPER_1_PIN, INPUT_PULLUP);
  pinMode(BUMPER_2_PIN, INPUT_PULLUP);
  pinMode(BUMPER_3_PIN, INPUT_PULLUP);
  pinMode(CLIFF_SENSOR_PIN, INPUT_PULLUP);

  ultrasonicA_.begin(
    ULTRASONIC_A_TRIGGER_PIN,
    ULTRASONIC_A_ECHO_PIN,
    now
  );
  ultrasonicB_.begin(
    ULTRASONIC_B_TRIGGER_PIN,
    ULTRASONIC_B_ECHO_PIN,
    now + ULTRASONIC_INTERVAL_MS / 2
  );

  update(now);
}

void SensorManager::update(unsigned long now) {
  pirFront_.update(now);
  pirBack_.update(now);
  pirReady_ = now - startedMs_ >= PIR_WARMUP_MS;
  if (pirReady()) {
    pirMask_ = (pirFront_.active() ? 1U << 0 : 0)
      | (pirBack_.active() ? 1U << 1 : 0);
  } else {
    pirMask_ = 0;
  }

  proximityMask_ = activeBit(BUMPER_1_PIN, PROXIMITY_ACTIVE_LEVEL, 0)
    | activeBit(BUMPER_2_PIN, PROXIMITY_ACTIVE_LEVEL, 1)
    | activeBit(BUMPER_3_PIN, PROXIMITY_ACTIVE_LEVEL, 2)
    | activeBit(CLIFF_SENSOR_PIN, PROXIMITY_ACTIVE_LEVEL, 3);

  const unsigned long nowUs = micros();
  if (activeUltrasonic_ == 0) {
    ultrasonicA_.update(now, nowUs);
    if (!ultrasonicA_.busy()) {
      activeUltrasonic_ = -1;
    }
  } else if (activeUltrasonic_ == 1) {
    ultrasonicB_.update(now, nowUs);
    if (!ultrasonicB_.busy()) {
      activeUltrasonic_ = -1;
    }
  }

  if (activeUltrasonic_ < 0) {
    if (ultrasonicA_.due(now)) {
      ultrasonicA_.start(nowUs);
      activeUltrasonic_ = 0;
    } else if (ultrasonicB_.due(now)) {
      ultrasonicB_.start(nowUs);
      activeUltrasonic_ = 1;
    }
  }
}

SensorData SensorManager::snapshot(unsigned long now) const {
  uint8_t flags = 0;
  if (pirReady()) {
    flags |= 1U << 0;
  }
  if (ultrasonicA_.valid()) {
    flags |= 1U << 1;
  }
  if (ultrasonicB_.valid()) {
    flags |= 1U << 2;
  }

  return {
    static_cast<uint32_t>(now),
    pirMask_,
    proximityMask_,
    localSafetyMask(),
    flags,
    ultrasonicA_.distanceMm(),
    ultrasonicB_.distanceMm(),
  };
}

bool SensorManager::pirReady() const {
  return pirReady_;
}

uint8_t SensorManager::pirMask() const {
  return pirMask_;
}

uint8_t SensorManager::proximityMask() const {
  return proximityMask_;
}

uint8_t SensorManager::localSafetyMask() const {
  const uint8_t bumperHazards = proximityMask_ & BUMPER_PROXIMITY_MASK;
  const uint8_t cliffHazard = (proximityMask_ & CLIFF_PROXIMITY_BIT) == 0
    ? CLIFF_PROXIMITY_BIT
    : 0;
  return bumperHazards | cliffHazard;
}

bool SensorManager::localHazardDetected() const {
  return localSafetyMask() != 0;
}

uint16_t SensorManager::ultrasonicAMm() const {
  return ultrasonicA_.distanceMm();
}

uint16_t SensorManager::ultrasonicBMm() const {
  return ultrasonicB_.distanceMm();
}

bool SensorManager::ultrasonicAValid() const {
  return ultrasonicA_.valid();
}

bool SensorManager::ultrasonicBValid() const {
  return ultrasonicB_.valid();
}

uint16_t SensorManager::ultrasonicASequence() const {
  return ultrasonicA_.measurementSequence();
}

uint16_t SensorManager::ultrasonicBSequence() const {
  return ultrasonicB_.measurementSequence();
}
