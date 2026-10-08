#include <Arduino.h>

#include "FirmwareConfig.h"
#include "FirmwareTypes.h"
#include "EnvironmentScan.h"
#include "Motion.h"
#include "Safety.h"
#include "Sensors.h"
#include "SerialCommunication.h"
#include "Servos.h"

SerialCommunication communication;
MotionController motion;
SensorManager sensors;
ServoController servos;
SafetyController safety;
EnvironmentScanner environmentScanner;

uint16_t lastCommandSequence = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastSensorStatusMs = 0;

void handleCommand(const Command &command) {
  const unsigned long now = millis();

  switch (command.type) {
    case CommandType::Hello:
      motion.stop();
      safety.onContact(now);
      communication.sendHelloAck(command.sequence);
      communication.sendMotionStatus(MotionMode::Stopped, SafetyReason::None);
      break;

    case CommandType::Heartbeat:
      safety.onContact(now);
      break;

    case CommandType::SetMotion:
      if (!motion.setTargets(command.leftMmS, command.rightMmS)) {
        communication.sendError(command.sequence, 1);
        break;
      }

      lastCommandSequence = command.sequence;
      safety.onContact(now);
      communication.sendAck(command.sequence);
      communication.sendMotionStatus(motion.mode(), SafetyReason::None);
      break;

    case CommandType::Stop:
      lastCommandSequence = command.sequence;
      safety.onContact(now);
      motion.stop();
      communication.sendAck(command.sequence);
      communication.sendMotionStatus(MotionMode::Stopped, SafetyReason::None);
      break;

    case CommandType::StartEnvironmentScan:
      if (!environmentScanner.start(now, servos)) {
        communication.sendError(command.sequence, 3);
        break;
      }
      communication.sendAck(command.sequence);
      communication.sendEnvironmentScanStatus(1);
      break;

    case CommandType::Unknown:
      communication.sendError(command.sequence, 2);
      break;
  }
}

void sendCurrentTelemetry(unsigned long now) {
  TelemetryData telemetry;
  telemetry.uptimeMs = now;
  telemetry.state = safety.watchdogTriggered()
    ? 2
    : (safety.localHazardTriggered() ? 3 : (motion.isMoving() ? 1 : 0));
  telemetry.leftTargetMmS = motion.leftTargetMmS();
  telemetry.rightTargetMmS = motion.rightTargetMmS();
  telemetry.leftPositionMm = motion.leftPositionMm();
  telemetry.rightPositionMm = motion.rightPositionMm();
  telemetry.distanceMm = sensors.ultrasonicAMm();
  telemetry.batteryMv = 0;
  telemetry.lastCommandSequence = lastCommandSequence;
  communication.sendTelemetry(telemetry);
}

void setup() {
  communication.begin();

  const unsigned long now = millis();
  motion.begin(now);
  sensors.begin(now);
  servos.begin(now);
  safety.begin(now);
  environmentScanner.begin();
}

void loop() {
  communication.processInput(handleCommand);

  // Command handlers may refresh the safety timestamp. Capture the loop time
  // afterwards so unsigned elapsed-time calculations cannot underflow.
  const unsigned long now = millis();

  motion.update(now);
  sensors.update(now);
  servos.update(now);

  EnvironmentScanSample scanSample;
  bool scanCompleted = false;
  if (environmentScanner.update(
        now,
        sensors,
        servos,
        scanSample,
        scanCompleted
      )) {
    communication.sendEnvironmentScanSample(scanSample);
  }
  if (scanCompleted) {
    communication.sendEnvironmentScanStatus(2);
  }

  const bool wasMoving = motion.isMoving();
  if (safety.update(
        now,
        communication.sessionActive(),
        sensors.localHazardDetected()
      )) {
    motion.stop();
    servos.stopAll();
    if (wasMoving) {
      communication.sendMotionStatus(MotionMode::Stopped, safety.reason());
    }
  }

  if (communication.sessionActive()
      && now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    sendCurrentTelemetry(now);
    lastTelemetryMs = now;
  }

  if (communication.sessionActive()
      && now - lastSensorStatusMs >= SENSOR_STATUS_INTERVAL_MS) {
    communication.sendSensorStatus(sensors.snapshot(now));
    lastSensorStatusMs = now;
  }
}
