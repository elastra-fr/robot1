#pragma once

#include <Arduino.h>

enum class CommandType : uint8_t {
  Hello,
  Heartbeat,
  SetMotion,
  Stop,
  Unknown,
};

struct Command {
  CommandType type;
  uint16_t sequence;
  int16_t leftMmS;
  int16_t rightMmS;
};

enum class MotionMode : uint8_t {
  Stopped = 0,
  Forward = 1,
  Backward = 2,
  Left = 3,
  Right = 4,
  Mixed = 5,
};

enum class SafetyReason : uint8_t {
  None = 0,
  Watchdog = 1,
  LocalHazard = 2,
};

struct TelemetryData {
  uint32_t uptimeMs;
  uint8_t state;
  int16_t leftTargetMmS;
  int16_t rightTargetMmS;
  int32_t leftPositionMm;
  int32_t rightPositionMm;
  uint16_t distanceMm;
  uint16_t batteryMv;
  uint16_t lastCommandSequence;
};

struct SensorData {
  uint32_t uptimeMs;
  uint8_t pirMask;
  uint8_t proximityMask;
  uint8_t localSafetyMask;
  uint8_t flags;
  uint16_t ultrasonicAMm;
  uint16_t ultrasonicBMm;
};
