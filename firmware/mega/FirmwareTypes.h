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
