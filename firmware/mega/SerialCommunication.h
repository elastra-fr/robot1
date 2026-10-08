#pragma once

#include <Arduino.h>

#include "FirmwareTypes.h"

using CommandHandler = void (*)(const Command &command);

class SerialCommunication {
 public:
  void begin();
  void processInput(CommandHandler handler);

  bool sessionActive() const;
  void sendHelloAck(uint16_t sequence);
  void sendAck(uint16_t sequence);
  void sendError(uint16_t sequence, uint8_t code);
  void sendTelemetry(const TelemetryData &telemetry);
  void sendSensorStatus(const SensorData &sensors);
  void sendMotionStatus(MotionMode mode, SafetyReason reason);

 private:
  static constexpr size_t HEADER_SIZE = 9;
  static constexpr size_t CRC_SIZE = 2;
  static constexpr size_t MAX_PAYLOAD_SIZE = 48;
  static constexpr size_t MAX_RAW_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;
  static constexpr size_t MAX_ENCODED_SIZE = 64;

  uint8_t encodedInput_[MAX_ENCODED_SIZE];
  size_t encodedInputLength_ = 0;
  bool droppingOversizedFrame_ = false;
  bool sessionActive_ = false;
  uint32_t activeSession_ = 0;
  uint16_t eventSequence_ = 1;

  void processEncodedFrame(CommandHandler handler);
  void processFrame(const uint8_t *raw, size_t rawLength, CommandHandler handler);
  void sendFrame(
    uint8_t messageType,
    uint16_t sequence,
    const uint8_t *payload,
    uint8_t payloadLength
  );
};
