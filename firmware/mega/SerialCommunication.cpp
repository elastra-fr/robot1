#include "SerialCommunication.h"

#include "FirmwareConfig.h"

namespace {

constexpr uint8_t PROTOCOL_VERSION = 1;

constexpr uint8_t MSG_HELLO = 0x01;
constexpr uint8_t MSG_HELLO_ACK = 0x02;
constexpr uint8_t MSG_HEARTBEAT = 0x03;
constexpr uint8_t MSG_SET_MOTION = 0x10;
constexpr uint8_t MSG_STOP = 0x11;
constexpr uint8_t MSG_ACK = 0x20;
constexpr uint8_t MSG_TELEMETRY = 0x30;
constexpr uint8_t MSG_SENSOR_STATUS = 0x31;
constexpr uint8_t MSG_MOTION_STATUS = 0x32;
constexpr uint8_t MSG_ERROR = 0x7f;

uint16_t crc16(const uint8_t *bytes, size_t length) {
  uint16_t crc = 0xffff;

  for (size_t index = 0; index < length; ++index) {
    crc ^= static_cast<uint16_t>(bytes[index]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) != 0
        ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
        : static_cast<uint16_t>(crc << 1);
    }
  }

  return crc;
}

size_t cobsEncode(
  const uint8_t *input,
  size_t inputLength,
  uint8_t *output,
  size_t outputCapacity
) {
  if (outputCapacity == 0) {
    return 0;
  }

  size_t outputIndex = 1;
  size_t codeIndex = 0;
  uint8_t code = 1;

  for (size_t inputIndex = 0; inputIndex < inputLength; ++inputIndex) {
    if (input[inputIndex] == 0) {
      output[codeIndex] = code;
      codeIndex = outputIndex;
      if (outputIndex >= outputCapacity) {
        return 0;
      }
      output[outputIndex++] = 0;
      code = 1;
    } else {
      if (outputIndex >= outputCapacity) {
        return 0;
      }
      output[outputIndex++] = input[inputIndex];
      ++code;

      if (code == 0xff) {
        output[codeIndex] = code;
        codeIndex = outputIndex;
        if (outputIndex >= outputCapacity) {
          return 0;
        }
        output[outputIndex++] = 0;
        code = 1;
      }
    }
  }

  output[codeIndex] = code;
  return outputIndex;
}

bool cobsDecode(
  const uint8_t *input,
  size_t inputLength,
  uint8_t *output,
  size_t outputCapacity,
  size_t &outputLength
) {
  size_t inputIndex = 0;
  size_t outputIndex = 0;

  while (inputIndex < inputLength) {
    const uint8_t code = input[inputIndex++];
    if (code == 0) {
      return false;
    }

    const size_t blockEnd = inputIndex + code - 1;
    if (blockEnd > inputLength) {
      return false;
    }

    while (inputIndex < blockEnd) {
      if (outputIndex >= outputCapacity) {
        return false;
      }
      output[outputIndex++] = input[inputIndex++];
    }

    if (code != 0xff && inputIndex < inputLength) {
      if (outputIndex >= outputCapacity) {
        return false;
      }
      output[outputIndex++] = 0;
    }
  }

  outputLength = outputIndex;
  return true;
}

uint16_t readU16(const uint8_t *bytes) {
  return static_cast<uint16_t>(bytes[0])
    | (static_cast<uint16_t>(bytes[1]) << 8);
}

int16_t readI16(const uint8_t *bytes) {
  return static_cast<int16_t>(readU16(bytes));
}

uint32_t readU32(const uint8_t *bytes) {
  return static_cast<uint32_t>(bytes[0])
    | (static_cast<uint32_t>(bytes[1]) << 8)
    | (static_cast<uint32_t>(bytes[2]) << 16)
    | (static_cast<uint32_t>(bytes[3]) << 24);
}

void writeU16(uint8_t *bytes, uint16_t value) {
  bytes[0] = static_cast<uint8_t>(value);
  bytes[1] = static_cast<uint8_t>(value >> 8);
}

void writeI16(uint8_t *bytes, int16_t value) {
  writeU16(bytes, static_cast<uint16_t>(value));
}

void writeU32(uint8_t *bytes, uint32_t value) {
  bytes[0] = static_cast<uint8_t>(value);
  bytes[1] = static_cast<uint8_t>(value >> 8);
  bytes[2] = static_cast<uint8_t>(value >> 16);
  bytes[3] = static_cast<uint8_t>(value >> 24);
}

void writeI32(uint8_t *bytes, int32_t value) {
  writeU32(bytes, static_cast<uint32_t>(value));
}

}  // namespace

void SerialCommunication::begin() {
  Serial.begin(SERIAL_BAUD_RATE);
}

void SerialCommunication::processInput(CommandHandler handler) {
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      return;
    }

    const uint8_t byte = static_cast<uint8_t>(value);
    if (byte == 0) {
      if (!droppingOversizedFrame_ && encodedInputLength_ > 0) {
        processEncodedFrame(handler);
      }

      encodedInputLength_ = 0;
      droppingOversizedFrame_ = false;
      continue;
    }

    if (droppingOversizedFrame_) {
      continue;
    }

    if (encodedInputLength_ >= sizeof(encodedInput_)) {
      encodedInputLength_ = 0;
      droppingOversizedFrame_ = true;
      continue;
    }

    encodedInput_[encodedInputLength_++] = byte;
  }
}

bool SerialCommunication::sessionActive() const {
  return sessionActive_;
}

void SerialCommunication::sendHelloAck(uint16_t sequence) {
  sendFrame(MSG_HELLO_ACK, sequence, nullptr, 0);
}

void SerialCommunication::sendAck(uint16_t sequence) {
  sendFrame(MSG_ACK, sequence, nullptr, 0);
}

void SerialCommunication::sendError(uint16_t sequence, uint8_t code) {
  sendFrame(MSG_ERROR, sequence, &code, 1);
}

void SerialCommunication::sendTelemetry(const TelemetryData &telemetry) {
  uint8_t payload[23];
  writeU32(&payload[0], telemetry.uptimeMs);
  payload[4] = telemetry.state;
  writeI16(&payload[5], telemetry.leftTargetMmS);
  writeI16(&payload[7], telemetry.rightTargetMmS);
  writeI32(&payload[9], telemetry.leftPositionMm);
  writeI32(&payload[13], telemetry.rightPositionMm);
  writeU16(&payload[17], telemetry.distanceMm);
  writeU16(&payload[19], telemetry.batteryMv);
  writeU16(&payload[21], telemetry.lastCommandSequence);

  sendFrame(MSG_TELEMETRY, eventSequence_++, payload, sizeof(payload));
}

void SerialCommunication::sendSensorStatus(const SensorData &sensors) {
  uint8_t payload[12];
  writeU32(&payload[0], sensors.uptimeMs);
  payload[4] = sensors.pirMask;
  payload[5] = sensors.proximityMask;
  payload[6] = sensors.localSafetyMask;
  payload[7] = sensors.flags;
  writeU16(&payload[8], sensors.ultrasonicAMm);
  writeU16(&payload[10], sensors.ultrasonicBMm);
  sendFrame(MSG_SENSOR_STATUS, eventSequence_++, payload, sizeof(payload));
}

void SerialCommunication::sendMotionStatus(
  MotionMode mode,
  SafetyReason reason
) {
  const uint8_t payload[] = {
    static_cast<uint8_t>(mode),
    static_cast<uint8_t>(reason),
  };
  sendFrame(MSG_MOTION_STATUS, eventSequence_++, payload, sizeof(payload));
}

void SerialCommunication::processEncodedFrame(CommandHandler handler) {
  uint8_t raw[MAX_RAW_SIZE];
  size_t rawLength = 0;
  if (cobsDecode(
        encodedInput_,
        encodedInputLength_,
        raw,
        sizeof(raw),
        rawLength
      )) {
    processFrame(raw, rawLength, handler);
  }
}

void SerialCommunication::processFrame(
  const uint8_t *raw,
  size_t rawLength,
  CommandHandler handler
) {
  if (rawLength < HEADER_SIZE + CRC_SIZE || raw[0] != PROTOCOL_VERSION) {
    return;
  }

  const uint8_t payloadLength = raw[8];
  if (payloadLength > MAX_PAYLOAD_SIZE
      || rawLength != HEADER_SIZE + payloadLength + CRC_SIZE) {
    return;
  }

  const size_t crcOffset = HEADER_SIZE + payloadLength;
  if (readU16(&raw[crcOffset]) != crc16(raw, crcOffset)) {
    return;
  }

  const uint8_t messageType = raw[1];
  const uint32_t session = readU32(&raw[2]);
  const uint16_t sequence = readU16(&raw[6]);
  const uint8_t *payload = &raw[HEADER_SIZE];

  if (messageType == MSG_HELLO && payloadLength == 0) {
    activeSession_ = session;
    sessionActive_ = true;
    handler({CommandType::Hello, sequence, 0, 0});
    return;
  }

  if (!sessionActive_ || session != activeSession_) {
    return;
  }

  if (messageType == MSG_HEARTBEAT && payloadLength == 0) {
    handler({CommandType::Heartbeat, sequence, 0, 0});
    return;
  }

  if (messageType == MSG_SET_MOTION && payloadLength == 4) {
    handler({
      CommandType::SetMotion,
      sequence,
      readI16(&payload[0]),
      readI16(&payload[2]),
    });
    return;
  }

  if (messageType == MSG_STOP && payloadLength == 0) {
    handler({CommandType::Stop, sequence, 0, 0});
    return;
  }

  handler({CommandType::Unknown, sequence, 0, 0});
}

void SerialCommunication::sendFrame(
  uint8_t messageType,
  uint16_t sequence,
  const uint8_t *payload,
  uint8_t payloadLength
) {
  if (payloadLength > MAX_PAYLOAD_SIZE) {
    return;
  }

  uint8_t raw[MAX_RAW_SIZE];
  raw[0] = PROTOCOL_VERSION;
  raw[1] = messageType;
  writeU32(&raw[2], activeSession_);
  writeU16(&raw[6], sequence);
  raw[8] = payloadLength;

  for (uint8_t index = 0; index < payloadLength; ++index) {
    raw[HEADER_SIZE + index] = payload[index];
  }

  const size_t crcOffset = HEADER_SIZE + payloadLength;
  writeU16(&raw[crcOffset], crc16(raw, crcOffset));

  uint8_t encoded[MAX_ENCODED_SIZE];
  const size_t encodedLength = cobsEncode(
    raw,
    crcOffset + CRC_SIZE,
    encoded,
    sizeof(encoded)
  );

  if (encodedLength == 0) {
    return;
  }

  Serial.write(encoded, encodedLength);
  Serial.write(static_cast<uint8_t>(0));
}
