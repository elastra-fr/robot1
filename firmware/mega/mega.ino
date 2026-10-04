#include <Arduino.h>

constexpr uint8_t PROTOCOL_VERSION = 1;
constexpr size_t HEADER_SIZE = 9;
constexpr size_t CRC_SIZE = 2;
constexpr size_t MAX_PAYLOAD_SIZE = 48;
constexpr size_t MAX_RAW_SIZE = HEADER_SIZE + MAX_PAYLOAD_SIZE + CRC_SIZE;
constexpr size_t MAX_ENCODED_SIZE = 64;

constexpr uint8_t MSG_HELLO = 0x01;
constexpr uint8_t MSG_HELLO_ACK = 0x02;
constexpr uint8_t MSG_HEARTBEAT = 0x03;
constexpr uint8_t MSG_SET_MOTION = 0x10;
constexpr uint8_t MSG_STOP = 0x11;
constexpr uint8_t MSG_ACK = 0x20;
constexpr uint8_t MSG_TELEMETRY = 0x30;
constexpr uint8_t MSG_ERROR = 0x7f;

constexpr unsigned long TELEMETRY_INTERVAL_MS = 100;
constexpr unsigned long WATCHDOG_TIMEOUT_MS = 1000;
constexpr int16_t MAX_SIMULATED_SPEED_MM_S = 1000;

uint8_t encodedInput[MAX_ENCODED_SIZE];
size_t encodedInputLength = 0;
bool droppingOversizedFrame = false;

bool sessionActive = false;
uint32_t activeSession = 0;
uint16_t telemetrySequence = 1;
uint16_t lastCommandSequence = 0;
unsigned long lastContactMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastSimulationMs = 0;
bool watchdogTriggered = false;

int16_t leftTargetMmS = 0;
int16_t rightTargetMmS = 0;
int32_t leftPositionMm = 0;
int32_t rightPositionMm = 0;
int32_t leftPositionRemainder = 0;
int32_t rightPositionRemainder = 0;

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

void sendFrame(
  uint8_t messageType,
  uint32_t session,
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
  writeU32(&raw[2], session);
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

void sendAck(uint16_t sequence) {
  sendFrame(MSG_ACK, activeSession, sequence, nullptr, 0);
}

void sendError(uint16_t sequence, uint8_t code) {
  sendFrame(MSG_ERROR, activeSession, sequence, &code, 1);
}

void stopSimulation(bool watchdog) {
  leftTargetMmS = 0;
  rightTargetMmS = 0;
  watchdogTriggered = watchdog;
  digitalWrite(LED_BUILTIN, LOW);
}

void processFrame(const uint8_t *raw, size_t rawLength) {
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
    activeSession = session;
    sessionActive = true;
    lastContactMs = millis();
    stopSimulation(false);
    sendFrame(MSG_HELLO_ACK, activeSession, sequence, nullptr, 0);
    return;
  }

  if (!sessionActive || session != activeSession) {
    return;
  }

  if (messageType == MSG_HEARTBEAT && payloadLength == 0) {
    lastContactMs = millis();
    watchdogTriggered = false;
    return;
  }

  if (messageType == MSG_SET_MOTION && payloadLength == 4) {
    const int16_t requestedLeft = readI16(&payload[0]);
    const int16_t requestedRight = readI16(&payload[2]);

    if (requestedLeft < -MAX_SIMULATED_SPEED_MM_S
        || requestedLeft > MAX_SIMULATED_SPEED_MM_S
        || requestedRight < -MAX_SIMULATED_SPEED_MM_S
        || requestedRight > MAX_SIMULATED_SPEED_MM_S) {
      sendError(sequence, 1);
      return;
    }

    leftTargetMmS = requestedLeft;
    rightTargetMmS = requestedRight;
    lastCommandSequence = sequence;
    lastContactMs = millis();
    watchdogTriggered = false;
    digitalWrite(
      LED_BUILTIN,
      leftTargetMmS != 0 || rightTargetMmS != 0 ? HIGH : LOW
    );
    sendAck(sequence);
    return;
  }

  if (messageType == MSG_STOP && payloadLength == 0) {
    lastCommandSequence = sequence;
    lastContactMs = millis();
    stopSimulation(false);
    sendAck(sequence);
    return;
  }

  sendError(sequence, 2);
}

void processSerialInput() {
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      return;
    }

    const uint8_t byte = static_cast<uint8_t>(value);
    if (byte == 0) {
      if (!droppingOversizedFrame && encodedInputLength > 0) {
        uint8_t raw[MAX_RAW_SIZE];
        size_t rawLength = 0;
        if (cobsDecode(
              encodedInput,
              encodedInputLength,
              raw,
              sizeof(raw),
              rawLength
            )) {
          processFrame(raw, rawLength);
        }
      }

      encodedInputLength = 0;
      droppingOversizedFrame = false;
      continue;
    }

    if (droppingOversizedFrame) {
      continue;
    }

    if (encodedInputLength >= sizeof(encodedInput)) {
      encodedInputLength = 0;
      droppingOversizedFrame = true;
      continue;
    }

    encodedInput[encodedInputLength++] = byte;
  }
}

void updateSimulation(unsigned long now) {
  const unsigned long elapsedMs = now - lastSimulationMs;
  if (elapsedMs == 0) {
    return;
  }

  const int32_t leftScaled = static_cast<int32_t>(leftTargetMmS) * elapsedMs
    + leftPositionRemainder;
  const int32_t rightScaled = static_cast<int32_t>(rightTargetMmS) * elapsedMs
    + rightPositionRemainder;

  leftPositionMm += leftScaled / 1000;
  rightPositionMm += rightScaled / 1000;
  leftPositionRemainder = leftScaled % 1000;
  rightPositionRemainder = rightScaled % 1000;
  lastSimulationMs = now;
}

uint16_t simulatedDistanceMm(unsigned long now) {
  const uint16_t phase = (now / 10) % 200;
  const uint16_t triangle = phase <= 100 ? phase : 200 - phase;
  return 200 + triangle * 5;
}

uint16_t simulatedBatteryMv(unsigned long now) {
  return 12000 - ((now / 1000) % 500);
}

void sendTelemetry(unsigned long now) {
  uint8_t payload[23];
  writeU32(&payload[0], now);
  payload[4] = watchdogTriggered
    ? 2
    : (leftTargetMmS != 0 || rightTargetMmS != 0 ? 1 : 0);
  writeI16(&payload[5], leftTargetMmS);
  writeI16(&payload[7], rightTargetMmS);
  writeI32(&payload[9], leftPositionMm);
  writeI32(&payload[13], rightPositionMm);
  writeU16(&payload[17], simulatedDistanceMm(now));
  writeU16(&payload[19], simulatedBatteryMv(now));
  writeU16(&payload[21], lastCommandSequence);

  sendFrame(
    MSG_TELEMETRY,
    activeSession,
    telemetrySequence++,
    payload,
    sizeof(payload)
  );
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  lastSimulationMs = millis();
}

void loop() {
  const unsigned long now = millis();

  processSerialInput();
  updateSimulation(now);

  if (sessionActive && now - lastContactMs > WATCHDOG_TIMEOUT_MS) {
    stopSimulation(true);
  }

  if (sessionActive && now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    sendTelemetry(now);
    lastTelemetryMs = now;
  }
}
