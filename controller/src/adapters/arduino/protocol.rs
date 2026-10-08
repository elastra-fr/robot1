use std::error::Error;
use std::fmt;

pub const VERSION: u8 = 1;
pub const MAX_PAYLOAD_SIZE: usize = 48;
const HEADER_SIZE: usize = 9;
const CRC_SIZE: usize = 2;
const MAX_ENCODED_SIZE: usize = 64;

pub mod message_type {
    pub const HELLO: u8 = 0x01;
    pub const HELLO_ACK: u8 = 0x02;
    pub const HEARTBEAT: u8 = 0x03;
    pub const SET_MOTION: u8 = 0x10;
    pub const STOP: u8 = 0x11;
    pub const START_ENVIRONMENT_SCAN: u8 = 0x12;
    pub const ACK: u8 = 0x20;
    pub const TELEMETRY: u8 = 0x30;
    pub const SENSOR_STATUS: u8 = 0x31;
    pub const MOTION_STATUS: u8 = 0x32;
    pub const ENVIRONMENT_SCAN_SAMPLE: u8 = 0x33;
    pub const ENVIRONMENT_SCAN_STATUS: u8 = 0x34;
    pub const ERROR: u8 = 0x7f;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub message_type: u8,
    pub session: u32,
    pub sequence: u16,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProtocolError {
    PayloadTooLarge,
    FrameTooLarge,
    InvalidCobs,
    FrameTooShort,
    UnsupportedVersion(u8),
    InvalidLength,
    InvalidCrc,
}

impl fmt::Display for ProtocolError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PayloadTooLarge => write!(formatter, "payload is too large"),
            Self::FrameTooLarge => write!(formatter, "encoded frame is too large"),
            Self::InvalidCobs => write!(formatter, "invalid COBS frame"),
            Self::FrameTooShort => write!(formatter, "frame is too short"),
            Self::UnsupportedVersion(version) => {
                write!(formatter, "unsupported protocol version {version}")
            }
            Self::InvalidLength => write!(formatter, "invalid payload length"),
            Self::InvalidCrc => write!(formatter, "invalid frame CRC"),
        }
    }
}

impl Error for ProtocolError {}

impl Frame {
    pub fn encode(&self) -> Result<Vec<u8>, ProtocolError> {
        if self.payload.len() > MAX_PAYLOAD_SIZE {
            return Err(ProtocolError::PayloadTooLarge);
        }

        let mut raw = Vec::with_capacity(HEADER_SIZE + self.payload.len() + CRC_SIZE);
        raw.push(VERSION);
        raw.push(self.message_type);
        raw.extend_from_slice(&self.session.to_le_bytes());
        raw.extend_from_slice(&self.sequence.to_le_bytes());
        raw.push(self.payload.len() as u8);
        raw.extend_from_slice(&self.payload);
        raw.extend_from_slice(&crc16(&raw).to_le_bytes());

        let mut encoded = cobs_encode(&raw);
        encoded.push(0);
        Ok(encoded)
    }

    fn decode(encoded: &[u8]) -> Result<Self, ProtocolError> {
        let raw = cobs_decode(encoded)?;

        if raw.len() < HEADER_SIZE + CRC_SIZE {
            return Err(ProtocolError::FrameTooShort);
        }

        if raw[0] != VERSION {
            return Err(ProtocolError::UnsupportedVersion(raw[0]));
        }

        let payload_size = raw[8] as usize;
        if payload_size > MAX_PAYLOAD_SIZE || raw.len() != HEADER_SIZE + payload_size + CRC_SIZE {
            return Err(ProtocolError::InvalidLength);
        }

        let crc_offset = raw.len() - CRC_SIZE;
        let received_crc = u16::from_le_bytes([raw[crc_offset], raw[crc_offset + 1]]);
        if crc16(&raw[..crc_offset]) != received_crc {
            return Err(ProtocolError::InvalidCrc);
        }

        Ok(Self {
            message_type: raw[1],
            session: u32::from_le_bytes([raw[2], raw[3], raw[4], raw[5]]),
            sequence: u16::from_le_bytes([raw[6], raw[7]]),
            payload: raw[HEADER_SIZE..crc_offset].to_vec(),
        })
    }
}

#[derive(Default)]
pub struct FrameDecoder {
    encoded: Vec<u8>,
    dropping_oversized_frame: bool,
}

impl FrameDecoder {
    pub fn push(&mut self, byte: u8) -> Option<Result<Frame, ProtocolError>> {
        if byte == 0 {
            if self.dropping_oversized_frame {
                self.dropping_oversized_frame = false;
                self.encoded.clear();
                return None;
            }

            if self.encoded.is_empty() {
                return None;
            }

            let encoded = std::mem::take(&mut self.encoded);
            return Some(Frame::decode(&encoded));
        }

        if self.dropping_oversized_frame {
            return None;
        }

        if self.encoded.len() >= MAX_ENCODED_SIZE {
            self.encoded.clear();
            self.dropping_oversized_frame = true;
            return Some(Err(ProtocolError::FrameTooLarge));
        }

        self.encoded.push(byte);
        None
    }
}

fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0xffff_u16;

    for byte in bytes {
        crc ^= u16::from(*byte) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }

    crc
}

fn cobs_encode(input: &[u8]) -> Vec<u8> {
    let mut output = Vec::with_capacity(input.len() + 2);
    output.push(0);

    let mut code_index = 0;
    let mut code = 1_u8;

    for byte in input {
        if *byte == 0 {
            output[code_index] = code;
            code_index = output.len();
            output.push(0);
            code = 1;
        } else {
            output.push(*byte);
            code += 1;

            if code == 0xff {
                output[code_index] = code;
                code_index = output.len();
                output.push(0);
                code = 1;
            }
        }
    }

    output[code_index] = code;
    output
}

fn cobs_decode(input: &[u8]) -> Result<Vec<u8>, ProtocolError> {
    let mut output = Vec::with_capacity(input.len());
    let mut index = 0;

    while index < input.len() {
        let code = input[index] as usize;
        if code == 0 {
            return Err(ProtocolError::InvalidCobs);
        }
        index += 1;

        let block_end = index + code - 1;
        if block_end > input.len() {
            return Err(ProtocolError::InvalidCobs);
        }

        output.extend_from_slice(&input[index..block_end]);
        index = block_end;

        if code != 0xff && index < input.len() {
            output.push(0);
        }
    }

    Ok(output)
}

#[cfg(test)]
mod tests {
    use super::{Frame, FrameDecoder, ProtocolError, cobs_decode, cobs_encode};

    #[test]
    fn round_trips_frame_containing_zero_bytes() {
        let frame = Frame {
            message_type: 0x10,
            session: 0x1200_3400,
            sequence: 7,
            payload: vec![0, 1, 0, 2],
        };
        let encoded = frame.encode().unwrap();
        assert_eq!(
            encoded,
            [
                0x03, 0x01, 0x10, 0x02, 0x34, 0x03, 0x12, 0x07, 0x02, 0x04, 0x02, 0x01, 0x04, 0x02,
                0xb3, 0x6f, 0x00,
            ]
        );
        let mut decoder = FrameDecoder::default();
        let mut decoded = None;

        for byte in encoded {
            if let Some(result) = decoder.push(byte) {
                decoded = Some(result.unwrap());
            }
        }

        assert_eq!(decoded, Some(frame));
    }

    #[test]
    fn rejects_corrupted_frame() {
        let frame = Frame {
            message_type: 0x03,
            session: 42,
            sequence: 9,
            payload: vec![1, 2, 3],
        };
        let encoded = frame.encode().unwrap();
        let mut raw = cobs_decode(&encoded[..encoded.len() - 1]).unwrap();
        raw[2] ^= 0x40;
        let mut encoded = cobs_encode(&raw);
        encoded.push(0);
        let mut decoder = FrameDecoder::default();
        let mut result = None;

        for byte in encoded {
            if let Some(event) = decoder.push(byte) {
                result = Some(event);
            }
        }

        assert!(matches!(result, Some(Err(ProtocolError::InvalidCrc))));
    }

    #[test]
    fn resynchronizes_after_oversized_frame() {
        let valid_frame = Frame {
            message_type: 0x02,
            session: 10,
            sequence: 1,
            payload: Vec::new(),
        };
        let mut bytes = vec![1; 80];
        bytes.push(0);
        bytes.extend(valid_frame.encode().unwrap());
        let mut decoder = FrameDecoder::default();
        let mut decoded = Vec::new();

        for byte in bytes {
            if let Some(Ok(frame)) = decoder.push(byte) {
                decoded.push(frame);
            }
        }

        assert_eq!(decoded, vec![valid_frame]);
    }
}
