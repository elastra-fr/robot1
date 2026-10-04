use super::protocol::{Frame, message_type};
use crate::domain::Telemetry;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ArduinoEvent {
    Ack { sequence: u16 },
    Telemetry(Telemetry),
    InvalidTelemetry,
    Error { sequence: u16, payload: Vec<u8> },
    Unexpected { message_type: u8 },
}

pub(super) fn decode_event(frame: Frame) -> ArduinoEvent {
    match frame.message_type {
        message_type::ACK => ArduinoEvent::Ack {
            sequence: frame.sequence,
        },
        message_type::TELEMETRY => decode_telemetry(&frame.payload)
            .map(ArduinoEvent::Telemetry)
            .unwrap_or(ArduinoEvent::InvalidTelemetry),
        message_type::ERROR => ArduinoEvent::Error {
            sequence: frame.sequence,
            payload: frame.payload,
        },
        message_type => ArduinoEvent::Unexpected { message_type },
    }
}

fn decode_telemetry(payload: &[u8]) -> Option<Telemetry> {
    if payload.len() != 23 {
        return None;
    }

    Some(Telemetry {
        uptime_ms: u32::from_le_bytes(payload[0..4].try_into().ok()?),
        state: payload[4],
        left_target_mm_s: i16::from_le_bytes(payload[5..7].try_into().ok()?),
        right_target_mm_s: i16::from_le_bytes(payload[7..9].try_into().ok()?),
        left_position_mm: i32::from_le_bytes(payload[9..13].try_into().ok()?),
        right_position_mm: i32::from_le_bytes(payload[13..17].try_into().ok()?),
        distance_mm: u16::from_le_bytes(payload[17..19].try_into().ok()?),
        battery_mv: u16::from_le_bytes(payload[19..21].try_into().ok()?),
        last_command_sequence: u16::from_le_bytes(payload[21..23].try_into().ok()?),
    })
}

#[cfg(test)]
mod tests {
    use super::decode_telemetry;

    #[test]
    fn decodes_simulated_telemetry() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&1_234_u32.to_le_bytes());
        payload.push(1);
        payload.extend_from_slice(&200_i16.to_le_bytes());
        payload.extend_from_slice(&(-100_i16).to_le_bytes());
        payload.extend_from_slice(&500_i32.to_le_bytes());
        payload.extend_from_slice(&(-250_i32).to_le_bytes());
        payload.extend_from_slice(&420_u16.to_le_bytes());
        payload.extend_from_slice(&11_900_u16.to_le_bytes());
        payload.extend_from_slice(&7_u16.to_le_bytes());

        let telemetry = decode_telemetry(&payload).unwrap();

        assert_eq!(telemetry.uptime_ms, 1_234);
        assert_eq!(telemetry.left_target_mm_s, 200);
        assert_eq!(telemetry.right_position_mm, -250);
        assert_eq!(telemetry.distance_mm, 420);
        assert_eq!(telemetry.last_command_sequence, 7);
    }

    #[test]
    fn rejects_telemetry_with_an_invalid_size() {
        assert!(decode_telemetry(&[0; 22]).is_none());
    }
}
