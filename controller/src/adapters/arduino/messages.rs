use super::protocol::{Frame, message_type};
use crate::domain::Telemetry;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ArduinoEvent {
    Ack { sequence: u16 },
    Telemetry(Telemetry),
    SensorStatus(SensorStatus),
    MotionStatus(MotionStatus),
    EnvironmentScanSample(EnvironmentScanSample),
    EnvironmentScanStatus(EnvironmentScanStatus),
    InvalidTelemetry,
    InvalidSensorStatus,
    InvalidMotionStatus,
    InvalidEnvironmentScanSample,
    InvalidEnvironmentScanStatus,
    Error { sequence: u16, payload: Vec<u8> },
    Unexpected { message_type: u8 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnvironmentScanSample {
    pub angle_a_deg: u16,
    pub distance_a_mm: u16,
    pub angle_b_deg: u16,
    pub distance_b_mm: u16,
    pub valid_flags: u8,
}

impl EnvironmentScanSample {
    pub fn sensor_a_valid(&self) -> bool {
        self.valid_flags & (1 << 0) != 0
    }

    pub fn sensor_b_valid(&self) -> bool {
        self.valid_flags & (1 << 1) != 0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnvironmentScanStatus {
    pub state: u8,
}

impl EnvironmentScanStatus {
    pub fn state_name(&self) -> &'static str {
        match self.state {
            1 => "started",
            2 => "complete",
            _ => "unknown",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SensorStatus {
    pub uptime_ms: u32,
    pub pir_mask: u8,
    pub proximity_mask: u8,
    pub local_safety_mask: u8,
    pub flags: u8,
    pub ultrasonic_a_mm: u16,
    pub ultrasonic_b_mm: u16,
}

impl SensorStatus {
    pub fn pir_ready(&self) -> bool {
        self.flags & (1 << 0) != 0
    }

    pub fn ultrasonic_a_valid(&self) -> bool {
        self.flags & (1 << 1) != 0
    }

    pub fn ultrasonic_b_valid(&self) -> bool {
        self.flags & (1 << 2) != 0
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MotionStatus {
    pub mode: u8,
    pub safety_reason: u8,
}

impl MotionStatus {
    pub fn mode_name(&self) -> &'static str {
        match self.mode {
            0 => "STOPPED",
            1 => "MOVE FORWARD",
            2 => "MOVE BACKWARD",
            3 => "TURN LEFT",
            4 => "TURN RIGHT",
            5 => "MIXED MOTION",
            _ => "UNKNOWN MOTION",
        }
    }

    pub fn safety_reason_name(&self) -> Option<&'static str> {
        match self.safety_reason {
            0 => None,
            1 => Some("watchdog"),
            2 => Some("local bumper/cliff sensor"),
            _ => Some("unknown safety reason"),
        }
    }
}

pub(super) fn decode_event(frame: Frame) -> ArduinoEvent {
    match frame.message_type {
        message_type::ACK => ArduinoEvent::Ack {
            sequence: frame.sequence,
        },
        message_type::TELEMETRY => decode_telemetry(&frame.payload)
            .map(ArduinoEvent::Telemetry)
            .unwrap_or(ArduinoEvent::InvalidTelemetry),
        message_type::SENSOR_STATUS => decode_sensor_status(&frame.payload)
            .map(ArduinoEvent::SensorStatus)
            .unwrap_or(ArduinoEvent::InvalidSensorStatus),
        message_type::MOTION_STATUS => decode_motion_status(&frame.payload)
            .map(ArduinoEvent::MotionStatus)
            .unwrap_or(ArduinoEvent::InvalidMotionStatus),
        message_type::ENVIRONMENT_SCAN_SAMPLE => decode_environment_scan_sample(&frame.payload)
            .map(ArduinoEvent::EnvironmentScanSample)
            .unwrap_or(ArduinoEvent::InvalidEnvironmentScanSample),
        message_type::ENVIRONMENT_SCAN_STATUS => decode_environment_scan_status(&frame.payload)
            .map(ArduinoEvent::EnvironmentScanStatus)
            .unwrap_or(ArduinoEvent::InvalidEnvironmentScanStatus),
        message_type::ERROR => ArduinoEvent::Error {
            sequence: frame.sequence,
            payload: frame.payload,
        },
        message_type => ArduinoEvent::Unexpected { message_type },
    }
}

fn decode_environment_scan_sample(payload: &[u8]) -> Option<EnvironmentScanSample> {
    if payload.len() != 9 {
        return None;
    }
    Some(EnvironmentScanSample {
        angle_a_deg: u16::from_le_bytes(payload[0..2].try_into().ok()?),
        distance_a_mm: u16::from_le_bytes(payload[2..4].try_into().ok()?),
        angle_b_deg: u16::from_le_bytes(payload[4..6].try_into().ok()?),
        distance_b_mm: u16::from_le_bytes(payload[6..8].try_into().ok()?),
        valid_flags: payload[8],
    })
}

fn decode_environment_scan_status(payload: &[u8]) -> Option<EnvironmentScanStatus> {
    (payload.len() == 1).then(|| EnvironmentScanStatus { state: payload[0] })
}

fn decode_sensor_status(payload: &[u8]) -> Option<SensorStatus> {
    if payload.len() != 12 {
        return None;
    }

    Some(SensorStatus {
        uptime_ms: u32::from_le_bytes(payload[0..4].try_into().ok()?),
        pir_mask: payload[4],
        proximity_mask: payload[5],
        local_safety_mask: payload[6],
        flags: payload[7],
        ultrasonic_a_mm: u16::from_le_bytes(payload[8..10].try_into().ok()?),
        ultrasonic_b_mm: u16::from_le_bytes(payload[10..12].try_into().ok()?),
    })
}

fn decode_motion_status(payload: &[u8]) -> Option<MotionStatus> {
    if payload.len() != 2 {
        return None;
    }
    Some(MotionStatus {
        mode: payload[0],
        safety_reason: payload[1],
    })
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
    use super::{
        decode_environment_scan_sample, decode_motion_status, decode_sensor_status,
        decode_telemetry,
    };

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

    #[test]
    fn decodes_sensor_status() {
        let status = decode_sensor_status(&[
            0xd2, 0x04, 0, 0, 0b01, 0b1001, 0b0001, 0b111, 0xa4, 0x01, 0xbc, 0x02,
        ])
        .unwrap();

        assert_eq!(status.uptime_ms, 1_234);
        assert!(status.pir_ready());
        assert!(status.ultrasonic_a_valid());
        assert!(status.ultrasonic_b_valid());
        assert_eq!(status.local_safety_mask, 1);
        assert_eq!(status.ultrasonic_a_mm, 420);
        assert_eq!(status.ultrasonic_b_mm, 700);
    }

    #[test]
    fn decodes_virtual_motion_status() {
        let status = decode_motion_status(&[4, 0]).unwrap();
        assert_eq!(status.mode_name(), "TURN RIGHT");
        assert_eq!(status.safety_reason_name(), None);
    }

    #[test]
    fn decodes_environment_scan_sample() {
        let sample =
            decode_environment_scan_sample(&[30, 0, 0xa4, 0x01, 0xd2, 0, 0xbc, 0x02, 0b11])
                .unwrap();

        assert_eq!(sample.angle_a_deg, 30);
        assert_eq!(sample.distance_a_mm, 420);
        assert_eq!(sample.angle_b_deg, 210);
        assert_eq!(sample.distance_b_mm, 700);
        assert!(sample.sensor_a_valid());
        assert!(sample.sensor_b_valid());
    }
}
