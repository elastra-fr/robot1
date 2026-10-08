#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Observation {
    ArduinoTelemetry(Telemetry),
    ArduinoSensors(SensorSnapshot),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SensorSnapshot {
    pub pir_ready: bool,
    pub pir_mask: u8,
    pub proximity_mask: u8,
    pub local_safety_mask: u8,
    pub ultrasonic_a_mm: Option<u16>,
    pub ultrasonic_b_mm: Option<u16>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Telemetry {
    pub uptime_ms: u32,
    pub state: u8,
    pub left_target_mm_s: i16,
    pub right_target_mm_s: i16,
    pub left_position_mm: i32,
    pub right_position_mm: i32,
    pub distance_mm: u16,
    pub battery_mv: u16,
    pub last_command_sequence: u16,
}

impl Telemetry {
    pub fn state_name(&self) -> &'static str {
        match self.state {
            0 => "idle",
            1 => "moving",
            2 => "watchdog",
            3 => "local-safety-stop",
            _ => "unknown",
        }
    }

    pub fn confirms_stop(&self, stop_sequence: u16) -> bool {
        matches!(self.state, 0 | 2 | 3)
            && self.left_target_mm_s == 0
            && self.right_target_mm_s == 0
            && self.last_command_sequence == stop_sequence
    }
}

#[cfg(test)]
mod tests {
    use super::Telemetry;

    fn telemetry(state: u8, left: i16, right: i16) -> Telemetry {
        Telemetry {
            uptime_ms: 0,
            state,
            left_target_mm_s: left,
            right_target_mm_s: right,
            left_position_mm: 0,
            right_position_mm: 0,
            distance_mm: u16::MAX,
            battery_mv: 0,
            last_command_sequence: 7,
        }
    }

    #[test]
    fn local_safety_stop_confirms_a_safe_session() {
        assert!(telemetry(3, 0, 0).confirms_stop(7));
        assert!(!telemetry(1, 150, -150).confirms_stop(7));
    }
}
