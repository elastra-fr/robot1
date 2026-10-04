#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Observation {
    ArduinoTelemetry(Telemetry),
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
            _ => "unknown",
        }
    }

    pub fn confirms_stop(&self, stop_sequence: u16) -> bool {
        self.state == 0
            && self.left_target_mm_s == 0
            && self.right_target_mm_s == 0
            && self.last_command_sequence == stop_sequence
    }
}
