#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActuatorCommand {
    SetWheelSpeeds { left_mm_s: i16, right_mm_s: i16 },
    Stop,
}
