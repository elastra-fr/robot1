use crate::domain::{MotionIntent, RobotState};
use std::time::Instant;

pub trait Behavior {
    fn start(&mut self, state: &RobotState, now: Instant) -> MotionIntent;
    fn update(&mut self, state: &RobotState, now: Instant) -> Option<MotionIntent>;
}
