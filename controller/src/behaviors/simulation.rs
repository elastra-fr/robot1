use crate::control::Behavior;
use crate::domain::{MotionIntent, RobotState};
use std::time::{Duration, Instant};

#[derive(Clone, Copy)]
struct MotionPhase {
    intent: MotionIntent,
    duration: Duration,
}

const MOTION_SCRIPT: [MotionPhase; 4] = [
    MotionPhase {
        intent: MotionIntent::new("forward", 200, 200),
        duration: Duration::from_secs(3),
    },
    MotionPhase {
        intent: MotionIntent::new("rotate right", 150, -150),
        duration: Duration::from_secs(2),
    },
    MotionPhase {
        intent: MotionIntent::new("reverse", -150, -150),
        duration: Duration::from_secs(3),
    },
    MotionPhase {
        intent: MotionIntent::new("stop", 0, 0),
        duration: Duration::from_secs(2),
    },
];

#[derive(Default)]
pub struct SimulationBehavior {
    phase_index: usize,
    phase_deadline: Option<Instant>,
}

impl Behavior for SimulationBehavior {
    fn start(&mut self, _state: &RobotState, now: Instant) -> MotionIntent {
        self.phase_index = 0;
        self.phase_deadline = Some(now + MOTION_SCRIPT[0].duration);
        MOTION_SCRIPT[0].intent
    }

    fn update(&mut self, _state: &RobotState, now: Instant) -> Option<MotionIntent> {
        let deadline = self.phase_deadline?;
        if now < deadline {
            return None;
        }

        self.phase_index = (self.phase_index + 1) % MOTION_SCRIPT.len();
        let phase = MOTION_SCRIPT[self.phase_index];
        self.phase_deadline = Some(now + phase.duration);
        Some(phase.intent)
    }
}

#[cfg(test)]
mod tests {
    use super::SimulationBehavior;
    use crate::control::Behavior;
    use crate::domain::RobotState;
    use std::time::{Duration, Instant};

    #[test]
    fn cycles_through_the_motion_script() {
        let state = RobotState::default();
        let start = Instant::now();
        let mut behavior = SimulationBehavior::default();

        assert_eq!(behavior.start(&state, start).name, "forward");
        assert!(
            behavior
                .update(&state, start + Duration::from_secs(2))
                .is_none()
        );
        assert_eq!(
            behavior
                .update(&state, start + Duration::from_secs(3))
                .unwrap()
                .name,
            "rotate right"
        );
    }
}
