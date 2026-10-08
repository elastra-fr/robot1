use crate::control::Behavior;
use crate::domain::{MotionIntent, RobotState};
use std::time::{Duration, Instant};

const ROTATION_DURATION: Duration = Duration::from_secs(1);
const ROTATION_SPEED_MM_S: i16 = 150;

const STOP: MotionIntent = MotionIntent::new("presence test idle", 0, 0);
const ROTATE_LEFT: MotionIntent = MotionIntent::new(
    "PIR front rotation test",
    -ROTATION_SPEED_MM_S,
    ROTATION_SPEED_MM_S,
);
const ROTATE_RIGHT: MotionIntent = MotionIntent::new(
    "PIR back rotation test",
    ROTATION_SPEED_MM_S,
    -ROTATION_SPEED_MM_S,
);

#[derive(Default)]
pub struct PresenceTestBehavior {
    armed: bool,
    rotation_deadline: Option<Instant>,
}

impl Behavior for PresenceTestBehavior {
    fn start(&mut self, _state: &RobotState, _now: Instant) -> MotionIntent {
        self.armed = false;
        self.rotation_deadline = None;
        STOP
    }

    fn update(&mut self, state: &RobotState, now: Instant) -> Option<MotionIntent> {
        if let Some(deadline) = self.rotation_deadline {
            if now < deadline {
                return None;
            }
            self.rotation_deadline = None;
            return Some(STOP);
        }

        let sensors = state.sensors()?;
        if !sensors.pir_ready {
            return None;
        }

        let presence = sensors.pir_mask & 0x03;
        if presence == 0 {
            self.armed = true;
            return None;
        }
        if !self.armed || sensors.local_safety_mask != 0 {
            return None;
        }
        if state
            .telemetry()
            .is_none_or(|telemetry| telemetry.state >= 2)
        {
            return None;
        }

        self.armed = false;
        let intent = match presence {
            0x01 => ROTATE_LEFT,
            0x02 => ROTATE_RIGHT,
            _ => return Some(STOP),
        };
        self.rotation_deadline = Some(now + ROTATION_DURATION);
        Some(intent)
    }
}

#[cfg(test)]
mod tests {
    use super::PresenceTestBehavior;
    use crate::control::Behavior;
    use crate::domain::{Observation, RobotState, SensorSnapshot, Telemetry};
    use std::time::{Duration, Instant};

    fn telemetry() -> Telemetry {
        Telemetry {
            uptime_ms: 100,
            state: 0,
            left_target_mm_s: 0,
            right_target_mm_s: 0,
            left_position_mm: 0,
            right_position_mm: 0,
            distance_mm: 500,
            battery_mv: 0,
            last_command_sequence: 1,
        }
    }

    fn sensors(pir_mask: u8) -> SensorSnapshot {
        SensorSnapshot {
            pir_ready: true,
            pir_mask,
            proximity_mask: 0x08,
            local_safety_mask: 0,
            ultrasonic_a_mm: Some(500),
            ultrasonic_b_mm: Some(600),
        }
    }

    #[test]
    fn rotates_once_per_debounced_presence_and_stops() {
        let start = Instant::now();
        let mut state = RobotState::default();
        state.apply(Observation::ArduinoTelemetry(telemetry()), start);
        state.apply(Observation::ArduinoSensors(sensors(0)), start);
        let mut behavior = PresenceTestBehavior::default();

        assert_eq!(behavior.start(&state, start).name, "presence test idle");
        assert!(behavior.update(&state, start).is_none());

        state.apply(Observation::ArduinoSensors(sensors(1)), start);
        let rotation = behavior.update(&state, start).unwrap();
        assert_eq!(rotation.name, "PIR front rotation test");
        assert_eq!((rotation.left_mm_s, rotation.right_mm_s), (-150, 150));
        assert!(
            behavior
                .update(&state, start + Duration::from_millis(999))
                .is_none()
        );
        assert_eq!(
            behavior
                .update(&state, start + Duration::from_secs(1))
                .unwrap()
                .name,
            "presence test idle"
        );
        assert!(
            behavior
                .update(&state, start + Duration::from_secs(2))
                .is_none()
        );

        state.apply(
            Observation::ArduinoSensors(sensors(0)),
            start + Duration::from_secs(2),
        );
        assert!(
            behavior
                .update(&state, start + Duration::from_secs(2))
                .is_none()
        );
        state.apply(
            Observation::ArduinoSensors(sensors(2)),
            start + Duration::from_secs(2),
        );
        let rotation = behavior
            .update(&state, start + Duration::from_secs(2))
            .unwrap();
        assert_eq!(rotation.name, "PIR back rotation test");
        assert_eq!((rotation.left_mm_s, rotation.right_mm_s), (150, -150));
    }
}
