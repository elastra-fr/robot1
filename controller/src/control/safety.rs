use crate::domain::{ActuatorCommand, ConnectionHealth, MotionIntent, RobotState};
use std::time::{Duration, Instant};

pub struct SafetyController {
    telemetry_timeout: Duration,
}

impl SafetyController {
    pub fn new(telemetry_timeout: Duration) -> Self {
        Self { telemetry_timeout }
    }

    pub fn requires_recovery(&self, state: &RobotState, now: Instant) -> bool {
        state.telemetry_is_stale(now, self.telemetry_timeout)
    }

    pub fn authorize(
        &self,
        state: &RobotState,
        intent: MotionIntent,
        now: Instant,
    ) -> ActuatorCommand {
        if state.connection() != ConnectionHealth::Connected
            || self.requires_recovery(state, now)
            || state
                .telemetry()
                .is_none_or(|telemetry| telemetry.state >= 2)
            || (intent.left_mm_s == 0 && intent.right_mm_s == 0)
        {
            ActuatorCommand::Stop
        } else {
            ActuatorCommand::SetWheelSpeeds {
                left_mm_s: intent.left_mm_s,
                right_mm_s: intent.right_mm_s,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::SafetyController;
    use crate::domain::{ActuatorCommand, MotionIntent, Observation, RobotState, Telemetry};
    use std::time::{Duration, Instant};

    #[test]
    fn stops_when_telemetry_is_stale() {
        let now = Instant::now();
        let mut state = RobotState::default();
        state.apply(
            Observation::ArduinoTelemetry(Telemetry {
                uptime_ms: 0,
                state: 0,
                left_target_mm_s: 0,
                right_target_mm_s: 0,
                left_position_mm: 0,
                right_position_mm: 0,
                distance_mm: 500,
                battery_mv: 12_000,
                last_command_sequence: 1,
            }),
            now,
        );
        let safety = SafetyController::new(Duration::from_millis(1_500));
        let intent = MotionIntent::new("forward", 200, 200);

        assert_eq!(
            safety.authorize(&state, intent, now),
            ActuatorCommand::SetWheelSpeeds {
                left_mm_s: 200,
                right_mm_s: 200
            }
        );
        assert_eq!(
            safety.authorize(&state, intent, now + Duration::from_millis(1_500)),
            ActuatorCommand::Stop
        );
    }

    #[test]
    fn stops_when_arduino_reports_local_safety() {
        let now = Instant::now();
        let mut state = RobotState::default();
        state.apply(
            Observation::ArduinoTelemetry(Telemetry {
                uptime_ms: 0,
                state: 3,
                left_target_mm_s: 0,
                right_target_mm_s: 0,
                left_position_mm: 0,
                right_position_mm: 0,
                distance_mm: 500,
                battery_mv: 0,
                last_command_sequence: 1,
            }),
            now,
        );
        let safety = SafetyController::new(Duration::from_millis(1_500));

        assert_eq!(
            safety.authorize(
                &state,
                MotionIntent::new("PIR rotation test", -150, 150),
                now
            ),
            ActuatorCommand::Stop
        );
    }
}
