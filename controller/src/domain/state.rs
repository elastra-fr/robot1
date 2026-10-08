use super::{ConnectionHealth, Observation, SensorSnapshot, Telemetry};
use std::time::{Duration, Instant};

pub struct RobotState {
    connection: ConnectionHealth,
    telemetry: Option<Telemetry>,
    telemetry_received_at: Option<Instant>,
    sensors: Option<SensorSnapshot>,
    sensors_received_at: Option<Instant>,
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            connection: ConnectionHealth::Starting,
            telemetry: None,
            telemetry_received_at: None,
            sensors: None,
            sensors_received_at: None,
        }
    }
}

impl RobotState {
    pub fn apply(&mut self, observation: Observation, received_at: Instant) {
        match observation {
            Observation::ArduinoTelemetry(telemetry) => {
                self.record_telemetry(telemetry, received_at);
            }
            Observation::ArduinoSensors(sensors) => {
                self.sensors = Some(sensors);
                self.sensors_received_at = Some(received_at);
            }
        }
    }

    fn record_telemetry(&mut self, telemetry: Telemetry, received_at: Instant) {
        self.telemetry = Some(telemetry);
        self.telemetry_received_at = Some(received_at);
        self.connection = ConnectionHealth::Connected;
    }

    pub fn mark_recovering(&mut self) {
        self.connection = ConnectionHealth::Recovering;
    }

    pub fn connection(&self) -> ConnectionHealth {
        self.connection
    }

    pub fn telemetry(&self) -> Option<&Telemetry> {
        self.telemetry.as_ref()
    }

    pub fn sensors(&self) -> Option<&SensorSnapshot> {
        self.sensors.as_ref()
    }

    pub fn sensors_received_at(&self) -> Option<Instant> {
        self.sensors_received_at
    }

    pub fn telemetry_is_stale(&self, now: Instant, timeout: Duration) -> bool {
        self.telemetry_received_at
            .is_none_or(|received_at| now.duration_since(received_at) >= timeout)
    }
}

#[cfg(test)]
mod tests {
    use super::RobotState;
    use crate::domain::{ConnectionHealth, Telemetry};
    use std::time::{Duration, Instant};

    fn telemetry() -> Telemetry {
        Telemetry {
            uptime_ms: 100,
            state: 0,
            left_target_mm_s: 0,
            right_target_mm_s: 0,
            left_position_mm: 10,
            right_position_mm: 20,
            distance_mm: 500,
            battery_mv: 12_000,
            last_command_sequence: 7,
        }
    }

    #[test]
    fn tracks_telemetry_freshness_and_connection_health() {
        let start = Instant::now();
        let mut state = RobotState::default();

        assert_eq!(state.connection(), ConnectionHealth::Starting);
        assert!(state.telemetry_is_stale(start, Duration::from_millis(1_500)));

        state.record_telemetry(telemetry(), start);
        assert_eq!(state.connection(), ConnectionHealth::Connected);
        assert!(!state.telemetry_is_stale(
            start + Duration::from_millis(1_499),
            Duration::from_millis(1_500)
        ));
        assert!(state.telemetry_is_stale(
            start + Duration::from_millis(1_500),
            Duration::from_millis(1_500)
        ));
    }
}
