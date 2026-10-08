use super::supervisor::ConnectionSupervisor;
use crate::adapters::arduino::{ArduinoEvent, ArduinoSession, SensorStatus};
use crate::behaviors::SimulationBehavior;
use crate::config::Config;
use crate::control::{Behavior, SafetyController};
use crate::domain::{ActuatorCommand, MotionIntent, Observation, RobotState, Telemetry};
use crate::ports::MotionOutput;
use std::io;
use std::time::{Duration, Instant};

pub struct Runtime {
    config: Config,
}

impl Runtime {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    pub fn run(self) -> Result<(), Box<dyn std::error::Error>> {
        let mut telemetry_log = TelemetryLog::new(self.config.telemetry_log_interval);
        let (mut arduino, initial_telemetry) = ArduinoSession::connect(
            self.config.arduino_port,
            self.config.boot_delay,
            self.config.serial_timeout,
            self.config.heartbeat_interval,
            &mut |event| telemetry_log.display(event, Instant::now()),
        )?;

        let mut state = RobotState::default();
        state.apply(
            Observation::ArduinoTelemetry(initial_telemetry),
            Instant::now(),
        );

        let safety = SafetyController::new(self.config.telemetry_timeout);
        let mut supervisor =
            ConnectionSupervisor::new(self.config.heartbeat_interval, Instant::now());
        let mut behavior = SimulationBehavior::default();

        println!("Protocol synchronized. Starting simulation loop (Ctrl+C to stop).");
        let intent = behavior.start(&state, Instant::now());
        apply_intent(&mut arduino, &safety, &state, intent, Instant::now())?;

        loop {
            if let Some(event) = arduino.poll()? {
                if let ArduinoEvent::Telemetry(telemetry) = &event {
                    state.apply(
                        Observation::ArduinoTelemetry(telemetry.clone()),
                        Instant::now(),
                    );
                }
                telemetry_log.display(&event, Instant::now());
            }

            let now = Instant::now();
            if safety.requires_recovery(&state, now) {
                eprintln!(
                    "No valid telemetry for {}ms; recovering Arduino session...",
                    self.config.telemetry_timeout.as_millis()
                );
                state.mark_recovering();
                telemetry_log = TelemetryLog::new(self.config.telemetry_log_interval);

                let (recovered_arduino, safe_telemetry) =
                    arduino.recover(&mut |event| telemetry_log.display(event, Instant::now()))?;
                arduino = recovered_arduino;
                let recovered_at = Instant::now();
                state.apply(Observation::ArduinoTelemetry(safe_telemetry), recovered_at);

                println!("Session recovered. Restarting simulation from a safe state.");
                let intent = behavior.start(&state, recovered_at);
                apply_intent(&mut arduino, &safety, &state, intent, recovered_at)?;
                supervisor.reset_after_recovery(recovered_at);
                continue;
            }

            if let Some(intent) = behavior.update(&state, now) {
                apply_intent(&mut arduino, &safety, &state, intent, now)?;
            }

            if supervisor.heartbeat_due(now) {
                arduino.send_heartbeat()?;
            }
        }
    }
}

fn apply_intent(
    output: &mut ArduinoSession,
    safety: &SafetyController,
    state: &RobotState,
    intent: MotionIntent,
    now: Instant,
) -> io::Result<()> {
    let command = safety.authorize(state, intent, now);
    let sequence = output.apply(command)?;

    match command {
        ActuatorCommand::Stop => {
            println!("Pi -> STOP seq={sequence} phase={}", intent.name);
        }
        ActuatorCommand::SetWheelSpeeds {
            left_mm_s,
            right_mm_s,
        } => {
            println!(
                "Pi -> SET_MOTION seq={sequence} phase={} left={left_mm_s}mm/s right={right_mm_s}mm/s",
                intent.name
            );
        }
    }

    Ok(())
}

struct TelemetryLog {
    interval: Duration,
    last_display: Option<Instant>,
    last_state: Option<u8>,
    last_command_sequence: Option<u16>,
    last_sensor_display: Option<Instant>,
    last_pir_mask: Option<u8>,
    last_proximity_mask: Option<u8>,
    last_local_safety_mask: Option<u8>,
    last_sensor_flags: Option<u8>,
}

impl TelemetryLog {
    fn new(interval: Duration) -> Self {
        Self {
            interval,
            last_display: None,
            last_state: None,
            last_command_sequence: None,
            last_sensor_display: None,
            last_pir_mask: None,
            last_proximity_mask: None,
            last_local_safety_mask: None,
            last_sensor_flags: None,
        }
    }

    fn display(&mut self, event: &ArduinoEvent, now: Instant) {
        match event {
            ArduinoEvent::Ack { sequence } => println!("Arduino -> ACK seq={sequence}"),
            ArduinoEvent::Telemetry(telemetry) => {
                if self.should_display(telemetry, now) {
                    println!(
                        "Telemetry t={}ms state={}({}) target=({},{})mm/s position=({},{})mm distance={}mm battery={}mV last_cmd={}",
                        telemetry.uptime_ms,
                        telemetry.state,
                        telemetry.state_name(),
                        telemetry.left_target_mm_s,
                        telemetry.right_target_mm_s,
                        telemetry.left_position_mm,
                        telemetry.right_position_mm,
                        telemetry.distance_mm,
                        telemetry.battery_mv,
                        telemetry.last_command_sequence
                    );
                }
            }
            ArduinoEvent::SensorStatus(sensors) => {
                if self.should_display_sensors(sensors, now) {
                    let pir_left = presence_name(sensors.pir_mask & (1 << 0) != 0);
                    let pir_right = presence_name(sensors.pir_mask & (1 << 1) != 0);
                    let ground = if sensors.proximity_mask & (1 << 3) != 0 {
                        "ground"
                    } else {
                        "VOID"
                    };
                    let safety = if sensors.local_safety_mask == 0 {
                        "clear"
                    } else {
                        "STOP"
                    };
                    println!(
                        "Sensors t={}ms pir_ready={} pir=(left:{},right:{}) bumpers=({},{},{}) cliff={} safety={} ultrasonic=(A:{},B:{})",
                        sensors.uptime_ms,
                        yes_no(sensors.pir_ready()),
                        pir_left,
                        pir_right,
                        detected(sensors.proximity_mask, 0),
                        detected(sensors.proximity_mask, 1),
                        detected(sensors.proximity_mask, 2),
                        ground,
                        safety,
                        distance_text(sensors.ultrasonic_a_mm, sensors.ultrasonic_a_valid()),
                        distance_text(sensors.ultrasonic_b_mm, sensors.ultrasonic_b_valid()),
                    );
                }
            }
            ArduinoEvent::MotionStatus(status) => {
                if let Some(reason) = status.safety_reason_name() {
                    println!(
                        "Arduino virtual motors -> {} (safety: {reason})",
                        status.mode_name()
                    );
                } else {
                    println!("Arduino virtual motors -> {}", status.mode_name());
                }
            }
            ArduinoEvent::InvalidTelemetry => {
                eprintln!("Arduino -> invalid TELEMETRY payload");
            }
            ArduinoEvent::InvalidSensorStatus => {
                eprintln!("Arduino -> invalid SENSOR_STATUS payload");
            }
            ArduinoEvent::InvalidMotionStatus => {
                eprintln!("Arduino -> invalid MOTION_STATUS payload");
            }
            ArduinoEvent::Error { sequence, payload } => {
                eprintln!("Arduino -> ERROR seq={sequence} payload={payload:02x?}");
            }
            ArduinoEvent::Unexpected { message_type } => {
                eprintln!("Arduino -> unexpected message type 0x{message_type:02x}");
            }
        }
    }

    fn should_display(&mut self, telemetry: &Telemetry, now: Instant) -> bool {
        let changed = self.last_state != Some(telemetry.state)
            || self.last_command_sequence != Some(telemetry.last_command_sequence);
        let periodic = self
            .last_display
            .is_none_or(|last_display| now.duration_since(last_display) >= self.interval);

        self.last_state = Some(telemetry.state);
        self.last_command_sequence = Some(telemetry.last_command_sequence);

        if changed || periodic {
            self.last_display = Some(now);
            true
        } else {
            false
        }
    }

    fn should_display_sensors(&mut self, sensors: &SensorStatus, now: Instant) -> bool {
        let changed = self.last_pir_mask != Some(sensors.pir_mask)
            || self.last_proximity_mask != Some(sensors.proximity_mask)
            || self.last_local_safety_mask != Some(sensors.local_safety_mask)
            || self.last_sensor_flags != Some(sensors.flags);
        let periodic = self
            .last_sensor_display
            .is_none_or(|last_display| now.duration_since(last_display) >= self.interval);

        self.last_pir_mask = Some(sensors.pir_mask);
        self.last_proximity_mask = Some(sensors.proximity_mask);
        self.last_local_safety_mask = Some(sensors.local_safety_mask);
        self.last_sensor_flags = Some(sensors.flags);

        if changed || periodic {
            self.last_sensor_display = Some(now);
            true
        } else {
            false
        }
    }
}

fn presence_name(detected: bool) -> &'static str {
    if detected { "PRESENCE" } else { "clear" }
}

fn detected(mask: u8, bit: u8) -> &'static str {
    if mask & (1 << bit) != 0 {
        "BLOCKED"
    } else {
        "clear"
    }
}

fn yes_no(value: bool) -> &'static str {
    if value { "yes" } else { "no" }
}

fn distance_text(distance_mm: u16, valid: bool) -> String {
    if valid {
        format!("{distance_mm}mm")
    } else {
        "invalid".to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::TelemetryLog;
    use crate::domain::Telemetry;
    use std::time::{Duration, Instant};

    #[test]
    fn limits_periodic_logs_without_hiding_transitions() {
        let start = Instant::now();
        let mut log = TelemetryLog::new(Duration::from_secs(1));
        let mut telemetry = Telemetry {
            uptime_ms: 0,
            state: 1,
            left_target_mm_s: 200,
            right_target_mm_s: 200,
            left_position_mm: 0,
            right_position_mm: 0,
            distance_mm: 500,
            battery_mv: 12_000,
            last_command_sequence: 1,
        };

        assert!(log.should_display(&telemetry, start));
        assert!(!log.should_display(&telemetry, start + Duration::from_millis(100)));

        telemetry.state = 2;
        assert!(log.should_display(&telemetry, start + Duration::from_millis(200)));

        telemetry.last_command_sequence = 2;
        assert!(log.should_display(&telemetry, start + Duration::from_millis(300)));
        assert!(log.should_display(&telemetry, start + Duration::from_millis(1_300)));
    }
}
