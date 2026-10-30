use super::supervisor::ConnectionSupervisor;
use crate::adapters::arduino::{ArduinoEvent, ArduinoSession, SensorStatus, UltrasonicDiagnostic};
use crate::behaviors::PresenceTestBehavior;
use crate::config::{Config, LogMode};
use crate::control::{Behavior, SafetyController};
use crate::domain::{
    ActuatorCommand, MotionIntent, Observation, RobotState, SensorSnapshot, Telemetry,
};
use crate::ports::MotionOutput;
use std::collections::VecDeque;
use std::io;
use std::time::{Duration, Instant};

const DISTANCE_DIAGNOSTIC_WINDOW: usize = 20;

pub struct Runtime {
    config: Config,
}

impl Runtime {
    pub fn new(config: Config) -> Self {
        Self { config }
    }

    pub fn run(self) -> Result<(), Box<dyn std::error::Error>> {
        let mut telemetry_log =
            TelemetryLog::new(self.config.telemetry_log_interval, self.config.log_mode);
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
        let mut behavior = PresenceTestBehavior::default();

        println!("[TEST] PIR rotation test ready (Ctrl+C to stop)");
        let intent = behavior.start(&state, Instant::now());
        apply_intent(&mut arduino, &safety, &state, intent, Instant::now())?;
        start_environment_scan(&mut arduino)?;

        loop {
            if let Some(event) = arduino.poll()? {
                let received_at = Instant::now();
                match &event {
                    ArduinoEvent::Telemetry(telemetry) => state.apply(
                        Observation::ArduinoTelemetry(telemetry.clone()),
                        received_at,
                    ),
                    ArduinoEvent::SensorStatus(sensors) => state.apply(
                        Observation::ArduinoSensors(SensorSnapshot {
                            pir_ready: sensors.pir_ready(),
                            pir_mask: sensors.pir_mask,
                            proximity_mask: sensors.proximity_mask,
                            local_safety_mask: sensors.local_safety_mask,
                            ultrasonic_a_mm: sensors
                                .ultrasonic_a_valid()
                                .then_some(sensors.ultrasonic_a_mm),
                            ultrasonic_b_mm: sensors
                                .ultrasonic_b_valid()
                                .then_some(sensors.ultrasonic_b_mm),
                        }),
                        received_at,
                    ),
                    _ => {}
                }
                telemetry_log.display(&event, received_at);
            }

            let now = Instant::now();
            if safety.requires_recovery(&state, now) {
                eprintln!(
                    "[RECOVERY] no valid telemetry for {}ms; reconnecting Arduino",
                    self.config.telemetry_timeout.as_millis()
                );
                state.mark_recovering();
                telemetry_log =
                    TelemetryLog::new(self.config.telemetry_log_interval, self.config.log_mode);

                let (recovered_arduino, safe_telemetry) =
                    arduino.recover(&mut |event| telemetry_log.display(event, Instant::now()))?;
                arduino = recovered_arduino;
                let recovered_at = Instant::now();
                state.apply(Observation::ArduinoTelemetry(safe_telemetry), recovered_at);

                println!("[TEST] session recovered; restarting from a safe state");
                let intent = behavior.start(&state, recovered_at);
                apply_intent(&mut arduino, &safety, &state, intent, recovered_at)?;
                start_environment_scan(&mut arduino)?;
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

fn start_environment_scan(arduino: &mut ArduinoSession) -> io::Result<()> {
    let sequence = arduino.start_environment_scan()?;
    println!("[TX] START_ENVIRONMENT_SCAN seq={sequence}");
    Ok(())
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
            println!("[TX] STOP seq={sequence} test=\"{}\"", intent.name);
        }
        ActuatorCommand::SetWheelSpeeds {
            left_mm_s,
            right_mm_s,
        } => {
            println!(
                "[TX] SET_MOTION seq={sequence} test=\"{}\" wheels=({left_mm_s},{right_mm_s})mm/s",
                intent.name
            );
        }
    }

    Ok(())
}

struct TelemetryLog {
    mode: LogMode,
    interval: Duration,
    last_display: Option<Instant>,
    last_state: Option<u8>,
    last_command_sequence: Option<u16>,
    last_sensor_display: Option<Instant>,
    last_pir_mask: Option<u8>,
    last_proximity_mask: Option<u8>,
    last_local_safety_mask: Option<u8>,
    last_sensor_flags: Option<u8>,
    last_distance_display: Option<Instant>,
    distance_samples: VecDeque<DistanceSample>,
    scan_measurements: usize,
    scan_valid_measurements: usize,
    scan_nearest: Option<ScanNearest>,
}

#[derive(Clone, Copy)]
struct ScanNearest {
    sensor: char,
    angle_deg: u16,
    distance_mm: u16,
}

#[derive(Clone, Copy)]
struct DistanceSample {
    distance_mm: u16,
    echo_duration_us: u32,
    valid: bool,
}

struct DistanceStats {
    valid_count: usize,
    average_mm: u16,
    average_echo_us: u32,
    minimum_mm: u16,
    maximum_mm: u16,
}

impl TelemetryLog {
    fn new(interval: Duration, mode: LogMode) -> Self {
        Self {
            mode,
            interval,
            last_display: None,
            last_state: None,
            last_command_sequence: None,
            last_sensor_display: None,
            last_pir_mask: None,
            last_proximity_mask: None,
            last_local_safety_mask: None,
            last_sensor_flags: None,
            last_distance_display: None,
            distance_samples: VecDeque::with_capacity(DISTANCE_DIAGNOSTIC_WINDOW),
            scan_measurements: 0,
            scan_valid_measurements: 0,
            scan_nearest: None,
        }
    }

    fn display(&mut self, event: &ArduinoEvent, now: Instant) {
        match event {
            ArduinoEvent::Ack { sequence } => println!("[RX] ACK seq={sequence}"),
            ArduinoEvent::Telemetry(telemetry) => {
                if self.should_display(telemetry, now) {
                    println!(
                        "[STATE] t={}ms mode={} target=({},{})mm/s position=({},{})mm distance={}mm battery={}mV last_cmd={}",
                        telemetry.uptime_ms,
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
                    let pir_front = presence_name(sensors.pir_mask & (1 << 0) != 0);
                    let pir_back = presence_name(sensors.pir_mask & (1 << 1) != 0);
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
                        "[SENSORS] t={}ms PIR={} front={} back={} | bumpers={}/{}/{} cliff={} safety={}{}",
                        sensors.uptime_ms,
                        if sensors.pir_ready() {
                            "READY"
                        } else {
                            "warming-up"
                        },
                        pir_front,
                        pir_back,
                        detected(sensors.proximity_mask, 0),
                        detected(sensors.proximity_mask, 1),
                        detected(sensors.proximity_mask, 2),
                        ground,
                        safety,
                        self.ultrasonic_suffix(sensors),
                    );
                }
            }
            ArduinoEvent::UltrasonicDiagnostic(diagnostic) => {
                self.record_distance(diagnostic);
                if self.should_display_distance(now) {
                    self.display_distance_diagnostic(diagnostic);
                }
            }
            ArduinoEvent::MotionStatus(status) => {
                if let Some(reason) = status.safety_reason_name() {
                    println!("[MOTION] {} (safety: {reason})", status.mode_name());
                } else {
                    println!("[MOTION] {}", status.mode_name());
                }
            }
            ArduinoEvent::EnvironmentScanSample(sample) => {
                self.record_scan_sample(sample);
                if self.mode == LogMode::Verbose {
                    println!(
                        "[SCAN] A {:03}deg={} | B {:03}deg={}",
                        sample.angle_a_deg,
                        distance_text(sample.distance_a_mm, sample.sensor_a_valid()),
                        sample.angle_b_deg,
                        distance_text(sample.distance_b_mm, sample.sensor_b_valid()),
                    );
                }
            }
            ArduinoEvent::EnvironmentScanStatus(status) => match status.state {
                1 => {
                    self.reset_scan();
                    println!("[SCAN] started");
                }
                2 => self.display_scan_summary(),
                _ => println!("[SCAN] status={}", status.state_name()),
            },
            ArduinoEvent::InvalidTelemetry => {
                eprintln!("[ERROR] invalid TELEMETRY payload");
            }
            ArduinoEvent::InvalidSensorStatus => {
                eprintln!("[ERROR] invalid SENSOR_STATUS payload");
            }
            ArduinoEvent::InvalidMotionStatus => {
                eprintln!("[ERROR] invalid MOTION_STATUS payload");
            }
            ArduinoEvent::InvalidEnvironmentScanSample => {
                eprintln!("[ERROR] invalid ENVIRONMENT_SCAN_SAMPLE payload");
            }
            ArduinoEvent::InvalidEnvironmentScanStatus => {
                eprintln!("[ERROR] invalid ENVIRONMENT_SCAN_STATUS payload");
            }
            ArduinoEvent::InvalidUltrasonicDiagnostic => {
                eprintln!("[ERROR] invalid ULTRASONIC_DIAGNOSTIC payload");
            }
            ArduinoEvent::Error { sequence, payload } => {
                eprintln!("[ERROR] Arduino seq={sequence} payload={payload:02x?}");
            }
            ArduinoEvent::Unexpected { message_type } => {
                eprintln!("[ERROR] unexpected message type 0x{message_type:02x}");
            }
        }
    }

    fn should_display(&mut self, telemetry: &Telemetry, now: Instant) -> bool {
        let changed = self.last_state != Some(telemetry.state)
            || self.last_command_sequence != Some(telemetry.last_command_sequence);
        let periodic = self.mode == LogMode::Verbose
            && self
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
        let meaningful_flags = sensors.flags & 0x01;
        let changed = self.last_pir_mask != Some(sensors.pir_mask)
            || self.last_proximity_mask != Some(sensors.proximity_mask)
            || self.last_local_safety_mask != Some(sensors.local_safety_mask)
            || self.last_sensor_flags != Some(meaningful_flags);
        let periodic = self.mode == LogMode::Verbose
            && self
                .last_sensor_display
                .is_none_or(|last_display| now.duration_since(last_display) >= self.interval);

        self.last_pir_mask = Some(sensors.pir_mask);
        self.last_proximity_mask = Some(sensors.proximity_mask);
        self.last_local_safety_mask = Some(sensors.local_safety_mask);
        self.last_sensor_flags = Some(meaningful_flags);

        if changed || periodic {
            self.last_sensor_display = Some(now);
            true
        } else {
            false
        }
    }

    fn ultrasonic_suffix(&self, sensors: &SensorStatus) -> String {
        if self.mode == LogMode::Verbose {
            format!(
                " | ultrasonic=A:{} B:{}",
                distance_text(sensors.ultrasonic_a_mm, sensors.ultrasonic_a_valid()),
                distance_text(sensors.ultrasonic_b_mm, sensors.ultrasonic_b_valid()),
            )
        } else {
            String::new()
        }
    }

    fn should_display_distance(&mut self, now: Instant) -> bool {
        if self
            .last_distance_display
            .is_some_and(|last| now.duration_since(last) < self.interval)
        {
            return false;
        }

        self.last_distance_display = Some(now);
        true
    }

    fn record_distance(&mut self, diagnostic: &UltrasonicDiagnostic) {
        if self.distance_samples.len() == DISTANCE_DIAGNOSTIC_WINDOW {
            self.distance_samples.pop_front();
        }
        self.distance_samples.push_back(DistanceSample {
            distance_mm: diagnostic.distance_mm,
            echo_duration_us: diagnostic.echo_duration_us,
            valid: diagnostic.valid(),
        });
    }

    fn distance_stats(&self) -> Option<DistanceStats> {
        let mut valid_count = 0_usize;
        let mut distance_sum = 0_u64;
        let mut echo_sum = 0_u64;
        let mut minimum_mm = u16::MAX;
        let mut maximum_mm = u16::MIN;

        for sample in self.distance_samples.iter().filter(|sample| sample.valid) {
            valid_count += 1;
            distance_sum += u64::from(sample.distance_mm);
            echo_sum += u64::from(sample.echo_duration_us);
            minimum_mm = minimum_mm.min(sample.distance_mm);
            maximum_mm = maximum_mm.max(sample.distance_mm);
        }

        if valid_count == 0 {
            return None;
        }

        let count = valid_count as u64;
        Some(DistanceStats {
            valid_count,
            average_mm: ((distance_sum + count / 2) / count) as u16,
            average_echo_us: ((echo_sum + count / 2) / count) as u32,
            minimum_mm,
            maximum_mm,
        })
    }

    fn display_distance_diagnostic(&self, diagnostic: &UltrasonicDiagnostic) {
        let total = self.distance_samples.len();
        let last = if diagnostic.valid() {
            format!(
                "A={}mm echo={}us",
                diagnostic.distance_mm, diagnostic.echo_duration_us
            )
        } else {
            format!("A=invalid reason={}", diagnostic.result_name())
        };

        if let Some(stats) = self.distance_stats() {
            println!(
                "[DISTANCE] {last} | valid={}/{} avg={}mm/{}us min={} max={} spread={}mm",
                stats.valid_count,
                total,
                stats.average_mm,
                stats.average_echo_us,
                stats.minimum_mm,
                stats.maximum_mm,
                stats.maximum_mm - stats.minimum_mm,
            );
        } else {
            println!("[DISTANCE] {last} | valid=0/{total}");
        }
    }

    fn reset_scan(&mut self) {
        self.scan_measurements = 0;
        self.scan_valid_measurements = 0;
        self.scan_nearest = None;
    }

    fn record_scan_sample(&mut self, sample: &crate::adapters::arduino::EnvironmentScanSample) {
        self.record_scan_measurement(
            'A',
            sample.angle_a_deg,
            sample.distance_a_mm,
            sample.sensor_a_valid(),
        );
        self.record_scan_measurement(
            'B',
            sample.angle_b_deg,
            sample.distance_b_mm,
            sample.sensor_b_valid(),
        );
    }

    fn record_scan_measurement(
        &mut self,
        sensor: char,
        angle_deg: u16,
        distance_mm: u16,
        valid: bool,
    ) {
        self.scan_measurements += 1;
        if !valid {
            return;
        }

        self.scan_valid_measurements += 1;
        if self
            .scan_nearest
            .is_none_or(|nearest| distance_mm < nearest.distance_mm)
        {
            self.scan_nearest = Some(ScanNearest {
                sensor,
                angle_deg,
                distance_mm,
            });
        }
    }

    fn display_scan_summary(&self) {
        let quality = if self.scan_measurements == 0 {
            "no samples".to_string()
        } else {
            format!(
                "{}/{} valid",
                self.scan_valid_measurements, self.scan_measurements
            )
        };
        let nearest = self.scan_nearest.map_or_else(
            || "nearest=none".to_string(),
            |nearest| {
                format!(
                    "nearest={}mm (sensor {}, {:03}deg)",
                    nearest.distance_mm, nearest.sensor, nearest.angle_deg
                )
            },
        );
        println!("[SCAN] complete: {quality}, {nearest}");
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
    use crate::adapters::arduino::{SensorStatus, UltrasonicDiagnostic};
    use crate::config::LogMode;
    use crate::domain::Telemetry;
    use std::time::{Duration, Instant};

    #[test]
    fn limits_periodic_logs_without_hiding_transitions() {
        let start = Instant::now();
        let mut log = TelemetryLog::new(Duration::from_secs(1), LogMode::Verbose);
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

    #[test]
    fn compact_logs_only_meaningful_sensor_changes() {
        let start = Instant::now();
        let mut log = TelemetryLog::new(Duration::from_secs(1), LogMode::Compact);
        let mut sensors = SensorStatus {
            uptime_ms: 0,
            pir_mask: 0,
            proximity_mask: 0x08,
            local_safety_mask: 0,
            flags: 0,
            ultrasonic_a_mm: 0,
            ultrasonic_b_mm: 0,
        };

        assert!(log.should_display_sensors(&sensors, start));
        sensors.flags |= 1 << 2;
        sensors.ultrasonic_b_mm = 250;
        assert!(!log.should_display_sensors(&sensors, start + Duration::from_secs(2)));

        sensors.flags |= 1;
        assert!(log.should_display_sensors(&sensors, start + Duration::from_secs(3)));
        sensors.pir_mask = 1;
        assert!(log.should_display_sensors(&sensors, start + Duration::from_secs(4)));
    }

    #[test]
    fn distance_is_displayed_once_per_interval() {
        let start = Instant::now();
        let mut log = TelemetryLog::new(Duration::from_secs(1), LogMode::Compact);

        assert!(log.should_display_distance(start));
        assert!(!log.should_display_distance(start + Duration::from_millis(999)));
        assert!(log.should_display_distance(start + Duration::from_secs(1)));
    }

    #[test]
    fn computes_rolling_distance_diagnostics() {
        let mut log = TelemetryLog::new(Duration::from_secs(1), LogMode::Compact);
        log.record_distance(&UltrasonicDiagnostic {
            distance_mm: 700,
            echo_duration_us: 4_082,
            result: 1,
        });
        log.record_distance(&UltrasonicDiagnostic {
            distance_mm: 710,
            echo_duration_us: 4_140,
            result: 1,
        });
        log.record_distance(&UltrasonicDiagnostic {
            distance_mm: u16::MAX,
            echo_duration_us: 0,
            result: 2,
        });

        let stats = log.distance_stats().unwrap();
        assert_eq!(stats.valid_count, 2);
        assert_eq!(stats.average_mm, 705);
        assert_eq!(stats.average_echo_us, 4_111);
        assert_eq!(stats.minimum_mm, 700);
        assert_eq!(stats.maximum_mm, 710);
    }
}
