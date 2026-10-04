mod protocol;

use protocol::{Frame, FrameDecoder, message_type};
use serialport::{ClearBuffer, SerialPort};
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const SERIAL_TIMEOUT: Duration = Duration::from_millis(50);
const DEFAULT_BOOT_DELAY: Duration = Duration::from_millis(2_000);
const HANDSHAKE_TIMEOUT: Duration = Duration::from_millis(500);
const HANDSHAKE_ATTEMPTS: usize = 10;
const HEARTBEAT_INTERVAL: Duration = Duration::from_millis(250);
const TELEMETRY_TIMEOUT: Duration = Duration::from_millis(1_500);
const SAFE_STATE_TIMEOUT: Duration = Duration::from_millis(1_500);
const TELEMETRY_LOG_INTERVAL: Duration = Duration::from_secs(1);

struct SerialLink {
    port: Box<dyn SerialPort>,
    decoder: FrameDecoder,
    pending_frames: VecDeque<Frame>,
    session: u32,
    next_sequence: u16,
}

impl SerialLink {
    fn open(port_name: &str, session: u32, boot_delay: Duration) -> io::Result<Self> {
        let port = serialport::new(port_name, 115_200)
            .timeout(SERIAL_TIMEOUT)
            .open()?;

        println!(
            "Waiting {}ms for the Arduino bootloader and firmware...",
            boot_delay.as_millis()
        );
        std::thread::sleep(boot_delay);
        port.clear(ClearBuffer::Input)?;

        Ok(Self {
            port,
            decoder: FrameDecoder::default(),
            pending_frames: VecDeque::new(),
            session,
            next_sequence: 1,
        })
    }

    fn send(&mut self, message_type: u8, payload: &[u8]) -> io::Result<u16> {
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.wrapping_add(1);

        let bytes = Frame {
            message_type,
            session: self.session,
            sequence,
            payload: payload.to_vec(),
        }
        .encode()
        .map_err(|error| io::Error::new(io::ErrorKind::InvalidInput, error))?;

        self.port.write_all(&bytes)?;
        self.port.flush()?;
        Ok(sequence)
    }

    fn poll(&mut self) -> io::Result<Option<Frame>> {
        if let Some(frame) = self.pending_frames.pop_front() {
            return Ok(Some(frame));
        }

        let mut buffer = [0_u8; 128];
        match self.port.read(&mut buffer) {
            Ok(0) => Ok(None),
            Ok(size) => {
                for byte in &buffer[..size] {
                    match self.decoder.push(*byte) {
                        Some(Ok(frame)) if frame.session == self.session => {
                            self.pending_frames.push_back(frame);
                        }
                        Some(Ok(frame)) => {
                            eprintln!(
                                "Ignoring stale session {:08x} (current {:08x})",
                                frame.session, self.session
                            );
                        }
                        Some(Err(error)) => eprintln!("Discarding invalid serial frame: {error}"),
                        None => {}
                    }
                }

                Ok(self.pending_frames.pop_front())
            }
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) =>
            {
                Ok(None)
            }
            Err(error) => Err(error),
        }
    }
}

#[derive(Clone, Copy)]
struct MotionPhase {
    name: &'static str,
    left_mm_s: i16,
    right_mm_s: i16,
    duration: Duration,
}

const MOTION_SCRIPT: [MotionPhase; 4] = [
    MotionPhase {
        name: "forward",
        left_mm_s: 200,
        right_mm_s: 200,
        duration: Duration::from_secs(3),
    },
    MotionPhase {
        name: "rotate right",
        left_mm_s: 150,
        right_mm_s: -150,
        duration: Duration::from_secs(2),
    },
    MotionPhase {
        name: "reverse",
        left_mm_s: -150,
        right_mm_s: -150,
        duration: Duration::from_secs(3),
    },
    MotionPhase {
        name: "stop",
        left_mm_s: 0,
        right_mm_s: 0,
        duration: Duration::from_secs(2),
    },
];

fn new_session() -> u32 {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let session = (now.as_secs() as u32) ^ now.subsec_nanos();
    session.max(1)
}

fn boot_delay() -> io::Result<Duration> {
    let Some(value) = std::env::var_os("ARDUINO_BOOT_DELAY_MS") else {
        return Ok(DEFAULT_BOOT_DELAY);
    };
    let value = value.to_string_lossy();
    let milliseconds = value.parse::<u64>().map_err(|_| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("invalid ARDUINO_BOOT_DELAY_MS value: {value}"),
        )
    })?;

    Ok(Duration::from_millis(milliseconds))
}

fn handshake(link: &mut SerialLink) -> io::Result<()> {
    println!("Synchronizing protocol session {:08x}...", link.session);

    for attempt in 1..=HANDSHAKE_ATTEMPTS {
        let sequence = link.send(message_type::HELLO, &[])?;
        println!("Pi -> HELLO seq={sequence} attempt={attempt}");
        let deadline = Instant::now() + HANDSHAKE_TIMEOUT;

        while Instant::now() < deadline {
            if let Some(frame) = link.poll()?
                && frame.message_type == message_type::HELLO_ACK
                && frame.sequence == sequence
            {
                println!("Arduino -> HELLO_ACK seq={sequence}");
                return Ok(());
            }
        }
    }

    Err(io::Error::new(
        io::ErrorKind::TimedOut,
        "Arduino did not acknowledge the protocol handshake",
    ))
}

fn confirm_safe_idle(
    link: &mut SerialLink,
    telemetry_log: &mut TelemetryLog,
) -> io::Result<Instant> {
    let stop_sequence = link.send(message_type::STOP, &[])?;
    println!("Pi -> STOP seq={stop_sequence} reason=session initialization");

    let deadline = Instant::now() + SAFE_STATE_TIMEOUT;
    let mut heartbeat_deadline = Instant::now() + HEARTBEAT_INTERVAL;
    let mut stop_acknowledged = false;
    let mut idle_confirmed = false;
    let mut last_telemetry = None;

    while Instant::now() < deadline {
        let now = Instant::now();
        if now >= heartbeat_deadline {
            link.send(message_type::HEARTBEAT, &[])?;
            heartbeat_deadline = now + HEARTBEAT_INTERVAL;
        }

        if let Some(frame) = link.poll()? {
            if frame.message_type == message_type::ACK && frame.sequence == stop_sequence {
                stop_acknowledged = true;
            }

            if frame.message_type == message_type::TELEMETRY
                && let Some(telemetry) = Telemetry::decode(&frame.payload)
            {
                last_telemetry = Some(Instant::now());
                idle_confirmed = telemetry.confirms_stop(stop_sequence);
            }

            display_frame(frame, telemetry_log);

            if stop_acknowledged && idle_confirmed {
                println!(
                    "Arduino safe state confirmed for session {:08x}",
                    link.session
                );
                return Ok(last_telemetry.unwrap_or_else(Instant::now));
            }
        }
    }

    Err(io::Error::new(
        io::ErrorKind::TimedOut,
        "Arduino did not confirm STOP and idle telemetry",
    ))
}

fn establish_safe_session(
    link: &mut SerialLink,
    telemetry_log: &mut TelemetryLog,
) -> io::Result<Instant> {
    handshake(link)?;
    confirm_safe_idle(link, telemetry_log)
}

fn send_motion(link: &mut SerialLink, phase: MotionPhase) -> io::Result<()> {
    if phase.left_mm_s == 0 && phase.right_mm_s == 0 {
        let sequence = link.send(message_type::STOP, &[])?;
        println!("Pi -> STOP seq={sequence} phase={}", phase.name);
        return Ok(());
    }

    let mut payload = Vec::with_capacity(4);
    payload.extend_from_slice(&phase.left_mm_s.to_le_bytes());
    payload.extend_from_slice(&phase.right_mm_s.to_le_bytes());
    let sequence = link.send(message_type::SET_MOTION, &payload)?;

    println!(
        "Pi -> SET_MOTION seq={sequence} phase={} left={}mm/s right={}mm/s",
        phase.name, phase.left_mm_s, phase.right_mm_s
    );
    Ok(())
}

fn display_frame(frame: Frame, telemetry_log: &mut TelemetryLog) -> bool {
    match frame.message_type {
        message_type::ACK => {
            println!("Arduino -> ACK seq={}", frame.sequence);
            false
        }
        message_type::TELEMETRY => match Telemetry::decode(&frame.payload) {
            Some(telemetry) => {
                if telemetry_log.should_display(&telemetry, Instant::now()) {
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
                true
            }
            None => {
                eprintln!("Arduino -> invalid TELEMETRY payload");
                false
            }
        },
        message_type::ERROR => {
            eprintln!(
                "Arduino -> ERROR seq={} payload={:02x?}",
                frame.sequence, frame.payload
            );
            false
        }
        other => {
            eprintln!("Arduino -> unexpected message type 0x{other:02x}");
            false
        }
    }
}

fn telemetry_timed_out(last_telemetry: Instant, now: Instant) -> bool {
    now.duration_since(last_telemetry) >= TELEMETRY_TIMEOUT
}

#[derive(Default)]
struct TelemetryLog {
    last_display: Option<Instant>,
    last_state: Option<u8>,
    last_command_sequence: Option<u16>,
}

impl TelemetryLog {
    fn should_display(&mut self, telemetry: &Telemetry, now: Instant) -> bool {
        let changed = self.last_state != Some(telemetry.state)
            || self.last_command_sequence != Some(telemetry.last_command_sequence);
        let periodic = self
            .last_display
            .is_none_or(|last_display| now.duration_since(last_display) >= TELEMETRY_LOG_INTERVAL);

        self.last_state = Some(telemetry.state);
        self.last_command_sequence = Some(telemetry.last_command_sequence);

        if changed || periodic {
            self.last_display = Some(now);
            true
        } else {
            false
        }
    }
}

struct Telemetry {
    uptime_ms: u32,
    state: u8,
    left_target_mm_s: i16,
    right_target_mm_s: i16,
    left_position_mm: i32,
    right_position_mm: i32,
    distance_mm: u16,
    battery_mv: u16,
    last_command_sequence: u16,
}

impl Telemetry {
    fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != 23 {
            return None;
        }

        Some(Self {
            uptime_ms: u32::from_le_bytes(payload[0..4].try_into().ok()?),
            state: payload[4],
            left_target_mm_s: i16::from_le_bytes(payload[5..7].try_into().ok()?),
            right_target_mm_s: i16::from_le_bytes(payload[7..9].try_into().ok()?),
            left_position_mm: i32::from_le_bytes(payload[9..13].try_into().ok()?),
            right_position_mm: i32::from_le_bytes(payload[13..17].try_into().ok()?),
            distance_mm: u16::from_le_bytes(payload[17..19].try_into().ok()?),
            battery_mv: u16::from_le_bytes(payload[19..21].try_into().ok()?),
            last_command_sequence: u16::from_le_bytes(payload[21..23].try_into().ok()?),
        })
    }

    fn state_name(&self) -> &'static str {
        match self.state {
            0 => "idle",
            1 => "moving",
            2 => "watchdog",
            _ => "unknown",
        }
    }

    fn confirms_stop(&self, stop_sequence: u16) -> bool {
        self.state == 0
            && self.left_target_mm_s == 0
            && self.right_target_mm_s == 0
            && self.last_command_sequence == stop_sequence
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port_name = std::env::var("ARDUINO_PORT").unwrap_or_else(|_| "/dev/ttyACM0".to_string());
    let session = new_session();
    let boot_delay = boot_delay()?;

    println!("Opening Arduino serial port: {port_name}");
    let mut link = SerialLink::open(&port_name, session, boot_delay)?;
    let mut telemetry_log = TelemetryLog::default();
    let mut last_telemetry = establish_safe_session(&mut link, &mut telemetry_log)?;

    println!("Protocol synchronized. Starting simulation loop (Ctrl+C to stop).");
    let mut phase_index = 0;
    let mut phase_deadline = Instant::now() + MOTION_SCRIPT[phase_index].duration;
    let mut heartbeat_deadline = Instant::now();
    send_motion(&mut link, MOTION_SCRIPT[phase_index])?;

    loop {
        if let Some(frame) = link.poll()?
            && display_frame(frame, &mut telemetry_log)
        {
            last_telemetry = Instant::now();
        }

        let now = Instant::now();
        if telemetry_timed_out(last_telemetry, now) {
            eprintln!(
                "No valid telemetry for {}ms; recovering Arduino session...",
                TELEMETRY_TIMEOUT.as_millis()
            );

            println!("Closing and reopening Arduino serial port: {port_name}");
            drop(link);
            link = SerialLink::open(&port_name, new_session(), boot_delay)?;
            telemetry_log = TelemetryLog::default();
            last_telemetry = establish_safe_session(&mut link, &mut telemetry_log)?;

            println!("Session recovered. Restarting simulation from a safe state.");
            phase_index = 0;
            send_motion(&mut link, MOTION_SCRIPT[phase_index])?;
            let recovered_at = Instant::now();
            phase_deadline = recovered_at + MOTION_SCRIPT[phase_index].duration;
            heartbeat_deadline = recovered_at + HEARTBEAT_INTERVAL;
            continue;
        }

        if now >= phase_deadline {
            phase_index = (phase_index + 1) % MOTION_SCRIPT.len();
            let phase = MOTION_SCRIPT[phase_index];
            send_motion(&mut link, phase)?;
            phase_deadline = now + phase.duration;
        }

        if now >= heartbeat_deadline {
            link.send(message_type::HEARTBEAT, &[])?;
            heartbeat_deadline = now + HEARTBEAT_INTERVAL;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Telemetry, TelemetryLog, telemetry_timed_out};
    use std::time::{Duration, Instant};

    #[test]
    fn decodes_simulated_telemetry() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&1_234_u32.to_le_bytes());
        payload.push(1);
        payload.extend_from_slice(&200_i16.to_le_bytes());
        payload.extend_from_slice(&(-100_i16).to_le_bytes());
        payload.extend_from_slice(&500_i32.to_le_bytes());
        payload.extend_from_slice(&(-250_i32).to_le_bytes());
        payload.extend_from_slice(&420_u16.to_le_bytes());
        payload.extend_from_slice(&11_900_u16.to_le_bytes());
        payload.extend_from_slice(&7_u16.to_le_bytes());

        let telemetry = Telemetry::decode(&payload).unwrap();

        assert_eq!(telemetry.uptime_ms, 1_234);
        assert_eq!(telemetry.left_target_mm_s, 200);
        assert_eq!(telemetry.right_position_mm, -250);
        assert_eq!(telemetry.distance_mm, 420);
        assert_eq!(telemetry.last_command_sequence, 7);
    }

    #[test]
    fn limits_periodic_logs_without_hiding_transitions() {
        let start = Instant::now();
        let mut log = TelemetryLog::default();
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
    fn detects_telemetry_timeout_at_limit() {
        let start = Instant::now();

        assert!(!telemetry_timed_out(
            start,
            start + Duration::from_millis(1_499)
        ));
        assert!(telemetry_timed_out(
            start,
            start + Duration::from_millis(1_500)
        ));
    }

    #[test]
    fn confirms_stop_only_for_matching_idle_telemetry() {
        let mut telemetry = Telemetry {
            uptime_ms: 100,
            state: 0,
            left_target_mm_s: 0,
            right_target_mm_s: 0,
            left_position_mm: 10,
            right_position_mm: 20,
            distance_mm: 500,
            battery_mv: 12_000,
            last_command_sequence: 7,
        };

        assert!(telemetry.confirms_stop(7));
        assert!(!telemetry.confirms_stop(8));

        telemetry.state = 1;
        assert!(!telemetry.confirms_stop(7));
    }
}
