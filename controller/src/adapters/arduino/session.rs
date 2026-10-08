use super::messages::{ArduinoEvent, decode_event};
use super::protocol::message_type;
use super::serial::SerialLink;
use crate::domain::{ActuatorCommand, Telemetry};
use crate::ports::MotionOutput;
use std::io;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

const HANDSHAKE_TIMEOUT: Duration = Duration::from_millis(500);
const HANDSHAKE_ATTEMPTS: usize = 10;
const SAFE_STATE_TIMEOUT: Duration = Duration::from_millis(1_500);

pub struct ArduinoSession {
    port_name: String,
    boot_delay: Duration,
    serial_timeout: Duration,
    heartbeat_interval: Duration,
    link: SerialLink,
}

impl ArduinoSession {
    pub fn connect(
        port_name: String,
        boot_delay: Duration,
        serial_timeout: Duration,
        heartbeat_interval: Duration,
        observer: &mut impl FnMut(&ArduinoEvent),
    ) -> io::Result<(Self, Telemetry)> {
        println!("Opening Arduino serial port: {port_name}");
        let link = SerialLink::open(&port_name, new_session(), boot_delay, serial_timeout)?;
        let mut session = Self {
            port_name,
            boot_delay,
            serial_timeout,
            heartbeat_interval,
            link,
        };
        let telemetry = session.establish_safe_session(observer)?;
        Ok((session, telemetry))
    }

    pub fn recover(
        self,
        observer: &mut impl FnMut(&ArduinoEvent),
    ) -> io::Result<(Self, Telemetry)> {
        let port_name = self.port_name.clone();
        let boot_delay = self.boot_delay;
        let serial_timeout = self.serial_timeout;
        let heartbeat_interval = self.heartbeat_interval;

        println!("Closing and reopening Arduino serial port: {port_name}");
        drop(self);

        Self::connect(
            port_name,
            boot_delay,
            serial_timeout,
            heartbeat_interval,
            observer,
        )
    }

    pub fn poll(&mut self) -> io::Result<Option<ArduinoEvent>> {
        self.link.poll().map(|frame| frame.map(decode_event))
    }

    pub fn send_heartbeat(&mut self) -> io::Result<()> {
        self.link.send(message_type::HEARTBEAT, &[])?;
        Ok(())
    }

    pub fn start_environment_scan(&mut self) -> io::Result<u16> {
        self.link.send(message_type::START_ENVIRONMENT_SCAN, &[])
    }

    fn establish_safe_session(
        &mut self,
        observer: &mut impl FnMut(&ArduinoEvent),
    ) -> io::Result<Telemetry> {
        self.handshake()?;
        self.confirm_safe_idle(observer)
    }

    fn handshake(&mut self) -> io::Result<()> {
        println!(
            "Synchronizing protocol session {:08x}...",
            self.link.session()
        );

        for attempt in 1..=HANDSHAKE_ATTEMPTS {
            let sequence = self.link.send(message_type::HELLO, &[])?;
            println!("Pi -> HELLO seq={sequence} attempt={attempt}");
            let deadline = Instant::now() + HANDSHAKE_TIMEOUT;

            while Instant::now() < deadline {
                if let Some(frame) = self.link.poll()?
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
        &mut self,
        observer: &mut impl FnMut(&ArduinoEvent),
    ) -> io::Result<Telemetry> {
        let stop_sequence = self.link.send(message_type::STOP, &[])?;
        println!("Pi -> STOP seq={stop_sequence} reason=session initialization");

        let deadline = Instant::now() + SAFE_STATE_TIMEOUT;
        let mut heartbeat_deadline = Instant::now() + self.heartbeat_interval;
        let mut stop_acknowledged = false;
        let mut safe_telemetry = None;

        while Instant::now() < deadline {
            let now = Instant::now();
            if now >= heartbeat_deadline {
                self.send_heartbeat()?;
                heartbeat_deadline = now + self.heartbeat_interval;
            }

            if let Some(frame) = self.link.poll()? {
                let event = decode_event(frame);
                match &event {
                    ArduinoEvent::Ack { sequence } if *sequence == stop_sequence => {
                        stop_acknowledged = true;
                    }
                    ArduinoEvent::Telemetry(telemetry)
                        if telemetry.confirms_stop(stop_sequence) =>
                    {
                        safe_telemetry = Some(telemetry.clone());
                    }
                    _ => {}
                }
                observer(&event);

                if stop_acknowledged && let Some(telemetry) = safe_telemetry {
                    println!(
                        "Arduino safe state confirmed for session {:08x}",
                        self.link.session()
                    );
                    return Ok(telemetry);
                }
            }
        }

        Err(io::Error::new(
            io::ErrorKind::TimedOut,
            "Arduino did not confirm STOP and safe telemetry",
        ))
    }
}

impl MotionOutput for ArduinoSession {
    type Error = io::Error;

    fn apply(&mut self, command: ActuatorCommand) -> Result<u16, Self::Error> {
        match command {
            ActuatorCommand::SetWheelSpeeds {
                left_mm_s,
                right_mm_s,
            } => {
                let mut payload = Vec::with_capacity(4);
                payload.extend_from_slice(&left_mm_s.to_le_bytes());
                payload.extend_from_slice(&right_mm_s.to_le_bytes());
                self.link.send(message_type::SET_MOTION, &payload)
            }
            ActuatorCommand::Stop => self.link.send(message_type::STOP, &[]),
        }
    }
}

fn new_session() -> u32 {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    let session = (now.as_secs() as u32) ^ now.subsec_nanos();
    session.max(1)
}
