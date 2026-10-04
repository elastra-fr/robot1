use super::protocol::{Frame, FrameDecoder};
use serialport::{ClearBuffer, SerialPort};
use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::time::Duration;

const BAUD_RATE: u32 = 115_200;

pub(super) struct SerialLink {
    port: Box<dyn SerialPort>,
    decoder: FrameDecoder,
    pending_frames: VecDeque<Frame>,
    session: u32,
    next_sequence: u16,
}

impl SerialLink {
    pub(super) fn open(
        port_name: &str,
        session: u32,
        boot_delay: Duration,
        serial_timeout: Duration,
    ) -> io::Result<Self> {
        let port = serialport::new(port_name, BAUD_RATE)
            .timeout(serial_timeout)
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

    pub(super) fn session(&self) -> u32 {
        self.session
    }

    pub(super) fn send(&mut self, message_type: u8, payload: &[u8]) -> io::Result<u16> {
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

    pub(super) fn poll(&mut self) -> io::Result<Option<Frame>> {
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
