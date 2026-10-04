use std::io::{self, BufRead, BufReader, Write};
use std::time::Duration;

fn read_message(reader: &mut impl BufRead) -> io::Result<String> {
    let mut line = String::new();

    if reader.read_line(&mut line)? == 0 {
        return Err(io::Error::new(
            io::ErrorKind::UnexpectedEof,
            "Arduino serial connection closed",
        ));
    }

    let message = line.trim().to_string();
    println!("Arduino -> {message}");

    Ok(message)
}

fn is_ready_message(message: &str) -> bool {
    let Some(fragment) = message.strip_suffix("READY") else {
        return false;
    };

    fragment.is_empty() || "READY".starts_with(fragment)
}

fn wait_for(reader: &mut impl BufRead, expected: &str) -> io::Result<()> {
    loop {
        let message = read_message(reader)?;
        if message == expected || (expected == "READY" && is_ready_message(&message)) {
            return Ok(());
        }
    }
}

fn send_command(
    reader: &mut impl BufRead,
    writer: &mut impl Write,
    command: &str,
    expected: &str,
) -> io::Result<()> {
    loop {
        println!("Pi -> {command}");
        writeln!(writer, "{command}")?;
        writer.flush()?;

        loop {
            let message = read_message(reader)?;

            if message == expected {
                return Ok(());
            }

            if is_ready_message(&message) {
                println!("Arduino restarted; resending {command}...");
                break;
            }

            if message.starts_with("ERR ") {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("Arduino rejected {command}: {message}"),
                ));
            }
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port_name = std::env::var("ARDUINO_PORT").unwrap_or_else(|_| "/dev/ttyACM0".to_string());

    println!("Opening Arduino serial port: {port_name}");
    let port = serialport::new(&port_name, 115_200)
        .timeout(Duration::from_secs(5))
        .open()?;

    let mut reader = BufReader::new(port.try_clone()?);
    let mut writer = port;

    println!("Waiting for Arduino...");
    wait_for(&mut reader, "READY")?;

    send_command(&mut reader, &mut writer, "PING", "PONG")?;

    println!("Handshake complete. Starting development loop (Ctrl+C to stop).");
    let mut led_on = false;

    loop {
        led_on = !led_on;
        let command = if led_on { "LED ON" } else { "LED OFF" };

        send_command(&mut reader, &mut writer, command, "OK")?;
        std::thread::sleep(Duration::from_secs(1));
    }
}

#[cfg(test)]
mod tests {
    use super::{send_command, wait_for};
    use std::io::{BufReader, Cursor};

    #[test]
    fn accepts_ready_merged_across_resets() {
        let responses = Cursor::new(b"READREADY\n");
        let mut reader = BufReader::new(responses);

        wait_for(&mut reader, "READY").unwrap();
    }

    #[test]
    fn resends_command_when_arduino_restarts() {
        let responses = Cursor::new(b"READREADY\nPONG\n");
        let mut reader = BufReader::new(responses);
        let mut commands = Vec::new();

        send_command(&mut reader, &mut commands, "PING", "PONG").unwrap();

        assert_eq!(commands, b"PING\nPING\n");
    }

    #[test]
    fn fails_when_arduino_rejects_command() {
        let responses = Cursor::new(b"ERR UNKNOWN_COMMAND\n");
        let mut reader = BufReader::new(responses);
        let mut commands = Vec::new();

        let error = send_command(&mut reader, &mut commands, "MOVE", "OK").unwrap_err();

        assert_eq!(error.kind(), std::io::ErrorKind::InvalidData);
        assert_eq!(commands, b"MOVE\n");
    }
}
