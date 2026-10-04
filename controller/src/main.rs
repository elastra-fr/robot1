use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

fn wait_for(
    reader: &mut BufReader<Box<dyn serialport::SerialPort>>,
    expected: &str,
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let mut line = String::new();
        reader.read_line(&mut line)?;

        let message = line.trim();
        println!("Arduino -> {message}");

        if message == expected {
            return Ok(());
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

    writer.write_all(b"PING\n")?;
    wait_for(&mut reader, "PONG")?;

    writer.write_all(b"LED ON\n")?;
    wait_for(&mut reader, "OK")?;

    std::thread::sleep(Duration::from_secs(1));

    writer.write_all(b"LED OFF\n")?;
    wait_for(&mut reader, "OK")?;

    Ok(())
}
