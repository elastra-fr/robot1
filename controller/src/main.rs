use std::io::{BufRead, BufReader, Write};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = serialport::new("/dev/ttyACM0", 115_200)
        .timeout(Duration::from_secs(5))
        .open()?;

    let mut reader = BufReader::new(port.try_clone()?);
    let mut writer = port;

    println!("Waiting for Arduino...");

    loop {
        let mut line = String::new();
        reader.read_line(&mut line)?;

        if line.trim() == "READY" {
            println!("Arduino ready");
            break;
        }
    }

    writer.write_all(b"PING\n")?;

    let mut response = String::new();
    reader.read_line(&mut response)?;

    println!("Arduino: {}", response.trim());

    writer.write_all(b"LED ON\n")?;

    response.clear();
    reader.read_line(&mut response)?;

    println!("LED ON: {}", response.trim());

    std::thread::sleep(Duration::from_secs(1));

    writer.write_all(b"LED OFF\n")?;

    response.clear();
    reader.read_line(&mut response)?;

    println!("LED OFF: {}", response.trim());

    Ok(())
}