use std::io::{Read, Write};
use std::thread;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut port = serialport::new("/dev/ttyACM0", 115_200)
        .timeout(Duration::from_secs(2))
        .open()?;

    // Le Mega redémarre à l'ouverture du port série
    thread::sleep(Duration::from_secs(2));

    port.write_all(b"PING\n")?;

    let mut buffer = [0u8; 128];
    let n = port.read(&mut buffer)?;

    println!(
        "Arduino: {}",
        String::from_utf8_lossy(&buffer[..n]).trim()
    );

    Ok(())
}