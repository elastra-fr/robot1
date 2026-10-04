use std::io;
use std::time::Duration;

const DEFAULT_ARDUINO_PORT: &str = "/dev/ttyACM0";
const DEFAULT_BOOT_DELAY: Duration = Duration::from_millis(2_000);
const DEFAULT_SERIAL_TIMEOUT: Duration = Duration::from_millis(50);
const DEFAULT_HEARTBEAT_INTERVAL: Duration = Duration::from_millis(250);
const DEFAULT_TELEMETRY_TIMEOUT: Duration = Duration::from_millis(1_500);
const DEFAULT_TELEMETRY_LOG_INTERVAL: Duration = Duration::from_secs(1);

#[derive(Debug, Clone)]
pub struct Config {
    pub arduino_port: String,
    pub boot_delay: Duration,
    pub serial_timeout: Duration,
    pub heartbeat_interval: Duration,
    pub telemetry_timeout: Duration,
    pub telemetry_log_interval: Duration,
}

impl Config {
    pub fn from_env() -> io::Result<Self> {
        Ok(Self {
            arduino_port: std::env::var("ARDUINO_PORT")
                .unwrap_or_else(|_| DEFAULT_ARDUINO_PORT.to_string()),
            boot_delay: duration_from_env("ARDUINO_BOOT_DELAY_MS", DEFAULT_BOOT_DELAY)?,
            serial_timeout: DEFAULT_SERIAL_TIMEOUT,
            heartbeat_interval: DEFAULT_HEARTBEAT_INTERVAL,
            telemetry_timeout: DEFAULT_TELEMETRY_TIMEOUT,
            telemetry_log_interval: DEFAULT_TELEMETRY_LOG_INTERVAL,
        })
    }
}

fn duration_from_env(name: &str, default: Duration) -> io::Result<Duration> {
    let Some(value) = std::env::var_os(name) else {
        return Ok(default);
    };
    let value = value.to_string_lossy();
    let milliseconds = value.parse::<u64>().map_err(|_| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("invalid {name} value: {value}"),
        )
    })?;

    Ok(Duration::from_millis(milliseconds))
}

#[cfg(test)]
mod tests {
    use super::duration_from_env;
    use std::time::Duration;

    #[test]
    fn keeps_default_duration_when_variable_is_missing() {
        let duration = duration_from_env(
            "ROBOT1_TEST_DURATION_THAT_MUST_NOT_EXIST",
            Duration::from_millis(42),
        )
        .unwrap();

        assert_eq!(duration, Duration::from_millis(42));
    }
}
