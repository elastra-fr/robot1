# Raspberry Pi + Arduino Robotics Stack

Experimental robotics stack combining a **Raspberry Pi** and an **Arduino-compatible board**.

The Raspberry Pi runs the high-level controller in **Rust**, while the Arduino handles low-level hardware control and timing-sensitive tasks. Communication between both layers uses **USB serial**.

## Architecture

```text
Development workstation
        │
        │ Git
        ▼
Raspberry Pi
│
├── Rust controller
├── high-level logic
├── networking / camera / navigation
└── USB serial
        │
        ▼
Arduino
├── motors
├── servos
├── sensors
├── encoders
└── local safety logic

Repository
project/
├── controller/   # Rust
├── firmware/     # Arduino
├── protocol/     # Serial protocol documentation
├── scripts/      # Build / deployment tools
└── README.md
```

## Current communication

Default serial configuration:

```text
115200 baud
/dev/ttyACM0 or /dev/ttyUSB0
```

Current protocol:

```text
Arduino -> READY
Pi      -> PING
Arduino -> PONG

Pi      -> LED ON
Arduino -> OK

Pi      -> LED OFF
Arduino -> OK
```

The end-to-end Rust ↔ Arduino communication has been validated.

## Development workflow

Development is done on a workstation.

```text
edit
 ↓
git push
 ↓
Raspberry Pi
 ↓
git pull
 ↓
build Rust
 ↓
compile / flash Arduino
 ↓
run controller
```

Rust:

```sh
cd controller
cargo build --release
```

Arduino:

```sh
arduino-cli compile \
  --fqbn arduino:avr:mega \
  firmware/mega

arduino-cli upload \
  --port /dev/ttyACM0 \
  --fqbn arduino:avr:mega \
  firmware/mega
```

Board type and serial port should remain configurable.

## Design principles

- high-level decisions run on the Raspberry Pi;
- timing-sensitive hardware control stays on the Arduino;
- local safety must not depend on Linux scheduling;
- the protocol should remain explicit and documented;
- Rust and Arduino firmware live in the same repository;
- the Raspberry Pi can update both the controller and the Arduino firmware.

## Next steps

- formalize the serial protocol;
- add structured status and sensor messages;
- distinguish responses from asynchronous events;
- add motor control and encoder feedback;
- add higher-level sensing and navigation.

## Deployment

The deployment scripts locate the repository automatically, so they can be run
from any current working directory.

Deploy the current checkout:

```sh
./scripts/deploy.sh
```

During development, when no systemd service is configured, the script runs the
controller in the foreground after uploading the firmware. This displays the
Arduino handshake and the continuous development loop directly. The command
keeps running until `Ctrl+C` and fails if communication does not work.

Pull fast-forward changes before deploying:

```sh
./scripts/update.sh
```

The defaults target an Arduino Mega on `/dev/ttyACM0`. Override them with
environment variables when needed:

```sh
ARDUINO_FQBN=arduino:avr:uno \
ARDUINO_PORT=/dev/ttyUSB0 \
FIRMWARE_DIR=firmware/uno \
./scripts/deploy.sh
```

Available configuration:

| Variable | Default | Description |
| --- | --- | --- |
| `ARDUINO_FQBN` | `arduino:avr:mega` | Arduino board identifier |
| `ARDUINO_PORT` | `/dev/ttyACM0` | Arduino serial port |
| `FIRMWARE_DIR` | `firmware/mega` | Firmware sketch directory |
| `CONTROLLER_DIR` | `controller` | Rust controller directory |
| `BUILD_PROFILE` | `release` | Cargo build profile |
| `SERVICE_NAME` | empty | Optional systemd service to stop and restart instead of running the controller directly |

When `SERVICE_NAME` is set, the current user must have permission to run
`systemctl stop` and `systemctl restart` for that service. When it is empty, the
controller runs in the foreground and its output remains visible in the
terminal.
