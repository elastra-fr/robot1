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

### Controller structure

The Rust controller separates hardware integration from robot decisions:

```text
Arduino adapter ─┐
Future vision  ──┼─> observations ─> RobotState ─> behavior
Future audio   ──┘                                  │
                                                   ▼
                                           intent and safety
                                                   │
                                                   ▼
                                           actuator command
                                                   │
                                                   ▼
                                                Arduino
```

`Runtime` is the single owner of `RobotState`. Hardware adapters produce typed,
timestamped observations instead of modifying shared state. Behaviors only read
the domain state and produce intentions. The safety layer validates an intention
before it becomes an actuator command.

The source tree reflects these boundaries:

```text
controller/src/
├── app/          # Runtime and component supervision
├── adapters/     # External hardware and protocol integration
├── behaviors/    # Replaceable high-level behaviors
├── control/      # Behavior interface and safety policy
├── domain/       # Robot state, observations, intentions and commands
├── ports/        # Interfaces implemented by external adapters
├── config.rs     # Runtime configuration
├── lib.rs        # Controller library
└── main.rs       # Process entry point only
```

### Arduino firmware structure

The Mega firmware keeps the main sketch as a small, non-blocking orchestrator:

```text
firmware/mega/
├── mega.ino                 # setup(), loop() and command dispatch
├── SerialCommunication.*   # USB serial protocol and telemetry
├── Motion.*                # motor targets and motion state
├── Sensors.*               # sensor acquisition (currently simulated)
├── Servos.*                # servo control boundary (not connected yet)
├── Safety.*                # local watchdog and future safety interlocks
├── FirmwareConfig.h        # shared timings and limits
└── FirmwareTypes.h         # commands and telemetry data
```

Hardware modules expose `begin()` and non-blocking `update()` operations. This
keeps serial processing and the local safety checks responsive while individual
sensors and actuators are introduced.

Timing-sensitive motor control, encoder feedback loops and the final watchdog
remain on the Arduino. Camera and microphone recognition will run as isolated,
on-demand workers on the Raspberry Pi. They will publish recognition results,
not raw continuous media streams, into the controller state.

## Current communication

Default serial configuration:

```text
115200 baud
/dev/ttyACM0 or /dev/ttyUSB0
```

Current protocol:

```text
Pi      -> HELLO
Arduino -> HELLO_ACK
Pi      -> SET_MOTION
Arduino -> ACK
Arduino -> TELEMETRY (10 Hz)
Pi      -> HEARTBEAT (4 Hz)
```

Protocol v1 uses binary COBS framing, session identifiers, sequence numbers and
CRC-16 validation. Its complete specification is in
[`protocol/serial-v1.md`](protocol/serial-v1.md).

The current firmware is a safe simulation: it does not drive motor outputs. It
simulates differential motion, encoder positions, a distance sensor and battery
voltage. The controller cycles through forward motion, rotation, reverse motion
and stop while displaying telemetry continuously.

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

## Git conventions

### Branches

`master` must remain buildable and deployable. Development is done on
short-lived branches created from an up-to-date `master` branch.

Branch names use lowercase kebab-case with one of these prefixes:

| Prefix | Purpose | Example |
| --- | --- | --- |
| `feat/` | New behavior or capability | `feat/simulated-control-loop` |
| `fix/` | Bug fix | `fix/serial-reconnect` |
| `refactor/` | Internal change without new behavior | `refactor/protocol-parser` |
| `docs/` | Documentation only | `docs/git-conventions` |
| `test/` | Tests and simulation tooling | `test/serial-frame-errors` |
| `chore/` | Maintenance and repository tooling | `chore/update-dependencies` |

Avoid personal names, machine names and issue descriptions containing sensitive
information in branch names.

### Commits

Commit messages follow the Conventional Commits format:

```text
<type>(<scope>): <short description>
```

Common types are `feat`, `fix`, `refactor`, `docs`, `test`, `chore`, `build`,
`ci` and `perf`. Recommended scopes are `controller`, `firmware`, `protocol`,
`deploy` and `docs`.

Examples:

```text
feat(controller): add simulated control loop
feat(firmware): publish simulated sensor data
fix(protocol): recover after a serial reset
docs(readme): document Git conventions
```

Use the imperative mood, keep the description concise and do not end it with a
period. Each commit should represent one coherent change. Mark breaking changes
with `!`, for example `feat(protocol)!: add binary message framing`, and explain
the migration in the commit body.

Before merging into `master`:

- format and test the Rust controller;
- compile the Arduino firmware;
- verify the controller-to-Arduino simulation on the target hardware;
- review the diff for credentials, personal paths and generated files;
- update the protocol documentation when messages or behavior change.

## Next steps

- add motor control and encoder feedback;
- validate protocol v1 under disconnects, corrupted frames and sustained load;
- move serial I/O into a dedicated controller worker;
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
| `ARDUINO_BOOT_DELAY_MS` | `2000` | Silent delay after opening the port, before the handshake |
| `FIRMWARE_DIR` | `firmware/mega` | Firmware sketch directory |
| `CONTROLLER_DIR` | `controller` | Rust controller directory |
| `BUILD_PROFILE` | `release` | Cargo build profile |
| `SERVICE_NAME` | empty | Optional systemd service to stop and restart instead of running the controller directly |

When `SERVICE_NAME` is set, the current user must have permission to run
`systemctl stop` and `systemctl restart` for that service. When it is empty, the
controller runs in the foreground and its output remains visible in the
terminal.
