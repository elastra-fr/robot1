# Serial protocol v1

This document describes the experimental binary protocol between the Raspberry
Pi controller and the Arduino firmware.

## Transport and framing

- USB serial at 115200 baud;
- little-endian integer encoding;
- COBS framing followed by a `0x00` delimiter;
- CRC-16/CCITT-FALSE (`poly=0x1021`, `init=0xffff`);
- maximum payload size of 48 bytes;
- malformed frames are discarded up to the next delimiter.

The decoded frame layout is:

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | Protocol version (`1`) |
| 1 | 1 | Message type |
| 2 | 4 | Session identifier |
| 6 | 2 | Sequence number |
| 8 | 1 | Payload length |
| 9 | N | Payload |
| 9 + N | 2 | CRC-16 |

## Session handshake

The controller creates a new non-zero session identifier whenever it starts.
The Arduino remains stopped and does not publish telemetry until the controller
initiates a session:

```text
Pi      -> HELLO(session, sequence)
Arduino -> HELLO_ACK(session, same sequence)
```

Opening the serial port resets classic Uno and Mega boards. The controller waits
for a configurable startup grace period before clearing stale input and sending
the first `HELLO` frame.

The controller retries `HELLO` when the Arduino is still rebooting. Frames from
older sessions are ignored. A new valid `HELLO` safely stops the current motion
before activating the new session.

## Messages

| Value | Name | Direction | Payload |
| --- | --- | --- | --- |
| `0x01` | `HELLO` | Pi → Arduino | Empty |
| `0x02` | `HELLO_ACK` | Arduino → Pi | Empty |
| `0x03` | `HEARTBEAT` | Pi → Arduino | Empty |
| `0x10` | `SET_MOTION` | Pi → Arduino | Left and right speeds (`i16`, mm/s) |
| `0x11` | `STOP` | Pi → Arduino | Empty |
| `0x12` | `START_ENVIRONMENT_SCAN` | Pi → Arduino | Empty |
| `0x20` | `ACK` | Arduino → Pi | Empty; sequence matches the command |
| `0x30` | `TELEMETRY` | Arduino → Pi | Telemetry structure below |
| `0x31` | `SENSOR_STATUS` | Arduino → Pi | Real sensor status structure below |
| `0x32` | `MOTION_STATUS` | Arduino → Pi | Virtual motion and safety reason |
| `0x33` | `ENVIRONMENT_SCAN_SAMPLE` | Arduino → Pi | Two opposite angular measurements |
| `0x34` | `ENVIRONMENT_SCAN_STATUS` | Arduino → Pi | Scan lifecycle state |
| `0x7f` | `ERROR` | Arduino → Pi | One-byte error code |

`SET_MOTION` is absolute and idempotent: a repeated frame sets the same target
instead of accumulating motion. The simulated firmware accepts speeds between
-1000 and 1000 mm/s.

## Telemetry payload

| Offset | Size | Type | Field |
| --- | --- | --- | --- |
| 0 | 4 | `u32` | Arduino uptime in milliseconds |
| 4 | 1 | `u8` | State: `0=idle`, `1=moving`, `2=watchdog stop`, `3=local safety stop` |
| 5 | 2 | `i16` | Left target speed in mm/s |
| 7 | 2 | `i16` | Right target speed in mm/s |
| 9 | 4 | `i32` | Simulated left position in mm |
| 13 | 4 | `i32` | Simulated right position in mm |
| 17 | 2 | `u16` | Ultrasonic A distance in mm, or `0xffff` if invalid |
| 19 | 2 | `u16` | Battery voltage in mV; zero until a real monitor is connected |
| 21 | 2 | `u16` | Last applied command sequence |

Telemetry is currently published at 10 Hz. It is not acknowledged: a stale
sample may be dropped in favor of a newer one. When no valid telemetry frame is
received for 1500 ms, the controller considers the session lost. It observes a
silent startup delay after closing and reopening the serial port, creates a new
session and performs a new handshake. It then sends `STOP` and requires both the
matching `ACK` and a safely stopped telemetry sample before restarting the
controller. State `3` (local safety stop) is accepted when both virtual wheel
targets are zero, allowing sensor diagnostics without connected safety sensors
while still forbidding movement.

## Sensor status payload

| Offset | Size | Type | Field |
| --- | --- | --- | --- |
| 0 | 4 | `u32` | Arduino uptime in milliseconds |
| 4 | 1 | `u8` | PIR presence bits: front=`bit 0`, back=`bit 1` |
| 5 | 1 | `u8` | 10 cm detection bits: bumpers=`bits 0..2`, ground=`bit 3` |
| 6 | 1 | `u8` | Local hazards: bumpers=`bits 0..2`, missing ground=`bit 3` |
| 7 | 1 | `u8` | Ready/valid flags: PIR=`bit 0`, ultrasonic A=`bit 1`, B=`bit 2` |
| 8 | 2 | `u16` | Ultrasonic A distance in mm, or `0xffff` |
| 10 | 2 | `u16` | Ultrasonic B distance in mm, or `0xffff` |

`SENSOR_STATUS` is published every 250 ms. The Pi prints changes immediately
and otherwise limits the development display to about one line per second. PIR
inputs have a 60-second startup stabilization period and a 300 ms debounce in
the Mega firmware.

## Motion status payload

| Offset | Size | Type | Field |
| --- | --- | --- | --- |
| 0 | 1 | `u8` | `0=stopped`, `1=forward`, `2=backward`, `3=left`, `4=right`, `5=mixed` |
| 1 | 1 | `u8` | Stop reason: `0=none`, `1=watchdog`, `2=local sensor` |

The motion remains virtual during sensor bring-up. This status provides visible
Arduino-side confirmation without writing unframed text onto the binary serial
link.

## Environment scan

`START_ENVIRONMENT_SCAN` is acknowledged like other commands. Error code `3`
means a scan is already running. The development controller requests one scan
after establishing a safe session.

The scan moves the shared servo progressively from 0 to 180 degrees. The two
ultrasonic sensors point in opposite directions, so every servo position yields
two directions covering a full 360 degrees.

`ENVIRONMENT_SCAN_SAMPLE` payload:

| Offset | Size | Type | Field |
| --- | --- | --- | --- |
| 0 | 2 | `u16` | Sensor A direction in degrees (`0..180`) |
| 2 | 2 | `u16` | Sensor A distance in mm, or `0xffff` |
| 4 | 2 | `u16` | Sensor B direction in degrees (`180..359`, then `0`) |
| 6 | 2 | `u16` | Sensor B distance in mm, or `0xffff` |
| 8 | 1 | `u8` | Valid measurements: A=`bit 0`, B=`bit 1` |

`ENVIRONMENT_SCAN_STATUS` contains one byte: `1=started`, `2=complete`. The
servo uses 5-degree steps, waits 150 ms after each step, and returns to its
90-degree center position when complete.

## Safety behavior

The controller sends a heartbeat every 250 ms. If the Arduino receives no valid
heartbeat or motion command for 1000 ms, it sets both simulated motor targets to
zero and publishes state `2`. Any of the three bumper sensors detecting an
object also stops motion locally. The fourth 10 cm sensor looks at the ground:
failure to detect the ground is treated as a cliff and publishes state `3`.
These protections are local to the firmware and do not depend on Linux
scheduling.
