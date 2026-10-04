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
| `0x20` | `ACK` | Arduino → Pi | Empty; sequence matches the command |
| `0x30` | `TELEMETRY` | Arduino → Pi | Telemetry structure below |
| `0x7f` | `ERROR` | Arduino → Pi | One-byte error code |

`SET_MOTION` is absolute and idempotent: a repeated frame sets the same target
instead of accumulating motion. The simulated firmware accepts speeds between
-1000 and 1000 mm/s.

## Telemetry payload

| Offset | Size | Type | Field |
| --- | --- | --- | --- |
| 0 | 4 | `u32` | Arduino uptime in milliseconds |
| 4 | 1 | `u8` | State: `0=idle`, `1=moving`, `2=watchdog stop` |
| 5 | 2 | `i16` | Left target speed in mm/s |
| 7 | 2 | `i16` | Right target speed in mm/s |
| 9 | 4 | `i32` | Simulated left position in mm |
| 13 | 4 | `i32` | Simulated right position in mm |
| 17 | 2 | `u16` | Simulated distance in mm |
| 19 | 2 | `u16` | Simulated battery voltage in mV |
| 21 | 2 | `u16` | Last applied command sequence |

Telemetry is currently published at 10 Hz. It is not acknowledged: a stale
sample may be dropped in favor of a newer one. The controller considers the
session lost and exits with an error when no valid telemetry frame is received
for 1500 ms.

## Safety behavior

The controller sends a heartbeat every 250 ms. If the Arduino receives no valid
heartbeat or motion command for 1000 ms, it sets both simulated motor targets to
zero and publishes state `2`. This watchdog is local to the firmware and does
not depend on Linux scheduling after the timeout expires.
