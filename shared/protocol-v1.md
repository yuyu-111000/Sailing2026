# Sailing2026 protocol v1

This document is the byte-level contract between the Python upper-computer
framework and the portable C++17 lower-computer core. It is deliberately
independent of UART, USB, TCP, CAN, or any specific MCU.

## Frame layout

All multi-byte integers and IEEE-754 floats are little-endian. The CRC is
CRC-16/CCITT-FALSE (`poly=0x1021`, `init=0xFFFF`) over every byte from
`version` through the end of `payload`; the two-byte magic and the CRC field
itself are excluded.

| Offset | Size | Field |
|---:|---:|---|
| 0 | 2 | Magic: ASCII `SA` (`53 41`) |
| 2 | 1 | Protocol version (`01`) |
| 3 | 1 | Message type |
| 4 | 1 | Flags; must be zero unless a message defines bits |
| 5 | 4 | Session ID (`uint32`) |
| 9 | 4 | Monotonic sequence (`uint32`) |
| 13 | 2 | Payload length (`uint16`, maximum 256) |
| 15 | 4 | Sender diagnostic monotonic time in milliseconds (`uint32`) |
| 19 | N | Payload |
| 19+N | 2 | CRC (`uint16`) |

Sender time is diagnostic only. A receiver starts command expiry from its own
local receive time and does not assume the two monotonic clocks are aligned.

## Message types

| Value | Name | Side effect |
|---:|---|---|
| `01` | `HEARTBEAT` | None; never refreshes the motion watchdog |
| `02` | `ARM` | Starts a new authorized motion session |
| `03` | `DISARM` | Immediately neutralizes outputs |
| `04` | `E_STOP` | Latches emergency stop |
| `05` | `CONTROL_SETPOINT` | Applies a time-bounded motion setpoint |
| `06` | `MISSION_STATUS` | Status only |
| `07` | `TELEMETRY` | Status only |
| `08` | `LAUNCH_ARM` | Arms the C2 launcher path; never fires |
| `09` | `FIRE_ONCE` | Requests one unique C2 fake shot |
| `0A` | `ACK` | Status only |
| `0B` | `NACK` | Status only |
| `0C` | `FAULT` | Reports a fault |

Unknown versions, types, non-zero undefined flags, over-length payloads, and
CRC failures are rejected without actuator side effects.

## Payloads used by the framework

`CONTROL_SETPOINT` is `<float32 propulsion, float32 steering, uint16
valid_for_ms>`. Both normalized values must be finite and in `[-1, 1]`.

`FIRE_ONCE` is `<uint32 shot_id, uint8 source, uint16 valid_for_ms>`, where
source `01` is `AUTO` and source `02` is `MANUAL_FUTURE_REMOTE`. A duplicate
`shot_id` is idempotently rejected.

## Safety boundary

CRC detects accidental transmission corruption; it is not authentication or
encryption. The lower computer remains the final authority for session,
sequence, watchdog, E-stop, mission mode, firing-zone, launcher-arm, cooldown,
and ammunition-budget checks. The current repository contains fake actuator
adapters only.

Machine-readable cross-language vectors live in
`shared/test-vectors/protocol-v1.tsv`.
