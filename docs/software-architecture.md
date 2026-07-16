# Software Architecture (Hardware-Independent Draft)

## Purpose

Provide a testable starting point before the physical platform is selected.
Algorithms depend on domain models and interfaces, never directly on a camera,
serial port, MCU HAL, motor driver, or launcher.

## Runtime data flow

```text
Sensors/camera -> adapters -> perception/state estimation -> mission state
                                                      |          machine
                                                      v             |
Lower telemetry <- framed link <- safety <- control command <- guidance
       |                                                       |
       +-> watchdog -> actuator adapters -> motor/steering/launcher
```

## Upper-computer responsibilities

1. Convert sensor inputs into typed observations.
2. Estimate vehicle and mission state.
3. Run either the C1 or C2 mission state machine.
4. Produce normalized propulsion and steering requests.
5. Authorize firing only in a valid C2 state.
6. Record decisions and telemetry for deterministic replay.

The initial framework supplies interfaces and deterministic fakes. Concrete
vision, infrared decoding, guidance, and control algorithms are later plugins.

## Lower-computer responsibilities

1. Decode and validate framed commands.
2. Reject unsupported versions, invalid CRCs, invalid lengths, and stale data.
3. Clamp normalized commands to safe ranges.
4. Apply commands through hardware adapter interfaces.
5. Enter failsafe when the command watchdog expires.
6. Keep the launcher inhibited unless every safety precondition is satisfied.

## Mission decomposition

### Shared safety lifecycle

```text
BOOT -> SELF_TEST -> DISARMED -> ARMED -> RUNNING -> COMPLETE
                                      |          -> ABORTED / FAULT
                                      +----------> E_STOP (latched)
```

Outputs are neutral outside `RUNNING`. Resetting a terminal state only returns
to `DISARMED`; it never restores motion or launch authorization automatically.

### C1-2

```text
SEARCH_GATE -> TRACK_GATE -> APPROACH_GATE -> VERIFY_PASS
     ^              |                              |
     +-- RECOVERY <-+              next gate <----+
                                                   +-> FINISHED (after gate 10)
```

Only evidence tagged with the current zero-based `gate_index` can advance the
counter. Repeated, skipped, reversed, or stale gate evidence is rejected.

### C2

```text
TRANSIT -> ENTER_FIRE_ZONE -> ACQUIRE_TARGET -> AIM_STABILIZE
                                                |
                                                v
SELECT_NEXT <- VERIFY_SHOT <- FIRE_READY ----> SAFE_HOLD
     |              |
     +-> next target+-> FINISHED
```

The `FIRE_READY` window is revoked as soon as firing-zone or target evidence is
lost. Firing is two-stage (`LAUNCH_ARM`, then unique `FIRE_ONCE`); the lower
computer still makes the final decision and enforces E-stop, freshness,
session/sequence, duplicate `shot_id`, cooldown, and ten-shot budget checks.
Navigation components remain shared with C1.

## Communication boundary

Protocol v1 is explicitly serialized little-endian data, not an in-memory
structure. It carries a session ID, 32-bit sequence, diagnostic timestamp,
bounded payload (256 bytes), and CRC-16/CCITT-FALSE. Python and C++ read the
same TSV golden vectors under `shared/test-vectors`.

## Deferred decisions

- Upper-computer hardware and operating system
- MCU family and HAL/RTOS
- Camera, infrared receiver, IMU, positioning sensors, and time source
- Propulsion and steering geometry
- Physical transport (UART, CAN, Ethernet, or another link)
- Control-loop rates and actuator limits
- Concrete perception, estimation, guidance, and aiming algorithms

These choices belong in adapters and configuration. They must not change the
mission or protocol-domain interfaces without an explicit version update.
