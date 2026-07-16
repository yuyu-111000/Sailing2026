# Sailing2026

Software framework for the C1-2 intelligent-navigation event and the C2
simulated-shore-fire-support event.

This first version is deliberately hardware-independent. It establishes module
boundaries, mission state machines, a versioned upper/lower communication
protocol, safety behavior, simulation fakes, and host-side tests. It does not
select a camera, IMU, upper computer, MCU, motor driver, steering mechanism, or
launcher interface.

## Repository layout

```text
upper_computer/   Python mission, perception, guidance, and communication core
lower_computer/   Portable C++17 realtime execution and safety core
shared/           Cross-language protocol contract and golden test vectors
docs/             Competition rules and project-facing design notes
scripts/          Local validation entry points
```

## Design boundary

- The upper computer decides *what the vehicle should do*.
- The lower computer decides *how to execute commands safely in realtime*.
- Hardware-specific code implements narrow adapter interfaces at the edges.
- C1-2 is the base mission. C2 reuses navigation and adds target acquisition,
  aiming, and firing authorization.
- Any stale, malformed, or unsafe command must degrade to neutral propulsion,
  neutral steering, and an inhibited launcher.

See [software architecture](docs/software-architecture.md) and the
[protocol contract](shared/protocol-v1.md) for the current design.

## Current runnable baseline

- Python 3.12 upper-computer state machines for the shared lifecycle, ordered
  ten-gate C1-2 route, and C2 transit/target/fake-fire flow.
- Portable C++17 lower-computer framing, fixed-capacity fakes, watchdog,
  E-stop latch, session/sequence validation, C1 launch prohibition, and C2
  two-stage single-shot protection.
- One shared set of Python/C++ protocol golden vectors.
- Deterministic, hardware-free C1 and C2 demonstrations.

Run every host-side check from PowerShell (pass tool paths when they are not on
`PATH`):

```powershell
.\scripts\validate.ps1 -Python "C:\path\to\python.exe" -Cxx "C:\path\to\g++.exe"
```

Run the fake mission demonstrations:

```powershell
.\scripts\run-demos.ps1 -Python "C:\path\to\python.exe"
```

This is an algorithm-ready scaffold, not a finished sailing algorithm. Vision,
infrared decoding, state estimation, route guidance, PID tuning, and every
physical hardware adapter remain intentionally deferred until the platform is
selected.
