# Lower Computer Core

Portable C++17 safety and protocol core for host testing before an MCU is
selected. It contains no vendor SDK, RTOS, GPIO, PWM, serial library, or real
launcher driver.

The important boundaries are:

- `protocol.*`: independent protocol-v1 serialization and bounded stream parsing.
- `hal.hpp`: narrow interfaces future hardware adapters must implement.
- `fakes.hpp`: host-only clocks, outputs, transport, and event collection.
- `safety_supervisor.*`: session/sequence checks, 300 s mission deadline,
  500 ms motion watchdog,
  E-stop latch, C1 launch prohibition, and C2 two-stage single-shot interlocks.

The ten-shot budget and used `shot_id` set survive disarm/reset/re-arm for the
lifetime of a supervisor instance. Starting a new competition round therefore
requires an explicit new supervisor/power-cycle boundary in this first draft.

Build and run the tests from the repository root:

```powershell
& 'D:\MinGW\mingw64\bin\g++.exe' -std=c++17 -Wall -Wextra -Wpedantic -Werror `
  -Ilower_computer/include lower_computer/src/protocol.cpp `
  lower_computer/src/safety_supervisor.cpp lower_computer/tests/test_main.cpp `
  -o build/lower_computer_tests.exe
& build/lower_computer_tests.exe shared/test-vectors/protocol-v1.tsv
```

Only fake outputs are linkable in this version. Adding real actuator adapters
requires a separate hardware selection and safety review.
