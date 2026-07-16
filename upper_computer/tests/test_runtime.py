from __future__ import annotations

import unittest

from sailing.mission import C1Mission, C2Mission, C2State
from sailing.models import ControlCommand, FireRequest, FireSource, GlobalState, MissionEvent
from sailing.protocol import MessageType, decode_control_payload, decode_fire_payload, decode_frame
from sailing.runtime import UpperRuntime
from sailing.safety import SafetyContext, SafetyGate
from sailing.simulation import FixedGuidance, ManualClock, MemoryTransport


class SafetyTests(unittest.TestCase):
    def test_neutralizes_disarmed_stale_and_estop_commands(self) -> None:
        gate = SafetyGate(command_timeout_seconds=0.5)
        desired = ControlCommand(2.0, -2.0)
        self.assertEqual(gate.apply(desired, SafetyContext(0.0, False)), ControlCommand.neutral())
        self.assertEqual(gate.apply(desired, SafetyContext(0.6, True)), ControlCommand.neutral())
        self.assertEqual(gate.apply(desired, SafetyContext(0.0, True, True)), ControlCommand.neutral())
        self.assertEqual(gate.apply(desired, SafetyContext(0.0, True)), ControlCommand(1.0, -1.0))
        with self.assertRaises(ValueError):
            gate.apply(desired, SafetyContext(-0.1, True))
        with self.assertRaises(ValueError):
            gate.apply(desired, SafetyContext(float("nan"), True))
        with self.assertRaises(ValueError):
            SafetyGate(float("nan"))


class RuntimeTests(unittest.TestCase):
    def make_runtime(self, mission: C1Mission | C2Mission) -> tuple[UpperRuntime, MemoryTransport, ManualClock]:
        transport = MemoryTransport()
        clock = ManualClock()
        runtime = UpperRuntime(
            mission,
            FixedGuidance(ControlCommand(0.6, -0.2)),
            SafetyGate(),
            transport,
            clock,
            initial_session_id=41,
        )
        return runtime, transport, clock

    def test_c1_runtime_starts_new_session_and_sends_safe_control(self) -> None:
        mission = C1Mission()
        runtime, transport, clock = self.make_runtime(mission)
        runtime.step(MissionEvent.BOOT_COMPLETE)
        neutral_frame = decode_frame(transport.writes[-1])
        neutral, _ = decode_control_payload(neutral_frame.payload)
        self.assertEqual(neutral, ControlCommand.neutral())
        runtime.step(MissionEvent.SELF_CHECK_OK)
        runtime.step(MissionEvent.ARM)
        arm_frame = decode_frame(transport.writes[-1])
        self.assertEqual(arm_frame.message_type, MessageType.ARM)
        self.assertEqual(runtime.session_id, 42)
        self.assertEqual(arm_frame.sequence, 0)
        clock.advance(0.125)
        runtime.step(MissionEvent.START)
        control_frame = decode_frame(transport.writes[-1])
        command, validity = decode_control_payload(control_frame.payload)
        self.assertAlmostEqual(command.propulsion, 0.6, places=6)
        self.assertAlmostEqual(command.steering, -0.2, places=6)
        self.assertEqual(validity, 500)
        self.assertEqual(control_frame.sender_time_ms, 125)
        self.assertEqual(control_frame.sequence, 1)
        self.assertFalse(runtime.arm_launcher())

    def test_runtime_c2_uses_two_stage_fake_fire(self) -> None:
        mission = C2Mission(c1_completed=True)
        runtime, transport, _ = self.make_runtime(mission)
        for event in (
            MissionEvent.BOOT_COMPLETE,
            MissionEvent.SELF_CHECK_OK,
            MissionEvent.ARM,
            MissionEvent.START,
            MissionEvent.ZONE_DETECTED,
            MissionEvent.ZONE_CONFIRMED,
            MissionEvent.TARGET_ACQUIRED,
            MissionEvent.AIM_LOCKED,
        ):
            runtime.step(event)
        self.assertEqual(mission.substate, C2State.FIRE_READY)
        self.assertTrue(runtime.arm_launcher())
        self.assertEqual(decode_frame(transport.writes[-1]).message_type, MessageType.LAUNCH_ARM)
        transition = runtime.fire_once(FireRequest(7, FireSource.AUTO, 250))
        fire_frame = decode_frame(transport.writes[-1])
        self.assertEqual(fire_frame.message_type, MessageType.FIRE_ONCE)
        self.assertEqual(decode_fire_payload(fire_frame.payload).shot_id, 7)
        self.assertTrue(transition.accepted)
        self.assertEqual(mission.substate, C2State.VERIFY_SHOT)
        with self.assertRaises(PermissionError):
            runtime.fire_once(FireRequest(8, FireSource.AUTO))

    def test_runtime_enforces_300_second_deadline(self) -> None:
        mission = C1Mission()
        runtime, transport, clock = self.make_runtime(mission)
        for event in (
            MissionEvent.BOOT_COMPLETE,
            MissionEvent.SELF_CHECK_OK,
            MissionEvent.ARM,
            MissionEvent.START,
        ):
            runtime.step(event)
        clock.advance(299.0)
        runtime.step()
        self.assertEqual(mission.global_state, GlobalState.RUNNING)
        clock.advance(1.0)
        transition = runtime.step()
        self.assertEqual(transition.event, MissionEvent.TIMEOUT)
        self.assertEqual(mission.global_state, GlobalState.FAULT)
        command, _ = decode_control_payload(decode_frame(transport.writes[-1]).payload)
        self.assertEqual(command, ControlCommand.neutral())

    def test_guidance_exception_faults_and_clears_motion(self) -> None:
        class FailingGuidance:
            def __init__(self) -> None:
                self.calls = 0

            def command(self) -> ControlCommand:
                self.calls += 1
                if self.calls >= 4:
                    raise RuntimeError("simulated algorithm failure")
                return ControlCommand(0.5, 0.0)

        mission = C1Mission()
        transport = MemoryTransport()
        runtime = UpperRuntime(
            mission,
            FailingGuidance(),
            SafetyGate(),
            transport,
            ManualClock(),
        )
        for event in (
            MissionEvent.BOOT_COMPLETE,
            MissionEvent.SELF_CHECK_OK,
            MissionEvent.ARM,
            MissionEvent.START,
        ):
            runtime.step(event)
        transition = runtime.step()
        self.assertEqual(transition.event, MissionEvent.INTERNAL_FAULT)
        self.assertEqual(mission.global_state, GlobalState.FAULT)
        command, _ = decode_control_payload(decode_frame(transport.writes[-1]).payload)
        self.assertEqual(command, ControlCommand.neutral())


class SimulationTests(unittest.TestCase):
    def test_manual_clock_only_moves_forward(self) -> None:
        clock = ManualClock()
        clock.advance(1.5)
        self.assertEqual(clock.monotonic(), 1.5)
        with self.assertRaises(ValueError):
            clock.advance(-1.0)
        with self.assertRaises(ValueError):
            clock.advance(float("nan"))
        with self.assertRaises(ValueError):
            ManualClock(float("nan"))


if __name__ == "__main__":
    unittest.main()
