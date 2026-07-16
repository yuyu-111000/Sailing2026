from __future__ import annotations

import unittest

from sailing.mission import C1Mission, C1State, C2Mission, C2State
from sailing.models import GlobalState, MissionEvent


def prepare(mission: C1Mission | C2Mission) -> None:
    assert mission.handle(MissionEvent.BOOT_COMPLETE).accepted
    assert mission.handle(MissionEvent.SELF_CHECK_OK).accepted


class C1MissionTests(unittest.TestCase):
    def test_completes_exactly_ten_ordered_gates(self) -> None:
        mission = C1Mission()
        prepare(mission)
        self.assertTrue(mission.handle(MissionEvent.ARM).accepted)
        self.assertTrue(mission.handle(MissionEvent.START).accepted)
        for gate_index in range(10):
            mission.handle(MissionEvent.GATE_OBSERVED)
            mission.handle(MissionEvent.GATE_TRACK_STABLE)
            mission.handle(MissionEvent.GATE_PASS_CANDIDATE, gate_index=gate_index)
            result = mission.handle(MissionEvent.GATE_PASS_CONFIRMED, gate_index=gate_index)
            self.assertTrue(result.accepted)
        self.assertEqual(mission.gates_passed, 10)
        self.assertEqual(mission.substate, C1State.FINISHED)
        self.assertEqual(mission.global_state, GlobalState.COMPLETE)

    def test_wrong_gate_and_duplicate_evidence_do_not_advance(self) -> None:
        mission = C1Mission(total_gates=2)
        prepare(mission)
        mission.handle(MissionEvent.ARM)
        mission.handle(MissionEvent.START)
        mission.handle(MissionEvent.GATE_OBSERVED)
        mission.handle(MissionEvent.GATE_TRACK_STABLE)
        rejected = mission.handle(MissionEvent.GATE_PASS_CANDIDATE, gate_index=1)
        self.assertFalse(rejected.accepted)
        self.assertEqual(mission.gates_passed, 0)
        mission.handle(MissionEvent.GATE_PASS_CANDIDATE, gate_index=0)
        mission.handle(MissionEvent.GATE_PASS_CONFIRMED, gate_index=0)
        duplicate = mission.handle(MissionEvent.GATE_PASS_CONFIRMED, gate_index=0)
        self.assertFalse(duplicate.accepted)
        self.assertEqual(mission.gates_passed, 1)

    def test_recovery_and_emergency_stop_are_explicit(self) -> None:
        mission = C1Mission()
        prepare(mission)
        mission.handle(MissionEvent.ARM)
        mission.handle(MissionEvent.START)
        mission.handle(MissionEvent.GATE_OBSERVED)
        self.assertTrue(mission.handle(MissionEvent.OBSERVATION_LOST).accepted)
        self.assertEqual(mission.substate, C1State.RECOVERY)
        mission.handle(MissionEvent.RECOVERED)
        self.assertEqual(mission.substate, C1State.SEARCH_GATE)
        mission.handle(MissionEvent.EMERGENCY_STOP)
        self.assertEqual(mission.global_state, GlobalState.E_STOP)
        rejected = mission.handle(MissionEvent.TIMEOUT)
        self.assertFalse(rejected.accepted)
        self.assertEqual(mission.global_state, GlobalState.E_STOP)
        mission.handle(MissionEvent.LINK_LOST)
        self.assertEqual(mission.global_state, GlobalState.E_STOP)
        self.assertTrue(mission.handle(MissionEvent.RESET).accepted)
        self.assertEqual(mission.global_state, GlobalState.DISARMED)


class C2MissionTests(unittest.TestCase):
    def test_c1_prerequisite_blocks_arm(self) -> None:
        mission = C2Mission(c1_completed=False)
        prepare(mission)
        result = mission.handle(MissionEvent.ARM)
        self.assertFalse(result.accepted)
        self.assertEqual(mission.global_state, GlobalState.DISARMED)

    def test_navigation_fire_window_and_completion(self) -> None:
        mission = C2Mission(c1_completed=True)
        prepare(mission)
        mission.handle(MissionEvent.ARM)
        mission.handle(MissionEvent.START)
        self.assertEqual(mission.substate, C2State.TRANSIT)
        for event in (
            MissionEvent.ZONE_DETECTED,
            MissionEvent.ZONE_CONFIRMED,
            MissionEvent.TARGET_ACQUIRED,
            MissionEvent.AIM_LOCKED,
        ):
            self.assertTrue(mission.handle(event).accepted)
        self.assertTrue(mission.launch_window_open)
        mission.handle(MissionEvent.FIRE_ACCEPTED)
        self.assertFalse(mission.launch_window_open)
        mission.handle(MissionEvent.SHOT_CONFIRMED)
        mission.handle(MissionEvent.TARGETS_COMPLETE)
        self.assertEqual(mission.substate, C2State.FINISHED)
        self.assertEqual(mission.global_state, GlobalState.COMPLETE)

    def test_lost_target_revokes_fire_window(self) -> None:
        mission = C2Mission(c1_completed=True)
        prepare(mission)
        mission.handle(MissionEvent.ARM)
        mission.handle(MissionEvent.START)
        for event in (
            MissionEvent.ZONE_DETECTED,
            MissionEvent.ZONE_CONFIRMED,
            MissionEvent.TARGET_ACQUIRED,
            MissionEvent.AIM_LOCKED,
        ):
            mission.handle(event)
        self.assertTrue(mission.launch_window_open)
        mission.handle(MissionEvent.OBSERVATION_LOST)
        self.assertEqual(mission.substate, C2State.SAFE_HOLD)
        self.assertFalse(mission.launch_window_open)


if __name__ == "__main__":
    unittest.main()
