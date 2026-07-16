"""Deterministic, hardware-free C1-2 and C2 mission state machines."""

from __future__ import annotations

from enum import Enum, auto

from .models import GlobalState, MissionEvent, MissionMode, Transition


class C1State(Enum):
    SEARCH_GATE = auto()
    TRACK_GATE = auto()
    APPROACH_GATE = auto()
    VERIFY_PASS = auto()
    RECOVERY = auto()
    FINISHED = auto()


class C2State(Enum):
    TRANSIT = auto()
    ENTER_FIRE_ZONE = auto()
    ACQUIRE_TARGET = auto()
    AIM_STABILIZE = auto()
    FIRE_READY = auto()
    VERIFY_SHOT = auto()
    SELECT_NEXT = auto()
    SAFE_HOLD = auto()
    FINISHED = auto()


_RESETTABLE = {
    GlobalState.COMPLETE,
    GlobalState.ABORTED,
    GlobalState.FAULT,
    GlobalState.E_STOP,
}


class _MissionBase:
    mode: MissionMode

    def __init__(self) -> None:
        self.global_state = GlobalState.BOOT
        self.substate: Enum | None = None

    @property
    def launch_window_open(self) -> bool:
        return False

    def _can_arm(self) -> tuple[bool, str]:
        return True, "arm accepted"

    def _initial_substate(self) -> Enum:
        raise NotImplementedError

    def _handle_running(self, event: MissionEvent, gate_index: int | None) -> tuple[bool, str]:
        raise NotImplementedError

    def handle(self, event: MissionEvent, *, gate_index: int | None = None) -> Transition:
        """Attempt one explicit transition and return an auditable result."""

        previous_global = self.global_state
        previous_substate = self.substate
        accepted = False
        reason = "event is not valid in the current state"

        if self.global_state is GlobalState.E_STOP and event not in {
            MissionEvent.EMERGENCY_STOP,
            MissionEvent.RESET,
        }:
            reason = "emergency stop remains latched until explicit reset"
        elif event is MissionEvent.EMERGENCY_STOP:
            self.global_state = GlobalState.E_STOP
            self.substate = None
            accepted, reason = True, "emergency stop latched"
        elif event in {
            MissionEvent.SELF_CHECK_FAILED,
            MissionEvent.TIMEOUT,
            MissionEvent.LINK_LOST,
            MissionEvent.INTERNAL_FAULT,
        } and self.global_state not in _RESETTABLE:
            self.global_state = GlobalState.FAULT
            self.substate = None
            accepted, reason = True, event.name.lower()
        elif event is MissionEvent.ABORT and self.global_state in {GlobalState.ARMED, GlobalState.RUNNING}:
            self.global_state = GlobalState.ABORTED
            self.substate = None
            accepted, reason = True, "mission aborted"
        elif event is MissionEvent.DISARM and self.global_state in {GlobalState.ARMED, GlobalState.RUNNING}:
            self.global_state = GlobalState.DISARMED
            self.substate = None
            accepted, reason = True, "explicit disarm"
        elif event is MissionEvent.RESET and self.global_state in _RESETTABLE:
            self.global_state = GlobalState.DISARMED
            self.substate = None
            accepted, reason = True, "terminal state reset to disarmed"
        elif self.global_state is GlobalState.BOOT and event is MissionEvent.BOOT_COMPLETE:
            self.global_state = GlobalState.SELF_TEST
            accepted, reason = True, "boot completed"
        elif self.global_state is GlobalState.SELF_TEST and event is MissionEvent.SELF_CHECK_OK:
            self.global_state = GlobalState.DISARMED
            accepted, reason = True, "self-test passed"
        elif self.global_state is GlobalState.DISARMED and event is MissionEvent.ARM:
            accepted, reason = self._can_arm()
            if accepted:
                self.global_state = GlobalState.ARMED
        elif self.global_state is GlobalState.ARMED and event is MissionEvent.START:
            self.global_state = GlobalState.RUNNING
            self.substate = self._initial_substate()
            accepted, reason = True, "mission started"
        elif self.global_state is GlobalState.RUNNING:
            accepted, reason = self._handle_running(event, gate_index)

        return Transition(
            previous_global=previous_global,
            current_global=self.global_state,
            previous_substate=previous_substate,
            current_substate=self.substate,
            event=event,
            accepted=accepted,
            reason=reason,
        )


class C1Mission(_MissionBase):
    """C1-2 sequence guard for the ten competition gates."""

    mode = MissionMode.C1

    def __init__(self, total_gates: int = 10) -> None:
        if total_gates <= 0:
            raise ValueError("total_gates must be positive")
        super().__init__()
        self.total_gates = total_gates
        self.gates_passed = 0

    @property
    def gate_index(self) -> int:
        """Zero-based index of the gate currently expected."""

        return self.gates_passed

    def _initial_substate(self) -> C1State:
        self.gates_passed = 0
        return C1State.SEARCH_GATE

    def _handle_running(self, event: MissionEvent, gate_index: int | None) -> tuple[bool, str]:
        if event is MissionEvent.OBSERVATION_LOST and self.substate in {
            C1State.TRACK_GATE,
            C1State.APPROACH_GATE,
            C1State.VERIFY_PASS,
        }:
            self.substate = C1State.RECOVERY
            return True, "current gate observation lost"
        if self.substate is C1State.RECOVERY and event is MissionEvent.RECOVERED:
            self.substate = C1State.SEARCH_GATE
            return True, "gate search resumed"
        if self.substate is C1State.SEARCH_GATE and event is MissionEvent.GATE_OBSERVED:
            self.substate = C1State.TRACK_GATE
            return True, "current gate observed"
        if self.substate is C1State.TRACK_GATE and event is MissionEvent.GATE_TRACK_STABLE:
            self.substate = C1State.APPROACH_GATE
            return True, "gate track stable"
        if self.substate is C1State.APPROACH_GATE and event is MissionEvent.GATE_PASS_CANDIDATE:
            if gate_index != self.gate_index:
                return False, "candidate does not match the current gate"
            self.substate = C1State.VERIFY_PASS
            return True, "gate pass candidate awaiting confirmation"
        if self.substate is C1State.VERIFY_PASS and event is MissionEvent.GATE_PASS_CONFIRMED:
            if gate_index != self.gate_index:
                return False, "confirmation does not match the current gate"
            self.gates_passed += 1
            if self.gates_passed == self.total_gates:
                self.substate = C1State.FINISHED
                self.global_state = GlobalState.COMPLETE
                return True, "all gates passed in order"
            self.substate = C1State.SEARCH_GATE
            return True, "gate accepted; searching for the next gate"
        return False, "event is not valid in the current C1 substate"


class C2Mission(_MissionBase):
    """C2 navigation and fake-launch authorization state skeleton."""

    mode = MissionMode.C2

    def __init__(self, *, c1_completed: bool = False) -> None:
        super().__init__()
        self.c1_completed = c1_completed

    @property
    def launch_window_open(self) -> bool:
        return self.global_state is GlobalState.RUNNING and self.substate is C2State.FIRE_READY

    def _can_arm(self) -> tuple[bool, str]:
        if not self.c1_completed:
            return False, "C2 requires an auditable C1 completion prerequisite"
        return True, "C1 prerequisite satisfied"

    def _initial_substate(self) -> C2State:
        return C2State.TRANSIT

    def _handle_running(self, event: MissionEvent, gate_index: int | None) -> tuple[bool, str]:
        del gate_index
        if event is MissionEvent.OBSERVATION_LOST and self.substate in {
            C2State.ENTER_FIRE_ZONE,
            C2State.ACQUIRE_TARGET,
            C2State.AIM_STABILIZE,
            C2State.FIRE_READY,
        }:
            self.substate = C2State.SAFE_HOLD
            return True, "zone or target evidence lost; launch authorization revoked"
        if self.substate is C2State.SAFE_HOLD and event is MissionEvent.RECOVERED:
            self.substate = C2State.ACQUIRE_TARGET
            return True, "fresh firing-zone evidence restored"

        transitions = {
            (C2State.TRANSIT, MissionEvent.ZONE_DETECTED): (
                C2State.ENTER_FIRE_ZONE,
                "candidate firing zone detected",
            ),
            (C2State.ENTER_FIRE_ZONE, MissionEvent.ZONE_CONFIRMED): (
                C2State.ACQUIRE_TARGET,
                "firing zone confirmed",
            ),
            (C2State.ACQUIRE_TARGET, MissionEvent.TARGET_ACQUIRED): (
                C2State.AIM_STABILIZE,
                "target acquired",
            ),
            (C2State.AIM_STABILIZE, MissionEvent.AIM_LOCKED): (
                C2State.FIRE_READY,
                "target lock stable",
            ),
            (C2State.FIRE_READY, MissionEvent.FIRE_ACCEPTED): (
                C2State.VERIFY_SHOT,
                "single-shot request accepted",
            ),
            (C2State.VERIFY_SHOT, MissionEvent.SHOT_CONFIRMED): (
                C2State.SELECT_NEXT,
                "fake shot completion confirmed",
            ),
            (C2State.SELECT_NEXT, MissionEvent.NEXT_TARGET): (
                C2State.ACQUIRE_TARGET,
                "selecting the next target",
            ),
        }
        next_state = transitions.get((self.substate, event))
        if next_state is not None:
            self.substate, reason = next_state
            return True, reason
        if self.substate is C2State.SELECT_NEXT and event is MissionEvent.TARGETS_COMPLETE:
            self.substate = C2State.FINISHED
            self.global_state = GlobalState.COMPLETE
            return True, "C2 task complete"
        return False, "event is not valid in the current C2 substate"
