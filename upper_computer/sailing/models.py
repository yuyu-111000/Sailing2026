"""Hardware-independent domain models shared by upper-computer modules."""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum, auto
import math


class MissionMode(Enum):
    """Run profile selected before arming."""

    C1 = "c1"
    C2 = "c2"
    SIMULATION = "simulation"


class GlobalState(Enum):
    """Safety-relevant lifecycle common to C1 and C2."""

    BOOT = auto()
    SELF_TEST = auto()
    DISARMED = auto()
    ARMED = auto()
    RUNNING = auto()
    COMPLETE = auto()
    ABORTED = auto()
    FAULT = auto()
    E_STOP = auto()


class MissionEvent(Enum):
    """Explicit inputs accepted by the deterministic mission state machines."""

    TICK = auto()
    BOOT_COMPLETE = auto()
    SELF_CHECK_OK = auto()
    SELF_CHECK_FAILED = auto()
    ARM = auto()
    DISARM = auto()
    START = auto()
    GATE_OBSERVED = auto()
    GATE_TRACK_STABLE = auto()
    GATE_PASS_CANDIDATE = auto()
    GATE_PASS_CONFIRMED = auto()
    ZONE_DETECTED = auto()
    ZONE_CONFIRMED = auto()
    TARGET_ACQUIRED = auto()
    AIM_LOCKED = auto()
    FIRE_ACCEPTED = auto()
    SHOT_CONFIRMED = auto()
    NEXT_TARGET = auto()
    TARGETS_COMPLETE = auto()
    OBSERVATION_LOST = auto()
    RECOVERED = auto()
    TIMEOUT = auto()
    LINK_LOST = auto()
    INTERNAL_FAULT = auto()
    ABORT = auto()
    EMERGENCY_STOP = auto()
    RESET = auto()


class FireSource(Enum):
    """Auditable origin of a C2 single-shot request."""

    AUTO = 1
    MANUAL_FUTURE_REMOTE = 2


@dataclass(frozen=True, slots=True)
class ControlCommand:
    """Normalized propulsion and steering request in the interval ``[-1, 1]``."""

    propulsion: float = 0.0
    steering: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.propulsion) or not math.isfinite(self.steering):
            raise ValueError("control values must be finite")

    def clamped(self) -> "ControlCommand":
        """Return a copy restricted to normalized actuator bounds."""

        return replace(
            self,
            propulsion=max(-1.0, min(1.0, self.propulsion)),
            steering=max(-1.0, min(1.0, self.steering)),
        )

    @classmethod
    def neutral(cls) -> "ControlCommand":
        """Return the safe motion command."""

        return cls()


@dataclass(frozen=True, slots=True)
class FireRequest:
    """One idempotent C2 shot request; real hardware is intentionally absent."""

    shot_id: int
    source: FireSource
    valid_for_ms: int = 250

    def __post_init__(self) -> None:
        if not 0 <= self.shot_id <= 0xFFFFFFFF:
            raise ValueError("shot_id must fit in uint32")
        if not 1 <= self.valid_for_ms <= 0xFFFF:
            raise ValueError("valid_for_ms must fit in non-zero uint16")


@dataclass(frozen=True, slots=True)
class Transition:
    """Observable result of one attempted state-machine event."""

    previous_global: GlobalState
    current_global: GlobalState
    previous_substate: Enum | None
    current_substate: Enum | None
    event: MissionEvent
    accepted: bool
    reason: str

    @property
    def changed(self) -> bool:
        """Whether either the global or mission substate changed."""

        return (
            self.previous_global is not self.current_global
            or self.previous_substate is not self.current_substate
        )
