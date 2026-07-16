"""Deterministic adapters for tests and algorithm prototyping."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from .models import ControlCommand


@dataclass(slots=True)
class ManualClock:
    now: float = 0.0

    def __post_init__(self) -> None:
        if not math.isfinite(self.now) or self.now < 0:
            raise ValueError("monotonic clock must start finite and non-negative")

    def monotonic(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("clock advance must be finite and non-negative")
        self.now += seconds


@dataclass(slots=True)
class MemoryTransport:
    writes: list[bytes] = field(default_factory=list)

    def write(self, data: bytes) -> None:
        self.writes.append(bytes(data))


@dataclass(slots=True)
class FixedGuidance:
    value: ControlCommand = field(default_factory=ControlCommand.neutral)

    def command(self) -> ControlCommand:
        return self.value
