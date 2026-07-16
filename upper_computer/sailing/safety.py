"""Upper-level motion safety gate; the lower computer repeats these checks."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .models import ControlCommand


@dataclass(frozen=True, slots=True)
class SafetyContext:
    """Safety facts evaluated for one upper-computer command."""

    link_age_seconds: float
    armed: bool
    emergency_stop: bool = False


class SafetyGate:
    """Neutralize stale, disarmed, or emergency motion requests."""

    def __init__(self, command_timeout_seconds: float = 0.25) -> None:
        if not math.isfinite(command_timeout_seconds) or command_timeout_seconds <= 0:
            raise ValueError("command timeout must be positive")
        self.command_timeout_seconds = command_timeout_seconds

    def apply(self, command: ControlCommand, context: SafetyContext) -> ControlCommand:
        """Return a bounded motion command safe to transmit."""

        if not math.isfinite(context.link_age_seconds) or context.link_age_seconds < 0:
            raise ValueError("link age must be finite and non-negative")
        if (
            context.emergency_stop
            or not context.armed
            or context.link_age_seconds > self.command_timeout_seconds
        ):
            return ControlCommand.neutral()
        return command.clamped()
