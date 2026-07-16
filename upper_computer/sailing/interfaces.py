"""Ports implemented later by hardware, algorithms, or simulation adapters."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from .models import ControlCommand


@runtime_checkable
class Clock(Protocol):
    """Monotonic time source."""

    def monotonic(self) -> float:
        """Return monotonically increasing seconds."""


@runtime_checkable
class ByteTransport(Protocol):
    """Ordered byte-stream transport such as UART, TCP, or a simulator pipe."""

    def write(self, data: bytes) -> None:
        """Write one encoded frame without changing its bytes."""


@runtime_checkable
class GuidanceProvider(Protocol):
    """Converts the current high-level state into a normalized command."""

    def command(self) -> ControlCommand:
        """Return the latest desired control command."""
