"""Composition root demonstrating the hardware-independent upper data flow."""

from __future__ import annotations

import math
from typing import Protocol

from .interfaces import ByteTransport, Clock, GuidanceProvider
from .models import ControlCommand, FireRequest, GlobalState, MissionEvent, MissionMode, Transition
from .protocol import (
    Frame,
    MessageType,
    encode_control_payload,
    encode_fire_payload,
    encode_frame,
)
from .safety import SafetyContext, SafetyGate


class Mission(Protocol):
    mode: MissionMode
    global_state: GlobalState
    launch_window_open: bool

    def handle(self, event: MissionEvent, *, gate_index: int | None = None) -> Transition: ...


class UpperRuntime:
    """Connect mission, guidance, safety, clock, and an abstract byte stream."""

    def __init__(
        self,
        mission: Mission,
        guidance: GuidanceProvider,
        safety: SafetyGate,
        transport: ByteTransport,
        clock: Clock,
        *,
        initial_session_id: int = 0,
        command_valid_for_ms: int = 500,
        mission_timeout_seconds: float = 300.0,
    ) -> None:
        if not 0 <= initial_session_id <= 0xFFFFFFFF:
            raise ValueError("initial_session_id must fit in uint32")
        if not 1 <= command_valid_for_ms <= 0xFFFF:
            raise ValueError("command_valid_for_ms must fit in non-zero uint16")
        if not math.isfinite(mission_timeout_seconds) or mission_timeout_seconds <= 0:
            raise ValueError("mission_timeout_seconds must be positive")
        self._mission = mission
        self._guidance = guidance
        self._safety = safety
        self._transport = transport
        self._clock = clock
        self._session_id = initial_session_id
        self._sequence = 0
        self._command_valid_for_ms = command_valid_for_ms
        self._mission_timeout_seconds = mission_timeout_seconds
        self._mission_started_at: float | None = None
        self._launcher_armed = False

    @property
    def session_id(self) -> int:
        return self._session_id

    @property
    def next_sequence(self) -> int:
        return self._sequence

    def _sender_time_ms(self) -> int:
        now = self._clock.monotonic()
        if not math.isfinite(now) or now < 0:
            raise ValueError("monotonic clock cannot be negative")
        return int(now * 1000) & 0xFFFFFFFF

    def _write(self, message_type: MessageType, payload: bytes = b"") -> None:
        frame = Frame(
            message_type=message_type,
            session_id=self._session_id,
            sequence=self._sequence,
            sender_time_ms=self._sender_time_ms(),
            payload=payload,
        )
        self._transport.write(encode_frame(frame))
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF

    def step(
        self,
        event: MissionEvent = MissionEvent.TICK,
        *,
        gate_index: int | None = None,
        link_age_seconds: float = 0.0,
    ) -> Transition:
        """Advance once and publish one corresponding safe protocol message."""

        now = self._clock.monotonic()
        if not math.isfinite(now) or now < 0:
            raise ValueError("monotonic clock cannot be negative")
        effective_event = event
        if (
            event is not MissionEvent.EMERGENCY_STOP
            and self._mission.global_state is GlobalState.RUNNING
            and self._mission_started_at is not None
            and now - self._mission_started_at >= self._mission_timeout_seconds
        ):
            effective_event = MissionEvent.TIMEOUT

        transition = self._mission.handle(effective_event, gate_index=gate_index)
        if effective_event is MissionEvent.START and transition.accepted:
            self._mission_started_at = now
        elif self._mission.global_state is not GlobalState.RUNNING:
            self._mission_started_at = None
        if not self._mission.launch_window_open:
            self._launcher_armed = False

        if effective_event is MissionEvent.ARM and transition.accepted:
            self._session_id = (self._session_id + 1) & 0xFFFFFFFF
            if self._session_id == 0:
                self._session_id = 1
            self._sequence = 0
            self._write(MessageType.ARM)
            return transition
        if effective_event is MissionEvent.DISARM and transition.accepted:
            self._launcher_armed = False
            self._write(MessageType.DISARM)
            return transition
        if effective_event is MissionEvent.EMERGENCY_STOP:
            self._launcher_armed = False
            self._write(MessageType.E_STOP)
            return transition

        try:
            command = self._safety.apply(
                self._guidance.command(),
                SafetyContext(
                    link_age_seconds=link_age_seconds,
                    armed=self._mission.global_state is GlobalState.RUNNING,
                    emergency_stop=self._mission.global_state is GlobalState.E_STOP,
                ),
            )
        except Exception:
            transition = self._mission.handle(MissionEvent.INTERNAL_FAULT)
            self._mission_started_at = None
            self._launcher_armed = False
            command = ControlCommand.neutral()
        self._write(
            MessageType.CONTROL_SETPOINT,
            encode_control_payload(command, valid_for_ms=self._command_valid_for_ms),
        )
        return transition

    def arm_launcher(self) -> bool:
        """Send the non-firing first phase of C2 launch authorization."""

        if self._mission.mode is not MissionMode.C2 or not self._mission.launch_window_open:
            return False
        self._write(MessageType.LAUNCH_ARM)
        self._launcher_armed = True
        return True

    def fire_once(self, request: FireRequest) -> Transition:
        """Send one unique fake-shot request after launcher-arm authorization."""

        if (
            self._mission.mode is not MissionMode.C2
            or not self._mission.launch_window_open
            or not self._launcher_armed
        ):
            raise PermissionError("C2 launcher is not armed in a valid firing window")
        self._write(MessageType.FIRE_ONCE, encode_fire_payload(request))
        self._launcher_armed = False
        return self._mission.handle(MissionEvent.FIRE_ACCEPTED)
