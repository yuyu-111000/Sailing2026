"""Deterministic host-only scenarios used before any hardware is selected."""

from __future__ import annotations

from typing import Any

from .mission import C1Mission, C2Mission
from .models import ControlCommand, FireRequest, FireSource, MissionEvent
from .runtime import UpperRuntime
from .safety import SafetyGate
from .simulation import FixedGuidance, ManualClock, MemoryTransport


def _runtime(mission: C1Mission | C2Mission) -> tuple[UpperRuntime, MemoryTransport]:
    transport = MemoryTransport()
    runtime = UpperRuntime(
        mission=mission,
        guidance=FixedGuidance(ControlCommand(0.35, 0.0)),
        safety=SafetyGate(),
        transport=transport,
        clock=ManualClock(),
        initial_session_id=100,
    )
    return runtime, transport


def run_c1_scenario() -> dict[str, Any]:
    """Replay ten ordered fake gates and return a compact summary."""

    mission = C1Mission()
    runtime, transport = _runtime(mission)
    for event in (
        MissionEvent.BOOT_COMPLETE,
        MissionEvent.SELF_CHECK_OK,
        MissionEvent.ARM,
        MissionEvent.START,
    ):
        runtime.step(event)
    for gate_index in range(10):
        runtime.step(MissionEvent.GATE_OBSERVED)
        runtime.step(MissionEvent.GATE_TRACK_STABLE)
        runtime.step(MissionEvent.GATE_PASS_CANDIDATE, gate_index=gate_index)
        runtime.step(MissionEvent.GATE_PASS_CONFIRMED, gate_index=gate_index)
    return {
        "mode": mission.mode.value,
        "global_state": mission.global_state.name,
        "substate": mission.substate.name if mission.substate else None,
        "gates_passed": mission.gates_passed,
        "frames_written": len(transport.writes),
        "hardware_used": False,
    }


def run_c2_scenario() -> dict[str, Any]:
    """Replay one fully interlocked fake C2 shot and mission completion."""

    mission = C2Mission(c1_completed=True)
    runtime, transport = _runtime(mission)
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
    if not runtime.arm_launcher():
        raise RuntimeError("scenario failed to open the fake launch window")
    request = FireRequest(shot_id=1, source=FireSource.AUTO)
    runtime.fire_once(request)
    runtime.step(MissionEvent.SHOT_CONFIRMED)
    runtime.step(MissionEvent.TARGETS_COMPLETE)
    return {
        "mode": mission.mode.value,
        "global_state": mission.global_state.name,
        "substate": mission.substate.name if mission.substate else None,
        "shot_id": request.shot_id,
        "fire_source": request.source.name,
        "frames_written": len(transport.writes),
        "hardware_used": False,
    }
