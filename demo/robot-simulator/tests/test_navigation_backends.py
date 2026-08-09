import json

import pytest

from simulator.config import SimulatorConfig
from simulator.motion import MotionSimulator
from simulator.navigation import NavigationSimulator
from simulator.navigation_backends import (
    NavigationBackendError,
    Ros2NavigationBackend,
    SimulatorNavigationBackend,
)


@pytest.mark.asyncio
async def test_simulator_backend_uses_same_compute_start_pause_cancel_flow() -> None:
    motion = MotionSimulator(SimulatorConfig())
    navigation = NavigationSimulator(motion)
    backend = SimulatorNavigationBackend(navigation, motion)
    loaded = await backend.execute("map.load", {"map_id": "MAP-NEW", "version": 3})
    assert loaded["current_state"] == "READY"
    plan = await backend.execute(
        "navigation.compute_path", {"goal": {"x": 6.5, "y": 6.0, "yaw": 0}}
    )
    assert plan["distance_m"] == 1.0
    assert len(plan["points"]) > 2
    started = await backend.execute(
        "navigation.start",
        {"mission_id": "mission-1", "goal": {"x": 6.5, "y": 6.0, "yaw": 0}},
    )
    assert started["current_state"] == "NAVIGATING"
    assert (await backend.execute("navigation.pause", {}))["current_state"] == "PAUSED"
    assert (await backend.execute("navigation.cancel", {}))["current_state"] == "CANCELED"


@pytest.mark.asyncio
async def test_manual_takeover_cancels_without_auto_resume() -> None:
    motion = MotionSimulator(SimulatorConfig())
    navigation = NavigationSimulator(motion)
    backend = SimulatorNavigationBackend(navigation, motion)
    await backend.execute(
        "navigation.start",
        {"mission_id": "mission-1", "goal": {"x": 8.0, "y": 6.0, "yaw": 0}},
    )
    await backend.manual_takeover()
    assert backend.current_state == "CANCELED"
    with pytest.raises(NavigationBackendError):
        await backend.execute("navigation.resume", {})


@pytest.mark.asyncio
async def test_expected_state_rejects_stale_command() -> None:
    motion = MotionSimulator(SimulatorConfig())
    backend = SimulatorNavigationBackend(NavigationSimulator(motion), motion)
    with pytest.raises(NavigationBackendError) as error:
        await backend.execute(
            "map.load",
            {"map_id": "MAP-NEW", "version": 1, "expected_state": "PAUSED"},
        )
    assert error.value.code == "STATE_CONFLICT"
    assert error.value.current_state == "READY"


@pytest.mark.asyncio
async def test_new_mapping_can_start_after_terminal_session() -> None:
    motion = MotionSimulator(SimulatorConfig())
    backend = SimulatorNavigationBackend(NavigationSimulator(motion), motion)
    await backend.execute("mapping.start", {"expected_state": "IDLE"})
    await backend.execute("mapping.cancel", {"expected_state": "MAPPING"})

    restarted = await backend.execute("mapping.start", {"expected_state": "IDLE"})

    assert restarted["current_state"] == "MAPPING"


@pytest.mark.asyncio
async def test_ros2_manual_control_does_not_cancel_mapping(monkeypatch) -> None:
    backend = Ros2NavigationBackend("/tmp/navigation.sock")
    backend._state = {"mode": "MAPPING", "state": "MAPPING"}

    async def unexpected_execute(command: str, payload: dict) -> dict:
        raise AssertionError(f"unexpected command: {command} {payload}")

    monkeypatch.setattr(backend, "execute", unexpected_execute)
    await backend.manual_takeover()


@pytest.mark.asyncio
async def test_ros2_manual_control_cancels_active_navigation(monkeypatch) -> None:
    backend = Ros2NavigationBackend("/tmp/navigation.sock")
    backend._state = {"mode": "NAVIGATION", "state": "NAVIGATING"}
    commands: list[str] = []

    async def record_execute(command: str, payload: dict) -> dict:
        commands.append(command)
        return {"status": "completed", "current_state": "CANCELED"}

    monkeypatch.setattr(backend, "execute", record_execute)
    await backend.manual_takeover()
    assert commands == ["navigation.cancel"]


def test_ros2_mapping_commands_allow_slow_posegraph_io() -> None:
    backend = Ros2NavigationBackend("/tmp/navigation.sock", timeout_seconds=20)

    assert backend._response_timeout("navigation.cancel") == 20
    assert backend._response_timeout("mapping.save_draft") == 90
    assert backend._response_timeout("mapping.finish") == 90
    assert backend._response_timeout("mapping.start") == 90


@pytest.mark.asyncio
async def test_ros2_backend_requests_safe_mode_switch_before_map_load(
    tmp_path, monkeypatch
) -> None:
    marker = tmp_path / "mode-request.json"
    backend = Ros2NavigationBackend(
        str(tmp_path / "navigation.sock"),
        mode_request_path=str(marker),
        mode_switch_timeout_seconds=2,
    )
    seen_payload: dict = {}

    async def fake_call(command: str, payload: dict, timeout: float) -> dict:
        del timeout
        if command == "system.status":
            mode = "NAVIGATION" if marker.exists() else "MAPPING"
            state = {
                "mode": mode,
                "state": "READY" if mode == "NAVIGATION" else "FINISHED",
                "nav2": "READY" if mode == "NAVIGATION" else "MAPPING",
            }
            backend._state.update(state)
            return {"status": "completed", "state": state}
        seen_payload.update(payload)
        return {
            "status": "completed",
            "current_state": "LOCALIZING",
            "state": {"mode": "NAVIGATION"},
        }

    monkeypatch.setattr(backend, "_call_adapter", fake_call)
    result = await backend.execute(
        "map.load",
        {"map_id": "MAP-NEW", "version": 1, "expected_state": "FINISHED"},
    )

    assert json.loads(marker.read_text())["mode"] == "NAVIGATION"
    assert seen_payload["expected_state"] == "READY"
    assert result["current_state"] == "LOCALIZING"


@pytest.mark.asyncio
async def test_mapping_finish_requests_navigation_without_waiting(
    tmp_path, monkeypatch
) -> None:
    marker = tmp_path / "mode-request.json"
    backend = Ros2NavigationBackend(
        str(tmp_path / "navigation.sock"), mode_request_path=str(marker)
    )

    async def fake_call(command: str, payload: dict, timeout: float) -> dict:
        del payload, timeout
        state = {"mode": "MAPPING", "state": "MAPPING", "nav2": "MAPPING"}
        if command == "system.status":
            return {"status": "completed", "state": state}
        return {
            "status": "completed",
            "current_state": "FINISHED",
            "state": state,
        }

    monkeypatch.setattr(backend, "_call_adapter", fake_call)
    await backend.execute("mapping.finish", {"expected_state": "MAPPING"})

    assert json.loads(marker.read_text())["mode"] == "NAVIGATION"
