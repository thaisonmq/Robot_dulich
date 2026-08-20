import asyncio
import importlib.util
import json
import math
from pathlib import Path

import pytest

from simulator.config import SimulatorConfig
from simulator.motion import MotionSimulator
from simulator.navigation import NavigationSimulator
from simulator.navigation_backends import (
    NavigationBackendError,
    Ros2NavigationBackend,
    SimulatorNavigationBackend,
    build_navigation_backend,
)


@pytest.mark.asyncio
async def test_simulator_backend_uses_same_compute_start_pause_cancel_flow() -> None:
    motion = MotionSimulator(SimulatorConfig())
    navigation = NavigationSimulator(motion)
    backend = SimulatorNavigationBackend(navigation, motion)
    loaded = await backend.execute("map.load", {"map_id": "MAP-NEW", "version": 3})
    assert loaded["current_state"] == "READY"
    with pytest.raises(NavigationBackendError) as error:
        await backend.execute(
            "navigation.compute_path", {"goal": {"x": 6.5, "y": 6.0, "yaw": 0}}
        )
    assert error.value.code == "MAP_VALIDATION_UNAVAILABLE"
    points = [{"x": 5.5, "y": 6.0}, {"x": 6.5, "y": 6.0}]
    started = await backend.execute(
        "navigation.start",
        {
            "mission_id": "mission-1",
            "goal": {"x": 6.5, "y": 6.0, "yaw": 0},
            "points": points,
        },
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
        {
            "mission_id": "mission-1",
            "goal": {"x": 8.0, "y": 6.0, "yaw": 0},
            "points": [{"x": 5.5, "y": 6.0}, {"x": 8.0, "y": 6.0}],
        },
    )
    await backend.manual_takeover()
    assert backend.current_state == "CANCELED"
    with pytest.raises(NavigationBackendError):
        await backend.execute("navigation.resume", {})


def test_hardware_motion_cannot_silently_use_simulator_navigation() -> None:
    motion = MotionSimulator(SimulatorConfig())
    with pytest.raises(NavigationBackendError) as error:
        build_navigation_backend(
            "simulator",
            NavigationSimulator(motion),
            motion,
            "/tmp/navigation.sock",
            motion_backend="ros2",
        )
    assert error.value.code == "NAVIGATION_BACKEND_UNSAFE"


def test_simulator_corner_contract_never_combines_turn_and_forward_arc() -> None:
    motion = MotionSimulator(SimulatorConfig(initial_yaw=math.pi / 2))
    navigation = NavigationSimulator(motion)
    navigation.start(
        "route",
        [{"x": 5.5, "y": 6.0}, {"x": 6.5, "y": 6.0}],
    )
    navigation.update()
    assert motion.linear_x == 0.0
    assert abs(motion.angular_z) > 0.05
    motion.pose.yaw = 0.02
    navigation.update()
    assert motion.linear_x > 0.0
    assert abs(motion.angular_z) <= 0.18


@pytest.mark.asyncio
async def test_auto_speed_mode_switches_at_runtime_and_rejects_invalid_mode() -> None:
    motion = MotionSimulator(SimulatorConfig())
    backend = SimulatorNavigationBackend(NavigationSimulator(motion), motion)

    result = await backend.execute(
        "navigation.speed_mode",
        {"mode": "FAST", "expected_state": "STALE_BROWSER_STATE"},
    )

    assert result["mode"] == "FAST"
    assert backend.state()["auto_speed_mode"] == "FAST"
    with pytest.raises(NavigationBackendError) as error:
        await backend.execute("navigation.speed_mode", {"mode": "TURBO"})
    assert error.value.code == "INVALID_SPEED_MODE"


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
async def test_map_deactivate_is_unconditional_when_health_state_is_stale() -> None:
    motion = MotionSimulator(SimulatorConfig())
    backend = SimulatorNavigationBackend(NavigationSimulator(motion), motion)

    result = await backend.execute(
        "map.deactivate",
        {"map_id": "MAP-001", "version": 1, "expected_state": "NAVIGATING"},
    )

    assert result["current_state"] == "NO_ACTIVE_MAP"
    assert backend.state()["map_id"] == ""


@pytest.mark.asyncio
async def test_new_mapping_can_start_after_terminal_session() -> None:
    motion = MotionSimulator(SimulatorConfig())
    backend = SimulatorNavigationBackend(NavigationSimulator(motion), motion)
    await backend.execute("mapping.start", {"expected_state": "IDLE"})
    await backend.execute("mapping.discard", {"expected_state": "MAPPING_RUNNING"})

    restarted = await backend.execute("mapping.start", {"expected_state": "IDLE"})

    assert restarted["current_state"] == "MAPPING_RUNNING"


@pytest.mark.asyncio
async def test_ros2_manual_control_does_not_cancel_mapping(monkeypatch) -> None:
    backend = Ros2NavigationBackend("/tmp/navigation.sock")
    backend._state = {"mode": "MAPPING", "state": "MAPPING_RUNNING"}

    async def unexpected_execute(command: str, payload: dict) -> dict:
        raise AssertionError(f"unexpected command: {command} {payload}")

    monkeypatch.setattr(backend, "execute", unexpected_execute)
    await backend.manual_takeover()


@pytest.mark.asyncio
async def test_ros2_manual_control_hands_off_and_preserves_active_destination(monkeypatch) -> None:
    backend = Ros2NavigationBackend("/tmp/navigation.sock")
    backend._state = {"mode": "NAVIGATION", "state": "NAVIGATING"}
    commands: list[str] = []

    async def record_execute(command: str, payload: dict) -> dict:
        commands.append(command)
        return {"status": "completed", "current_state": "CANCELED"}

    monkeypatch.setattr(backend, "execute", record_execute)
    await backend.manual_takeover()
    assert commands == ["navigation.manual_handoff"]


def test_ros2_mapping_commands_allow_slow_posegraph_io() -> None:
    backend = Ros2NavigationBackend("/tmp/navigation.sock", timeout_seconds=20)

    assert backend._response_timeout("navigation.cancel") == 20
    assert backend._response_timeout("mapping.save_draft") == 90
    assert backend._response_timeout("mapping.save") == 90
    assert backend._response_timeout("mapping.finish") == 90
    assert backend._response_timeout("mapping.start") == 90


@pytest.mark.asyncio
async def test_ros2_speed_mode_never_requests_a_stack_restart(monkeypatch) -> None:
    backend = Ros2NavigationBackend("/tmp/navigation.sock")
    calls: list[tuple[str, dict]] = []

    async def fake_call(command: str, payload: dict, timeout: float) -> dict:
        del timeout
        calls.append((command, payload))
        return {
            "status": "completed",
            "mode": "SLOW",
            "state": {"mode": "MAPPING", "state": "MAPPING_RUNNING"},
        }

    async def forbidden_mode_switch(command: str) -> bool:
        raise AssertionError(f"speed mode tried to restart stack for {command}")

    monkeypatch.setattr(backend, "_call_adapter", fake_call)
    monkeypatch.setattr(backend, "_ensure_mode", forbidden_mode_switch)

    # execute still calls _ensure_mode; exercise the real decision separately
    # and then stub it to its expected no-switch result.
    assert backend._required_mode("navigation.speed_mode") is None
    monkeypatch.setattr(backend, "_ensure_mode", lambda command: asyncio.sleep(0, result=False))
    result = await backend.execute("navigation.speed_mode", {"mode": "SLOW"})
    assert result["mode"] == "SLOW"
    assert calls == [("navigation.speed_mode", {"mode": "SLOW"})]


def test_ros2_mapping_start_retry_filter_only_accepts_startup_races() -> None:
    assert Ros2NavigationBackend._retryable_mapping_start({
        "error_code": "MAPPING_AUTHORITY_CONFLICT",
        "error_message": "Another ROS mapping authority is active: _NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_",
    })
    assert Ros2NavigationBackend._retryable_mapping_start({
        "error_code": "SCAN_STALE",
    })
    assert not Ros2NavigationBackend._retryable_mapping_start({
        "error_code": "MAPPING_AUTHORITY_CONFLICT",
        "error_message": "Another ROS mapping authority is active: /other/slam_toolbox",
    })


@pytest.mark.asyncio
async def test_ros2_mapping_start_retries_transient_dds_authority_after_switch(
    tmp_path, monkeypatch
) -> None:
    marker = tmp_path / "mode-request.json"
    backend = Ros2NavigationBackend(
        str(tmp_path / "navigation.sock"),
        mode_request_path=str(marker),
        mode_switch_timeout_seconds=2,
    )
    status_calls = 0
    start_calls = 0

    async def no_sleep(_: float) -> None:
        return None

    async def fake_call(command: str, payload: dict, timeout: float) -> dict:
        nonlocal status_calls, start_calls
        del timeout
        if command == "system.status":
            status_calls += 1
            if status_calls == 1:
                raise OSError("old adapter stopped")
            state = {"mode": "MAPPING", "state": "IDLE", "nav2": "MAPPING"}
            backend._state.update(state)
            return {"status": "completed", "state": state}
        start_calls += 1
        assert payload["expected_state"] == "IDLE"
        if start_calls == 1:
            return {
                "status": "rejected",
                "current_state": "IDLE",
                "error_code": "MAPPING_AUTHORITY_CONFLICT",
                "error_message": "Another ROS mapping authority is active: _NODE_NAMESPACE_UNKNOWN_/_NODE_NAME_UNKNOWN_",
                "state": {"mode": "MAPPING", "state": "IDLE"},
            }
        return {
            "status": "completed",
            "current_state": "MAPPING_RUNNING",
            "state": {"mode": "MAPPING", "state": "MAPPING_RUNNING"},
        }

    monkeypatch.setattr(backend, "_call_adapter", fake_call)
    monkeypatch.setattr("simulator.navigation_backends.asyncio.sleep", no_sleep)

    result = await backend.execute("mapping.start", {"expected_state": "IDLE"})

    assert result["current_state"] == "MAPPING_RUNNING"
    assert start_calls == 2


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
async def test_ros2_backend_surfaces_mode_supervisor_fault_immediately(
    tmp_path, monkeypatch
) -> None:
    marker = tmp_path / "mode-request.json"
    backend = Ros2NavigationBackend(
        str(tmp_path / "navigation.sock"),
        mode_request_path=str(marker),
        mode_switch_timeout_seconds=30,
    )

    async def old_adapter(command: str, payload: dict, timeout: float) -> dict:
        del payload, timeout
        assert command == "system.status"
        return {
            "status": "completed",
            "state": {"mode": "MAPPING", "state": "MAPPING", "nav2": "MAPPING"},
        }

    async def publish_fault(_: float) -> None:
        request = json.loads(marker.read_text())
        marker.with_name("mode-status.json").write_text(json.dumps({
            "request_id": request["request_id"],
            "mode": "NAVIGATION",
            "status": "FAULT",
            "error": "managed-motion base authority is unsafe",
        }))

    monkeypatch.setattr(backend, "_call_adapter", old_adapter)
    monkeypatch.setattr("simulator.navigation_backends.asyncio.sleep", publish_fault)

    with pytest.raises(NavigationBackendError) as captured:
        await backend.execute("map.relocalize", {"expected_state": "READY"})

    assert captured.value.code == "MODE_SWITCH_FAILED"
    assert "base authority is unsafe" in str(captured.value)


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


@pytest.mark.asyncio
async def test_ros2_map_deactivate_waits_until_idle_runtime(tmp_path, monkeypatch) -> None:
    marker = tmp_path / "mode-request.json"
    backend = Ros2NavigationBackend(
        str(tmp_path / "navigation.sock"),
        mode_request_path=str(marker),
        mode_switch_timeout_seconds=2,
    )

    async def fake_call(command: str, payload: dict, timeout: float) -> dict:
        del payload, timeout
        if command == "system.status":
            return {
                "status": "completed",
                "state": {"mode": "NAVIGATION", "state": "READY", "nav2": "READY"},
            }
        return {
            "status": "completed",
            "current_state": "NO_ACTIVE_MAP",
            "state": {"mode": "NAVIGATION", "state": "NO_ACTIVE_MAP"},
        }

    async def fake_supervisor() -> None:
        while not marker.exists():
            await asyncio.sleep(0.01)
        while json.loads(marker.read_text()).get("mode") != "IDLE":
            await asyncio.sleep(0.01)
        request = json.loads(marker.read_text())
        marker.with_name("mode-status.json").write_text(json.dumps({
            "request_id": request["request_id"],
            "mode": "IDLE",
            "status": "READY",
            "state": {
                "mode": "IDLE",
                "state": "NO_ACTIVE_MAP",
                "nav2": "STOPPED",
                "localized": False,
            },
        }))

    monkeypatch.setattr(backend, "_call_adapter", fake_call)
    _, result = await asyncio.gather(
        fake_supervisor(),
        backend.execute("map.deactivate", {"expected_state": "READY"}),
    )

    assert result["current_state"] == "NO_ACTIVE_MAP"
    assert backend.state()["mode"] == "IDLE"
    assert backend.state()["nav2"] == "STOPPED"


def test_mode_supervisor_idle_stops_both_ros_authorities(tmp_path, monkeypatch) -> None:
    project_dir = Path(__file__).parents[1]
    monkeypatch.setenv("ROVERA_PROJECT_DIR", str(project_dir))
    monkeypatch.setenv("ROVERA_STATE_DIR", str(tmp_path))
    script = project_dir / "scripts" / "mode_supervisor.py"
    spec = importlib.util.spec_from_file_location("test_mode_supervisor", script)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    calls: list[tuple[tuple[str, ...], str]] = []

    monkeypatch.setattr(module, "validate_base_runtime", lambda: "managed-motion")
    monkeypatch.setattr(module, "adapter_status", lambda: (_ for _ in ()).throw(OSError()))
    monkeypatch.setattr(
        module,
        "compose",
        lambda files, profile, *args, **kwargs: calls.append((tuple(args), profile)),
    )
    monkeypatch.setattr(module, "remove_stale_socket", lambda: None)

    state = module.switch_mode({"mode": "IDLE"})

    assert state == {
        "mode": "IDLE",
        "state": "NO_ACTIVE_MAP",
        "nav2": "STOPPED",
        "localized": False,
        "localization_state": "IDLE",
    }
    assert calls == [
        (("stop", "navigation-stack"), "navigation"),
        (("stop", "mapping-stack"), "legacy-coexistence"),
    ]


def test_mode_supervisor_accepts_guarded_managed_motion_without_vendor(
    tmp_path, monkeypatch
) -> None:
    project_dir = Path(__file__).parents[1]
    monkeypatch.setenv("ROVERA_PROJECT_DIR", str(project_dir))
    monkeypatch.setenv("ROVERA_STATE_DIR", str(tmp_path))
    script = project_dir / "scripts" / "mode_supervisor.py"
    spec = importlib.util.spec_from_file_location("managed_mode_supervisor", script)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    values = {
        "ROVERA_CONTROL_MODE": "managed-motion",
        "ROVERA_CMD_VEL_MODE": "exclusive",
        "ROVERA_EXCLUSIVE_CMD_VEL_ACK": module.REQUIRED_EXCLUSIVE_ACK,
    }
    monkeypatch.setattr(module, "configured_value", values.get)
    monkeypatch.setattr(module, "runtime_process_counts", lambda: {
        "vendor_base": 0,
        "serial_agent": 1,
        "managed_bridge": 1,
        "motion_safety": 1,
    })

    assert module.validate_base_runtime() == "managed-motion"


def test_mode_supervisor_rejects_managed_motion_without_safety(
    tmp_path, monkeypatch
) -> None:
    project_dir = Path(__file__).parents[1]
    monkeypatch.setenv("ROVERA_PROJECT_DIR", str(project_dir))
    monkeypatch.setenv("ROVERA_STATE_DIR", str(tmp_path))
    script = project_dir / "scripts" / "mode_supervisor.py"
    spec = importlib.util.spec_from_file_location("unsafe_mode_supervisor", script)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    values = {
        "ROVERA_CONTROL_MODE": "managed-motion",
        "ROVERA_CMD_VEL_MODE": "exclusive",
        "ROVERA_EXCLUSIVE_CMD_VEL_ACK": module.REQUIRED_EXCLUSIVE_ACK,
    }
    monkeypatch.setattr(module, "configured_value", values.get)
    monkeypatch.setattr(module, "runtime_process_counts", lambda: {
        "vendor_base": 0,
        "serial_agent": 1,
        "managed_bridge": 1,
        "motion_safety": 0,
    })

    with pytest.raises(RuntimeError, match="motion_safety=0"):
        module.validate_base_runtime()


def test_motion_safety_uses_only_normalized_scan_and_publishes_atomic_status() -> None:
    project = Path(__file__).parents[1]
    safety_node = (project / "motion-safety/safety_node.py").read_text()
    adapter = (project / "navigation-stack/adapter_node.py").read_text()

    assert 'LaserScan, "/scan/normalized"' in safety_node
    assert 'LaserScan, "/scan", self._on_scan' not in safety_node
    assert 'String, "/safety/status"' in adapter
    assert 'self.status_state = self.create_publisher(String, "/safety/status"' in safety_node
    # Compatibility topics remain available for non-navigation consumers.
    assert '"/safety/health"' in safety_node
    assert '"/safety/directional_mask"' in safety_node


def test_visualization_delta_is_route_aware_and_explicit_about_clear() -> None:
    from simulator.client import navigation_visualization_delta

    first, state = navigation_visualization_delta(
        {
            "revision": 1,
            "map_id": "MAP-A",
            "map_version": 1,
            "route_id": "R1",
            "global_path": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
            "dynamic_obstacles": [],
        },
        None,
    )
    assert first["route_id"] == "R1"
    assert "global_path" in first

    obstacle_only, state = navigation_visualization_delta(
        {
            "revision": 2,
            "map_id": "MAP-A",
            "map_version": 1,
            "route_id": "R1",
            "global_path": first["global_path"],
            "dynamic_obstacles": [{"x": 0.5, "y": 0.0}],
        },
        state,
    )
    assert obstacle_only == {
        "revision": 2,
        "map_id": "MAP-A",
        "map_version": 1,
        "route_id": "R1",
        "dynamic_obstacles": [{"x": 0.5, "y": 0.0}],
    }

    route_change, state = navigation_visualization_delta(
        {
            "revision": 3,
            "map_id": "MAP-A",
            "map_version": 1,
            "route_id": "R2",
            # Route identity still forces a full path even when rounded
            # geometry happens to match the previous route.
            "global_path": first["global_path"],
            "dynamic_obstacles": obstacle_only["dynamic_obstacles"],
        },
        state,
    )
    assert route_change["route_id"] == "R2"
    assert route_change["global_path"] == first["global_path"]

    explicit_clear, _ = navigation_visualization_delta(
        {
            "revision": 4,
            "map_id": "MAP-A",
            "map_version": 1,
            "route_id": "R2",
            "global_path": [],
            "dynamic_obstacles": obstacle_only["dynamic_obstacles"],
        },
        state,
    )
    assert explicit_clear["route_id"] == "R2"
    assert explicit_clear["global_path"] == []
