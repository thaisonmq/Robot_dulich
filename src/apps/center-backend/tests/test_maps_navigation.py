import hashlib
import io
import json
import math
import tarfile
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import HTTPException
from sqlalchemy import create_engine
from sqlalchemy.orm import Session

from app.api.maps import (
    MappingInitialPose,
    _mapping_pose_hint,
    _map_view,
    _mapping_action_reached,
    _mapping_pose_in_version,
    _mapping_start_health_failures,
    _posegraph_basename,
)
from app.api import maps as maps_api
from app.api import navigation as navigation_api
from app.core.config import Settings
from app.api.navigation import (
    ComputePathRequest,
    InitialPoseRequest,
    RelocalizeRequest,
    _mission_start_rejection,
    _normalized_route_candidates,
    _robot_by_public_id,
    _selected_candidate,
)
from app.api.websockets import persist_robot_runtime_event
from app.models.database import Base, SessionLocal
from app.models.entities import MapRecord, MappingSession, MapVersion, NavigationMission, Robot
from app.services.map_storage import InvalidMapBundle, MapBundleStore
from app.services.state_machines import (
    InvalidTransition,
    mapping_transition,
    navigation_transition,
)


def test_navigation_resolves_robot_by_public_robot_id() -> None:
    engine = create_engine("sqlite://")
    Base.metadata.create_all(engine)
    with Session(engine) as database:
        database.add(
            Robot(
                id="internal-robot-uuid",
                robot_id="ROBOT-001",
                name="Navigation test robot",
                site_id="test",
                map_id="MAP-OLD",
            )
        )
        database.commit()
        robot = _robot_by_public_id(database, "ROBOT-001")
        assert robot is not None
        assert robot.robot_id == "ROBOT-001"
        assert robot.id == "internal-robot-uuid"


def test_compute_path_auto_recovery_is_explicit_and_opt_in() -> None:
    common = {
        "request_id": "request-auto-recovery",
        "robot_id": "ROBOT-001",
        "session_id": "session-auto-recovery",
        "expected_state": "READY",
        "map_id": "MAP-TEST",
        "version": 1,
        "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0},
    }

    assert ComputePathRequest(**common).auto_recover is False
    assert ComputePathRequest(**common, auto_recover=True).auto_recover is True


@pytest.mark.asyncio
async def test_compute_path_persists_recovery_instead_of_plan_failed(monkeypatch) -> None:
    captured: dict = {}

    async def fake_command(database, settings, **kwargs):
        del database, settings
        captured.update(kwargs)
        return {
            "status": "accepted",
            "current_state": "WAITING_FOR_DYNAMIC_CLEAR",
            "points": [],
            "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0},
            "recovery_pending": True,
            "recovery_reason": "NO_EXACT_STOP_TURN_ROUTE",
        }

    class FakeDatabase:
        mission = None

        def scalar(self, _query):
            return None

        def add(self, mission):
            self.mission = mission

        def commit(self):
            return None

        def refresh(self, mission):
            now = datetime.now(timezone.utc)
            mission.created_at = now
            mission.updated_at = now

    database = FakeDatabase()
    monkeypatch.setattr(navigation_api, "_valid_lease", lambda *args: None)
    monkeypatch.setattr(
        navigation_api,
        "_active_version",
        lambda *args: SimpleNamespace(
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
            width=100,
            height=100,
            resolution=0.05,
        ),
    )
    monkeypatch.setattr(
        navigation_api,
        "_robot_by_public_id",
        lambda *args: SimpleNamespace(
            map_id="MAP-TEST", active_map_version=1
        ),
    )
    monkeypatch.setattr(navigation_api, "_command", fake_command)
    request = ComputePathRequest(
        request_id="request-recovery-mission",
        robot_id="ROBOT-001",
        session_id="session-recovery-mission",
        expected_state="READY",
        map_id="MAP-TEST",
        version=1,
        goal={"x": 1.0, "y": 2.0, "yaw": 0.0},
        auto_recover=True,
    )

    result = await navigation_api.compute_path(
        request,
        user_id="operator-1",
        database=database,
        settings=Settings(),
    )

    assert captured["payload"]["auto_recover"] is True
    assert captured["payload"]["mission_id"] == result["mission_id"]
    assert database.mission.status == "RECOVERY"
    assert database.mission.path == []
    assert database.mission.error_code is None
    assert result["status"] == "RECOVERY"


def test_navigation_route_id_is_deterministic_from_executable_geometry() -> None:
    path = [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.5}]

    assert navigation_api._route_id_for_path(path) == navigation_api._route_id_for_path(
        list(path)
    )
    assert navigation_api._route_id_for_path(path).startswith("route-")
    assert navigation_api._route_id_for_path(path) != navigation_api._route_id_for_path(
        [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.6}]
    )


def test_selected_preview_route_is_server_authoritative() -> None:
    primary = [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}]
    alternative = [
        {"x": 0.0, "y": 0.0},
        {"x": 0.5, "y": 0.5},
        {"x": 1.0, "y": 0.0},
    ]
    candidates = _normalized_route_candidates([
        {"route_id": "route-primary", "points": primary, "valid": True},
        {
            "route_id": "route-alternative",
            "points": alternative,
            "valid": True,
            "minimum_static_clearance": 0.22,
        },
        {"route_id": "route-invalid", "points": alternative, "valid": False},
    ])

    route_id, points = _selected_candidate(
        candidates,
        "route-alternative",
        primary,
    )

    assert route_id == "route-alternative"
    assert points == alternative
    assert candidates[1]["minimum_clearance"] == 0.22
    with pytest.raises(ValueError, match="UNKNOWN_ROUTE_ID"):
        _selected_candidate(candidates, "route-from-browser", primary)
    legacy_route_id = navigation_api._route_id_for_path(primary)
    assert _selected_candidate([], legacy_route_id, primary) == (
        legacy_route_id,
        primary,
    )


def test_navigation_planning_timeout_exceeds_pi_planner_budget() -> None:
    assert Settings().navigation_planning_timeout_seconds == 30.0


def test_relocalization_rotation_requires_explicit_authorization() -> None:
    request = RelocalizeRequest(
        request_id="request-passive-localization",
        robot_id="ROBOT-001",
        session_id="session-passive-localization",
        expected_state="READY",
        map_id="MAP-001",
        version=1,
    )

    assert request.allow_rotation is False
    assert request.force_global is False


def test_auto_navigation_can_request_fresh_global_localization() -> None:
    request = RelocalizeRequest(
        request_id="request-global-localization",
        robot_id="ROBOT-001",
        session_id="session-global-localization",
        expected_state="READY",
        map_id="MAP-001",
        version=1,
        allow_rotation=True,
        force_global=True,
    )

    assert request.allow_rotation is True
    assert request.force_global is True


@pytest.mark.asyncio
async def test_relocalize_allows_bounded_ros_mode_switch(monkeypatch) -> None:
    captured: dict = {}

    async def fake_command(database, settings, **kwargs):
        del database, settings
        captured.update(kwargs)
        return {"status": "accepted", "current_state": "LOCALIZING_GLOBAL"}

    monkeypatch.setattr(navigation_api, "_valid_lease", lambda *args: None)
    monkeypatch.setattr(navigation_api, "_active_version", lambda *args: object())
    monkeypatch.setattr(navigation_api, "_command", fake_command)
    settings = Settings(mapping_command_timeout_seconds=73.0)
    request = RelocalizeRequest(
        request_id="request-mode-switch-budget",
        robot_id="ROBOT-001",
        session_id="session-mode-switch-budget",
        expected_state="READY",
        map_id="MAP-001",
        version=1,
        allow_rotation=True,
        force_global=True,
    )

    result = await navigation_api.relocalize(
        request,
        user_id="operator-1",
        database=object(),
        settings=settings,
    )

    assert result["status"] == "accepted"
    assert captured["command_type"] == "map.relocalize"
    assert captured["timeout_seconds"] == 73.0


@pytest.mark.asyncio
async def test_approximate_pose_is_forwarded_as_search_center_without_yaw(monkeypatch) -> None:
    captured: dict = {}

    async def fake_command(database, settings, **kwargs):
        del database, settings
        captured.update(kwargs)
        return {"status": "accepted", "current_state": "LOCALIZING_APPROXIMATE_POSE"}

    monkeypatch.setattr(navigation_api, "_valid_lease", lambda *args: None)
    monkeypatch.setattr(
        navigation_api,
        "_active_version",
        lambda *args: SimpleNamespace(
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
            width=100,
            height=100,
            resolution=0.05,
        ),
    )
    monkeypatch.setattr(navigation_api, "_command", fake_command)
    request = InitialPoseRequest(
        request_id="request-approximate-search-hint",
        robot_id="ROBOT-001",
        session_id="session-approximate-search-hint",
        expected_state="AMBIGUOUS",
        map_id="MAP-001",
        version=1,
        pose={"x": 2.0, "y": 3.0, "yaw": 2.4},
    )

    result = await navigation_api.set_initial_pose(
        request,
        user_id="operator-1",
        database=object(),
        settings=Settings(mapping_command_timeout_seconds=73.0),
    )

    assert result["status"] == "accepted"
    assert captured["command_type"] == "map.set_initial_pose"
    assert captured["payload"]["pose"] == {"x": 2.0, "y": 3.0, "yaw": 0.0}
    assert captured["timeout_seconds"] == 73.0


def test_plan_failed_or_empty_navigation_mission_cannot_start() -> None:
    blocked = NavigationMission(
        mission_id="mission-blocked-route",
        request_id="request-blocked-route",
        robot_id="ROBOT-001",
        control_session_id="session-blocked-route",
        map_id="MAP-TEST",
        map_version=1,
        status="PLAN_FAILED",
        goal={"x": 1.0, "y": 2.0, "yaw": 0.0},
        path=[],
        error_code="NO_PATH",
        error_message="Nav2 returned an empty path",
    )
    rejection = _mission_start_rejection(blocked)
    assert rejection is not None
    assert rejection["code"] == "NO_PATH"
    assert "đường hợp lệ" in rejection["message"]

    blocked.status = "READY"
    blocked.path = [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 2.0}]
    assert _mission_start_rejection(blocked) is None


def test_plan_failed_mission_preserves_specific_footprint_diagnostic() -> None:
    blocked = NavigationMission(
        mission_id="mission-footprint",
        request_id="request-footprint",
        robot_id="ROBOT-001",
        control_session_id="session-footprint",
        map_id="MAP-TEST",
        map_version=1,
        status="PLAN_FAILED",
        goal={"x": 1.0, "y": 1.0, "yaw": 0.0},
        path=[],
        error_code="GOAL_FOOTPRINT_BLOCKED",
        error_message="Điểm đến không đủ khoảng trống cho toàn bộ thân robot.",
    )

    assert _mission_start_rejection(blocked) == {
        "code": "GOAL_FOOTPRINT_BLOCKED",
        "message": "Điểm đến không đủ khoảng trống cho toàn bộ thân robot.",
        "status": "PLAN_FAILED",
    }


def test_plan_failure_messages_distinguish_unknown_and_blocked_start() -> None:
    mission = NavigationMission(
        mission_id="mission-plan-diagnostic",
        request_id="request-plan-diagnostic",
        robot_id="ROBOT-001",
        control_session_id="session-plan-diagnostic",
        map_id="MAP-TEST",
        map_version=1,
        status="PLAN_FAILED",
        goal={"x": 1.0, "y": 1.0, "yaw": 0.0},
        path=[],
        error_code="UNKNOWN_SPACE",
        error_message="generic",
    )
    rejection = _mission_start_rejection(mission)
    assert rejection is not None
    assert "chưa được lập bản đồ" in rejection["message"]

    mission.error_code = "START_BLOCKED"
    rejection = _mission_start_rejection(mission)
    assert rejection is not None
    assert "vùng xuất phát" in rejection["message"]

    mission.error_code = "ROUTE_CLEARANCE_INSUFFICIENT"
    rejection = _mission_start_rejection(mission)
    assert rejection is not None
    assert "hard safety margin" in rejection["message"]


def bundle_bytes(*, unsafe: bool = False) -> bytes:
    output = io.BytesIO()
    artifacts = {
        "map.yaml": b"image: map.pgm\nresolution: 0.05\norigin: [-1, -2, 0.1]\n",
        "map.pgm": b"P2\n2 2\n255\n254 0 254 0\n",
    }
    metadata = {
        "map_id": "MAP-TEST",
        "name": "Test map",
        "version": 1,
        "robot_id": "ROBOT-001",
        "created_at": 1.0,
        "updated_at": 1.0,
        "resolution": 0.05,
        "origin": {"x": -1.0, "y": -2.0, "yaw": 0.1},
        "width": 2,
        "height": 2,
        "frame_id": "map",
        "checksum": hashlib.sha256(artifacts["map.pgm"]).hexdigest(),
        "has_posegraph": False,
        "slam_mode": "slam_toolbox_online_async",
        "files": {
            name: hashlib.sha256(payload).hexdigest()
            for name, payload in artifacts.items()
        },
    }
    with tarfile.open(fileobj=output, mode="w:gz") as archive:
        for name, payload in {
            **artifacts,
            "metadata.json": json.dumps(metadata).encode(),
            "../escape": b"unsafe",
        }.items():
            if name == "../escape" and not unsafe:
                continue
            info = tarfile.TarInfo(name)
            info.size = len(payload)
            archive.addfile(info, io.BytesIO(payload))
    return output.getvalue()


def test_map_bundle_checksum_and_atomic_version_install(tmp_path: Path) -> None:
    content = bundle_bytes()
    checksum = hashlib.sha256(content).hexdigest()
    store = MapBundleStore(tmp_path, max_bytes=1024 * 1024)
    path, actual, metadata = store.save(
        io.BytesIO(content), map_id="MAP-TEST", version=1, expected_checksum=checksum
    )
    assert path == tmp_path / "MAP-TEST" / "v1" / "map-bundle.tar.gz"
    assert actual == checksum
    assert metadata["origin"]["yaw"] == 0.1

    with pytest.raises(InvalidMapBundle, match="checksum"):
        store.save(
            io.BytesIO(content),
            map_id="MAP-TEST",
            version=2,
            expected_checksum="0" * 64,
        )


def test_map_bundle_rejects_path_traversal(tmp_path: Path) -> None:
    content = bundle_bytes(unsafe=True)
    with pytest.raises(InvalidMapBundle, match="unsafe path"):
        MapBundleStore(tmp_path, 1024 * 1024).save(
            io.BytesIO(content),
            map_id="MAP-TEST",
            version=1,
            expected_checksum=hashlib.sha256(content).hexdigest(),
        )
    assert not (tmp_path / "escape").exists()


def test_mapping_and_navigation_state_machines_are_idempotent_and_strict() -> None:
    assert mapping_transition("MAPPING_STARTING", "MAPPING_RUNNING") == "MAPPING_RUNNING"
    assert mapping_transition("MAPPING_STARTING", "MAPPING_LOCALIZING") == "MAPPING_LOCALIZING"
    assert mapping_transition("MAPPING_LOCALIZING", "MAPPING_RUNNING") == "MAPPING_RUNNING"
    assert mapping_transition("MAPPING_RUNNING", "MAPPING_STOPPED_UNSAVED") == "MAPPING_STOPPED_UNSAVED"
    assert mapping_transition("MAPPING_STOPPED_UNSAVED", "MAPPING_SAVING") == "MAPPING_SAVING"
    assert mapping_transition("MAPPING_SAVING", "FINISHED") == "FINISHED"
    assert mapping_transition("MAPPING", "PAUSED") == "PAUSED"
    assert mapping_transition("PAUSED", "MAPPING_RUNNING") == "MAPPING_RUNNING"
    assert mapping_transition("MAPPING", "MAPPING") == "MAPPING"
    assert mapping_transition("MAPPING", "FINISHING") == "FINISHING"
    assert mapping_transition("FINISHING", "FINISHED") == "FINISHED"
    with pytest.raises(InvalidTransition):
        mapping_transition("FINISHED", "MAPPING")
    assert navigation_transition("READY", "PLANNING") == "PLANNING"
    assert navigation_transition("READY", "COMPUTING_ALTERNATIVES") == "COMPUTING_ALTERNATIVES"
    assert navigation_transition("ROUTE_SELECTION", "READY") == "READY"
    assert navigation_transition("PLANNING", "PLAN_FAILED") == "PLAN_FAILED"
    assert navigation_transition("MAP_LOADING", "LOCALIZING_LAST_POSE") == "LOCALIZING_LAST_POSE"
    assert navigation_transition("LOCALIZING_LAST_POSE", "LOCALIZING_GLOBAL") == "LOCALIZING_GLOBAL"
    assert navigation_transition("LOCALIZING_GLOBAL", "LOCALIZING_ROTATING") == "LOCALIZING_ROTATING"
    assert navigation_transition("LOCALIZING_ROTATING", "READY") == "READY"
    assert navigation_transition("NAVIGATING", "PAUSED") == "PAUSED"
    assert navigation_transition("RECOVERY", "PAUSED") == "PAUSED"
    assert navigation_transition("RECOVERY", "ROUTE_SELECTION") == "ROUTE_SELECTION"
    assert navigation_transition("ROUTE_SELECTION", "BLOCKED") == "BLOCKED"
    assert navigation_transition("BLOCKED", "COMPUTING_ALTERNATIVES") == "COMPUTING_ALTERNATIVES"
    assert navigation_transition("COMPUTING_ALTERNATIVES", "READY") == "READY"
    with pytest.raises(InvalidTransition):
        navigation_transition("ARRIVED", "NAVIGATING")


def test_mapping_action_accepts_authoritative_idempotent_terminal_state() -> None:
    canceled = {
        "status": "rejected",
        "current_state": "CANCELED",
        "error_code": "STATE_CONFLICT",
    }
    assert _mapping_action_reached(canceled, "cancel", "CANCELED") is True
    assert _mapping_action_reached(canceled, "finish", "FINISHED") is False
    # Save Draft cannot be inferred from an unchanged MAPPING runtime state.
    assert _mapping_action_reached(
        {"status": "rejected", "current_state": "MAPPING"},
        "save-draft",
        "MAPPING",
    ) is False


def test_mapping_start_from_idle_defers_health_gate_to_started_slam_adapter() -> None:
    assert _mapping_start_health_failures(
        {"source": "robot"},
        {
            "mode": "IDLE",
            "scan_fresh": False,
            "odometry_ready": False,
            "lidar_tf_ready": False,
            "safety": "UNKNOWN",
        },
    ) == []


def test_mapping_start_keeps_strict_gate_when_ros_runtime_is_active() -> None:
    failures = _mapping_start_health_failures(
        {"source": "robot"},
        {
            "mode": "NAVIGATION",
            "scan_fresh": True,
            "odometry_ready": True,
            "lidar_tf_ready": True,
            "safety": "UNKNOWN",
            "estop": False,
        },
    )

    assert failures == ["Motion safety chưa sẵn sàng."]


def test_map_version_only_continues_with_complete_posegraph_pair() -> None:
    version = MapVersion(
        map_id="MAP-CONTINUE-PAIR",
        version=1,
        status="VALIDATING",
        checksum="a" * 64,
        storage_path="/tmp/map.tar.gz",
        resolution=0.05,
        origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
        width=10,
        height=10,
        metadata_json={"files": {"posegraph.posegraph": "x", "posegraph.data": "y"}},
        created_by_robot="ROBOT-001",
    )
    assert _posegraph_basename(version) == "posegraph"
    version.metadata_json = {"files": {"posegraph.posegraph": "x"}}
    assert _posegraph_basename(version) is None


def test_continued_mapping_pose_must_be_finite_and_inside_source_version() -> None:
    version = MapVersion(
        map_id="MAP-CONTINUE-POSE",
        version=3,
        status="VALIDATING",
        checksum="a" * 64,
        storage_path="/tmp/map.tar.gz",
        resolution=0.5,
        origin={"x": 1.0, "y": 2.0, "yaw": math.pi / 2},
        width=10,
        height=8,
        metadata_json={"files": {"posegraph.posegraph": "x", "posegraph.data": "y"}},
        created_by_robot="ROBOT-001",
    )

    # Local map point (2, 3) transformed by the 90-degree map origin.
    assert _mapping_pose_in_version(
        version,
        MappingInitialPose(x=-2.0, y=4.0, yaw=0.4),
    )
    assert not _mapping_pose_in_version(
        version,
        MappingInitialPose(x=1.0, y=1.9, yaw=0.4),
    )
    with pytest.raises(HTTPException) as missing:
        _mapping_pose_hint(version, None)
    assert getattr(missing.value, "status_code", None) == 422
    assert "Phải chọn vùng gần đúng" in str(getattr(missing.value, "detail", ""))
    with pytest.raises(ValueError, match="số hữu hạn"):
        MappingInitialPose(x=float("nan"), y=0.0, yaw=0.0)


def test_faulted_power_loss_session_is_exposed_as_recoverable() -> None:
    engine = create_engine("sqlite://")
    Base.metadata.create_all(engine)
    with Session(engine) as database:
        record = MapRecord(
            map_id="MAP-AUTOSAVE-RECOVERY",
            name="M2-T5",
            image_url="",
            width_pixels=0,
            height_pixels=0,
            resolution_m_per_pixel=0.05,
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
            status="DRAFT",
        )
        session = MappingSession(
            session_id="SESSION-AUTOSAVE-RECOVERY",
            map_id=record.map_id,
            version=1,
            robot_id="ROBOT-001",
            user_id="operator-1",
            status="FAULT",
            error_code="MAPPING_RUNTIME_RESET",
            error_message="runtime reset after power loss",
            metadata_json={"name": "M2-T5", "site_id": "MQ", "floor_id": "5"},
        )
        database.add_all((record, session))
        database.commit()

        detail = _map_view(database, record, details=True)

    assert detail["mapping_session"] is None
    assert detail["recoverable_mapping_session"]["session_id"] == "SESSION-AUTOSAVE-RECOVERY"
    assert detail["recoverable_mapping_session"]["error_code"] == "MAPPING_RUNTIME_RESET"


@pytest.mark.asyncio
async def test_recover_action_dispatches_idle_autosave_recovery(monkeypatch) -> None:
    engine = create_engine("sqlite://")
    Base.metadata.create_all(engine)
    captured: dict = {}

    async def dispatch(database, **kwargs):
        del database
        captured.update(kwargs)
        return {"status": "completed", "current_state": "PAUSED"}

    monkeypatch.setattr(maps_api, "_dispatch_idempotent", dispatch)
    with Session(engine) as database:
        record = MapRecord(
            map_id="MAP-AUTOSAVE-ACTION",
            name="M2-T5",
            image_url="",
            width_pixels=0,
            height_pixels=0,
            resolution_m_per_pixel=0.05,
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
            status="DRAFT",
        )
        session = MappingSession(
            session_id="SESSION-AUTOSAVE-ACTION",
            map_id=record.map_id,
            version=1,
            robot_id="ROBOT-001",
            user_id="operator-1",
            status="FAULT",
            error_code="MAPPING_RUNTIME_RESET",
            metadata_json={"name": "M2-T5", "site_id": "MQ", "floor_id": "5"},
        )
        database.add_all((record, session))
        database.commit()

        result = await maps_api.mapping_action(
            session.session_id,
            "recover",
            maps_api.MappingCommandRequest(
                request_id="request-autosave-recovery",
                expected_state="FAULT",
            ),
            "operator-1",
            database,
            Settings(),
        )

        assert result["status"] == "PAUSED"
        assert database.get(MappingSession, session.session_id).status == "PAUSED"
    assert captured["command_type"] == "mapping.recover"
    assert captured["expected_state"] == "IDLE"
    assert captured["payload"]["map_id"] == "MAP-AUTOSAVE-ACTION"


def test_navigation_status_reconciles_active_mapping_session() -> None:
    map_id = "MAP-RUNTIME-RECONCILE"
    session_id = "mapping-runtime-reconcile"
    with SessionLocal.begin() as database:
        database.add(
            MapRecord(
                map_id=map_id,
                name="Runtime map",
                image_url="",
                width_pixels=0,
                height_pixels=0,
                resolution_m_per_pixel=0.05,
                origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
                status="DRAFT",
            )
        )
        database.add(
            MappingSession(
                session_id=session_id,
                map_id=map_id,
                version=1,
                robot_id="ROBOT-001",
                user_id="test-user",
                status="MAPPING",
            )
        )

    persist_robot_runtime_event(
        "ROBOT-001",
        "navigation.status",
        {"mode": "MAPPING", "state": "CANCELED"},
    )

    with SessionLocal() as database:
        assert database.get(MappingSession, session_id).status == "CANCELED"


def test_idle_mapping_runtime_releases_orphaned_session_lock() -> None:
    map_id = "MAP-RUNTIME-IDLE"
    session_id = "mapping-runtime-idle"
    with SessionLocal.begin() as database:
        database.add(
            MapRecord(
                map_id=map_id,
                name="Interrupted map",
                image_url="",
                width_pixels=0,
                height_pixels=0,
                resolution_m_per_pixel=0.05,
                origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
                status="DRAFT",
            )
        )
        database.add(
            MappingSession(
                session_id=session_id,
                map_id=map_id,
                version=1,
                robot_id="ROBOT-IDLE-TEST",
                user_id="test-user",
                status="MAPPING",
            )
        )

    persist_robot_runtime_event(
        "ROBOT-IDLE-TEST",
        "navigation.status",
        {"mode": "MAPPING", "state": "IDLE"},
    )

    with SessionLocal() as database:
        session = database.get(MappingSession, session_id)
        assert session.status == "FAULT"
        assert session.error_code == "MAPPING_RUNTIME_RESET"


def test_idle_runtime_does_not_fault_mapping_while_start_command_is_pending() -> None:
    map_id = "MAP-RUNTIME-STARTING"
    session_id = "mapping-runtime-starting"
    with SessionLocal.begin() as database:
        database.add(
            MapRecord(
                map_id=map_id,
                name="Starting map",
                image_url="",
                width_pixels=0,
                height_pixels=0,
                resolution_m_per_pixel=0.05,
                origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
                status="DRAFT",
            )
        )
        database.add(
            MappingSession(
                session_id=session_id,
                map_id=map_id,
                version=2,
                robot_id="ROBOT-STARTING-TEST",
                user_id="test-user",
                status="STARTING",
                error_code="MAPPING_RUNTIME_RESET",
                error_message="stale",
            )
        )

    persist_robot_runtime_event(
        "ROBOT-STARTING-TEST",
        "navigation.status",
        {"mode": "MAPPING", "state": "IDLE"},
    )
    with SessionLocal() as database:
        assert database.get(MappingSession, session_id).status == "STARTING"

    persist_robot_runtime_event(
        "ROBOT-STARTING-TEST",
        "navigation.status",
        {"mode": "MAPPING", "state": "MAPPING"},
    )
    with SessionLocal() as database:
        session = database.get(MappingSession, session_id)
        assert session.status == "MAPPING"
        assert session.error_code is None
        assert session.error_message is None


def test_navigation_runtime_preserves_failed_and_ignores_map_only_states() -> None:
    map_id = "MAP-MISSION-RUNTIME"
    mission_id = "mission-runtime-state"
    with SessionLocal.begin() as database:
        database.add(MapRecord(
            map_id=map_id,
            name="Mission state map",
            image_url="",
            width_pixels=1,
            height_pixels=1,
            resolution_m_per_pixel=0.05,
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
            status="ACTIVE",
        ))
        database.add(NavigationMission(
            mission_id=mission_id,
            request_id="request-runtime-state",
            robot_id="ROBOT-MISSION-STATE",
            control_session_id="session-runtime-state",
            map_id=map_id,
            map_version=1,
            status="NAVIGATING",
            goal={"x": 0.5, "y": 0.5, "yaw": 0.0},
            path=[],
        ))

    persist_robot_runtime_event(
        "ROBOT-MISSION-STATE",
        "navigation.status",
        {"mission_id": mission_id, "state": "FAILED"},
    )
    with SessionLocal() as database:
        assert database.get(NavigationMission, mission_id).status == "FAILED"

    persist_robot_runtime_event(
        "ROBOT-MISSION-STATE",
        "navigation.status",
        {"mission_id": mission_id, "state": "NO_ACTIVE_MAP"},
    )
    with SessionLocal() as database:
        assert database.get(NavigationMission, mission_id).status == "FAILED"


def test_robot_health_reconciles_a_succeeded_navigation_mission() -> None:
    map_id = "MAP-MISSION-HEALTH"
    mission_id = "mission-health-succeeded"
    robot_id = "ROBOT-MISSION-HEALTH"
    with SessionLocal.begin() as database:
        database.add(MapRecord(
            map_id=map_id,
            name="Mission health map",
            image_url="",
            width_pixels=1,
            height_pixels=1,
            resolution_m_per_pixel=0.05,
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
            status="ACTIVE",
        ))
        database.add(NavigationMission(
            mission_id=mission_id,
            request_id="request-health-succeeded",
            robot_id=robot_id,
            control_session_id="session-health-succeeded",
            map_id=map_id,
            map_version=1,
            status="NAVIGATING",
            goal={"x": 0.5, "y": 0.5, "yaw": 0.0},
            path=[],
        ))

    persist_robot_runtime_event(
        robot_id,
        "robot.health",
        {
            "mode": "NAVIGATION",
            "map_state": "SUCCEEDED",
            "navigation": "idle",
            "mission_id": mission_id,
        },
    )

    with SessionLocal() as database:
        mission = database.get(NavigationMission, mission_id)
        assert mission.status == "SUCCEEDED"
        assert mission.error_code is None
        assert mission.error_message is None

@pytest.mark.parametrize(
    ("runtime_state", "expected_status"),
    [
        ("WAIT_FOR_DYNAMIC_CLEAR", "RECOVERY"),
        ("WAITING_FOR_DYNAMIC_CLEAR", "RECOVERY"),
        ("DYNAMIC_REPLAN", "RECOVERY"),
        ("SENSOR_TIME_INVALID", "RECOVERY"),
        ("VERIFYING", "RECOVERY"),
        ("LOCALIZATION_REQUIRED", "LOCALIZATION_LOST"),
    ],
)
def test_navigation_runtime_projects_edge_recovery_states(
    runtime_state: str,
    expected_status: str,
) -> None:
    map_id = f"MAP-{runtime_state}"
    mission_id = f"mission-{runtime_state.lower()}"
    robot_id = f"ROBOT-{runtime_state}"
    with SessionLocal.begin() as database:
        database.add(MapRecord(
            map_id=map_id,
            name=f"Runtime state {runtime_state}",
            image_url="",
            width_pixels=1,
            height_pixels=1,
            resolution_m_per_pixel=0.05,
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
            status="ACTIVE",
        ))
        database.add(NavigationMission(
            mission_id=mission_id,
            request_id=f"request-{runtime_state.lower()}",
            robot_id=robot_id,
            control_session_id=f"session-{runtime_state.lower()}",
            map_id=map_id,
            map_version=1,
            status="NAVIGATING",
            goal={"x": 0.5, "y": 0.5, "yaw": 0.0},
            path=[],
        ))

    persist_robot_runtime_event(
        robot_id,
        "navigation.status",
        {"mission_id": mission_id, "state": runtime_state},
    )

    with SessionLocal() as database:
        assert database.get(NavigationMission, mission_id).status == expected_status
