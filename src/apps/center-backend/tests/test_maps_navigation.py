import hashlib
import io
import json
import tarfile
from pathlib import Path

import pytest

from app.api.maps import _mapping_action_reached, _posegraph_basename
from app.api.websockets import persist_robot_runtime_event
from app.models.database import SessionLocal
from app.models.entities import MapRecord, MappingSession, MapVersion
from app.services.map_storage import InvalidMapBundle, MapBundleStore
from app.services.state_machines import (
    InvalidTransition,
    mapping_transition,
    navigation_transition,
)


def bundle_bytes(*, unsafe: bool = False) -> bytes:
    output = io.BytesIO()
    metadata = {
        "map_id": "MAP-TEST",
        "version": 1,
        "resolution": 0.05,
        "origin": {"x": -1.0, "y": -2.0, "yaw": 0.1},
        "width": 2,
        "height": 2,
    }
    with tarfile.open(fileobj=output, mode="w:gz") as archive:
        for name, payload in {
            "map.yaml": b"image: map.pgm\nresolution: 0.05\norigin: [-1, -2, 0.1]\n",
            "map.pgm": b"P2\n2 2\n255\n254 0 254 0\n",
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
    assert mapping_transition("MAPPING", "PAUSED") == "PAUSED"
    assert mapping_transition("MAPPING", "MAPPING") == "MAPPING"
    assert mapping_transition("MAPPING", "FINISHING") == "FINISHING"
    assert mapping_transition("FINISHING", "FINISHED") == "FINISHED"
    with pytest.raises(InvalidTransition):
        mapping_transition("FINISHED", "MAPPING")
    assert navigation_transition("READY", "PLANNING") == "PLANNING"
    assert navigation_transition("NAVIGATING", "PAUSED") == "PAUSED"
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
