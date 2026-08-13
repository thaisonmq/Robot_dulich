import hashlib
import io
import json
import tarfile
import time
from pathlib import Path

import httpx
import pytest

from simulator.map_cache import MapCacheError, RobotMapCacheManager


def _bundle() -> bytes:
    output = io.BytesIO()
    artifacts = {
        "map.yaml": b"image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n",
        "map.pgm": b"P2\n1 1\n255\n254\n",
    }
    files = {name: hashlib.sha256(payload).hexdigest() for name, payload in artifacts.items()}
    metadata = {
        "map_id": "MAP-TWO",
        "name": "Test map",
        "version": 2,
        "robot_id": "ROBOT-001",
        "created_at": 1.0,
        "updated_at": 1.0,
        "resolution": 0.1,
        "width": 1,
        "height": 1,
        "origin": {"x": 0, "y": 0, "yaw": 0},
        "frame_id": "map",
        "checksum": files["map.pgm"],
        "has_posegraph": False,
        "slam_mode": "slam_toolbox_online_async",
        "files": files,
    }
    with tarfile.open(fileobj=output, mode="w:gz") as archive:
        for name, payload in {
            **artifacts,
            "metadata.json": json.dumps(metadata).encode(),
        }.items():
            member = tarfile.TarInfo(name)
            member.size = len(payload)
            archive.addfile(member, io.BytesIO(payload))
    return output.getvalue()


async def _token() -> str:
    return "test-token"


def test_registry_last_pose_active_tombstone_and_no_resurrection(tmp_path: Path) -> None:
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    local = tmp_path / "MAP-ONE" / "v1"
    local.mkdir(parents=True)
    created = tmp_path / "created" / "MAP-ONE" / "v1"
    created.mkdir(parents=True)
    cache.mark_active("MAP-ONE", 1, "a" * 64, local)
    cache.save_last_pose("MAP-ONE", 1, {"x": 1, "y": -2, "yaw": 0.4})
    assert cache.last_pose("MAP-ONE", 1)["x"] == 1
    assert cache.health()["localCount"] == 1

    cache.delete_local("MAP-ONE")
    assert not local.exists() and not created.exists()
    assert cache.active() is None
    assert cache.health()["pendingDeletion"] == 1
    with pytest.raises(MapCacheError, match="deleted map"):
        cache.mark_local("MAP-ONE", 1, "a" * 64, "SYNC_PENDING")
    cache.acknowledge_tombstone("MAP-ONE")
    assert cache.health()["pendingDeletion"] == 0


def test_registry_rejects_traversal_and_reserved_deletion_targets(tmp_path: Path) -> None:
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    with pytest.raises(MapCacheError):
        cache.delete_local("../outside")
    with pytest.raises(MapCacheError):
        cache.delete_local("created")


def test_new_map_activation_uses_terminal_pose_until_navigation_pose_exists(tmp_path: Path) -> None:
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    destination = tmp_path / "MAP-NEW" / "v1"
    destination.mkdir(parents=True)
    (destination / "metadata.json").write_text(json.dumps({
        "map_id": "MAP-NEW",
        "version": 1,
        "updated_at": time.time(),
        "terminal_pose": {"x": 1.25, "y": -2.5, "yaw": 0.75},
    }))

    assert cache.activation_pose("MAP-NEW", 1, destination) == {
        "x": 1.25,
        "y": -2.5,
        "yaw": 0.75,
        "covariance": 1.0,
        "source": "mapping_terminal_pose",
    }

    cache.save_last_pose("MAP-NEW", 1, {"x": 2, "y": 3, "yaw": 1.5})
    recent = cache.activation_pose("MAP-NEW", 1, destination)
    assert recent["x"] == 2
    assert recent["covariance"] == 0.25
    assert recent["source"] == "recent_navigation_pose"


def test_runtime_restores_active_map_without_reusing_robot_pose(tmp_path: Path) -> None:
    destination = tmp_path / "MAP-A" / "v2"
    destination.mkdir(parents=True)
    (destination / "map.yaml").write_text(
        "image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n"
    )
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    cache.mark_active("MAP-A", 2, "a" * 64, destination)
    cache.save_last_pose("MAP-A", 2, {"x": 1.0, "y": -2.0, "yaw": 0.5})

    assert cache.active_load_payload() == {
        "expected_state": "NO_ACTIVE_MAP",
        "map_id": "MAP-A",
        "version": 2,
        "map_path": str(destination),
    }


@pytest.mark.asyncio
async def test_download_checksum_and_atomic_install(tmp_path: Path, monkeypatch) -> None:
    content = _bundle()
    checksum = hashlib.sha256(content).hexdigest()

    def handler(request: httpx.Request) -> httpx.Response:
        assert request.headers["Authorization"] == "Bearer test-token"
        return httpx.Response(200, content=content)

    real_client = httpx.AsyncClient
    monkeypatch.setattr(
        "simulator.map_cache.httpx.AsyncClient",
        lambda **kwargs: real_client(transport=httpx.MockTransport(handler), **kwargs),
    )
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    destination = await cache.ensure(
        map_id="MAP-TWO", version=2, checksum=checksum, download_url="/download"
    )
    assert (destination / "map.yaml").is_file()
    assert (destination / ".sha256").read_text() == checksum

    (destination / "keep-me").write_text("old active remains")
    with pytest.raises(MapCacheError, match="checksum mismatch"):
        await cache.ensure(
            map_id="MAP-TWO", version=2, checksum="0" * 64, download_url="/download"
        )
    assert (destination / "keep-me").read_text() == "old active remains"
