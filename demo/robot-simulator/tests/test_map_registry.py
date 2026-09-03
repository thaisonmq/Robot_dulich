import asyncio
import hashlib
import io
import json
import tarfile
import time
from pathlib import Path

import httpx
import pytest

from simulator.client import RobotConnectionClient
from simulator.config import SimulatorConfig
from simulator.map_cache import MapCacheError, RobotMapCacheManager
from simulator.navigation_backends import NavigationBackendError


def _bundle(
    *,
    map_id: str = "MAP-TWO",
    version: int = 2,
    yaml_content: bytes | None = None,
    metadata_overrides: dict | None = None,
    extra_artifacts: dict[str, bytes] | None = None,
    duplicate_member: str | None = None,
) -> bytes:
    output = io.BytesIO()
    artifacts = {
        "map.yaml": yaml_content
        or b"image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n",
        "map.pgm": b"P2\n1 1\n255\n254\n",
        **(extra_artifacts or {}),
    }
    files = {name: hashlib.sha256(payload).hexdigest() for name, payload in artifacts.items()}
    metadata = {
        "map_id": map_id,
        "name": "Test map",
        "version": version,
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
    metadata.update(metadata_overrides or {})
    members = [
        *artifacts.items(),
        ("metadata.json", json.dumps(metadata).encode()),
    ]
    if duplicate_member is not None:
        source_name = duplicate_member.removeprefix("./")
        members.append((duplicate_member, artifacts[source_name]))
    with tarfile.open(fileobj=output, mode="w:gz") as archive:
        for name, payload in members:
            member = tarfile.TarInfo(name)
            member.size = len(payload)
            archive.addfile(member, io.BytesIO(payload))
    return output.getvalue()


def _install_bundle(
    destination: Path,
    content: bytes,
    *,
    map_id: str,
    version: int,
    checksum: str,
) -> None:
    from simulator import map_cache as map_cache_module

    archive = destination.parent / "bundle.tar.gz"
    archive.parent.mkdir(parents=True, exist_ok=True)
    archive.write_bytes(content)
    destination.mkdir(parents=True)
    map_cache_module._safe_extract(archive, destination)
    map_cache_module._validate_unpacked(destination, map_id, version)
    (destination / map_cache_module.CACHE_BUNDLE_NAME).write_bytes(content)
    (destination / ".sha256").write_text(checksum)


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


def test_tombstone_is_durable_before_runtime_artifacts_are_purged(
    tmp_path: Path,
) -> None:
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    local = tmp_path / "MAP-RACE" / "v1"
    local.mkdir(parents=True)
    (local / "map.yaml").write_text("image: map.pgm\n")
    cache.mark_active("MAP-RACE", 1, "a" * 64, local)

    cache.record_tombstone("MAP-RACE", deleted_at=123.0)

    assert local.exists()
    assert cache.active() is None
    assert cache.is_tombstoned("MAP-RACE")
    assert cache.active_load_payload() is None

    cache.purge_tombstoned_artifacts("MAP-RACE")
    assert not local.exists()


def test_replayed_acknowledged_tombstone_does_not_rewrite_registry(
    tmp_path: Path,
    monkeypatch,
) -> None:
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    cache.record_tombstone("MAP-REPLAY", deleted_at=123.0)
    cache.acknowledge_tombstone("MAP-REPLAY")

    def unexpected_write(_registry: dict) -> None:
        raise AssertionError("acknowledged tombstone must not rewrite flash")

    monkeypatch.setattr(cache, "_write_registry", unexpected_write)
    cache.record_tombstone("MAP-REPLAY", deleted_at=123.0)
    cache.acknowledge_tombstone("MAP-REPLAY")


@pytest.mark.asyncio
async def test_tombstone_wins_if_it_arrives_during_map_download(
    tmp_path: Path,
    monkeypatch,
) -> None:
    content = _bundle()
    checksum = hashlib.sha256(content).hexdigest()

    def handler(_request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, content=content)

    real_client = httpx.AsyncClient
    monkeypatch.setattr(
        "simulator.map_cache.httpx.AsyncClient",
        lambda **kwargs: real_client(transport=httpx.MockTransport(handler), **kwargs),
    )
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    from simulator import map_cache as map_cache_module

    validate = map_cache_module._validate_unpacked

    def validate_then_delete(
        path: Path, map_id: str, version: int, **kwargs
    ) -> None:
        validate(path, map_id, version, **kwargs)
        cache.record_tombstone(map_id, deleted_at=456.0)

    monkeypatch.setattr(map_cache_module, "_validate_unpacked", validate_then_delete)

    with pytest.raises(MapCacheError, match="deleted map cannot be installed"):
        await cache.ensure(
            map_id="MAP-TWO",
            version=2,
            checksum=checksum,
            download_url="/download",
        )

    assert not (tmp_path / "MAP-TWO" / "v2").exists()
    assert cache.is_tombstoned("MAP-TWO")


@pytest.mark.asyncio
async def test_reconnect_tombstone_barrier_precedes_active_map_restore(
    tmp_path: Path,
    monkeypatch,
) -> None:
    config = SimulatorConfig(
        motion_backend="simulator",
        navigation_backend="simulator",
        map_cache_dir=str(tmp_path / "cache"),
        robot_state_file=str(tmp_path / "device.json"),
    )
    client = RobotConnectionClient(config)
    destination = Path(config.map_cache_dir) / "MAP-DELETED" / "v1"
    destination.mkdir(parents=True)
    (destination / "map.yaml").write_text("image: map.pgm\n")
    client.map_cache.mark_active(
        "MAP-DELETED", 1, "a" * 64, destination
    )
    client.navigation_backend.loaded_map_id = "MAP-DELETED"  # type: ignore[attr-defined]
    client.navigation_backend.loaded_version = 1  # type: ignore[attr-defined]
    client.navigation_backend.current_state = "NO_ACTIVE_MAP"  # type: ignore[attr-defined]

    # The navigation poll wins the scheduler race, but restore remains closed
    # until the reconnect snapshot has been reconciled.
    assert await client._restore_active_navigation_map(
        {"state": "NO_ACTIVE_MAP", "mode": "NAVIGATION"}
    ) is None
    assert client.map_cache.active() is not None

    requests: list[tuple[str, str]] = []

    def handler(request: httpx.Request) -> httpx.Response:
        requests.append((request.method, request.url.path))
        if request.method == "GET":
            return httpx.Response(
                200,
                json={
                    "snapshot_at": "2026-08-29T00:00:00Z",
                    "items": [
                        {
                            "map_id": "MAP-DELETED",
                            "deleted_at_unix": 123.0,
                        }
                    ],
                },
            )
        return httpx.Response(200, json={"status": "DELETED"})

    real_client = httpx.AsyncClient
    monkeypatch.setattr(
        "simulator.client.httpx.AsyncClient",
        lambda **kwargs: real_client(transport=httpx.MockTransport(handler), **kwargs),
    )

    async def token() -> str:
        return "robot-token"

    monkeypatch.setattr(client, "_robot_bearer_token", token)

    assert await client._synchronize_map_registry_once() == 1
    assert client.map_cache.active() is None
    assert client.map_cache.is_tombstoned("MAP-DELETED")
    assert not destination.exists()
    assert requests == [
        ("GET", "/api/maps/tombstones"),
        ("POST", "/api/maps/tombstones/MAP-DELETED/ack"),
    ]


def test_registry_barrier_rejects_motion_granting_commands(tmp_path: Path) -> None:
    client = RobotConnectionClient(
        SimulatorConfig(
            motion_backend="simulator",
            navigation_backend="simulator",
            map_cache_dir=str(tmp_path / "cache"),
            robot_state_file=str(tmp_path / "device.json"),
        )
    )

    with pytest.raises(
        NavigationBackendError, match="tombstones have not been reconciled"
    ):
        client._require_map_registry_ready("navigation.start")
    client._require_map_registry_ready("navigation.cancel")
    client.map_registry_ready.set()
    client._require_map_registry_ready("navigation.start")


@pytest.mark.asyncio
async def test_registry_loop_stays_closed_until_a_full_sync_succeeds(
    tmp_path: Path,
    monkeypatch,
) -> None:
    client = RobotConnectionClient(
        SimulatorConfig(
            motion_backend="simulator",
            navigation_backend="simulator",
            map_cache_dir=str(tmp_path / "cache"),
            robot_state_file=str(tmp_path / "device.json"),
        )
    )
    attempts = 0

    async def synchronize() -> int:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise MapCacheError("Center unavailable")
        return 0

    async def sleep(_delay: float) -> None:
        if attempts == 1:
            assert not client.map_registry_ready.is_set()
            assert client.map_registry_sync_status == "ERROR"
            return
        raise asyncio.CancelledError

    monkeypatch.setattr(client, "_synchronize_map_registry_once", synchronize)
    monkeypatch.setattr("simulator.client.asyncio.sleep", sleep)

    with pytest.raises(asyncio.CancelledError):
        await client._map_registry_sync_loop()

    assert attempts == 2
    assert client.map_registry_ready.is_set()
    assert client.map_registry_sync_status == "READY"


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

    cache.save_last_pose("MAP-NEW", 1, {
        "x": 2, "y": 3, "yaw": 1.5, "verification_version": 2,
    })
    recent = cache.activation_pose("MAP-NEW", 1, destination)
    assert recent["x"] == 2
    assert recent["covariance"] == 0.25
    assert recent["source"] == "recent_navigation_pose"


def test_runtime_restores_active_map_with_recent_verified_navigation_pose(
    tmp_path: Path,
) -> None:
    destination = tmp_path / "MAP-A" / "v2"
    content = _bundle(map_id="MAP-A", version=2)
    checksum = hashlib.sha256(content).hexdigest()
    _install_bundle(
        destination,
        content,
        map_id="MAP-A",
        version=2,
        checksum=checksum,
    )
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)
    cache.mark_active("MAP-A", 2, checksum, destination)
    # Legacy READY records predate independent global alias verification and
    # must never seed AMCL as a trusted pose after an upgrade.
    cache.save_last_pose("MAP-A", 2, {"x": 9.0, "y": 9.0, "yaw": 0.5})
    assert "last_known_pose" not in cache.active_load_payload()
    cache.save_last_pose("MAP-A", 2, {
        "x": 1.0,
        "y": -2.0,
        "yaw": 0.5,
        "verification_version": 2,
    })

    assert cache.active_load_payload() == {
        "expected_state": "NO_ACTIVE_MAP",
        "map_id": "MAP-A",
        "version": 2,
        "map_path": str(destination),
        "last_known_pose": {
            "map_id": "MAP-A",
            "map_version": 2,
            "x": 1.0,
            "y": -2.0,
            "yaw": 0.5,
            "covariance": 0.25,
            "verification_version": 2,
            "timestamp": cache.last_pose("MAP-A", 2)["timestamp"],
            "source": "recent_navigation_pose",
        },
    }

    forged_image = b"P2\n1 1\n255\n0\n"
    (destination / "map.pgm").write_bytes(forged_image)
    forged_metadata = json.loads((destination / "metadata.json").read_text())
    forged_hash = hashlib.sha256(forged_image).hexdigest()
    forged_metadata["files"]["map.pgm"] = forged_hash
    forged_metadata["checksum"] = forged_hash
    (destination / "metadata.json").write_text(json.dumps(forged_metadata))
    with pytest.raises(MapCacheError, match="verified bundle"):
        cache.active_load_payload()


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

    # A matching marker is not enough: corrupt artifacts must be revalidated
    # and replaced from the authoritative bundle before load.
    (destination / "map.pgm").write_bytes(b"corrupt")
    repaired = await cache.ensure(
        map_id="MAP-TWO", version=2, checksum=checksum, download_url="/download"
    )
    assert (repaired / "map.pgm").read_bytes() == b"P2\n1 1\n255\n254\n"

    (destination / "keep-me").write_text("old active remains")
    with pytest.raises(MapCacheError, match="checksum mismatch"):
        await cache.ensure(
            map_id="MAP-TWO", version=2, checksum="0" * 64, download_url="/download"
        )
    assert (destination / "keep-me").read_text() == "old active remains"


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("content", "message"),
    [
        (
            _bundle(yaml_content=(
                b"image: ../outside.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n"
            )),
            "bundle basename",
        ),
        (
            _bundle(metadata_overrides={"width": 2}),
            "dimensions do not match",
        ),
        (
            _bundle(duplicate_member="./map.yaml"),
            "duplicate member",
        ),
    ],
)
async def test_edge_rejects_semantically_invalid_or_ambiguous_bundle(
    tmp_path: Path,
    monkeypatch,
    content: bytes,
    message: str,
) -> None:
    checksum = hashlib.sha256(content).hexdigest()

    def handler(_request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, content=content)

    real_client = httpx.AsyncClient
    monkeypatch.setattr(
        "simulator.map_cache.httpx.AsyncClient",
        lambda **kwargs: real_client(
            transport=httpx.MockTransport(handler), **kwargs
        ),
    )
    cache = RobotMapCacheManager(tmp_path, "https://center", _token)

    with pytest.raises(MapCacheError, match=message):
        await cache.ensure(
            map_id="MAP-TWO",
            version=2,
            checksum=checksum,
            download_url="/download",
        )
    assert not (tmp_path / "MAP-TWO" / "v2").exists()


@pytest.mark.asyncio
async def test_edge_enforces_compressed_and_unpacked_bundle_limits(
    tmp_path: Path,
    monkeypatch,
) -> None:
    content = _bundle(extra_artifacts={"padding.bin": b"0" * 100_000})
    checksum = hashlib.sha256(content).hexdigest()

    def handler(_request: httpx.Request) -> httpx.Response:
        return httpx.Response(200, content=content)

    real_client = httpx.AsyncClient
    monkeypatch.setattr(
        "simulator.map_cache.httpx.AsyncClient",
        lambda **kwargs: real_client(
            transport=httpx.MockTransport(handler), **kwargs
        ),
    )
    compressed = RobotMapCacheManager(
        tmp_path / "compressed",
        "https://center",
        _token,
        max_bundle_bytes=max(1, len(content) - 1),
    )
    with pytest.raises(MapCacheError, match="compressed size limit"):
        await compressed.ensure(
            map_id="MAP-TWO",
            version=2,
            checksum=checksum,
            download_url="/download",
        )

    expanded = RobotMapCacheManager(
        tmp_path / "expanded",
        "https://center",
        _token,
        max_uncompressed_bytes=50_000,
    )
    with pytest.raises(MapCacheError, match="uncompressed size limit"):
        await expanded.ensure(
            map_id="MAP-TWO",
            version=2,
            checksum=checksum,
            download_url="/download",
        )
