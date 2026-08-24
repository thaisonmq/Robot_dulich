from __future__ import annotations

import asyncio
import hashlib
import json
import math
import os
import re
import shutil
import tarfile
import threading
import time
from pathlib import Path, PurePosixPath
from tempfile import mkdtemp
from typing import Awaitable, Callable

import httpx


class MapCacheError(RuntimeError):
    pass


MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{1,63}$")
RESERVED_MAP_IDS = {"created", "registry", "staging", "autosave"}
REQUIRED_METADATA_FIELDS = {
    "map_id", "name", "version", "robot_id", "created_at", "updated_at",
    "resolution", "width", "height", "origin", "frame_id", "checksum",
    "has_posegraph", "slam_mode", "files",
}


def _safe_extract(archive_path: Path, destination: Path) -> None:
    with tarfile.open(archive_path, "r:*") as archive:
        for member in archive.getmembers():
            path = PurePosixPath(member.name)
            if (
                path.is_absolute() or ".." in path.parts or member.issym()
                or member.islnk() or not (member.isfile() or member.isdir())
            ):
                raise MapCacheError("map bundle contains an unsafe path")
        # Paths and member types were checked above. Avoid tarfile's `filter=`
        # argument because Raspberry Pi OS / ROS Humble still uses Python 3.10.
        archive.extractall(destination)


def _validate_unpacked(destination: Path, map_id: str, version: int) -> None:
    """Validate identity and artifact hashes before atomic installation."""
    try:
        metadata = json.loads((destination / "metadata.json").read_text())
        if not isinstance(metadata, dict):
            raise MapCacheError("map metadata must contain an object")
        missing = REQUIRED_METADATA_FIELDS - metadata.keys()
        if missing:
            raise MapCacheError(f"map metadata is missing: {', '.join(sorted(missing))}")
        if str(metadata["map_id"]) != map_id or int(metadata["version"]) != version:
            raise MapCacheError("map metadata identity mismatch")
        if metadata["frame_id"] != "map":
            raise MapCacheError("map metadata frame_id must be map")
        if (
            float(metadata["resolution"]) <= 0
            or int(metadata["width"]) <= 0
            or int(metadata["height"]) <= 0
        ):
            raise MapCacheError("map metadata geometry is invalid")
        declared = metadata["files"]
        if not isinstance(declared, dict) or not declared:
            raise MapCacheError("map metadata files is invalid")
        actual_files = {
            path.relative_to(destination).as_posix()
            for path in destination.rglob("*") if path.is_file()
        }
        undeclared = actual_files - {"metadata.json"} - set(declared)
        if undeclared:
            raise MapCacheError(
                f"map bundle contains undeclared artifacts: {', '.join(sorted(undeclared))}"
            )
        if not (destination / "map.yaml").is_file():
            raise MapCacheError("map.yaml is missing")
        image_names = {"map.pgm", "map.png"} & set(declared)
        if not image_names:
            raise MapCacheError("saved occupancy image is missing")
        primary_checksum = str(metadata["checksum"]).lower()
        if len(primary_checksum) != 64 or not all(char in "0123456789abcdef" for char in primary_checksum):
            raise MapCacheError("map metadata checksum is invalid")
        if not any(str(declared[name]).lower() == primary_checksum for name in image_names):
            raise MapCacheError("map metadata checksum does not identify the occupancy image")
        for filename, expected_checksum in declared.items():
            relative = PurePosixPath(str(filename))
            if relative.is_absolute() or ".." in relative.parts:
                raise MapCacheError("map metadata contains an unsafe artifact path")
            artifact = destination.joinpath(*relative.parts)
            if not artifact.is_file():
                raise MapCacheError(f"declared map artifact is missing: {filename}")
            actual_checksum = hashlib.sha256(artifact.read_bytes()).hexdigest()
            if actual_checksum != str(expected_checksum).lower():
                raise MapCacheError(f"map artifact checksum mismatch: {filename}")
    except (FileNotFoundError, OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        if isinstance(exc, MapCacheError):
            raise
        raise MapCacheError("map metadata is invalid") from exc


class RobotMapCacheManager:
    def __init__(
        self,
        root: str | Path,
        center_api_url: str,
        token_provider: Callable[[], Awaitable[str]],
        *,
        verify: bool | str = True,
    ) -> None:
        self.root = Path(root)
        self.center_api_url = center_api_url.rstrip("/")
        self.token_provider = token_provider
        self.verify = verify
        self.registry_path = self.root / "registry.json"
        self._lock = asyncio.Lock()
        self._registry_lock = threading.RLock()

    @staticmethod
    def _validate_identity(map_id: str, version: int) -> None:
        if (
            not MAP_ID_PATTERN.fullmatch(map_id)
            or map_id.lower() in RESERVED_MAP_IDS
            or version < 1
        ):
            raise MapCacheError("invalid map identity")

    def _read_registry(self) -> dict:
        try:
            value = json.loads(self.registry_path.read_text())
            if isinstance(value, dict):
                return value
        except (FileNotFoundError, OSError, ValueError, json.JSONDecodeError):
            pass
        return {"schema_version": 1, "maps": {}, "active": None, "last_known_pose": {}, "tombstones": {}}

    def _write_registry(self, value: dict) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        temporary = self.registry_path.with_name(f".{self.registry_path.name}.{os.getpid()}.tmp")
        with temporary.open("w") as output:
            json.dump(value, output, separators=(",", ":"), sort_keys=True)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, self.registry_path)

    @staticmethod
    def _key(map_id: str, version: int) -> str:
        return f"{map_id}:v{version}"

    def mark_local(self, map_id: str, version: int, checksum: str, status: str) -> None:
        self._validate_identity(map_id, version)
        with self._registry_lock:
            registry = self._read_registry()
            if map_id in registry.get("tombstones", {}):
                raise MapCacheError("deleted map cannot be restored locally")
            registry.setdefault("maps", {})[self._key(map_id, version)] = {
                "map_id": map_id,
                "version": version,
                "checksum": checksum,
                "local_status": "LOCAL_ONLY" if status != "SYNCED" else "AVAILABLE",
                "sync_status": status,
                "updated_at": time.time(),
            }
            self._write_registry(registry)

    def mark_synced(self, map_id: str, version: int, checksum: str) -> None:
        self.mark_local(map_id, version, checksum, "SYNCED")

    def mark_active(self, map_id: str, version: int, checksum: str, path: Path) -> None:
        with self._registry_lock:
            self.mark_synced(map_id, version, checksum)
            registry = self._read_registry()
            registry["active"] = {
                "map_id": map_id,
                "version": version,
                "checksum": checksum,
                "path": str(path),
                "activated_at": time.time(),
            }
            self._write_registry(registry)

    def active(self) -> dict | None:
        active = self._read_registry().get("active")
        return dict(active) if isinstance(active, dict) else None

    def save_last_pose(self, map_id: str, version: int, pose: dict) -> None:
        self._validate_identity(map_id, version)
        with self._registry_lock:
            registry = self._read_registry()
            if map_id in registry.get("tombstones", {}):
                return
            registry.setdefault("last_known_pose", {})[self._key(map_id, version)] = {
                "map_id": map_id,
                "map_version": version,
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose["yaw"]),
                "covariance": float(pose.get("covariance", 0.25)),
                "verification_version": int(
                    pose.get("verification_version", 0)
                ),
                "timestamp": time.time(),
            }
            self._write_registry(registry)

    def last_pose(self, map_id: str, version: int, *, max_age_seconds: float = 604800) -> dict | None:
        value = self._read_registry().get("last_known_pose", {}).get(self._key(map_id, version))
        if not isinstance(value, dict) or time.time() - float(value.get("timestamp", 0)) > max_age_seconds:
            return None
        return dict(value)

    def activation_pose(self, map_id: str, version: int, destination: Path) -> dict | None:
        """Return the best verified initial pose for AMCL map activation.

        A pose observed during a previous navigation run has priority. For a
        newly mapped version there is no navigation history yet, but the
        immutable bundle records the robot's terminal SLAM pose. Using that
        pose avoids throwing away the strongest localization hint immediately
        after a map is saved.
        """
        # A navigation pose is persisted only after 30 seconds of independent
        # covariance, rolling-stability, scan/map and clock verification. Keep
        # that bounded covariance and heading for the fast local verification
        # phase. The adapter still rejects it and falls back to a full global
        # search when current LiDAR evidence does not agree (for example when
        # the chassis was carried while powered off).
        pose = self.last_pose(map_id, version, max_age_seconds=3600)
        if (
            pose is not None
            and int(pose.get("verification_version", 0)) >= 2
        ):
            pose["covariance"] = max(
                0.04, min(0.25, float(pose.get("covariance", 0.25)))
            )
            pose["source"] = "recent_navigation_pose"
            return pose
        try:
            metadata = json.loads((destination / "metadata.json").read_text())
            candidate = metadata.get("terminal_pose")
            updated_at = float(metadata["updated_at"])
            if (
                str(metadata.get("map_id")) != map_id
                or int(metadata.get("version", 0)) != version
                or not isinstance(candidate, dict)
                or not math.isfinite(updated_at)
                or not -300 <= time.time() - updated_at <= 3600
                or not all(
                    math.isfinite(float(candidate[axis]))
                    for axis in ("x", "y", "yaw")
                )
            ):
                return None
            return {
                "x": float(candidate["x"]),
                "y": float(candidate["y"]),
                "yaw": float(candidate["yaw"]),
                "covariance": 1.0,
                "source": "mapping_terminal_pose",
            }
        except (FileNotFoundError, OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            return None

    def active_load_payload(self, *, expected_state: str = "NO_ACTIVE_MAP") -> dict | None:
        """Build a verified adapter payload for restoring the persisted active map."""
        active = self.active()
        if active is None:
            return None
        try:
            map_id = str(active["map_id"])
            version = int(active["version"])
            destination = Path(str(active["path"])).resolve()
            self._validate_identity(map_id, version)
            destination.relative_to(self.root.resolve())
        except (KeyError, TypeError, ValueError) as exc:
            raise MapCacheError("active map registry entry is invalid") from exc
        registry = self._read_registry()
        if map_id in registry.get("tombstones", {}):
            raise MapCacheError("deleted map cannot be restored as active")
        if not (destination / "map.yaml").is_file():
            raise MapCacheError("active map.yaml is missing")
        payload = {
            "expected_state": expected_state,
            "map_id": map_id,
            "version": version,
            "map_path": str(destination),
        }
        recent_pose = self.last_pose(map_id, version, max_age_seconds=3600)
        if (
            recent_pose is not None
            and int(recent_pose.get("verification_version", 0)) >= 2
        ):
            recent_pose["covariance"] = max(
                0.01, min(0.25, float(recent_pose.get("covariance", 0.25)))
            )
            recent_pose["source"] = "recent_navigation_pose"
            payload["last_known_pose"] = recent_pose
        return payload

    def delete_local(self, map_id: str, *, deleted_at: float | None = None) -> None:
        if not MAP_ID_PATTERN.fullmatch(map_id) or map_id.lower() in RESERVED_MAP_IDS:
            raise MapCacheError("invalid map identity")
        for parent in (self.root, self.root / "created"):
            target = (parent / map_id).resolve()
            if target.parent != parent.resolve():
                raise MapCacheError("unsafe map deletion target")
            if target.exists():
                shutil.rmtree(target)
        with self._registry_lock:
            registry = self._read_registry()
            registry["maps"] = {
                key: item for key, item in registry.get("maps", {}).items()
                if item.get("map_id") != map_id
            }
            registry["last_known_pose"] = {
                key: item for key, item in registry.get("last_known_pose", {}).items()
                if item.get("map_id") != map_id
            }
            active = registry.get("active")
            if isinstance(active, dict) and active.get("map_id") == map_id:
                registry["active"] = None
            registry.setdefault("tombstones", {})[map_id] = {
                "map_id": map_id,
                "deleted_at": deleted_at or time.time(),
                "status": "DELETION_PENDING",
            }
            self._write_registry(registry)

    def acknowledge_tombstone(self, map_id: str) -> None:
        if not MAP_ID_PATTERN.fullmatch(map_id) or map_id.lower() in RESERVED_MAP_IDS:
            raise MapCacheError("invalid map identity")
        with self._registry_lock:
            registry = self._read_registry()
            item = registry.get("tombstones", {}).get(map_id)
            if isinstance(item, dict):
                item["status"] = "DELETED"
                item["acknowledged_at"] = time.time()
                self._write_registry(registry)

    def health(self) -> dict[str, int]:
        registry = self._read_registry()
        maps = list(registry.get("maps", {}).values())
        return {
            "localCount": len(maps),
            "pendingSync": sum(item.get("sync_status") == "SYNC_PENDING" for item in maps),
            "pendingDeletion": sum(
                item.get("status") == "DELETION_PENDING"
                for item in registry.get("tombstones", {}).values()
            ),
        }

    async def ensure(
        self,
        *,
        map_id: str,
        version: int,
        checksum: str,
        download_url: str,
    ) -> Path:
        self._validate_identity(map_id, version)
        destination = self.root / map_id / f"v{version}"
        marker = destination / ".sha256"
        if marker.is_file() and marker.read_text().strip() == checksum:
            self.mark_synced(map_id, version, checksum)
            return destination
        async with self._lock:
            self.root.mkdir(parents=True, exist_ok=True)
            if marker.is_file() and marker.read_text().strip() == checksum:
                self.mark_synced(map_id, version, checksum)
                return destination
            staging = Path(mkdtemp(prefix=f".{map_id}-v{version}-", dir=self.root))
            archive_path = staging / "bundle.tar.gz"
            try:
                token = await self.token_provider()
                digest = hashlib.sha256()
                url = download_url if download_url.startswith("http") else f"{self.center_api_url}{download_url}"
                async with httpx.AsyncClient(verify=self.verify, timeout=60) as client:
                    async with client.stream("GET", url, headers={"Authorization": f"Bearer {token}"}) as response:
                        response.raise_for_status()
                        with archive_path.open("wb") as output:
                            async for chunk in response.aiter_bytes(1024 * 1024):
                                digest.update(chunk)
                                output.write(chunk)
                if digest.hexdigest() != checksum:
                    raise MapCacheError("downloaded map checksum mismatch")
                unpacked = staging / "unpacked"
                unpacked.mkdir()
                await asyncio.to_thread(_safe_extract, archive_path, unpacked)
                await asyncio.to_thread(_validate_unpacked, unpacked, map_id, version)
                (unpacked / ".sha256").write_text(checksum)
                destination.parent.mkdir(parents=True, exist_ok=True)
                previous = destination.with_name(f".{destination.name}.previous")
                if previous.exists():
                    shutil.rmtree(previous)
                if destination.exists():
                    os.replace(destination, previous)
                os.replace(unpacked, destination)
                if previous.exists():
                    shutil.rmtree(previous)
                self.mark_synced(map_id, version, checksum)
                return destination
            except httpx.HTTPStatusError as exc:
                raise MapCacheError(
                    f"map download failed with HTTP {exc.response.status_code}"
                ) from exc
            except httpx.HTTPError as exc:
                raise MapCacheError("map download failed") from exc
            finally:
                shutil.rmtree(staging, ignore_errors=True)
