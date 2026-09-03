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
import warnings
from collections.abc import Awaitable, Callable
from pathlib import Path, PurePosixPath
from tempfile import mkdtemp
from typing import Any

import httpx
import yaml
from PIL import Image


class MapCacheError(RuntimeError):
    pass


MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{1,63}$")
RESERVED_MAP_IDS = {"created", "registry", "staging", "autosave"}
REQUIRED_METADATA_FIELDS = {
    "map_id", "name", "version", "robot_id", "created_at", "updated_at",
    "resolution", "width", "height", "origin", "frame_id", "checksum",
    "has_posegraph", "slam_mode", "files",
}
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
DEFAULT_MAX_BUNDLE_BYTES = 512 * 1024 * 1024
DEFAULT_MAX_UNCOMPRESSED_BYTES = 1024 * 1024 * 1024
DEFAULT_MAX_MEMBER_BYTES = 768 * 1024 * 1024
DEFAULT_MAX_MEMBERS = 64
DEFAULT_MAX_COMPRESSION_RATIO = 2000.0
DEFAULT_MAX_IMAGE_PIXELS = 100_000_000
MAX_METADATA_BYTES = 1024 * 1024
MAX_MAP_YAML_BYTES = 64 * 1024
CACHE_BUNDLE_NAME = ".map-bundle.tar.gz"


def _canonical_member_name(name: str) -> str:
    normalized = str(name)
    while normalized.startswith("./"):
        normalized = normalized[2:]
    path = PurePosixPath(normalized)
    if (
        not normalized
        or path.is_absolute()
        or len(path.parts) != 1
        or path.name != normalized
        or path.name in {".", ".."}
    ):
        raise MapCacheError("map bundle contains an unsafe path or nested path")
    return normalized


def _reject_json_constant(value: str) -> None:
    raise MapCacheError(f"map metadata contains non-finite JSON: {value}")


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapCacheError(f"{field} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MapCacheError(f"{field} must be a finite number") from exc
    if not math.isfinite(number):
        raise MapCacheError(f"{field} must be a finite number")
    return number


def _positive_integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise MapCacheError(f"{field} must be a positive integer")
    return value


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_extract(
    archive_path: Path,
    destination: Path,
    *,
    max_uncompressed_bytes: int = DEFAULT_MAX_UNCOMPRESSED_BYTES,
    max_member_bytes: int = DEFAULT_MAX_MEMBER_BYTES,
    max_members: int = DEFAULT_MAX_MEMBERS,
    max_compression_ratio: float = DEFAULT_MAX_COMPRESSION_RATIO,
) -> None:
    try:
        with tarfile.open(archive_path, "r:*") as archive:
            members: dict[str, tarfile.TarInfo] = {}
            uncompressed_bytes = 0
            for member in archive:
                if not member.isfile():
                    raise MapCacheError("map bundle contains an unsafe member type")
                canonical = _canonical_member_name(member.name)
                if canonical in members:
                    raise MapCacheError(
                        f"map bundle contains duplicate member: {canonical}"
                    )
                if member.size < 0 or member.size > max_member_bytes:
                    raise MapCacheError(
                        f"map bundle member is too large: {canonical}"
                    )
                members[canonical] = member
                if len(members) > max_members:
                    raise MapCacheError("map bundle contains too many members")
                uncompressed_bytes += member.size
                if uncompressed_bytes > max_uncompressed_bytes:
                    raise MapCacheError(
                        "map bundle exceeds the uncompressed size limit"
                    )
            compressed_bytes = max(1, archive_path.stat().st_size)
            if uncompressed_bytes / compressed_bytes > max_compression_ratio:
                raise MapCacheError("map bundle exceeds the compression ratio limit")
            for canonical, member in members.items():
                source = archive.extractfile(member)
                if source is None:
                    raise MapCacheError(
                        f"map bundle member cannot be read: {canonical}"
                    )
                artifact = destination / canonical
                with artifact.open("xb") as output:
                    copied = 0
                    for chunk in iter(
                        lambda source=source: source.read(1024 * 1024), b""
                    ):
                        copied += len(chunk)
                        if copied > member.size:
                            raise MapCacheError(
                                f"map bundle member size is invalid: {canonical}"
                            )
                        output.write(chunk)
                    if copied != member.size:
                        raise MapCacheError(
                            f"map bundle member size is invalid: {canonical}"
                        )
                    output.flush()
                    os.fsync(output.fileno())
    except (tarfile.TarError, EOFError, OSError) as exc:
        raise MapCacheError("map bundle archive is invalid") from exc


def _validate_unpacked(
    destination: Path,
    map_id: str,
    version: int,
    *,
    max_image_pixels: int = DEFAULT_MAX_IMAGE_PIXELS,
) -> None:
    """Validate identity and artifact hashes before atomic installation."""
    try:
        metadata_path = destination / "metadata.json"
        if (
            metadata_path.is_symlink()
            or not metadata_path.is_file()
            or metadata_path.stat().st_size > MAX_METADATA_BYTES
        ):
            raise MapCacheError("map metadata is missing or too large")
        metadata = json.loads(
            metadata_path.read_text(), parse_constant=_reject_json_constant
        )
        if not isinstance(metadata, dict):
            raise MapCacheError("map metadata must contain an object")
        missing = REQUIRED_METADATA_FIELDS - metadata.keys()
        if missing:
            raise MapCacheError(f"map metadata is missing: {', '.join(sorted(missing))}")
        metadata_version = _positive_integer(
            metadata["version"], "map metadata version"
        )
        if metadata["map_id"] != map_id or metadata_version != version:
            raise MapCacheError("map metadata identity mismatch")
        if metadata["frame_id"] != "map":
            raise MapCacheError("map metadata frame_id must be map")
        if not isinstance(metadata.get("name"), str) or not 1 <= len(metadata["name"]) <= 120:
            raise MapCacheError("map metadata name is invalid")
        if (
            not isinstance(metadata.get("robot_id"), str)
            or not 3 <= len(metadata["robot_id"]) <= 64
        ):
            raise MapCacheError("map metadata robot_id is invalid")
        if (
            not isinstance(metadata.get("slam_mode"), str)
            or not 1 <= len(metadata["slam_mode"]) <= 64
        ):
            raise MapCacheError("map metadata slam_mode is invalid")
        width = _positive_integer(metadata["width"], "map metadata width")
        height = _positive_integer(metadata["height"], "map metadata height")
        if width * height > max_image_pixels:
            raise MapCacheError("occupancy image exceeds the pixel limit")
        resolution = _finite_number(
            metadata["resolution"], "map metadata resolution"
        )
        if resolution <= 0:
            raise MapCacheError("map metadata resolution must be positive")
        for field in ("created_at", "updated_at"):
            _finite_number(metadata[field], f"map metadata {field}")
        origin = metadata.get("origin")
        if not isinstance(origin, dict) or not all(
            axis in origin for axis in ("x", "y", "yaw")
        ):
            raise MapCacheError("map metadata origin is invalid")
        metadata_origin = tuple(
            _finite_number(origin[axis], f"map metadata origin.{axis}")
            for axis in ("x", "y", "yaw")
        )
        declared = metadata["files"]
        if not isinstance(declared, dict) or not declared:
            raise MapCacheError("map metadata files is invalid")
        actual_files: set[str] = set()
        for path in destination.rglob("*"):
            if path.is_symlink():
                raise MapCacheError("map cache contains a symlink")
            if not path.is_file():
                continue
            relative = path.relative_to(destination)
            if len(relative.parts) != 1:
                raise MapCacheError("map cache contains a nested artifact")
            if relative.name not in {".sha256", CACHE_BUNDLE_NAME}:
                actual_files.add(relative.name)
        if set(declared) != actual_files - {"metadata.json"}:
            raise MapCacheError("map metadata files do not match cached artifacts")
        yaml_path = destination / "map.yaml"
        if (
            yaml_path.is_symlink()
            or not yaml_path.is_file()
            or yaml_path.stat().st_size > MAX_MAP_YAML_BYTES
        ):
            raise MapCacheError("map.yaml is missing")
        primary_checksum = str(metadata["checksum"]).lower()
        if not SHA256_PATTERN.fullmatch(primary_checksum):
            raise MapCacheError("map metadata checksum is invalid")
        for filename, expected_checksum in declared.items():
            if not isinstance(filename, str) or _canonical_member_name(filename) != filename:
                raise MapCacheError("map metadata contains an unsafe artifact path")
            expected = str(expected_checksum).lower()
            if not SHA256_PATTERN.fullmatch(expected):
                raise MapCacheError(f"invalid map artifact checksum: {filename}")
            artifact = destination / filename
            if artifact.is_symlink() or not artifact.is_file():
                raise MapCacheError(f"declared map artifact is missing: {filename}")
            if _file_sha256(artifact) != expected:
                raise MapCacheError(f"map artifact checksum mismatch: {filename}")

        try:
            map_yaml = yaml.safe_load(yaml_path.read_text())
        except yaml.YAMLError as exc:
            raise MapCacheError("map.yaml is invalid") from exc
        if not isinstance(map_yaml, dict):
            raise MapCacheError("map.yaml must contain an object")
        image_name = map_yaml.get("image")
        if not isinstance(image_name, str):
            raise MapCacheError("map.yaml image must be a bundle basename")
        try:
            image_name = _canonical_member_name(image_name)
        except MapCacheError as exc:
            raise MapCacheError(
                "map.yaml image must be a bundle basename"
            ) from exc
        if image_name not in {"map.pgm", "map.png"} or image_name not in declared:
            raise MapCacheError(
                "map.yaml image is not the declared occupancy image"
            )
        yaml_resolution = _finite_number(
            map_yaml.get("resolution"), "map.yaml resolution"
        )
        yaml_origin = map_yaml.get("origin")
        if not isinstance(yaml_origin, list) or len(yaml_origin) != 3:
            raise MapCacheError("map.yaml origin must contain x, y and yaw")
        parsed_yaml_origin = tuple(
            _finite_number(value, f"map.yaml origin[{index}]")
            for index, value in enumerate(yaml_origin)
        )
        if not math.isclose(yaml_resolution, resolution, rel_tol=0.0, abs_tol=1e-9):
            raise MapCacheError("map.yaml resolution does not match metadata")
        if any(
            not math.isclose(left, right, rel_tol=0.0, abs_tol=1e-9)
            for left, right in zip(parsed_yaml_origin, metadata_origin)
        ):
            raise MapCacheError("map.yaml origin does not match metadata")
        mode = str(map_yaml.get("mode", "trinary")).lower()
        if mode not in {"trinary", "scale", "raw"}:
            raise MapCacheError("map.yaml mode is unsupported")
        negate = map_yaml.get("negate", 0)
        if isinstance(negate, bool) or negate not in {0, 1}:
            raise MapCacheError("map.yaml negate must be 0 or 1")
        occupied = _finite_number(
            map_yaml.get("occupied_thresh", 0.65), "map.yaml occupied_thresh"
        )
        free = _finite_number(
            map_yaml.get("free_thresh", 0.196), "map.yaml free_thresh"
        )
        if not 0.0 <= free < occupied <= 1.0:
            raise MapCacheError("map.yaml occupancy thresholds are invalid")

        image_path = destination / image_name
        try:
            with warnings.catch_warnings():
                warnings.simplefilter("error", Image.DecompressionBombWarning)
                with Image.open(image_path) as image:
                    image.load()
                    image_width, image_height = image.size
        except (
            Image.DecompressionBombError,
            Image.DecompressionBombWarning,
            OSError,
        ) as exc:
            raise MapCacheError("occupancy image is invalid or unsafe") from exc
        if image_width * image_height > max_image_pixels:
            raise MapCacheError("occupancy image exceeds the pixel limit")
        if image_width != width or image_height != height:
            raise MapCacheError(
                "occupancy image dimensions do not match metadata"
            )
        if str(declared[image_name]).lower() != primary_checksum:
            raise MapCacheError(
                "map metadata checksum does not identify the YAML occupancy image"
            )
        if metadata.get("checksum_scope", image_name) != image_name:
            raise MapCacheError(
                "map metadata checksum_scope does not match map.yaml image"
            )

        has_posegraph = metadata.get("has_posegraph")
        if not isinstance(has_posegraph, bool):
            raise MapCacheError("map metadata has_posegraph must be boolean")
        artifacts = set(declared)
        posegraphs = {
            filename.removesuffix(".posegraph")
            for filename in artifacts
            if filename.endswith(".posegraph")
        }
        posegraph_data = {
            filename.removesuffix(".data")
            for filename in artifacts
            if filename.endswith(".data")
        }
        if posegraphs != posegraph_data:
            raise MapCacheError("serialized posegraph artifacts are incomplete")
        posegraph_bases = posegraphs & posegraph_data
        if has_posegraph != bool(posegraph_bases):
            raise MapCacheError(
                "map metadata has_posegraph does not match its artifacts"
            )
        terminal_pose = metadata.get("terminal_pose")
        if terminal_pose is not None:
            if not isinstance(terminal_pose, dict) or not all(
                axis in terminal_pose for axis in ("x", "y", "yaw")
            ):
                raise MapCacheError("map metadata terminal_pose is invalid")
            for axis in ("x", "y", "yaw"):
                _finite_number(
                    terminal_pose[axis], f"map metadata terminal_pose.{axis}"
                )
    except (
        FileNotFoundError,
        OSError,
        ValueError,
        TypeError,
        json.JSONDecodeError,
        RecursionError,
        UnicodeDecodeError,
    ) as exc:
        if isinstance(exc, MapCacheError):
            raise
        raise MapCacheError("map metadata is invalid") from exc


def _validate_installed_cache(
    destination: Path,
    map_id: str,
    version: int,
    checksum: str,
    *,
    max_bundle_bytes: int,
    max_uncompressed_bytes: int,
    max_member_bytes: int,
    max_members: int,
    max_compression_ratio: float,
    max_image_pixels: int,
) -> None:
    archive_path = destination / CACHE_BUNDLE_NAME
    metadata_path = destination / "metadata.json"
    if (
        archive_path.is_symlink()
        or not archive_path.is_file()
        or archive_path.stat().st_size > max_bundle_bytes
        or _file_sha256(archive_path) != checksum
    ):
        raise MapCacheError("cached bundle checksum is invalid")
    try:
        with tarfile.open(archive_path, "r:*") as archive:
            members: dict[str, tarfile.TarInfo] = {}
            uncompressed_bytes = 0
            for member in archive:
                if not member.isfile():
                    raise MapCacheError("cached bundle contains an unsafe member type")
                canonical = _canonical_member_name(member.name)
                if canonical in members:
                    raise MapCacheError(
                        f"cached bundle contains duplicate member: {canonical}"
                    )
                if member.size < 0 or member.size > max_member_bytes:
                    raise MapCacheError(
                        f"cached bundle member is too large: {canonical}"
                    )
                members[canonical] = member
                if len(members) > max_members:
                    raise MapCacheError("cached bundle contains too many members")
                uncompressed_bytes += member.size
                if uncompressed_bytes > max_uncompressed_bytes:
                    raise MapCacheError(
                        "cached bundle exceeds the uncompressed size limit"
                    )
            if (
                uncompressed_bytes / max(1, archive_path.stat().st_size)
                > max_compression_ratio
            ):
                raise MapCacheError("cached bundle exceeds the compression ratio limit")
            metadata_member = members.get("metadata.json")
            if metadata_member is None or metadata_member.size > MAX_METADATA_BYTES:
                raise MapCacheError("cached bundle metadata is missing or too large")
            source = archive.extractfile(metadata_member)
            if source is None:
                raise MapCacheError("cached bundle metadata cannot be read")
            archived_metadata_hash = hashlib.sha256(source.read()).hexdigest()
    except (tarfile.TarError, EOFError, OSError) as exc:
        raise MapCacheError("cached bundle archive is invalid") from exc
    if (
        metadata_path.is_symlink()
        or not metadata_path.is_file()
        or _file_sha256(metadata_path) != archived_metadata_hash
    ):
        raise MapCacheError("cached metadata does not match the verified bundle")
    _validate_unpacked(
        destination,
        map_id,
        version,
        max_image_pixels=max_image_pixels,
    )


class RobotMapCacheManager:
    def __init__(
        self,
        root: str | Path,
        center_api_url: str,
        token_provider: Callable[[], Awaitable[str]],
        *,
        verify: bool | str = True,
        max_bundle_bytes: int = DEFAULT_MAX_BUNDLE_BYTES,
        max_uncompressed_bytes: int = DEFAULT_MAX_UNCOMPRESSED_BYTES,
        max_member_bytes: int = DEFAULT_MAX_MEMBER_BYTES,
        max_members: int = DEFAULT_MAX_MEMBERS,
        max_compression_ratio: float = DEFAULT_MAX_COMPRESSION_RATIO,
        max_image_pixels: int = DEFAULT_MAX_IMAGE_PIXELS,
    ) -> None:
        self.root = Path(root)
        self.center_api_url = center_api_url.rstrip("/")
        self.token_provider = token_provider
        self.verify = verify
        self.max_bundle_bytes = max_bundle_bytes
        self.max_uncompressed_bytes = max_uncompressed_bytes
        self.max_member_bytes = max_member_bytes
        self.max_members = max_members
        self.max_compression_ratio = max_compression_ratio
        self.max_image_pixels = max_image_pixels
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
        # fsyncing only the file does not make the rename durable across a
        # sudden power loss. The directory entry is part of the tombstone
        # barrier and must reach stable storage before restore can reopen.
        directory_fd = os.open(self.root, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)

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

    def is_tombstoned(self, map_id: str) -> bool:
        return map_id in self._read_registry().get("tombstones", {})

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
            expected_destination = (
                self.root.resolve() / map_id / f"v{version}"
            )
            if destination != expected_destination:
                raise ValueError("active map path does not match its identity")
        except (KeyError, TypeError, ValueError) as exc:
            raise MapCacheError("active map registry entry is invalid") from exc
        registry = self._read_registry()
        if map_id in registry.get("tombstones", {}):
            raise MapCacheError("deleted map cannot be restored as active")
        expected_checksum = str(active.get("checksum", "")).lower()
        marker = destination / ".sha256"
        try:
            marker_valid = bool(
                SHA256_PATTERN.fullmatch(expected_checksum)
                and not marker.is_symlink()
                and marker.is_file()
                and marker.read_text().strip().lower() == expected_checksum
            )
        except OSError:
            marker_valid = False
        if not marker_valid:
            raise MapCacheError("active map cache checksum marker is invalid")
        _validate_installed_cache(
            destination,
            map_id,
            version,
            expected_checksum,
            max_bundle_bytes=self.max_bundle_bytes,
            max_uncompressed_bytes=self.max_uncompressed_bytes,
            max_member_bytes=self.max_member_bytes,
            max_members=self.max_members,
            max_compression_ratio=self.max_compression_ratio,
            max_image_pixels=self.max_image_pixels,
        )
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

    def record_tombstone(
        self, map_id: str, *, deleted_at: float | None = None
    ) -> None:
        """Persist deletion authority before touching runtime or artifacts."""
        if not MAP_ID_PATTERN.fullmatch(map_id) or map_id.lower() in RESERVED_MAP_IDS:
            raise MapCacheError("invalid map identity")
        deletion_timestamp = time.time() if deleted_at is None else deleted_at
        with self._registry_lock:
            registry = self._read_registry()
            existing = registry.get("tombstones", {}).get(map_id)
            map_references = any(
                item.get("map_id") == map_id
                for item in registry.get("maps", {}).values()
            )
            pose_references = any(
                item.get("map_id") == map_id
                for item in registry.get("last_known_pose", {}).values()
            )
            active = registry.get("active")
            active_reference = bool(
                isinstance(active, dict) and active.get("map_id") == map_id
            )
            try:
                existing_deleted_at = float(existing.get("deleted_at", 0.0))
            except (AttributeError, TypeError, ValueError):
                existing_deleted_at = 0.0
            if (
                isinstance(existing, dict)
                and existing.get("status") == "DELETED"
                and existing_deleted_at >= deletion_timestamp
                and not map_references
                and not pose_references
                and not active_reference
            ):
                # The authoritative full snapshot is polled periodically. Do
                # not turn an already durable ACK back into PENDING and fsync
                # the same registry forever.
                return
            registry["maps"] = {
                key: item for key, item in registry.get("maps", {}).items()
                if item.get("map_id") != map_id
            }
            registry["last_known_pose"] = {
                key: item for key, item in registry.get("last_known_pose", {}).items()
                if item.get("map_id") != map_id
            }
            if isinstance(active, dict) and active.get("map_id") == map_id:
                registry["active"] = None
            registry.setdefault("tombstones", {})[map_id] = {
                "map_id": map_id,
                "deleted_at": deletion_timestamp,
                "status": "DELETION_PENDING",
            }
            self._write_registry(registry)

    def purge_tombstoned_artifacts(self, map_id: str) -> None:
        """Remove local files only after a durable tombstone is present."""
        if not MAP_ID_PATTERN.fullmatch(map_id) or map_id.lower() in RESERVED_MAP_IDS:
            raise MapCacheError("invalid map identity")
        if not self.is_tombstoned(map_id):
            raise MapCacheError("map deletion requires a persisted tombstone")
        for parent in (self.root, self.root / "created"):
            target = (parent / map_id).resolve()
            if target.parent != parent.resolve():
                raise MapCacheError("unsafe map deletion target")
            if target.exists():
                shutil.rmtree(target)

    def delete_local(self, map_id: str, *, deleted_at: float | None = None) -> None:
        self.record_tombstone(map_id, deleted_at=deleted_at)
        self.purge_tombstoned_artifacts(map_id)

    def acknowledge_tombstone(self, map_id: str) -> None:
        if not MAP_ID_PATTERN.fullmatch(map_id) or map_id.lower() in RESERVED_MAP_IDS:
            raise MapCacheError("invalid map identity")
        with self._registry_lock:
            registry = self._read_registry()
            item = registry.get("tombstones", {}).get(map_id)
            if isinstance(item, dict):
                if item.get("status") == "DELETED":
                    return
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
        checksum = checksum.lower()
        if not SHA256_PATTERN.fullmatch(checksum):
            raise MapCacheError("invalid map bundle checksum")
        if self.is_tombstoned(map_id):
            raise MapCacheError("deleted map cannot be downloaded")
        destination = self.root / map_id / f"v{version}"
        marker = destination / ".sha256"
        if self._cached_version_is_valid(
            destination, marker, map_id, version, checksum
        ):
            self.mark_synced(map_id, version, checksum)
            return destination
        async with self._lock:
            self.root.mkdir(parents=True, exist_ok=True)
            if self._cached_version_is_valid(
                destination, marker, map_id, version, checksum
            ):
                self.mark_synced(map_id, version, checksum)
                return destination
            staging = Path(mkdtemp(prefix=f".{map_id}-v{version}-", dir=self.root))
            archive_path = staging / "bundle.tar.gz"
            try:
                token = await self.token_provider()
                digest = hashlib.sha256()
                url = download_url if download_url.startswith("http") else f"{self.center_api_url}{download_url}"
                async with (
                    httpx.AsyncClient(verify=self.verify, timeout=60) as client,
                    client.stream(
                        "GET",
                        url,
                        headers={"Authorization": f"Bearer {token}"},
                    ) as response,
                ):
                        response.raise_for_status()
                        with archive_path.open("wb") as output:
                            downloaded_bytes = 0
                            async for chunk in response.aiter_bytes(1024 * 1024):
                                downloaded_bytes += len(chunk)
                                if downloaded_bytes > self.max_bundle_bytes:
                                    raise MapCacheError(
                                        "downloaded map exceeds the compressed size limit"
                                    )
                                digest.update(chunk)
                                output.write(chunk)
                if digest.hexdigest() != checksum:
                    raise MapCacheError("downloaded map checksum mismatch")
                unpacked = staging / "unpacked"
                unpacked.mkdir()
                await asyncio.to_thread(
                    _safe_extract,
                    archive_path,
                    unpacked,
                    max_uncompressed_bytes=self.max_uncompressed_bytes,
                    max_member_bytes=self.max_member_bytes,
                    max_members=self.max_members,
                    max_compression_ratio=self.max_compression_ratio,
                )
                await asyncio.to_thread(
                    _validate_unpacked,
                    unpacked,
                    map_id,
                    version,
                    max_image_pixels=self.max_image_pixels,
                )
                os.replace(archive_path, unpacked / CACHE_BUNDLE_NAME)
                marker_staging = unpacked / ".sha256"
                with marker_staging.open("x") as marker_output:
                    marker_output.write(checksum)
                    marker_output.flush()
                    os.fsync(marker_output.fileno())
                # A tombstone may arrive while the network download or bundle
                # validation is awaiting. Recheck immediately before the
                # atomic install so deleted artifacts cannot reappear on disk.
                if self.is_tombstoned(map_id):
                    raise MapCacheError("deleted map cannot be installed")
                destination.parent.mkdir(parents=True, exist_ok=True)
                previous = destination.with_name(f".{destination.name}.previous")
                if previous.exists():
                    shutil.rmtree(previous)
                if destination.exists():
                    os.replace(destination, previous)
                os.replace(unpacked, destination)
                directory_fd = os.open(
                    destination.parent, os.O_RDONLY | os.O_DIRECTORY
                )
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
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

    def _cached_version_is_valid(
        self,
        destination: Path,
        marker: Path,
        map_id: str,
        version: int,
        checksum: str,
    ) -> bool:
        try:
            if (
                marker.is_symlink()
                or not marker.is_file()
                or marker.read_text().strip().lower() != checksum
            ):
                return False
            _validate_installed_cache(
                destination,
                map_id,
                version,
                checksum,
                max_bundle_bytes=self.max_bundle_bytes,
                max_uncompressed_bytes=self.max_uncompressed_bytes,
                max_member_bytes=self.max_member_bytes,
                max_members=self.max_members,
                max_compression_ratio=self.max_compression_ratio,
                max_image_pixels=self.max_image_pixels,
            )
            return True
        except (MapCacheError, OSError):
            return False
