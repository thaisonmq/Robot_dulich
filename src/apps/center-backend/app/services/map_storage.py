from __future__ import annotations

import hashlib
import json
import math
import os
import re
import shutil
import tarfile
import warnings
from pathlib import Path, PurePosixPath
from tempfile import NamedTemporaryFile
from typing import Any, BinaryIO

import yaml
from PIL import Image

REQUIRED_BUNDLE_FILES = frozenset({"map.yaml", "metadata.json"})
REQUIRED_METADATA_FIELDS = frozenset({
    "map_id", "name", "version", "robot_id", "created_at", "updated_at",
    "resolution", "width", "height", "origin", "frame_id", "checksum",
    "has_posegraph", "slam_mode",
})
MAP_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{1,63}$")
SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
DEFAULT_MAX_UNCOMPRESSED_BYTES = 1024 * 1024 * 1024
DEFAULT_MAX_MEMBER_BYTES = 768 * 1024 * 1024
DEFAULT_MAX_MEMBERS = 64
DEFAULT_MAX_COMPRESSION_RATIO = 2000.0
DEFAULT_MAX_IMAGE_PIXELS = 100_000_000
MAX_METADATA_BYTES = 1024 * 1024
MAX_MAP_YAML_BYTES = 64 * 1024


class InvalidMapBundle(ValueError):
    pass


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
        raise InvalidMapBundle("bundle contains an unsafe path or nested path")
    return normalized


def _reject_json_constant(value: str) -> None:
    raise InvalidMapBundle(f"metadata contains non-finite JSON: {value}")


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InvalidMapBundle(f"{field} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise InvalidMapBundle(f"{field} must be a finite number") from exc
    if not math.isfinite(number):
        raise InvalidMapBundle(f"{field} must be a finite number")
    return number


def _positive_integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise InvalidMapBundle(f"{field} must be a positive integer")
    return value


def _read_member(
    archive: tarfile.TarFile,
    member: tarfile.TarInfo,
    maximum_bytes: int,
) -> bytes:
    if member.size > maximum_bytes:
        raise InvalidMapBundle(f"bundle member is too large: {member.name}")
    source = archive.extractfile(member)
    if source is None:
        raise InvalidMapBundle(f"bundle member cannot be read: {member.name}")
    content = source.read(maximum_bytes + 1)
    if len(content) > maximum_bytes or len(content) != member.size:
        raise InvalidMapBundle(f"bundle member size is invalid: {member.name}")
    return content


def _validate_semantics(
    archive: tarfile.TarFile,
    members: dict[str, tarfile.TarInfo],
    metadata: dict[str, Any],
    *,
    expected_map_id: str | None,
    expected_version: int | None,
    max_image_pixels: int,
) -> None:
    if metadata.get("frame_id") != "map":
        raise InvalidMapBundle("metadata frame_id must be map")
    if expected_map_id is not None and metadata.get("map_id") != expected_map_id:
        raise InvalidMapBundle("metadata map/version does not match upload target")
    if expected_version is not None and metadata.get("version") != expected_version:
        raise InvalidMapBundle("metadata map/version does not match upload target")
    if not isinstance(metadata.get("map_id"), str) or not MAP_ID_PATTERN.fullmatch(
        metadata["map_id"]
    ):
        raise InvalidMapBundle("metadata map_id is invalid")
    if not isinstance(metadata.get("name"), str) or not 1 <= len(metadata["name"]) <= 120:
        raise InvalidMapBundle("metadata name is invalid")
    if (
        not isinstance(metadata.get("robot_id"), str)
        or not 3 <= len(metadata["robot_id"]) <= 64
    ):
        raise InvalidMapBundle("metadata robot_id is invalid")
    if (
        not isinstance(metadata.get("slam_mode"), str)
        or not 1 <= len(metadata["slam_mode"]) <= 64
    ):
        raise InvalidMapBundle("metadata slam_mode is invalid")
    _positive_integer(metadata.get("version"), "metadata version")
    width = _positive_integer(metadata.get("width"), "metadata width")
    height = _positive_integer(metadata.get("height"), "metadata height")
    if width * height > max_image_pixels:
        raise InvalidMapBundle("occupancy image exceeds the pixel limit")
    resolution = _finite_number(metadata.get("resolution"), "metadata resolution")
    if resolution <= 0:
        raise InvalidMapBundle("metadata resolution must be positive")
    for field in ("created_at", "updated_at"):
        _finite_number(metadata.get(field), f"metadata {field}")
    origin = metadata.get("origin")
    if not isinstance(origin, dict) or not all(axis in origin for axis in ("x", "y", "yaw")):
        raise InvalidMapBundle("metadata origin is invalid")
    metadata_origin = tuple(
        _finite_number(origin[axis], f"metadata origin.{axis}")
        for axis in ("x", "y", "yaw")
    )

    yaml_content = _read_member(archive, members["map.yaml"], MAX_MAP_YAML_BYTES)
    try:
        map_yaml = yaml.safe_load(yaml_content)
    except yaml.YAMLError as exc:
        raise InvalidMapBundle("map.yaml is invalid") from exc
    if not isinstance(map_yaml, dict):
        raise InvalidMapBundle("map.yaml must contain an object")
    image_name = map_yaml.get("image")
    if not isinstance(image_name, str):
        raise InvalidMapBundle("map.yaml image must be a bundle basename")
    try:
        image_name = _canonical_member_name(image_name)
    except InvalidMapBundle as exc:
        raise InvalidMapBundle("map.yaml image must be a bundle basename") from exc
    if image_name not in {"map.pgm", "map.png"} or image_name not in members:
        raise InvalidMapBundle("map.yaml image is not the declared occupancy image")
    yaml_resolution = _finite_number(map_yaml.get("resolution"), "map.yaml resolution")
    yaml_origin = map_yaml.get("origin")
    if not isinstance(yaml_origin, list) or len(yaml_origin) != 3:
        raise InvalidMapBundle("map.yaml origin must contain x, y and yaw")
    parsed_yaml_origin = tuple(
        _finite_number(value, f"map.yaml origin[{index}]")
        for index, value in enumerate(yaml_origin)
    )
    if not math.isclose(yaml_resolution, resolution, rel_tol=0.0, abs_tol=1e-9):
        raise InvalidMapBundle("map.yaml resolution does not match metadata")
    if any(
        not math.isclose(left, right, rel_tol=0.0, abs_tol=1e-9)
        for left, right in zip(parsed_yaml_origin, metadata_origin, strict=True)
    ):
        raise InvalidMapBundle("map.yaml origin does not match metadata")
    mode = str(map_yaml.get("mode", "trinary")).lower()
    if mode not in {"trinary", "scale", "raw"}:
        raise InvalidMapBundle("map.yaml mode is unsupported")
    negate = map_yaml.get("negate", 0)
    if isinstance(negate, bool) or negate not in {0, 1}:
        raise InvalidMapBundle("map.yaml negate must be 0 or 1")
    occupied = _finite_number(
        map_yaml.get("occupied_thresh", 0.65), "map.yaml occupied_thresh"
    )
    free = _finite_number(
        map_yaml.get("free_thresh", 0.196), "map.yaml free_thresh"
    )
    if not 0.0 <= free < occupied <= 1.0:
        raise InvalidMapBundle("map.yaml occupancy thresholds are invalid")

    image_source = archive.extractfile(members[image_name])
    if image_source is None:
        raise InvalidMapBundle("occupancy image cannot be read")
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("error", Image.DecompressionBombWarning)
            with Image.open(image_source) as image:
                image.load()
                image_width, image_height = image.size
    except (Image.DecompressionBombError, Image.DecompressionBombWarning, OSError) as exc:
        raise InvalidMapBundle("occupancy image is invalid or unsafe") from exc
    if image_width * image_height > max_image_pixels:
        raise InvalidMapBundle("occupancy image exceeds the pixel limit")
    if image_width != width or image_height != height:
        raise InvalidMapBundle("occupancy image dimensions do not match metadata")

    declared_files = metadata.get("files")
    if not isinstance(declared_files, dict) or not declared_files:
        raise InvalidMapBundle("metadata files must contain SHA-256 entries")
    actual_artifacts = set(members) - {"metadata.json"}
    if set(declared_files) != actual_artifacts:
        raise InvalidMapBundle("metadata files do not match the bundle artifacts")
    for filename, expected in declared_files.items():
        if not isinstance(filename, str) or _canonical_member_name(filename) != filename:
            raise InvalidMapBundle("metadata contains an unsafe artifact path")
        expected_hash = str(expected).lower()
        if not SHA256_PATTERN.fullmatch(expected_hash):
            raise InvalidMapBundle(f"invalid checksum entry for {filename}")
        source = archive.extractfile(members[filename])
        if source is None:
            raise InvalidMapBundle(f"cannot read {filename}")
        digest = hashlib.sha256()
        for chunk in iter(
            lambda source=source: source.read(1024 * 1024), b""
        ):
            digest.update(chunk)
        if digest.hexdigest() != expected_hash:
            raise InvalidMapBundle(f"artifact checksum mismatch: {filename}")
    metadata_checksum = str(metadata.get("checksum", "")).lower()
    if (
        not SHA256_PATTERN.fullmatch(metadata_checksum)
        or str(declared_files[image_name]).lower() != metadata_checksum
    ):
        raise InvalidMapBundle("metadata checksum must identify the YAML occupancy image")
    if metadata.get("checksum_scope", image_name) != image_name:
        raise InvalidMapBundle("metadata checksum_scope does not match map.yaml image")

    has_posegraph = metadata.get("has_posegraph")
    if not isinstance(has_posegraph, bool):
        raise InvalidMapBundle("metadata has_posegraph must be boolean")
    posegraphs = {
        filename.removesuffix(".posegraph")
        for filename in actual_artifacts
        if filename.endswith(".posegraph")
    }
    posegraph_data = {
        filename.removesuffix(".data")
        for filename in actual_artifacts
        if filename.endswith(".data")
    }
    if posegraphs != posegraph_data:
        raise InvalidMapBundle("serialized posegraph artifacts are incomplete")
    posegraph_bases = posegraphs & posegraph_data
    if has_posegraph != bool(posegraph_bases):
        raise InvalidMapBundle("metadata has_posegraph does not match its artifacts")
    terminal_pose = metadata.get("terminal_pose")
    if terminal_pose is not None:
        if not isinstance(terminal_pose, dict) or not all(
            axis in terminal_pose for axis in ("x", "y", "yaw")
        ):
            raise InvalidMapBundle("metadata terminal_pose is invalid")
        for axis in ("x", "y", "yaw"):
            _finite_number(terminal_pose[axis], f"metadata terminal_pose.{axis}")


def inspect_map_bundle(
    path: Path,
    *,
    expected_map_id: str | None = None,
    expected_version: int | None = None,
    max_uncompressed_bytes: int = DEFAULT_MAX_UNCOMPRESSED_BYTES,
    max_member_bytes: int = DEFAULT_MAX_MEMBER_BYTES,
    max_members: int = DEFAULT_MAX_MEMBERS,
    max_compression_ratio: float = DEFAULT_MAX_COMPRESSION_RATIO,
    max_image_pixels: int = DEFAULT_MAX_IMAGE_PIXELS,
) -> dict[str, Any]:
    try:
        with tarfile.open(path, "r:*") as archive:
            members: dict[str, tarfile.TarInfo] = {}
            uncompressed_bytes = 0
            for member in archive:
                if not member.isfile():
                    raise InvalidMapBundle("bundle contains an unsafe member type")
                canonical = _canonical_member_name(member.name)
                if canonical in members:
                    raise InvalidMapBundle(f"bundle contains duplicate member: {canonical}")
                if member.size < 0 or member.size > max_member_bytes:
                    raise InvalidMapBundle(f"bundle member is too large: {canonical}")
                members[canonical] = member
                if len(members) > max_members:
                    raise InvalidMapBundle("bundle contains too many members")
                uncompressed_bytes += member.size
                if uncompressed_bytes > max_uncompressed_bytes:
                    raise InvalidMapBundle("bundle exceeds the uncompressed size limit")
            compressed_bytes = max(1, path.stat().st_size)
            if uncompressed_bytes / compressed_bytes > max_compression_ratio:
                raise InvalidMapBundle("bundle exceeds the compression ratio limit")
            missing = REQUIRED_BUNDLE_FILES - set(members)
            if missing:
                raise InvalidMapBundle(f"bundle is missing: {', '.join(sorted(missing))}")
            metadata = json.loads(
                _read_member(archive, members["metadata.json"], MAX_METADATA_BYTES),
                parse_constant=_reject_json_constant,
            )
            if not isinstance(metadata, dict):
                raise InvalidMapBundle("metadata.json must contain an object")
            missing_metadata = REQUIRED_METADATA_FIELDS - metadata.keys()
            if missing_metadata:
                raise InvalidMapBundle(
                    f"metadata.json is missing: {', '.join(sorted(missing_metadata))}"
                )
            _validate_semantics(
                archive,
                members,
                metadata,
                expected_map_id=expected_map_id,
                expected_version=expected_version,
                max_image_pixels=max_image_pixels,
            )
            return metadata
    except (
        tarfile.TarError,
        json.JSONDecodeError,
        OSError,
        RecursionError,
        UnicodeDecodeError,
    ) as exc:
        raise InvalidMapBundle("invalid map bundle") from exc


class MapBundleStore:
    def __init__(
        self,
        root: str | Path,
        max_bytes: int,
        *,
        max_uncompressed_bytes: int = DEFAULT_MAX_UNCOMPRESSED_BYTES,
        max_member_bytes: int = DEFAULT_MAX_MEMBER_BYTES,
        max_members: int = DEFAULT_MAX_MEMBERS,
        max_compression_ratio: float = DEFAULT_MAX_COMPRESSION_RATIO,
        max_image_pixels: int = DEFAULT_MAX_IMAGE_PIXELS,
    ) -> None:
        self.root = Path(root).resolve()
        self.max_bytes = max_bytes
        self.max_uncompressed_bytes = max_uncompressed_bytes
        self.max_member_bytes = max_member_bytes
        self.max_members = max_members
        self.max_compression_ratio = max_compression_ratio
        self.max_image_pixels = max_image_pixels
        self.root.mkdir(parents=True, exist_ok=True)

    def save(
        self,
        stream: BinaryIO,
        *,
        map_id: str,
        version: int,
        expected_checksum: str,
    ) -> tuple[Path, str, dict[str, Any]]:
        if not MAP_ID_PATTERN.fullmatch(map_id) or version < 1:
            raise InvalidMapBundle("invalid map storage identity")
        map_directory = (self.root / map_id).resolve()
        if map_directory.parent != self.root:
            raise InvalidMapBundle("invalid map storage identity")
        target_directory = map_directory / f"v{version}"
        target_directory.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256()
        size = 0
        with NamedTemporaryFile(
            mode="wb", prefix="upload-", suffix=".tmp", dir=target_directory, delete=False
        ) as temporary:
            temporary_path = Path(temporary.name)
            while True:
                chunk = stream.read(1024 * 1024)
                if not chunk:
                    break
                size += len(chunk)
                if size > self.max_bytes:
                    temporary.close()
                    temporary_path.unlink(missing_ok=True)
                    raise InvalidMapBundle("map bundle exceeds configured size limit")
                digest.update(chunk)
                temporary.write(chunk)
            temporary.flush()
            os.fsync(temporary.fileno())
        checksum = digest.hexdigest()
        if checksum != expected_checksum.lower():
            temporary_path.unlink(missing_ok=True)
            raise InvalidMapBundle("map bundle checksum mismatch")
        try:
            metadata = inspect_map_bundle(
                temporary_path,
                expected_map_id=map_id,
                expected_version=version,
                max_uncompressed_bytes=self.max_uncompressed_bytes,
                max_member_bytes=self.max_member_bytes,
                max_members=self.max_members,
                max_compression_ratio=self.max_compression_ratio,
                max_image_pixels=self.max_image_pixels,
            )
        except InvalidMapBundle:
            temporary_path.unlink(missing_ok=True)
            raise
        destination = target_directory / "map-bundle.tar.gz"
        os.replace(temporary_path, destination)
        directory = os.open(target_directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
        return destination, checksum, metadata

    def delete_map(self, map_id: str) -> None:
        """Remove one validated map directory without accepting broad paths."""
        target = (self.root / map_id).resolve()
        if target.parent != self.root or target == self.root:
            raise ValueError("invalid map storage target")
        if target.exists():
            shutil.rmtree(target)
