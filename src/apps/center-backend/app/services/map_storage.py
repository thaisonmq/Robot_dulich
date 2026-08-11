from __future__ import annotations

import hashlib
import json
import os
import shutil
import tarfile
from pathlib import Path, PurePosixPath
from tempfile import NamedTemporaryFile
from typing import BinaryIO, Any


REQUIRED_BUNDLE_FILES = frozenset({"map.yaml", "metadata.json"})
REQUIRED_METADATA_FIELDS = frozenset({
    "map_id", "name", "version", "robot_id", "created_at", "updated_at",
    "resolution", "width", "height", "origin", "frame_id", "checksum",
    "has_posegraph", "slam_mode",
})


class InvalidMapBundle(ValueError):
    pass


def _safe_member(name: str) -> bool:
    path = PurePosixPath(name)
    return bool(name) and not path.is_absolute() and ".." not in path.parts


def inspect_map_bundle(path: Path) -> dict[str, Any]:
    try:
        with tarfile.open(path, "r:*") as archive:
            if any(
                member.issym() or member.islnk()
                or not (member.isfile() or member.isdir())
                for member in archive.getmembers()
            ):
                raise InvalidMapBundle("bundle contains an unsafe member type")
            files = {member.name.lstrip("./") for member in archive.getmembers() if member.isfile()}
            if not all(_safe_member(member.name) for member in archive.getmembers()):
                raise InvalidMapBundle("bundle contains an unsafe path")
            missing = REQUIRED_BUNDLE_FILES - files
            if missing:
                raise InvalidMapBundle(f"bundle is missing: {', '.join(sorted(missing))}")
            metadata_member = next(
                member for member in archive.getmembers()
                if member.isfile() and member.name.lstrip("./") == "metadata.json"
            )
            extracted = archive.extractfile(metadata_member)
            if extracted is None:
                raise InvalidMapBundle("metadata.json cannot be read")
            metadata = json.loads(extracted.read(1_048_577))
            if not isinstance(metadata, dict):
                raise InvalidMapBundle("metadata.json must contain an object")
            missing_metadata = REQUIRED_METADATA_FIELDS - metadata.keys()
            if missing_metadata:
                raise InvalidMapBundle(
                    f"metadata.json is missing: {', '.join(sorted(missing_metadata))}"
                )
            if metadata.get("frame_id") != "map":
                raise InvalidMapBundle("metadata frame_id must be map")
            if not ({"map.pgm", "map.png"} & files):
                raise InvalidMapBundle("bundle must contain map.pgm or map.png")
            declared_files = metadata.get("files")
            if not isinstance(declared_files, dict):
                raise InvalidMapBundle("metadata files must contain SHA-256 entries")
            undeclared = (files - {"metadata.json"}) - set(declared_files)
            if undeclared:
                raise InvalidMapBundle(
                    f"metadata files is missing: {', '.join(sorted(undeclared))}"
                )
            members = {
                member.name.lstrip("./"): member
                for member in archive.getmembers() if member.isfile()
            }
            for filename, expected in declared_files.items():
                if filename not in members or not isinstance(expected, str) or len(expected) != 64:
                    raise InvalidMapBundle(f"invalid checksum entry for {filename}")
                source = archive.extractfile(members[filename])
                if source is None:
                    raise InvalidMapBundle(f"cannot read {filename}")
                digest = hashlib.sha256()
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    digest.update(chunk)
                if digest.hexdigest() != expected.lower():
                    raise InvalidMapBundle(f"artifact checksum mismatch: {filename}")
            primary_images = {name for name in ("map.pgm", "map.png") if name in declared_files}
            metadata_checksum = str(metadata.get("checksum", "")).lower()
            if (
                len(metadata_checksum) != 64
                or not primary_images
                or not any(
                    str(declared_files[name]).lower() == metadata_checksum
                    for name in primary_images
                )
            ):
                raise InvalidMapBundle("metadata checksum must identify the occupancy image")
            return metadata
    except (tarfile.TarError, json.JSONDecodeError, OSError) as exc:
        raise InvalidMapBundle("invalid map bundle") from exc


class MapBundleStore:
    def __init__(self, root: str | Path, max_bytes: int) -> None:
        self.root = Path(root).resolve()
        self.max_bytes = max_bytes
        self.root.mkdir(parents=True, exist_ok=True)

    def save(
        self,
        stream: BinaryIO,
        *,
        map_id: str,
        version: int,
        expected_checksum: str,
    ) -> tuple[Path, str, dict[str, Any]]:
        target_directory = self.root / map_id / f"v{version}"
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
            metadata = inspect_map_bundle(temporary_path)
        except InvalidMapBundle:
            temporary_path.unlink(missing_ok=True)
            raise
        destination = target_directory / "map-bundle.tar.gz"
        os.replace(temporary_path, destination)
        return destination, checksum, metadata

    def delete_map(self, map_id: str) -> None:
        """Remove one validated map directory without accepting broad paths."""
        target = (self.root / map_id).resolve()
        if target.parent != self.root or target == self.root:
            raise ValueError("invalid map storage target")
        if target.exists():
            shutil.rmtree(target)
