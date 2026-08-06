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


class InvalidMapBundle(ValueError):
    pass


def _safe_member(name: str) -> bool:
    path = PurePosixPath(name)
    return bool(name) and not path.is_absolute() and ".." not in path.parts


def inspect_map_bundle(path: Path) -> dict[str, Any]:
    try:
        with tarfile.open(path, "r:*") as archive:
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
            if not ({"map.pgm", "map.png"} & files):
                raise InvalidMapBundle("bundle must contain map.pgm or map.png")
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
        metadata = inspect_map_bundle(temporary_path)
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
