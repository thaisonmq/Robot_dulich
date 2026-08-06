from __future__ import annotations

import asyncio
import hashlib
import os
import shutil
import tarfile
from pathlib import Path, PurePosixPath
from tempfile import mkdtemp
from typing import Awaitable, Callable

import httpx


class MapCacheError(RuntimeError):
    pass


def _safe_extract(archive_path: Path, destination: Path) -> None:
    with tarfile.open(archive_path, "r:*") as archive:
        for member in archive.getmembers():
            path = PurePosixPath(member.name)
            if path.is_absolute() or ".." in path.parts or member.issym() or member.islnk():
                raise MapCacheError("map bundle contains an unsafe path")
        archive.extractall(destination, filter="data")


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
        self.root.mkdir(parents=True, exist_ok=True)
        self._lock = asyncio.Lock()

    async def ensure(
        self,
        *,
        map_id: str,
        version: int,
        checksum: str,
        download_url: str,
    ) -> Path:
        destination = self.root / map_id / f"v{version}"
        marker = destination / ".sha256"
        if marker.is_file() and marker.read_text().strip() == checksum:
            return destination
        async with self._lock:
            if marker.is_file() and marker.read_text().strip() == checksum:
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
                return destination
            except httpx.HTTPStatusError as exc:
                raise MapCacheError(
                    f"map download failed with HTTP {exc.response.status_code}"
                ) from exc
            except httpx.HTTPError as exc:
                raise MapCacheError("map download failed") from exc
            finally:
                shutil.rmtree(staging, ignore_errors=True)
