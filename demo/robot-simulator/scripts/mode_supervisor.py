#!/usr/bin/env python3
"""Safely hand one shared ROS graph between Rovera SLAM and Nav2.

This process runs on the host. Containers never receive the Docker socket.
Only one adapter/map authority is allowed to run at a time, while the vendor
Yahboom base runtime and serial Agent are deliberately left untouched.
"""

from __future__ import annotations

import fcntl
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


PROJECT_DIR = Path(os.environ["ROVERA_PROJECT_DIR"]).resolve()
STATE_DIR = Path(os.environ["ROVERA_STATE_DIR"]).resolve()
REQUEST_PATH = STATE_DIR / "navigation" / "mode-request.json"
STATUS_PATH = STATE_DIR / "navigation" / "mode-status.json"
SOCKET_PATH = STATE_DIR / "navigation" / "navigation.sock"
LOCK_PATH = STATE_DIR / "navigation" / "mode-supervisor.lock"
POLL_SECONDS = 0.25


def run(*args: str, timeout: float = 90.0, check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=PROJECT_DIR,
        env={**os.environ, "ROVERA_USE_VENDOR_BASE_RUNTIME": "1"},
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=check,
    )


def compose(files: list[str], profile: str, *args: str, timeout: float = 90.0) -> None:
    command = ["docker", "compose", "--env-file", ".env"]
    for filename in files:
        command.extend(("-f", filename))
    command.extend(("--profile", profile, *args))
    result = run(*command, timeout=timeout)
    if result.stdout.strip():
        print(result.stdout.rstrip(), flush=True)


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    temporary.write_text(json.dumps(value, separators=(",", ":")))
    os.replace(temporary, path)


def vendor_base_runtime_count() -> int:
    result = run("docker", "ps", "-q")
    count = 0
    for container_id in result.stdout.split():
        top = run(
            "docker", "top", container_id, "-eo", "pid,args", check=False, timeout=5
        ).stdout
        if "yahboomcar_bringup_launch.py" in top:
            count += 1
    return count


def adapter_status(timeout: float = 2.0) -> dict[str, Any]:
    client = socket.socket(socket.AF_UNIX)
    client.settimeout(timeout)
    try:
        client.connect(str(SOCKET_PATH))
        client.sendall(b'{"command":"system.status","payload":{}}\n')
        chunks = bytearray()
        while not chunks.endswith(b"\n"):
            chunk = client.recv(1_048_576)
            if not chunk:
                break
            chunks.extend(chunk)
        return json.loads(chunks)
    finally:
        client.close()


def wait_for_mode(mode: str, timeout: float = 75.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    last_state: dict[str, Any] = {}
    while time.monotonic() < deadline:
        try:
            response = adapter_status()
            last_state = dict(response.get("state") or {})
            nav_ready = mode != "NAVIGATION" or last_state.get("nav2") == "READY"
            if last_state.get("mode") == mode and nav_ready:
                return last_state
        except (OSError, ValueError, json.JSONDecodeError):
            pass
        time.sleep(0.5)
    raise RuntimeError(f"adapter did not become {mode}; last_state={last_state}")


def remove_stale_socket() -> None:
    for _ in range(20):
        if not SOCKET_PATH.exists():
            return
        try:
            SOCKET_PATH.unlink()
            return
        except OSError:
            time.sleep(0.1)
    raise RuntimeError(f"cannot remove stale socket {SOCKET_PATH}")


def switch_mode(request: dict[str, Any]) -> dict[str, Any]:
    mode = str(request.get("mode", "")).upper()
    if mode not in {"MAPPING", "NAVIGATION"}:
        raise ValueError(f"unsupported mode: {mode}")
    vendor_count = vendor_base_runtime_count()
    if vendor_count != 1:
        raise RuntimeError(
            f"expected exactly one Yahboom base runtime, found {vendor_count}; "
            "refusing ROS switch"
        )

    current: dict[str, Any] = {}
    try:
        current = dict(adapter_status().get("state") or {})
    except (OSError, ValueError, json.JSONDecodeError):
        pass
    if current.get("mode") == mode and (
        mode != "NAVIGATION" or current.get("nav2") == "READY"
    ):
        return current

    navigation_files = ["compose.yaml", "compose.navigation.yml"]
    mapping_files = ["compose.yaml", "compose.coexistence.yml"]
    if mode == "NAVIGATION":
        compose(mapping_files, "legacy-coexistence", "stop", "mapping-stack")
        remove_stale_socket()
        compose(
            navigation_files,
            "navigation",
            "up", "-d", "--no-deps", "--force-recreate", "navigation-stack",
        )
    else:
        compose(navigation_files, "navigation", "stop", "navigation-stack")
        remove_stale_socket()
        compose(
            mapping_files,
            "legacy-coexistence",
            "up", "-d", "--no-deps", "--force-recreate", "mapping-stack",
        )
    return wait_for_mode(mode)


def main() -> int:
    REQUEST_PATH.parent.mkdir(parents=True, exist_ok=True)
    with LOCK_PATH.open("w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("another mode supervisor is already running", file=sys.stderr)
            return 73
        last_request_id = ""
        while True:
            try:
                request = json.loads(REQUEST_PATH.read_text())
                request_id = str(request.get("request_id", ""))
                if request_id and request_id != last_request_id:
                    last_request_id = request_id
                    atomic_json(STATUS_PATH, {
                        "request_id": request_id,
                        "mode": request.get("mode"),
                        "status": "SWITCHING",
                        "updated_at_unix": time.time(),
                    })
                    try:
                        state = switch_mode(request)
                        atomic_json(STATUS_PATH, {
                            "request_id": request_id,
                            "mode": request.get("mode"),
                            "status": "READY",
                            "state": state,
                            "updated_at_unix": time.time(),
                        })
                        print(f"safe ROS mode switch complete: {request.get('mode')}", flush=True)
                    except Exception as exc:
                        atomic_json(STATUS_PATH, {
                            "request_id": request_id,
                            "mode": request.get("mode"),
                            "status": "FAULT",
                            "error": str(exc),
                            "updated_at_unix": time.time(),
                        })
                        print(f"safe ROS mode switch failed: {exc}", file=sys.stderr, flush=True)
            except FileNotFoundError:
                pass
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                print(f"invalid mode request: {exc}", file=sys.stderr, flush=True)
            time.sleep(POLL_SECONDS)


if __name__ == "__main__":
    raise SystemExit(main())
