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
REQUIRED_EXCLUSIVE_ACK = "I_ACCEPT_EXCLUSIVE_CMD_VEL_OWNERSHIP"


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


def configured_value(name: str) -> str:
    """Read deployment mode from the service environment or project .env."""
    direct = os.environ.get(name)
    if direct is not None:
        return direct.strip()
    try:
        lines = (PROJECT_DIR / ".env").read_text().splitlines()
    except OSError:
        return ""
    for raw_line in reversed(lines):
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key.removeprefix("export ").strip() == name:
            return value.strip().strip("\"'")
    return ""


def runtime_process_counts() -> dict[str, int]:
    """Count running hardware/control authorities once per container."""
    result = run("docker", "ps", "-q")
    counts = {
        "vendor_base": 0,
        "serial_agent": 0,
        "managed_bridge": 0,
        "motion_safety": 0,
    }
    for container_id in result.stdout.split():
        top = run(
            "docker", "top", container_id, "-eo", "pid,args", check=False, timeout=5
        ).stdout
        if "yahboomcar_bringup_launch.py" in top:
            counts["vendor_base"] += 1
        if (
            "micro_ros_agent" in top
            and " serial " in f" {top} "
            and "/dev/ttyUSB0" in top
        ):
            counts["serial_agent"] += 1
        if "control_bridge.py" in top:
            counts["managed_bridge"] += 1
        if "/rovera_motion_safety/safety_node" in top:
            counts["motion_safety"] += 1
    return counts


def validate_base_runtime() -> str:
    """Fail closed while accepting both supported Pi control architectures.

    The managed-motion cutover deliberately disables the legacy Yahboom
    desktop runtime. In that mode the serial Agent, control bridge and final
    motion-safety node collectively form the single base authority. Requiring
    ``yahboomcar_bringup_launch.py`` there incorrectly prevents every
    SLAM/Nav2 switch even though the guarded replacement is healthy.
    """
    counts = runtime_process_counts()
    control_mode = configured_value("ROVERA_CONTROL_MODE").lower()
    command_mode = configured_value("ROVERA_CMD_VEL_MODE").lower()
    exclusive_ack = configured_value("ROVERA_EXCLUSIVE_CMD_VEL_ACK")
    if control_mode == "managed-motion":
        configuration_errors = []
        if command_mode != "exclusive":
            configuration_errors.append(
                f"ROVERA_CMD_VEL_MODE={command_mode or '<missing>'}"
            )
        if exclusive_ack != REQUIRED_EXCLUSIVE_ACK:
            configuration_errors.append("exclusive ownership ACK is missing")
        expected = {
            "serial_agent": 1,
            "managed_bridge": 1,
            "motion_safety": 1,
        }
        runtime_errors = [
            f"{name}={counts[name]} (expected {expected_count})"
            for name, expected_count in expected.items()
            if counts[name] != expected_count
        ]
        if counts["vendor_base"] > 1:
            runtime_errors.append(
                f"vendor_base={counts['vendor_base']} (expected at most 1)"
            )
        errors = [*configuration_errors, *runtime_errors]
        if errors:
            raise RuntimeError(
                "managed-motion base authority is unsafe: " + "; ".join(errors)
            )
        return "managed-motion"

    if counts["vendor_base"] != 1:
        raise RuntimeError(
            "expected exactly one Yahboom base runtime in legacy mode, "
            f"found {counts['vendor_base']}; refusing ROS switch"
        )
    return "legacy-yahboom"


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
    if mode == "IDLE":
        return {
            "mode": "IDLE",
            "state": "NO_ACTIVE_MAP",
            "nav2": "STOPPED",
            "localized": False,
            "localization_state": "IDLE",
        }
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
    if mode not in {"IDLE", "MAPPING", "NAVIGATION"}:
        raise ValueError(f"unsupported mode: {mode}")
    validate_base_runtime()

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
    if mode == "IDLE":
        # A deleted/deactivated active map must leave no localization or map
        # authority behind. The vendor base and motion-safety runtime remain
        # alive, so manual control and E-stop are still available.
        compose(navigation_files, "navigation", "stop", "navigation-stack")
        compose(
            mapping_files, "legacy-coexistence", "stop",
            "rviz-bridge", "mapping-stack",
        )
        remove_stale_socket()
    elif mode == "NAVIGATION":
        compose(
            mapping_files, "legacy-coexistence", "stop",
            "rviz-bridge", "mapping-stack",
        )
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
            "up", "-d", "--no-deps", "--force-recreate",
            "mapping-stack", "rviz-bridge",
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
