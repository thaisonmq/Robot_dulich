import asyncio
import hashlib
import hmac
import json
import logging
import math
import os
import platform
import random
import ssl
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.parse import quote, urlencode, urlsplit, urlunsplit

import httpx
import websockets

from simulator.camera_ptz import CameraPtzController
from simulator.config import SimulatorConfig
from simulator.media import MediaPublisher, redact_media_source
from simulator.media_devices import (
    discover_media_sources,
    discover_video_sources,
    prepare_audio_output,
    prepare_audio_source,
    probe_audio_output,
    probe_audio_source,
    select_v4l2_mode,
)
from simulator.messages import Message, make_message
from simulator.motion import MotionSimulator
from simulator.motion_driver import (
    MotionDisabledError,
    MotionDriver,
    build_motion_driver,
)
from simulator.navigation import NavigationSimulator
from simulator.navigation_backends import (
    NavigationBackendError,
    build_navigation_backend,
)
from simulator.map_cache import MapCacheError, RobotMapCacheManager


def mapping_autosave_posegraph(
    map_cache_dir: str,
    map_id: str,
    version: int,
) -> Path:
    """Resolve one complete, local-only SLAM autosave without trusting Center paths."""
    RobotMapCacheManager._validate_identity(map_id, version)
    map_root = Path(map_cache_dir).parent
    posegraph = map_root / ".autosave" / f"{map_id}-latest"
    artifacts = (posegraph.with_suffix(".posegraph"), posegraph.with_suffix(".data"))
    if any(not artifact.is_file() or artifact.stat().st_size <= 0 for artifact in artifacts):
        raise MapCacheError("robot has no complete mapping autosave for this map")
    return posegraph


logger = logging.getLogger("simulator.gateway")


def localization_pose_safe_to_persist(state: dict[str, Any]) -> bool:
    """Persist only a pose that survived sustained independent verification."""
    diagnostics = state.get("localization_diagnostics")
    if not isinstance(diagnostics, dict):
        return False
    stability = diagnostics.get("pose_stability")
    sensor_time = diagnostics.get("sensor_time")
    if not isinstance(stability, dict) or not isinstance(sensor_time, dict):
        return False
    score = float(diagnostics.get("scan_map_score", 0.0))
    threshold = float(diagnostics.get("scan_map_threshold", 1.0))
    return (
        bool(state.get("localized"))
        and state.get("localization_state") == "READY"
        and int(state.get("localization_verification_version", 0)) >= 2
        and bool(stability.get("passed"))
        and sensor_time.get("clock_state") == "SYNCED"
        and score >= threshold
        and float(diagnostics.get("ready_evidence_hold_ms") or 0.0) >= 30_000.0
    )


def navigation_visualization_delta(
    visualization: dict[str, Any],
    previous: dict[str, Any] | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Build a route-aware visualization delta with explicit clear semantics."""
    identity = (
        str(visualization.get("map_id", "")),
        int(visualization.get("map_version", 0) or 0),
    )
    route_id = str(visualization.get("route_id", ""))
    path = list(visualization.get("global_path") or [])
    obstacles = list(visualization.get("dynamic_obstacles") or [])
    changed: dict[str, Any] = {
        "revision": int(visualization.get("revision", -1)),
        "map_id": identity[0],
        "map_version": identity[1],
        "route_id": route_id,
    }
    route_changed = bool(
        previous is None
        or identity != previous["identity"]
        or route_id != previous["route_id"]
    )
    if route_changed or path != previous["global_path"]:
        # [] is intentionally retained: omission means unchanged, not clear.
        changed["global_path"] = path
    if (
        previous is None
        or identity != previous["identity"]
        or obstacles != previous["dynamic_obstacles"]
    ):
        changed["dynamic_obstacles"] = obstacles
    return changed, {
        "identity": identity,
        "route_id": route_id,
        "global_path": path,
        "dynamic_obstacles": obstacles,
    }


def bounded_navigation_trajectory(
    trajectory: Any, *, maximum_points: int = 40
) -> list[dict[str, Any]]:
    """Uniformly sample a trail before placing it in frequent WS messages."""
    values = [dict(point) for point in (trajectory or []) if isinstance(point, dict)]
    limit = max(2, int(maximum_points))
    if len(values) <= limit:
        return values
    last_index = len(values) - 1
    indexes = {
        round(index * last_index / (limit - 1))
        for index in range(limit)
    }
    return [values[index] for index in sorted(indexes)]


def bounded_navigation_command_details(
    details: dict[str, Any] | None,
) -> dict[str, Any]:
    """Bound the nested runtime state before sending a command acknowledgement."""
    result = dict(details or {})
    state = result.get("state")
    if not isinstance(state, dict):
        return result
    bounded_state = dict(state)
    bounded_state["trajectory"] = bounded_navigation_trajectory(
        bounded_state.get("trajectory")
    )
    result["state"] = bounded_state
    return result


class RobotConnectionClient:
    def __init__(self, config: SimulatorConfig) -> None:
        self.config = config
        self.motion = MotionSimulator(config)
        self.motion_driver: MotionDriver = build_motion_driver(config, self.motion)
        self.navigation = NavigationSimulator(self.motion)
        self.navigation_backend = build_navigation_backend(
            config.navigation_backend,
            self.navigation,
            self.motion,
            config.navigation_socket_path,
            motion_backend=config.motion_backend,
        )
        self.map_cache = RobotMapCacheManager(
            config.map_cache_dir,
            config.center_api_url,
            self._robot_bearer_token,
            verify=self.http_verify,
        )
        self.media = MediaPublisher(config, self._media_token)
        self.camera_ptz = CameraPtzController()
        self._ptz_task: asyncio.Task[bool] | None = None
        self._background_tasks: set[asyncio.Task[Any]] = set()
        self.sequence = 0
        self.processed_ids: set[str] = set()
        self.processed_order: deque[str] = deque()
        self.running = True
        self.socket: Any = None
        self.media_restart_requested = asyncio.Event()
        self.media_lease_changed = asyncio.Event()
        self.media_leases: dict[str, float] = {}
        self._camera_inventory_cache: list[dict[str, Any]] = []
        self.robot_access_token = ""
        self._used_enrollment_token_hash = ""
        self._configured_robot_id = config.robot_id
        self._configured_robot_credential = config.robot_credential
        self._configured_enrollment_token = config.robot_enrollment_token
        self._configured_configuration = self._configuration_snapshot()
        self._state_loaded = False
        self._last_control_latency_log_monotonic = 0.0
        self._load_device_state()

    @property
    def url(self) -> str:
        separator = "&" if "?" in self.config.center_robot_ws_url else "?"
        return self.config.center_robot_ws_url + separator + urlencode(
            {"robot_id": self.config.robot_id}
        )

    @property
    def state_path(self) -> Path:
        if self.config.robot_state_file:
            return Path(self.config.robot_state_file).expanduser()
        return Path.home() / ".config" / "rovera" / "device.json"

    def _load_device_state(self) -> None:
        try:
            state = json.loads(self.state_path.read_text())
        except FileNotFoundError:
            return
        except OSError as exc:
            logger.warning("cannot read device state path=%s error=%s", self.state_path, exc)
            return
        except ValueError as exc:
            logger.warning("ignoring invalid device state path=%s error=%s", self.state_path, exc)
            return
        if not isinstance(state, dict):
            logger.warning("ignoring non-object device state path=%s", self.state_path)
            return
        if not self._state_belongs_to_this_device(state):
            return
        robot_id = str(state.get("robot_id", ""))
        credential = str(state.get("credential", ""))
        used_token_hash = str(state.get("enrollment_token_hash", ""))
        if self.config.robot_enrollment_token:
            supplied_token_hash = hashlib.sha256(
                self.config.robot_enrollment_token.encode()
            ).hexdigest()
            if not used_token_hash or not hmac.compare_digest(
                supplied_token_hash, used_token_hash
            ):
                return
            self.config.robot_enrollment_token = ""
        if robot_id and credential:
            self.config.robot_id = robot_id
            self.config.robot_credential = credential
            self._used_enrollment_token_hash = used_token_hash
            self._state_loaded = True
            configuration = state.get("configuration")
            if isinstance(configuration, dict):
                try:
                    self._apply_configuration(configuration)
                except (TypeError, ValueError) as exc:
                    logger.warning("ignoring invalid saved configuration error=%s", exc)
            if not state.get("device_fingerprint"):
                try:
                    self._save_device_state()
                    logger.info("migrated legacy device state with local fingerprint")
                except OSError as exc:
                    logger.warning("cannot migrate legacy device state error=%s", exc)

    def _state_belongs_to_this_device(self, state: dict[str, Any]) -> bool:
        stored_fingerprint = str(state.get("device_fingerprint", "")).strip()
        current_fingerprint = self._device_fingerprint()
        if stored_fingerprint:
            if hmac.compare_digest(stored_fingerprint, current_fingerprint):
                return True
            logger.warning(
                "ignoring device state copied from another machine path=%s",
                self.state_path,
            )
            return False

        # Compatibility for state files created before fingerprints existed.
        # A copied legacy file still contains the previous robot address.
        configuration = state.get("configuration")
        stored_address = (
            str(configuration.get("device_ip", "")).strip()
            if isinstance(configuration, dict)
            else ""
        )
        configured_address = self.config.robot_management_address.strip()
        if (
            stored_address
            and configured_address
            and stored_address.casefold() != configured_address.casefold()
        ):
            logger.warning(
                "ignoring legacy device state for another address "
                "saved_address=%s configured_address=%s",
                stored_address,
                configured_address,
            )
            return False
        return True

    def _configuration_snapshot(self) -> dict[str, Any]:
        video_source = self.config.simulator_media_source
        if self.config.simulator_media_source_type == "test":
            video_source = "generated://test-pattern"
        elif (
            self.config.simulator_media_source_type == "camera"
            and not video_source
        ):
            video_source = self.config.simulator_camera_device
        return {
            "device_ip": self.config.device_ip,
            "video_source_type": self.config.simulator_media_source_type,
            "video_source": video_source,
            "video_profile": self.config.video_profile,
            "rtsp_transport": self.config.rtsp_transport,
            "camera_label": self.config.camera_label,
            "audio_source_type": self.config.simulator_audio_source_type,
            "audio_source": self.config.simulator_audio_source,
            "microphone_label": self.config.microphone_label,
            "audio_output_type": self.config.simulator_audio_output_type,
            "audio_output": self.config.simulator_audio_output,
            "speaker_label": self.config.speaker_label,
        }

    def _save_device_state(self) -> None:
        path = self.state_path
        path.parent.mkdir(parents=True, exist_ok=True)
        self._move_invalid_state_target(path)
        temporary = path.with_suffix(".tmp")
        self._move_invalid_state_target(temporary)
        temporary.write_text(
            json.dumps(
                {
                    "robot_id": self.config.robot_id,
                    "credential": self.config.robot_credential,
                    "device_fingerprint": self._device_fingerprint(),
                    "enrollment_token_hash": self._used_enrollment_token_hash,
                    "configuration": self._configuration_snapshot(),
                }
            )
        )
        temporary.chmod(0o600)
        os.replace(temporary, path)
        self._state_loaded = True

    @staticmethod
    def _move_invalid_state_target(path: Path) -> None:
        if not path.is_symlink() and (not path.exists() or path.is_file()):
            return
        timestamp = int(time.time())
        backup = path.with_name(f"{path.name}.invalid-{timestamp}")
        counter = 1
        while backup.exists() or backup.is_symlink():
            backup = path.with_name(f"{path.name}.invalid-{timestamp}-{counter}")
            counter += 1
        path.rename(backup)
        logger.warning(
            "moved non-file device state target path=%s backup=%s",
            path,
            backup,
        )

    def _reset_identity_if_state_removed(self) -> bool:
        if not self._state_loaded or self.state_path.is_file():
            return False
        logger.warning(
            "device state was removed; clearing in-memory identity and reclaiming"
        )
        self.config.robot_id = self._configured_robot_id
        self.config.robot_credential = self._configured_robot_credential
        self.config.robot_enrollment_token = self._configured_enrollment_token
        self._used_enrollment_token_hash = ""
        self.robot_access_token = ""
        self._state_loaded = False
        try:
            self._apply_configuration(self._configured_configuration)
        except (TypeError, ValueError) as exc:
            logger.warning("cannot restore environment configuration error=%s", exc)
        return True

    def _device_fingerprint(self) -> str:
        machine_id = ""
        try:
            machine_id = Path("/etc/machine-id").read_text().strip()
        except OSError:
            pass
        return f"{platform.node()}:{machine_id or 'unknown-device'}"

    @property
    def http_verify(self) -> bool | str:
        return self.config.center_tls_ca_file or self.config.center_tls_verify

    def websocket_ssl_context(self) -> ssl.SSLContext | None:
        if not self.url.startswith("wss://"):
            return None
        if not self.config.center_tls_verify:
            context = ssl.create_default_context()
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
            return context
        return ssl.create_default_context(
            cafile=self.config.center_tls_ca_file or None
        )

    async def _authenticate(self) -> None:
        self._reset_identity_if_state_removed()
        if self.config.robot_enrollment_token:
            await self._enroll()
        elif not self.config.robot_credential:
            if self.config.robot_username and self.config.robot_password:
                await self._claim()
            else:
                await self._enroll()
        async with httpx.AsyncClient(
            base_url=self.config.center_api_url.rstrip("/"),
            verify=self.http_verify,
            timeout=8,
        ) as client:
            response = await client.post(
                "/api/robot-auth/token",
                json={
                    "robot_id": self.config.robot_id,
                    "credential": self.config.robot_credential,
                },
            )
            if (
                response.status_code == 401
                and self.config.robot_username
                and self.config.robot_password
            ):
                self.config.robot_credential = ""
                await self._claim()
                response = await client.post(
                    "/api/robot-auth/token",
                    json={
                        "robot_id": self.config.robot_id,
                        "credential": self.config.robot_credential,
                    },
                )
            response.raise_for_status()
            self.robot_access_token = str(response.json()["access_token"])
            if not self.state_path.is_file():
                self._save_device_state()

    async def _robot_bearer_token(self) -> str:
        # Map transfer is infrequent and may happen long after the persistent
        # gateway WebSocket was opened. Always refresh here so an expired JWT
        # cannot turn a continuation/save into a 90-second command timeout.
        await self._authenticate()
        return self.robot_access_token

    async def _claim(self) -> None:
        management_address = (
            self.config.robot_management_address.strip()
            or self.config.device_ip.strip()
        )
        if not management_address:
            raise RuntimeError("Robot chưa khai báo địa chỉ quản lý")
        async with httpx.AsyncClient(
            base_url=self.config.center_api_url.rstrip("/"),
            verify=self.http_verify,
            timeout=15,
        ) as client:
            response = await client.post(
                "/api/robot-auth/claim",
                json={
                    "management_address": management_address,
                    "username": self.config.robot_username,
                    "password": self.config.robot_password,
                    "device_fingerprint": self._device_fingerprint(),
                },
            )
            response.raise_for_status()
            claimed = response.json()
            self.config.robot_id = str(claimed["robot_id"])
            self.config.robot_credential = str(claimed["credential"])
            self._save_device_state()
            logger.info("robot credential claim completed robot_id=%s", self.config.robot_id)

    async def _enroll(self) -> None:
        if not self.config.robot_enrollment_token:
            raise RuntimeError(
                "Robot chưa được ghép nối; cần ROBOT_ENROLLMENT_TOKEN"
            )
        enrollment_token = self.config.robot_enrollment_token
        async with httpx.AsyncClient(
            base_url=self.config.center_api_url.rstrip("/"),
            verify=self.http_verify,
            timeout=8,
        ) as client:
            response = await client.post(
                "/api/robot-auth/enroll",
                json={
                    "enrollment_token": enrollment_token,
                    "device_fingerprint": self._device_fingerprint(),
                },
            )
            response.raise_for_status()
            enrollment = response.json()
            self.config.robot_id = str(enrollment["robot_id"])
            self.config.robot_credential = str(enrollment["credential"])
            self._used_enrollment_token_hash = hashlib.sha256(
                enrollment_token.encode()
            ).hexdigest()
            self.config.robot_enrollment_token = ""
            self._save_device_state()
            logger.info("robot enrollment completed robot_id=%s", self.config.robot_id)

    async def _media_token(self, purpose: str = "main") -> str:
        if purpose not in {"main", "video"}:
            raise ValueError(f"unsupported media token purpose: {purpose}")
        if not self.robot_access_token:
            await self._authenticate()
        async with httpx.AsyncClient(
            base_url=self.config.center_api_url.rstrip("/"),
            verify=self.http_verify,
            timeout=8,
        ) as client:
            response = await client.post(
                "/api/robot-auth/media-token",
                params={"purpose": purpose},
                headers={"Authorization": f"Bearer {self.robot_access_token}"},
            )
            if response.status_code == 401:
                await self._authenticate()
                response = await client.post(
                    "/api/robot-auth/media-token",
                    params={"purpose": purpose},
                    headers={"Authorization": f"Bearer {self.robot_access_token}"},
                )
            response.raise_for_status()
            media = response.json()
            self.config.livekit_url = str(media["url"])
            return str(media["token"])

    async def run(self) -> None:
        delay = 1.0
        while self.running:
            try:
                await self._authenticate()
                async with websockets.connect(
                    self.url,
                    additional_headers={
                        "Authorization": f"Bearer {self.robot_access_token}"
                    },
                    ssl=self.websocket_ssl_context(),
                    open_timeout=8,
                    ping_interval=10,
                    ping_timeout=8,
                    max_size=65_536,
                ) as socket:
                    self.socket = socket
                    delay = 1.0
                    logger.info("gateway connected robot_id=%s", self.config.robot_id)
                    warm_task = asyncio.create_task(self._warm_media_source())
                    try:
                        await self._connected(socket)
                    finally:
                        if not warm_task.done():
                            warm_task.cancel()
                        await asyncio.gather(warm_task, return_exceptions=True)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning("gateway reconnect robot_id=%s error=%s", self.config.robot_id, exc)
            finally:
                self._stop_motion("gateway_disconnected")
                await self.camera_ptz.stop(*self._ptz_source())
                self.socket = None
                self.media_leases.clear()
                self.media_lease_changed.set()
                await self.media.disconnect()
            await asyncio.sleep(delay + random.uniform(0, min(1, delay / 4)))
            delay = min(15.0, delay * 2)

    async def _connected(self, socket: Any) -> None:
        local_address = getattr(socket, "local_address", None)
        if (
            local_address
            and self.config.device_ip in {"", "127.0.0.1", "localhost"}
        ):
            self.config.device_ip = str(local_address[0])
        tasks = [
            asyncio.create_task(self._receive_loop(socket)),
            asyncio.create_task(self._simulation_loop()),
            asyncio.create_task(self._telemetry_loop(socket)),
            asyncio.create_task(self._heartbeat_loop(socket)),
            asyncio.create_task(self._media_loop()),
            asyncio.create_task(self._mapping_upload_retry_loop()),
            asyncio.create_task(self._map_registry_sync_loop()),
            asyncio.create_task(self._navigation_runtime_loop(socket)),
        ]
        done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
        for task in pending:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        for task in tuple(self._background_tasks):
            task.cancel()
        await asyncio.gather(*self._background_tasks, return_exceptions=True)
        self._background_tasks.clear()
        for task in done:
            if task.exception():
                raise task.exception()

    async def _media_loop(self) -> None:
        if not self.config.media_enabled:
            return
        delay = 1.0
        while True:
            self._prune_media_leases()
            if not self.media_leases:
                if self.media.connected:
                    logger.info("media lease ended; camera stopped")
                    await self.media.disconnect()
                self.media_lease_changed.clear()
                try:
                    await asyncio.wait_for(
                        self.media_lease_changed.wait(), timeout=1.0
                    )
                except asyncio.TimeoutError:
                    pass
                continue
            if self.media_restart_requested.is_set():
                self.media_restart_requested.clear()
                await self.media.disconnect()
            if self.media.connected:
                delay = 1.0
                self.media_lease_changed.clear()
                try:
                    await asyncio.wait_for(
                        self.media_lease_changed.wait(), timeout=1.0
                    )
                except asyncio.TimeoutError:
                    pass
                continue
            try:
                logger.info(
                    "media lease active; starting camera leases=%d",
                    len(self.media_leases),
                )
                await self.media.connect()
                delay = 1.0
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                logger.warning(
                    "media unavailable; retrying in %.1fs while control remains active: %s",
                    delay,
                    exc,
                )
                await self.media.disconnect()
                self.media_lease_changed.clear()
                try:
                    await asyncio.wait_for(
                        self.media_lease_changed.wait(), timeout=delay
                    )
                except asyncio.TimeoutError:
                    pass
                delay = min(15.0, delay * 2)

    async def _warm_media_source(self) -> None:
        try:
            await asyncio.to_thread(self.media.warm_video_source)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            logger.warning("video source warmup skipped error=%s", exc)
        else:
            logger.info("video source warmup completed")

    def _prune_media_leases(self) -> None:
        now = time.monotonic()
        for lease_id, expires_at in list(self.media_leases.items()):
            if expires_at <= now:
                self.media_leases.pop(lease_id, None)

    def _start_media_lease(self, payload: dict[str, Any]) -> str:
        lease_id = str(payload.get("lease_id", "")).strip()
        if not lease_id or len(lease_id) > 160:
            raise ValueError("Media lease không hợp lệ")
        try:
            ttl_seconds = float(payload.get("ttl_seconds", 30))
        except (TypeError, ValueError) as exc:
            raise ValueError("Thời hạn media lease không hợp lệ") from exc
        ttl_seconds = min(120.0, max(5.0, ttl_seconds))
        self.media_leases[lease_id] = time.monotonic() + ttl_seconds
        self.media_lease_changed.set()
        return lease_id

    def _stop_media_lease(self, payload: dict[str, Any]) -> str:
        lease_id = str(payload.get("lease_id", "")).strip()
        if not lease_id:
            raise ValueError("Media lease không hợp lệ")
        self.media_leases.pop(lease_id, None)
        self.media_lease_changed.set()
        return lease_id

    async def _receive_loop(self, socket: Any) -> None:
        async for raw in socket:
            message = Message.model_validate_json(raw)
            if message.message_type == "gateway.welcome":
                continue
            message_id = str(message.message_id)
            if message_id in self.processed_ids:
                await self._ack(socket, message, "accepted")
                continue
            self._remember(message_id)
            if message.expired():
                await self._ack(socket, message, "expired")
                continue
            if message.message_type == "control.velocity":
                obstacle_avoidance_enabled = message.payload.get(
                    "obstacle_avoidance_enabled", True
                )
                if not isinstance(obstacle_avoidance_enabled, bool):
                    self._stop_motion("invalid_obstacle_avoidance_mode")
                    await self._ack(socket, message, "rejected")
                    continue
                try:
                    linear_x = float(message.payload.get("linear_x", 0))
                    angular_z = float(message.payload.get("angular_z", 0))
                except (TypeError, ValueError):
                    self._stop_motion("invalid_velocity")
                    await self._ack(socket, message, "rejected")
                    continue
                if not math.isfinite(linear_x) or not math.isfinite(angular_z):
                    self._stop_motion("invalid_velocity")
                    await self._ack(socket, message, "rejected")
                    continue
                navigation_state = self.navigation_backend.state()
                if str(navigation_state.get("mode", "")).upper() == "MAPPING":
                    linear_x = max(
                        -self.config.mapping_max_reverse_speed,
                        min(self.config.mapping_max_forward_speed, linear_x),
                    )
                    angular_z = max(
                        -self.config.mapping_max_angular_speed,
                        min(self.config.mapping_max_angular_speed, angular_z),
                    )
                if self.config.navigation_backend == "simulator":
                    await self.navigation_backend.manual_takeover()
                else:
                    # Safety arbitration switches to the manual source at once;
                    # Nav2 cancellation is independent and must not delay input.
                    self._spawn_background(self.navigation_backend.manual_takeover())
                dispatch_started = time.monotonic()
                try:
                    self.motion_driver.set_velocity(
                        linear_x,
                        angular_z,
                        obstacle_avoidance_enabled=obstacle_avoidance_enabled,
                    )
                except MotionDisabledError as exc:
                    await self._ack(
                        socket,
                        message,
                        "rejected",
                        {
                            "error_code": "MOTION_DISABLED",
                            "error_message": str(exc),
                        },
                    )
                    continue
                dispatch_finished = time.monotonic()
                if (
                    dispatch_finished - self._last_control_latency_log_monotonic
                    >= 1.0
                ):
                    command_age_ms = (
                        datetime.now(timezone.utc) - message.timestamp
                    ).total_seconds() * 1000
                    logger.info(
                        "control latency browser_to_edge_ms=%.1f "
                        "edge_dispatch_ms=%.3f sequence=%d",
                        command_age_ms,
                        (dispatch_finished - dispatch_started) * 1000,
                        message.sequence,
                    )
                    self._last_control_latency_log_monotonic = dispatch_finished
                await self._ack(socket, message, "accepted")
            elif message.message_type == "control.stop":
                self._stop_motion(str(message.payload.get("reason", "control_stop")))
                self._dispatch_ptz({"operation": "stop"})
                await self._ack(socket, message, "completed")
            elif message.message_type == "camera.ptz":
                accepted = self._dispatch_ptz(message.payload)
                await self._ack(socket, message, "accepted" if accepted else "rejected")
            elif message.message_type == "media.start":
                try:
                    self._start_media_lease(message.payload)
                    await self._ack(socket, message, "accepted")
                except ValueError:
                    await self._ack(socket, message, "rejected")
            elif message.message_type == "media.stop":
                try:
                    self._stop_media_lease(message.payload)
                    await self._ack(socket, message, "completed")
                except ValueError:
                    await self._ack(socket, message, "rejected")
            elif (
                message.message_type.startswith("mapping.")
                or message.message_type.startswith("map.")
                or message.message_type in {
                    "navigation.compute_path",
                    "navigation.start",
                    "navigation.pause",
                    "navigation.resume",
                    "navigation.cancel",
                    "navigation.manual_handoff",
                    "navigation.alternatives",
                    "navigation.select_route",
                    "navigation.route_selection_back",
                    "navigation.goal",
                    "navigation.speed_mode",
                }
            ):
                if (
                    self.config.motion_backend == "ros2"
                    and self.config.navigation_backend != "ros2"
                ):
                    # Reject this invalid hardware configuration immediately.
                    # The normal ROS/Nav2 path stays asynchronous below so map
                    # I/O can never delay a manual stop command.
                    await self._handle_navigation_command(socket, message)
                    continue
                # Map I/O and Nav2 actions can take seconds. Run them outside
                # the receive loop so stop/manual commands remain immediate.
                self._spawn_background(
                    self._handle_navigation_command(socket, message)
                )
            elif message.message_type == "configuration.get":
                await self._configuration_state(
                    socket, str(message.payload.get("request_id", ""))
                )
            elif message.message_type == "configuration.update":
                request_id = str(message.payload.get("request_id", ""))
                try:
                    self._apply_configuration(message.payload)
                    self._save_device_state()
                    await self._configuration_state(socket, request_id)
                    self.media_restart_requested.set()
                    asyncio.create_task(self._warm_media_source())
                except (OSError, TypeError, ValueError) as exc:
                    await self._configuration_state(socket, request_id, str(exc))
            elif message.message_type == "diagnostics.ping":
                await self._diagnostics_result(
                    socket,
                    str(message.payload.get("request_id", "")),
                    "connection",
                    {
                        "ok": True,
                        "gateway": "online",
                        "media": "online" if self.media.connected else "offline",
                        "device_ip": self.config.device_ip,
                    },
                )
            elif message.message_type == "media.sources.get":
                await self._media_sources(
                    socket,
                    str(message.payload.get("request_id", "")),
                    str(message.payload.get("media_kind", "all")),
                )
            elif message.message_type == "media.onvif.scan":
                # Discovery can wait for multicast replies and several camera
                # profile requests. Keep the gateway receive loop responsive
                # to control and heartbeat traffic while it runs.
                asyncio.create_task(
                    self._onvif_sources(
                        socket,
                        str(message.payload.get("request_id", "")),
                        str(message.payload.get("target_host", "")),
                        str(message.payload.get("username", "")),
                        str(message.payload.get("password", "")),
                    )
                )
            elif message.message_type == "media.cameras.get":
                # V4L2 probing may take several seconds.  Keeping it inline
                # blocks this websocket's receive loop and can delay an
                # unrelated navigation command beyond its ACK envelope.
                self._spawn_background(
                    self._camera_sources(
                        socket,
                        str(message.payload.get("request_id", "")),
                    )
                )
            elif message.message_type == "media.source.select":
                request_id = str(message.payload.get("request_id", ""))
                try:
                    self._select_live_camera(message.payload)
                    self.media_restart_requested.set()
                    self.media_lease_changed.set()
                    await self._media_source_state(socket, request_id)
                except (TypeError, ValueError) as exc:
                    await self._media_source_state(
                        socket, request_id, str(exc)
                    )
            elif message.message_type == "media.probe":
                request_id = str(message.payload.get("request_id", ""))
                result = await self._probe_media(message.payload)
                await self._diagnostics_result(
                    socket, request_id, str(message.payload.get("media_kind", "")), result
                )
            else:
                await self._ack(socket, message, "rejected")

    def _public_video_source(self) -> str:
        if self.config.simulator_media_source_type == "test":
            return "generated://test-pattern"
        if self.config.simulator_media_source_type == "camera":
            return (
                self.config.simulator_media_source
                or self.config.simulator_camera_device
            )
        if self.config.simulator_media_source_type == "file":
            return self.config.simulator_media_source
        source = self.config.simulator_media_source.strip()
        if not source:
            return "rtsp://camera.local/live"
        parsed = urlsplit(source)
        path = parsed.path
        if self.config.simulator_rtsp_path and path in ("", "/"):
            path = "/" + self.config.simulator_rtsp_path.lstrip("/")
        hostname = parsed.hostname or "camera.local"
        port = f":{parsed.port}" if parsed.port else ""
        return urlunsplit(
            (parsed.scheme or "rtsp", f"{hostname}{port}", path, parsed.query, parsed.fragment)
        )

    def _source_with_preserved_credentials(self, source: str) -> str:
        return self._source_with_credentials_from(
            source, self.config.simulator_media_source
        )

    @staticmethod
    def _source_with_credentials_from(source: str, current_source: str) -> str:
        requested = urlsplit(source)
        current = urlsplit(current_source)
        if (
            requested.username is not None
            or current.username is None
            or not requested.hostname
            or not current.hostname
            or requested.hostname.casefold() != current.hostname.casefold()
        ):
            return source
        credentials = quote(current.username, safe="")
        if current.password is not None:
            credentials += ":" + quote(current.password, safe="")
        hostname = requested.hostname or ""
        port = f":{requested.port}" if requested.port else ""
        return urlunsplit(
            (
                requested.scheme,
                f"{credentials}@{hostname}{port}",
                requested.path,
                requested.query,
                requested.fragment,
            )
        )

    def _public_audio_source(self) -> str:
        source = self.config.simulator_audio_source.strip()
        parsed = urlsplit(source)
        if not parsed.hostname or parsed.username is None:
            return source
        port = f":{parsed.port}" if parsed.port else ""
        return urlunsplit(
            (
                parsed.scheme,
                f"{parsed.hostname}{port}",
                parsed.path,
                parsed.query,
                parsed.fragment,
            )
        )

    def _apply_configuration(self, payload: dict[str, Any]) -> None:
        profile = str(payload.get("video_profile", ""))
        profiles = {
            # Leave Wi-Fi airtime for low-latency control while preserving
            # good 1080p25 quality on the robot's MJPEG camera.
            "full_hd": (1920, 1080, 6_000_000),
            # Superfast x264 preserves the selected camera mode at this rate
            # while leaving airtime for control on congested 2.4 GHz Wi-Fi.
            "balanced": (1280, 720, 2_000_000),
            "low_bandwidth": (854, 480, 1_200_000),
        }
        if profile not in profiles:
            raise ValueError("Profile video không hợp lệ")
        transport = str(payload.get("rtsp_transport", ""))
        if transport not in {"auto", "tcp", "udp"}:
            raise ValueError("Giao thức RTSP không hợp lệ")
        source_type = str(payload.get("video_source_type", ""))
        if source_type not in {"rtsp", "camera", "file", "test"}:
            raise ValueError("Loại nguồn video không hợp lệ")
        source = str(payload.get("video_source", "")).strip()
        if source_type == "rtsp" and not source.lower().startswith(
            ("rtsp://", "rtsps://")
        ):
            raise ValueError("Nguồn RTSP không hợp lệ")
        if source_type == "camera" and not source.startswith("/dev/"):
            raise ValueError("Thiết bị camera USB không hợp lệ")
        if not source:
            raise ValueError("Nguồn video không được để trống")
        audio_source_type = str(
            payload.get("audio_source_type", "silent")
        )
        if audio_source_type not in {"silent", "device", "file"}:
            raise ValueError("Loại nguồn microphone không hợp lệ")
        requested_audio_source = str(payload.get("audio_source", "")).strip()
        if audio_source_type != "silent" and not requested_audio_source:
            raise ValueError("Hãy chọn nguồn microphone")
        if audio_source_type == "device":
            unavailable_reason = prepare_audio_source(requested_audio_source)
            if unavailable_reason:
                raise ValueError(unavailable_reason)
        audio_output_type = str(
            payload.get("audio_output_type", "disabled")
        )
        if audio_output_type not in {"disabled", "device"}:
            raise ValueError("Loại đầu ra loa không hợp lệ")
        requested_audio_output = str(payload.get("audio_output", "")).strip()
        if audio_output_type == "device" and not requested_audio_output:
            raise ValueError("Hãy chọn loa trên robot")
        if audio_output_type == "device":
            unavailable_reason = prepare_audio_output(requested_audio_output)
            if unavailable_reason:
                raise ValueError(unavailable_reason)
        self.config.device_ip = str(payload.get("device_ip", "")).strip()
        self.config.camera_label = str(payload.get("camera_label", "")).strip()
        self.config.simulator_audio_source_type = audio_source_type
        self.config.simulator_audio_source = self._source_with_credentials_from(
            requested_audio_source, self.config.simulator_audio_source
        )
        self.config.microphone_label = str(
            payload.get("microphone_label", "Microphone chính")
        ).strip()
        self.config.simulator_audio_output_type = audio_output_type
        self.config.simulator_audio_output = requested_audio_output
        self.config.speaker_label = str(
            payload.get("speaker_label", "Loa chính")
        ).strip()
        self.config.video_profile = profile
        self.config.rtsp_transport = transport
        self.config.video_width, self.config.video_height, self.config.video_bitrate = (
            profiles[profile]
        )
        self.config.simulator_media_source = (
            self.camera_ptz.credentialed_source(
                self._source_with_preserved_credentials(source)
            )
            if source_type == "rtsp"
            else source
        )
        self.config.simulator_rtsp_path = ""
        self.config.simulator_media_source_type = source_type

    async def _configuration_state(
        self, socket: Any, request_id: str, error: str | None = None
    ) -> None:
        await socket.send(
            json.dumps(
                make_message(
                    "configuration.state",
                    self.config.robot_id,
                    self._next_sequence(),
                    {
                        "request_id": request_id,
                        "ok": error is None,
                        "error": error,
                        "robot_id": self.config.robot_id,
                        "device_ip": self.config.device_ip,
                        "video_source_type": self.config.simulator_media_source_type,
                        "video_source": self._public_video_source(),
                        "video_profile": self.config.video_profile,
                        "rtsp_transport": self.config.rtsp_transport,
                        "camera_label": self.config.camera_label,
                        "audio_source_type": self.config.simulator_audio_source_type,
                        "audio_source": self._public_audio_source(),
                        "microphone_label": self.config.microphone_label,
                        "audio_output_type": self.config.simulator_audio_output_type,
                        "audio_output": self.config.simulator_audio_output,
                        "speaker_label": self.config.speaker_label,
                        "software_version": "sim-1.0",
                        "connection_status": "online",
                    },
                )
            )
        )

    async def _media_sources(
        self, socket: Any, request_id: str, media_kind: str = "all"
    ) -> None:
        sources = await asyncio.to_thread(discover_media_sources, media_kind)
        await socket.send(
            json.dumps(
                make_message(
                    "media.sources",
                    self.config.robot_id,
                    self._next_sequence(),
                    {
                        "request_id": request_id,
                        "ok": True,
                        "media_kind": media_kind,
                        **sources,
                    },
                )
            )
        )

    async def _onvif_sources(
        self,
        socket: Any,
        request_id: str,
        target_host: str = "",
        username: str = "",
        password: str = "",
    ) -> None:
        devices = await self.camera_ptz.scan_onvif(
            self.config.device_ip,
            self.config.simulator_media_source,
            target_host,
            username,
            password,
        )
        await socket.send(
            json.dumps(
                make_message(
                    "media.onvif.devices",
                    self.config.robot_id,
                    self._next_sequence(),
                    {
                        "request_id": request_id,
                        "ok": True,
                        "devices": devices,
                    },
                )
            )
        )

    async def _camera_sources(self, socket: Any, request_id: str) -> None:
        selected_source = ""
        if self.config.simulator_media_source_type == "camera":
            selected_source = (
                self.config.simulator_media_source
                or self.config.simulator_camera_device
            )
        if selected_source and self.media_leases:
            # Never probe V4L2 while the active publisher owns the device.
            # Return its known source immediately; a 4-second inventory probe
            # previously looked like an unrelated robot command ACK failure.
            selected = next(
                (
                    dict(source)
                    for source in self._camera_inventory_cache
                    if str(source.get("value", "")) == selected_source
                ),
                {
                    "type": self.config.simulator_media_source_type,
                    "value": selected_source,
                    "label": self.config.camera_label,
                    "ptz": {},
                },
            )
            await socket.send(
                json.dumps(
                    make_message(
                        "media.cameras",
                        self.config.robot_id,
                        self._next_sequence(),
                        {
                            "request_id": request_id,
                            "ok": True,
                            "video_sources": [selected],
                            "selected_source": selected_source,
                        },
                    )
                )
            )
            return
        # The live-view request and camera-list request arrive together when a
        # dashboard opens. Do not race the publisher by probing its exclusive
        # V4L2 device in another FFmpeg process. Other candidates are still
        # opened and must return a real frame before they are shown.
        if selected_source and self.media_leases:
            sources, _rejected = await asyncio.to_thread(
                discover_video_sources,
                {selected_source},
            )
        else:
            sources, _rejected = await asyncio.to_thread(discover_video_sources)
        if self.config.simulator_media_source_type == "rtsp":
            selected_source = self._public_video_source()
            sources.insert(
                0,
                {
                    "type": "rtsp",
                    "value": selected_source,
                    "label": self.config.camera_label,
                },
            )
        for source in sources:
            capability_source = str(source.get("value", ""))
            if source.get("type") == "rtsp" and capability_source == selected_source:
                capability_source = self.config.simulator_media_source
            source["ptz"] = await self.camera_ptz.capabilities(
                str(source.get("type", "camera")), capability_source
            )
            if source.get("type") == "camera":
                modes = list(source.get("capture_modes") or [])
                source["recommended_mode"] = select_v4l2_mode(
                    modes,
                    self.config.simulator_camera_width or self.config.video_width,
                    self.config.simulator_camera_height or self.config.video_height,
                    self.config.simulator_camera_fps or self.config.video_fps,
                    self.config.simulator_camera_format,
                )
        self._camera_inventory_cache = [dict(source) for source in sources]
        await socket.send(
            json.dumps(
                make_message(
                    "media.cameras",
                    self.config.robot_id,
                    self._next_sequence(),
                    {
                        "request_id": request_id,
                        "ok": True,
                        "video_sources": sources,
                        "selected_source": selected_source,
                    },
                )
            )
        )

    def _select_live_camera(self, payload: dict[str, Any]) -> None:
        source_type = str(payload.get("source_type", "camera"))
        source = str(payload.get("source", "")).strip()
        if source_type != "camera" or not source.startswith("/dev/video"):
            raise ValueError("Nguồn camera trực tiếp không hợp lệ")
        available = {
            item["value"] for item in discover_video_sources()[0]
        }
        if source not in available:
            raise ValueError("Camera không trả về hình ảnh hoặc đã bị rút khỏi robot")
        self.config.simulator_media_source_type = "camera"
        self.config.simulator_media_source = source
        self.config.camera_label = (
            str(payload.get("label", "")).strip() or self.config.camera_label
        )

    async def _media_source_state(
        self, socket: Any, request_id: str, error: str | None = None
    ) -> None:
        await socket.send(
            json.dumps(
                make_message(
                    "media.source.state",
                    self.config.robot_id,
                    self._next_sequence(),
                    {
                        "request_id": request_id,
                        "ok": error is None,
                        "error": error,
                        "selected_source": (
                            self.config.simulator_media_source
                            if error is None
                            else ""
                        ),
                    },
                )
            )
        )

    async def _probe_media(self, payload: dict[str, Any]) -> dict[str, Any]:
        started = time.monotonic()
        kind = str(payload.get("media_kind", ""))
        configuration = dict(payload.get("configuration") or {})
        success_detail = "Nguồn media hoạt động"
        if kind == "video" and configuration.get("video_source_type") == "test":
            return {"ok": True, "latency_ms": 0, "detail": "Nguồn kiểm thử sẵn sàng"}
        if kind == "audio" and configuration.get("audio_source_type") == "silent":
            return {
                "ok": False,
                "latency_ms": 0,
                "detail": "Chưa chọn microphone để kiểm tra",
            }
        if kind == "speaker" and configuration.get("audio_output_type") == "disabled":
            return {
                "ok": False,
                "latency_ms": 0,
                "detail": "Chưa chọn loa để kiểm tra",
            }
        process: asyncio.subprocess.Process | None = None
        try:
            if kind == "video":
                source_type = str(configuration.get("video_source_type", "rtsp"))
                source = str(configuration.get("video_source", ""))
                if source_type == "camera":
                    resolved_source = source
                elif source_type == "rtsp":
                    resolved_source = self.camera_ptz.credentialed_source(
                        self._source_with_preserved_credentials(source)
                    )
                else:
                    resolved_source = source
                probe_config = self.config.model_copy(deep=True)
                probe_config.simulator_media_source_type = source_type
                probe_config.simulator_media_source = resolved_source
                probe_config.rtsp_transport = str(
                    configuration.get("rtsp_transport", "auto")
                )
                probe_publisher = MediaPublisher(probe_config)
                probe = await asyncio.wait_for(
                    asyncio.to_thread(probe_publisher._probe_video_source),
                    timeout=8,
                )
                reasons = probe_publisher._video_transcode_reasons(probe)
                route = "transcode" if reasons else "passthrough"
                return {
                    "ok": True,
                    "latency_ms": round((time.monotonic() - started) * 1000),
                    "detail": "Nguồn video đã hoàn tất kiểm tra realtime",
                    "codec": probe.codec,
                    "width": probe.width,
                    "height": probe.height,
                    "fps_metadata": (
                        str(probe.fps) if probe.fps is not None else None
                    ),
                    "fps_measured": (
                        str(probe.measured_fps)
                        if probe.measured_fps is not None
                        else None
                    ),
                    "has_b_frames": probe.has_b_frames,
                    "timing_reliable": probe.timing_reliable,
                    "timing_reason": probe.timing_reason,
                    "packet_count": probe.packet_count,
                    "median_frame_interval_ms": round(
                        probe.median_frame_interval_ms, 2
                    ),
                    "p95_frame_interval_ms": round(
                        probe.p95_frame_interval_ms, 2
                    ),
                    "frame_interval_jitter_ms": round(
                        probe.frame_interval_jitter_ms, 2
                    ),
                    "bitrate": probe.bitrate,
                    "largest_access_unit": probe.largest_access_unit,
                    "largest_keyframe": probe.largest_keyframe,
                    "cadence_bursty": probe.cadence_bursty,
                    "passthrough_safe": probe.passthrough_safe,
                    "needs_normalization": bool(reasons),
                    "route": route,
                    "encoder": "copy" if not reasons else probe_config.video_encoder,
                    "warnings": reasons,
                }
            elif kind == "audio":
                audio_type = str(configuration.get("audio_source_type", "device"))
                audio_source = str(configuration.get("audio_source", "default"))
                if audio_type == "device":
                    ok, detail = await asyncio.to_thread(
                        probe_audio_source, audio_source
                    )
                    return {
                        "ok": ok,
                        "latency_ms": round(
                            (time.monotonic() - started) * 1000
                        ),
                        "detail": detail,
                    }
                if audio_type == "file":
                    audio_source = self._source_with_credentials_from(
                        audio_source, self.config.simulator_audio_source
                    )
                    success_detail = "Tệp hoặc stream âm thanh đã giải mã thành công"
                input_args = ["-i", audio_source]
                command = [
                    "ffmpeg", "-hide_banner", "-loglevel", "error",
                    *input_args, "-t", "2", "-vn", "-f", "null", "-",
                ]
            elif kind == "speaker":
                audio_output = str(configuration.get("audio_output", ""))
                ok, detail = await asyncio.to_thread(
                    probe_audio_output, audio_output, audible=True
                )
                return {
                    "ok": ok,
                    "latency_ms": round((time.monotonic() - started) * 1000),
                    "detail": detail,
                }
            else:
                return {
                    "ok": False,
                    "latency_ms": 0,
                    "detail": "Loại media kiểm tra không hợp lệ",
                }
            process = await asyncio.create_subprocess_exec(
                *command,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.PIPE,
            )
            _, stderr = await asyncio.wait_for(process.communicate(), timeout=7)
            detail = stderr.decode(errors="replace").strip()[-300:]
            source = str(
                configuration.get("video_source")
                or configuration.get("audio_source")
                or configuration.get("audio_output")
                or ""
            )
            if source:
                detail = detail.replace(source, "<media-source>")
            return {
                "ok": process.returncode == 0,
                "latency_ms": round((time.monotonic() - started) * 1000),
                "detail": detail or (
                    success_detail if process.returncode == 0
                    else "Không đọc được nguồn media"
                ),
            }
        except asyncio.TimeoutError:
            if process and process.returncode is None:
                process.kill()
                await process.wait()
            return {
                "ok": False,
                "latency_ms": round((time.monotonic() - started) * 1000),
                "detail": (
                    "Nguồn video không phản hồi trong 8 giây"
                    if kind == "video"
                    else "Nguồn media không phản hồi trong 7 giây"
                ),
            }
        except Exception as exc:
            return {
                "ok": False,
                "latency_ms": round((time.monotonic() - started) * 1000),
                "detail": redact_media_source(
                    exc,
                    str(
                        configuration.get("video_source")
                        or configuration.get("audio_source")
                        or ""
                    ),
                ),
            }

    async def _diagnostics_result(
        self,
        socket: Any,
        request_id: str,
        diagnostic: str,
        result: dict[str, Any],
    ) -> None:
        await socket.send(
            json.dumps(
                make_message(
                    "diagnostics.result",
                    self.config.robot_id,
                    self._next_sequence(),
                    {
                        "request_id": request_id,
                        "diagnostic": diagnostic,
                        **result,
                    },
                )
            )
        )

    def _remember(self, message_id: str) -> None:
        self.processed_ids.add(message_id)
        self.processed_order.append(message_id)
        while len(self.processed_order) > 2048:
            self.processed_ids.discard(self.processed_order.popleft())

    async def _execute_navigation_command(
        self, command: str, payload: dict[str, Any]
    ) -> dict[str, Any]:
        recovering_autosave = False
        if command == "map.resync":
            map_id = str(payload.get("map_id", ""))
            version = int(payload.get("version", 0))
            self.map_cache._validate_identity(map_id, version)
            bundle = (
                Path(self.config.map_cache_dir) / "created" / map_id
                / f"v{version}" / "map-bundle.tar.gz"
            )
            if not bundle.is_file():
                raise NavigationBackendError(
                    "LOCAL_MAP_MISSING",
                    "Pi không còn bundle local để đồng bộ lại",
                    current_state=str(self.navigation_backend.state().get("state", "FAULT")),
                )
            upload = {
                "map_id": map_id,
                "version": version,
                "robot_id": self.config.robot_id,
                "bundle_path": str(bundle),
            }
            self._queue_mapping_upload(upload)
            self._spawn_background(self._upload_mapping_bundle_safely(upload))
            return {
                "status": "accepted",
                "current_state": str(self.navigation_backend.state().get("state", "IDLE")),
                "sync_status": "SYNC_PENDING",
            }
        if command == "mapping.recover":
            if self.config.navigation_backend != "ros2":
                raise NavigationBackendError(
                    "AUTOSAVE_RECOVERY_UNAVAILABLE",
                    "Mapping autosave recovery requires the ROS 2 backend",
                    current_state=str(
                        self.navigation_backend.state().get("state", "FAULT")
                    ),
                )
            map_id = str(payload.get("map_id", ""))
            version = int(payload.get("version", 0))
            posegraph = mapping_autosave_posegraph(
                self.config.map_cache_dir,
                map_id,
                version,
            )
            recovering_autosave = True
            command = "mapping.start"
            payload = {
                **payload,
                "posegraph_path": str(posegraph),
                # A powered-off runtime has no authoritative terminal pose.
                # Resuming from the first graph node permits an immediate save;
                # collecting more scans requires returning to the original
                # mapping start pose first.
                "initial_pose": None,
            }
        if command in {
            "mapping.start", "mapping.stop", "mapping.finish", "mapping.save",
            "map.load", "map.deactivate", "navigation.cancel",
        }:
            # Mode changes and map replacement are stationary operations. Send
            # an immediate zero through the existing motion owner before the
            # host supervisor replaces any ROS stack.
            self._stop_motion(f"safe_{command.replace('.', '_')}")
        if (
            self.config.motion_backend == "ros2"
            and self.config.navigation_backend != "ros2"
        ):
            raise NavigationBackendError(
                "NAVIGATION_BACKEND_UNAVAILABLE",
                "Real motion requires the ROS 2 navigation backend",
                current_state="FAULT",
            )
        command_payload = dict(payload)
        if command == "mapping.start":
            state = self.navigation_backend.state()
            current = str(state.get("state", "")).upper()
            if current in {"NAVIGATING", "PAUSED", "BLOCKED", "RECOVERY"}:
                await self.navigation_backend.execute(
                    "navigation.cancel", {"expected_state": current, "reason": "mapping_start"}
                )
        if command == "map.load" and self.config.navigation_backend == "ros2":
            destination = await self.map_cache.ensure(
                map_id=str(payload["map_id"]),
                version=int(payload["version"]),
                checksum=str(payload["checksum"]),
                download_url=str(payload["download_url"]),
            )
            command_payload["map_path"] = str(destination)
            recent_pose = self.map_cache.last_pose(
                str(payload["map_id"]),
                int(payload["version"]),
                max_age_seconds=3600,
            )
            if (
                recent_pose is not None
                and int(recent_pose.get("verification_version", 0)) >= 2
            ):
                # This is only a bounded AMCL seed. The adapter still requires
                # fresh LiDAR K-of-N, covariance, stability and uniqueness
                # evidence before exposing a map pose or READY.
                recent_pose["covariance"] = max(
                    0.01, min(0.25, float(recent_pose.get("covariance", 0.25)))
                )
                recent_pose["source"] = "recent_navigation_pose"
                command_payload["last_known_pose"] = recent_pose
        if (
            command == "mapping.start"
            and self.config.navigation_backend == "ros2"
            and payload.get("source_version")
        ):
            destination = await self.map_cache.ensure(
                map_id=str(payload["map_id"]),
                version=int(payload["source_version"]),
                checksum=str(payload["source_checksum"]),
                download_url=str(payload["source_download_url"]),
            )
            basename = str(payload.get("posegraph_basename") or "posegraph")
            if Path(basename).name != basename:
                raise MapCacheError("invalid pose-graph basename")
            posegraph = destination / basename
            if not posegraph.with_suffix(".posegraph").is_file() or not posegraph.with_suffix(".data").is_file():
                raise MapCacheError("downloaded map has no serialized pose-graph")
            command_payload["posegraph_path"] = str(posegraph)
        result = await self.navigation_backend.execute(command, command_payload)
        if (
            recovering_autosave
            and result.get("status") in {"accepted", "completed"}
        ):
            # Do not ingest scans at the charging location as though the robot
            # were still at the graph's first node. The operator must choose
            # Resume explicitly after returning the chassis to the mapping
            # start, or save the recovered graph without adding new scans.
            result = await self.navigation_backend.execute(
                "mapping.pause",
                {"expected_state": str(result.get("current_state", "MAPPING_RUNNING"))},
            )
            result["recovered_from_autosave"] = True
        if command == "map.load" and result.get("status") in {"accepted", "completed"}:
            self.map_cache.mark_active(
                str(payload["map_id"]),
                int(payload["version"]),
                str(payload["checksum"]),
                Path(str(command_payload["map_path"])),
            )
        if command == "map.deactivate" and payload.get("delete_local"):
            self.map_cache.delete_local(
                str(payload["map_id"]),
                deleted_at=float(payload.get("deleted_at") or time.time()),
            )
        bundle_path = str(result.get("bundle_path", ""))
        if command in {"mapping.save", "mapping.save_draft", "mapping.finish"} and bundle_path:
            upload = {
                "map_id": str(payload["map_id"]),
                "version": int(payload["version"]),
                "robot_id": self.config.robot_id,
                "bundle_path": bundle_path,
            }
            self._queue_mapping_upload(upload)
            self._spawn_background(self._upload_mapping_bundle_safely(upload))
            result["upload_status"] = "PENDING"
        return result

    def _spawn_background(self, operation: Any) -> None:
        task = asyncio.create_task(operation)
        self._background_tasks.add(task)

        def finished(completed: asyncio.Task[Any]) -> None:
            self._background_tasks.discard(completed)
            if completed.cancelled():
                return
            try:
                completed.result()
            except Exception as exc:
                logger.warning("background operation failed error=%s", exc)

        task.add_done_callback(finished)

    async def _handle_navigation_command(self, socket: Any, message: Message) -> None:
        try:
            result = await self._execute_navigation_command(
                message.message_type, message.payload
            )
            await self._ack(
                socket,
                message,
                str(result.get("status", "accepted")),
                result,
            )
        except (NavigationBackendError, MapCacheError) as exc:
            if getattr(exc, "code", "") == "NAVIGATION_BACKEND_UNAVAILABLE":
                self.navigation.status = "failed"
            if message.message_type != "navigation.speed_mode":
                self._stop_motion(
                    "navigation_unsupported"
                    if getattr(exc, "code", "") == "NAVIGATION_BACKEND_UNAVAILABLE"
                    else "navigation_command_rejected"
                )
            await self._ack(
                socket,
                message,
                "rejected",
                {
                    "current_state": getattr(exc, "current_state", "FAULT"),
                    "error_code": getattr(exc, "code", "MAP_CACHE_ERROR"),
                    "error_message": str(exc),
                },
            )
        await self._navigation_status(socket)

    @staticmethod
    def _file_sha256(path: Path) -> str:
        digest = hashlib.sha256()
        with path.open("rb") as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    async def _upload_mapping_bundle(self, upload: dict[str, Any]) -> None:
        path = Path(str(upload["bundle_path"]))
        if not path.is_file():
            raise FileNotFoundError(path)
        checksum = await asyncio.to_thread(self._file_sha256, path)
        token = await self._robot_bearer_token()

        def send() -> None:
            with path.open("rb") as source, httpx.Client(
                base_url=self.config.center_api_url.rstrip("/"),
                verify=self.http_verify,
                timeout=120,
            ) as client:
                response = client.post(
                    f"/api/maps/{upload['map_id']}/versions",
                    headers={"Authorization": f"Bearer {token}"},
                    data={
                        "version": str(upload["version"]),
                        "robot_id": str(upload["robot_id"]),
                        "checksum": checksum,
                    },
                    files={"bundle": (path.name, source, "application/gzip")},
                )
                response.raise_for_status()
                remote_checksum = str(response.json().get("checksum", "")).lower()
                if remote_checksum != checksum.lower():
                    raise MapCacheError("Center acknowledged a different map checksum")

        await asyncio.to_thread(send)
        self.map_cache.mark_synced(
            str(upload["map_id"]), int(upload["version"]), checksum
        )
        path.with_name(".upload-pending.json").unlink(missing_ok=True)

    async def _upload_mapping_bundle_safely(self, upload: dict[str, Any]) -> None:
        try:
            await self._upload_mapping_bundle(upload)
            logger.info("mapping bundle uploaded path=%s", upload["bundle_path"])
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            # The persistent marker is intentionally retained for reconnect.
            logger.warning(
                "mapping bundle queued for retry path=%s error=%s",
                upload["bundle_path"],
                exc,
            )

    def _queue_mapping_upload(self, upload: dict[str, Any]) -> None:
        marker = Path(str(upload["bundle_path"])).with_name(".upload-pending.json")
        marker.write_text(json.dumps(upload))
        try:
            checksum = self._file_sha256(Path(str(upload["bundle_path"])))
            self.map_cache.mark_local(
                str(upload["map_id"]), int(upload["version"]), checksum, "SYNC_PENDING"
            )
        except (OSError, MapCacheError) as exc:
            logger.warning("cannot update local map registry error=%s", exc)

    async def _mapping_upload_retry_loop(self) -> None:
        root = Path(self.config.map_cache_dir) / "created"
        while True:
            for marker in (root.glob("*/*/.upload-pending.json") if root.exists() else ()):
                try:
                    upload = json.loads(marker.read_text())
                    await self._upload_mapping_bundle(upload)
                    logger.info("mapping bundle upload recovered path=%s", upload["bundle_path"])
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    logger.warning("mapping bundle retry pending marker=%s error=%s", marker, exc)
            await asyncio.sleep(10)

    async def _map_registry_sync_loop(self) -> None:
        """Apply Center tombstones after reconnect so deleted maps never resurrect."""
        while True:
            try:
                token = await self._robot_bearer_token()
                async with httpx.AsyncClient(
                    base_url=self.config.center_api_url.rstrip("/"),
                    verify=self.http_verify,
                    timeout=20,
                ) as client:
                    response = await client.get(
                        "/api/maps/tombstones",
                        headers={"Authorization": f"Bearer {token}"},
                    )
                    response.raise_for_status()
                    for item in response.json().get("items", []):
                        map_id = str(item.get("map_id", ""))
                        if not map_id:
                            continue
                        active = self.map_cache.active()
                        backend_state = self.navigation_backend.state()
                        runtime_has_map = str(backend_state.get("map_id") or "") == map_id
                        if runtime_has_map or (active and active.get("map_id") == map_id):
                            try:
                                await self.navigation_backend.execute(
                                    "map.deactivate",
                                    {"expected_state": backend_state.get("state", "")},
                                )
                            except NavigationBackendError as exc:
                                self._stop_motion("map_tombstone_deactivate_failed")
                                # Persist the tombstone immediately so a reboot
                                # cannot resurrect the map, but do not ACK
                                # Center until the ROS runtime is confirmed IDLE.
                                self.map_cache.delete_local(
                                    map_id,
                                    deleted_at=float(item.get("deleted_at_unix") or time.time()),
                                )
                                logger.warning(
                                    "map tombstone waiting for runtime deactivation map_id=%s error=%s",
                                    map_id,
                                    exc,
                                )
                                continue
                        self.map_cache.delete_local(
                            map_id, deleted_at=float(item.get("deleted_at_unix") or time.time())
                        )
                        ack = await client.post(
                            f"/api/maps/tombstones/{map_id}/ack",
                            headers={"Authorization": f"Bearer {token}"},
                            json={"robot_id": self.config.robot_id},
                        )
                        ack.raise_for_status()
                        self.map_cache.acknowledge_tombstone(map_id)
            except asyncio.CancelledError:
                raise
            except (httpx.HTTPError, OSError, ValueError, MapCacheError) as exc:
                logger.debug("map registry sync pending error=%s", exc)
            await asyncio.sleep(30)

    async def _restore_active_navigation_map(
        self, state: dict[str, Any]
    ) -> dict[str, Any] | None:
        """Restore the verified active map after a Nav2/adapter restart.

        The active assignment and bundle survive container and Web session
        restarts, while the map_server process does not. Waiting for an
        operator to click Activate again left later Control sessions without
        a map->base pose even though the exact map was already cached.
        """
        if str(state.get("state", "")).upper() != "NO_ACTIVE_MAP":
            return None
        if str(state.get("mode", "NAVIGATION")).upper() != "NAVIGATION":
            return None
        payload = self.map_cache.active_load_payload(expected_state="NO_ACTIVE_MAP")
        if payload is None:
            return None
        self._stop_motion("restore_active_navigation_map")
        result = await self.navigation_backend.execute("map.load", payload)
        logger.info(
            "restored active navigation map map_id=%s version=%s",
            payload["map_id"],
            payload["version"],
        )
        return result

    async def _navigation_runtime_loop(self, socket: Any) -> None:
        if self.config.navigation_backend != "ros2":
            return
        last_visualization_revision = -1
        last_visualization_identity: tuple[str, int] | None = None
        last_route_id: str | None = None
        last_global_path: list[dict[str, Any]] | None = None
        last_dynamic_obstacles: list[dict[str, Any]] | None = None
        last_pose_save = 0.0
        last_poll_warning = 0.0
        last_restore_attempt = 0.0
        adapter_was_unavailable = False
        while True:
            try:
                result = await self.navigation_backend.execute("system.status", {})
                if adapter_was_unavailable:
                    logger.info("ROS 2 navigation adapter connection recovered")
                    adapter_was_unavailable = False
                state = dict(result.get("state") or {})
                state["trajectory"] = bounded_navigation_trajectory(
                    state.get("trajectory")
                )
                if (
                    str(state.get("state", "")).upper() == "NO_ACTIVE_MAP"
                    and time.monotonic() - last_restore_attempt >= 10.0
                ):
                    last_restore_attempt = time.monotonic()
                    try:
                        restored = await self._restore_active_navigation_map(state)
                        if restored is not None:
                            result = restored
                            state = dict(restored.get("state") or state)
                    except (NavigationBackendError, MapCacheError, OSError, ValueError) as exc:
                        logger.warning("active map restore pending error=%s", exc)
                # A map-restore command returns a fresh state object, so apply
                # the same transport budget after that replacement as well.
                state["trajectory"] = bounded_navigation_trajectory(
                    state.get("trajectory")
                )
                await socket.send(
                    json.dumps(
                        make_message(
                            "navigation.status",
                            self.config.robot_id,
                            self._next_sequence(),
                            {"status": state.get("state", "FAULT"), **state},
                        )
                    )
                )
                visualization = result.get("visualization")
                if isinstance(visualization, dict):
                    revision = int(visualization.get("revision", -1))
                    if revision != last_visualization_revision:
                        last_visualization_revision = revision
                        previous = (
                            None
                            if last_visualization_identity is None
                            else {
                                "identity": last_visualization_identity,
                                "route_id": last_route_id,
                                "global_path": last_global_path,
                                "dynamic_obstacles": last_dynamic_obstacles,
                            }
                        )
                        changed, delta_state = navigation_visualization_delta(
                            visualization, previous
                        )
                        last_visualization_identity = delta_state["identity"]
                        last_route_id = delta_state["route_id"]
                        last_global_path = delta_state["global_path"]
                        last_dynamic_obstacles = delta_state["dynamic_obstacles"]
                        await socket.send(
                            json.dumps(
                                make_message(
                                    "navigation.visualization",
                                    self.config.robot_id,
                                    self._next_sequence(),
                                    changed,
                                )
                            )
                        )
                pose = result.get("pose")
                if isinstance(pose, dict):
                    await socket.send(
                        json.dumps(
                            make_message(
                                "robot.pose",
                                self.config.robot_id,
                                self._next_sequence(),
                                {
                                    "map_id": state.get("map_id", self.config.map_id),
                                    "map_version": int(state.get("map_version", 0) or 0),
                                    "x": pose.get("x", 0),
                                    "y": pose.get("y", 0),
                                    "yaw": pose.get("yaw", 0),
                                    "linear_velocity": 0,
                                    "angular_velocity": 0,
                                    "timestamp": time.time(),
                                    "localized": bool(state.get("localized")),
                                    "confidence": float(state.get("localization_confidence", 0)),
                                },
                            )
                        )
                    )
                    if (
                        localization_pose_safe_to_persist(state)
                        and state.get("map_id")
                        and int(state.get("map_version", 0) or 0) > 0
                        and time.monotonic() - last_pose_save >= 5.0
                    ):
                        last_pose_save = time.monotonic()
                        self.map_cache.save_last_pose(
                            str(state["map_id"]),
                            int(state["map_version"]),
                            {
                                **pose,
                                "verification_version": int(
                                    state.get("localization_verification_version", 0)
                                ),
                                "covariance": max(
                                    0.01,
                                    1.0 - float(state.get("localization_confidence", 0)),
                                ),
                            },
                        )
            except asyncio.CancelledError:
                raise
            except NavigationBackendError as exc:
                now = time.monotonic()
                adapter_was_unavailable = True
                if now - last_poll_warning >= 5.0:
                    logger.warning("navigation status poll failed error=%s", exc)
                    last_poll_warning = now
            await asyncio.sleep(0.2)

    async def _ack(
        self,
        socket: Any,
        command: Message,
        status: str,
        details: dict[str, Any] | None = None,
    ) -> None:
        request_id = str(command.payload.get("request_id", ""))
        payload = {
            "command_message_id": str(command.message_id),
            "request_id": request_id,
            "status": status,
            **bounded_navigation_command_details(details),
        }
        await socket.send(
            json.dumps(
                make_message(
                    "command.ack",
                    self.config.robot_id,
                    self._next_sequence(),
                    payload,
                    command.session_id,
                )
            )
        )

    async def _simulation_loop(self) -> None:
        period = 1 / self.config.simulation_hz
        previous = time.monotonic()
        while True:
            started = time.monotonic()
            dt = min(0.1, started - previous)
            previous = started
            if self.config.motion_backend == "simulator":
                self.navigation.update()
                self.motion.step(dt)
            self.motion_driver.watchdog(started)
            await asyncio.sleep(max(0, period - (time.monotonic() - started)))

    async def _telemetry_loop(self, socket: Any) -> None:
        previous_status = self.navigation.status
        while True:
            if self.config.navigation_backend == "simulator":
                await socket.send(
                    json.dumps(
                        make_message(
                            "robot.pose",
                            self.config.robot_id,
                            self._next_sequence(),
                            self.motion.as_payload(),
                        )
                    )
                )
            backend_state = self.navigation_backend.state()
            registry_health = self.map_cache.health()
            await socket.send(
                json.dumps(
                    make_message(
                        "robot.health",
                        self.config.robot_id,
                        self._next_sequence(),
                        {
                            "battery_percent": 78,
                            "network_rtt_ms": random.randint(36, 68),
                            "packet_loss_percent": round(random.uniform(0.0, 0.7), 2),
                            "camera": "online" if self.media.connected else "offline",
                            "audio": (
                                "online"
                                if self.media.connected
                                and (
                                    self.config.simulator_audio_source_type == "silent"
                                    or self.media.audio_capture_healthy
                                )
                                else "offline"
                            ),
                            "audio_playback": (
                                "online"
                                if self.media.audio_playback_healthy
                                else "offline"
                            ),
                            "aec": "active" if self.media.aec_active else "inactive",
                            "navigation": self.navigation.status,
                            "simulator": "running",
                            "motion_backend": self.config.motion_backend,
                            "navigation_backend": self.config.navigation_backend,
                            "map_state": backend_state.get("state"),
                            "map_id": backend_state.get("map_id", ""),
                            "mode": backend_state.get("mode"),
                            "map_version": backend_state.get("map_version", 0),
                            "localized": backend_state.get("localized", False),
                            "localization_state": backend_state.get("localization_state", "IDLE"),
                            "localization_confidence": backend_state.get("localization_confidence", 0),
                            "localization_diagnostics": backend_state.get(
                                "localization_diagnostics"
                            ),
                            "nav2": backend_state.get("nav2", "UNAVAILABLE"),
                            "auto_speed_mode": backend_state.get(
                                "auto_speed_mode", "NORMAL"
                            ),
                            "auto_speed_profile": backend_state.get(
                                "auto_speed_profile"
                            ),
                            "replan_frequency_hz": backend_state.get(
                                "replan_frequency_hz", 0.0
                            ),
                            "navigation_metrics": backend_state.get(
                                "navigation_metrics"
                            ),
                            "safety": (
                                "HEALTHY"
                                if self.config.motion_backend == "simulator"
                                else "READ_ONLY"
                                if self.config.motion_backend == "disabled"
                                else backend_state.get("safety", "UNKNOWN")
                            ),
                            "scan_fresh": (
                                True
                                if self.config.motion_backend == "simulator"
                                else backend_state.get("scan_fresh", False)
                            ),
                            "sensor_clock_state": backend_state.get(
                                "sensor_clock_state", "CLOCK_SYNCING"
                            ),
                            "sensor_time_healthy": backend_state.get(
                                "sensor_time_healthy",
                                self.config.motion_backend == "simulator",
                            ),
                            "sensor_time_failure_reason": backend_state.get(
                                "sensor_time_failure_reason", ""
                            ),
                            "sensor_time_diagnostics": backend_state.get(
                                "sensor_time_diagnostics", {}
                            ),
                            "scan_arrival_fresh": backend_state.get(
                                "scan_arrival_fresh", False
                            ),
                            "scan_timestamp_valid": backend_state.get(
                                "scan_timestamp_valid", False
                            ),
                            "scan_clock_skew_seconds": backend_state.get(
                                "scan_clock_skew_seconds", 0.0
                            ),
                            "odom_arrival_fresh": backend_state.get(
                                "odom_arrival_fresh", False
                            ),
                            "odom_timestamp_valid": backend_state.get(
                                "odom_timestamp_valid", False
                            ),
                            "odom_clock_skew_seconds": backend_state.get(
                                "odom_clock_skew_seconds", 0.0
                            ),
                            "imu_arrival_fresh": backend_state.get(
                                "imu_arrival_fresh", False
                            ),
                            "imu_timestamp_valid": backend_state.get(
                                "imu_timestamp_valid", False
                            ),
                            "imu_clock_skew_seconds": backend_state.get(
                                "imu_clock_skew_seconds", 0.0
                            ),
                            "odometry_ready": backend_state.get(
                                "odometry_ready",
                                self.config.motion_backend == "simulator",
                            ),
                            "lidar_tf_ready": backend_state.get(
                                "lidar_tf_ready",
                                self.config.motion_backend == "simulator",
                            ),
                            "estop": bool(backend_state.get("estop", False)),
                            "collision_fault": False,
                            "mapping": backend_state.get("mapping"),
                            "map_registry": registry_health,
                            "footprint": backend_state.get("footprint"),
                            "corridor": backend_state.get("corridor"),
                            # Keep the browser bound to the authoritative edge
                            # mission even if its local route preview is replaced
                            # by a telemetry refresh while choosing alternatives.
                            "mission_id": backend_state.get("mission_id", ""),
                            "route_candidates": backend_state.get("route_candidates", []),
                            "selected_route_id": backend_state.get("selected_route_id", ""),
                            "manual_handoff_reason": backend_state.get("manual_handoff_reason", ""),
                            "trajectory": bounded_navigation_trajectory(
                                backend_state.get("trajectory")
                            ),
                        },
                    )
                )
            )
            if previous_status != self.navigation.status:
                previous_status = self.navigation.status
                await self._navigation_status(socket)
            await asyncio.sleep(1 / self.config.telemetry_hz)

    async def _navigation_status(self, socket: Any) -> None:
        backend_state = self.navigation_backend.state()
        backend_state["trajectory"] = bounded_navigation_trajectory(
            backend_state.get("trajectory")
        )
        await socket.send(
            json.dumps(
                make_message(
                    "navigation.status",
                    self.config.robot_id,
                    self._next_sequence(),
                    {
                        "route_id": self.navigation.route_id,
                        "status": self.navigation.status,
                        "point_index": self.navigation.point_index,
                        **backend_state,
                    },
                )
            )
        )

    async def _heartbeat_loop(self, socket: Any) -> None:
        while True:
            if self._reset_identity_if_state_removed():
                raise RuntimeError("device state removed; reconnecting to reclaim")
            await socket.send(
                json.dumps(
                    make_message(
                        "robot.heartbeat",
                        self.config.robot_id,
                        self._next_sequence(),
                        {"uptime": time.monotonic()},
                    )
                )
            )
            await asyncio.sleep(self.config.heartbeat_seconds)

    def _next_sequence(self) -> int:
        self.sequence += 1
        return self.sequence

    def _stop_motion(self, reason: str) -> None:
        self.motion_driver.stop(reason)
        if self.motion_driver is not self.motion:
            self.motion.stop(reason)

    def _ptz_source(self) -> tuple[str, str]:
        source_type = self.config.simulator_media_source_type
        source = self.config.simulator_media_source
        if source_type == "camera" and not source:
            source = self.config.simulator_camera_device
        return source_type, source

    def _dispatch_ptz(self, payload: dict[str, Any]) -> bool:
        source_type, source = self._ptz_source()
        if source_type not in {"rtsp", "camera"} or not source:
            return False
        # PTZ must never block velocity, stop, heartbeat, or media messages.
        # Only the newest UI intent matters; cancelling an in-flight request
        # prevents a queue of stale move commands from building up.
        if self._ptz_task is not None and not self._ptz_task.done():
            self._ptz_task.cancel()
        task = asyncio.create_task(
            self.camera_ptz.command(source_type, source, dict(payload))
        )
        self._ptz_task = task

        def command_finished(completed: asyncio.Task[bool]) -> None:
            if completed.cancelled():
                return
            try:
                accepted = completed.result()
            except Exception as exc:
                logger.debug("PTZ command failed error=%s", exc)
            else:
                if not accepted:
                    logger.debug("PTZ command rejected source_type=%s", source_type)

        task.add_done_callback(command_finished)
        return True

    async def stop(self) -> None:
        self.running = False
        self._stop_motion("edge_shutdown")
        if self._ptz_task is not None and not self._ptz_task.done():
            self._ptz_task.cancel()
            await asyncio.gather(self._ptz_task, return_exceptions=True)
        await self.camera_ptz.stop(*self._ptz_source())
        await self.camera_ptz.close()
        if self.socket:
            await self.socket.close()
        self.motion_driver.close()
