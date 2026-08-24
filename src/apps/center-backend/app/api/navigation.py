from __future__ import annotations

import hashlib
import json
import math
from datetime import datetime, timezone
from typing import Literal
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field, field_validator
from sqlalchemy import select
from sqlalchemy.orm import Session

from app.core.config import Settings, get_settings
from app.core.security import current_user
from app.models.database import get_db
from app.models.entities import (
    CommandReceipt,
    Destination,
    MapRecord,
    MapVersion,
    NavigationMission,
    POI,
    Robot,
    RobotMapCache,
)
from app.schemas.messages import (
    NavigationCancelRequest,
    NavigationGoalRequest,
    NavigationPreviewRequest,
)
from app.services.hub import hub
from app.services.state_machines import InvalidTransition, navigation_transition

router = APIRouter(prefix="/api/navigation", tags=["navigation"])

PLAN_FAILURE_MESSAGES = {
    "START_BLOCKED": "Không thể lập đường: vùng xuất phát bị costmap đánh dấu là vật cản.",
    "GOAL_BLOCKED": "Không thể lập đường: điểm đến bị costmap đánh dấu là vật cản.",
    "NO_VALID_PATH": "Không tìm thấy đường hợp lệ tới điểm đích.",
    "NO_PATH": "Không tìm thấy đường hợp lệ tới điểm đích.",
    "UNKNOWN_SPACE": "Không thể lập đường vì lộ trình đi qua vùng chưa được lập bản đồ.",
    "PLANNER_TIMEOUT": "Bộ lập đường không phản hồi đúng thời gian.",
    "TF_ERROR": "Không thể xác định vị trí robot trên bản đồ để lập đường.",
    "COSTMAP_NOT_READY": "Costmap chưa sẵn sàng; vui lòng thử lại sau khi dữ liệu LiDAR được cập nhật.",
    "ROUTE_CLEARANCE_INSUFFICIENT": (
        "Không có lộ trình vượt hard safety margin của footprint thật, "
        "độ bất định localization và độ phân giải bản đồ."
    ),
}


class GoalPose(BaseModel):
    x: float
    y: float
    yaw: float = 0.0

    @field_validator("x", "y", "yaw")
    @classmethod
    def finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("Tọa độ phải là số hữu hạn")
        return value


class NavigationCommandBase(BaseModel):
    request_id: str = Field(min_length=8, max_length=64)
    robot_id: str = Field(min_length=3, max_length=64)
    session_id: str = Field(min_length=8, max_length=128)
    expected_state: str = Field(max_length=32)


class MapLoadRequest(NavigationCommandBase):
    map_id: str = Field(min_length=2, max_length=64)
    version: int = Field(ge=1)


class InitialPoseRequest(NavigationCommandBase):
    map_id: str = Field(min_length=2, max_length=64)
    version: int = Field(ge=1)
    pose: GoalPose


class RelocalizeRequest(NavigationCommandBase):
    map_id: str = Field(min_length=2, max_length=64)
    version: int = Field(ge=1)
    allow_rotation: bool = False
    force_global: bool = False


class ComputePathRequest(NavigationCommandBase):
    map_id: str = Field(min_length=2, max_length=64)
    version: int = Field(ge=1)
    goal: GoalPose


class MissionCommandRequest(NavigationCommandBase):
    mission_id: str = Field(min_length=8, max_length=64)
    route_id: str | None = Field(default=None, min_length=3, max_length=96)


class SpeedModeRequest(NavigationCommandBase):
    mode: Literal["SLOW", "NORMAL", "FAST"]


def _route_id_for_path(path: list[dict[str, float]]) -> str:
    digest = hashlib.sha1(
        json.dumps(path, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()[:12]
    return f"route-{digest}"


def _normalized_route_candidates(value: object) -> list[dict]:
    if not isinstance(value, list):
        return []
    candidates: list[dict] = []
    for raw in value:
        if not isinstance(raw, dict) or raw.get("valid") is False:
            continue
        route_id = str(raw.get("route_id") or "")
        points = raw.get("points")
        if not route_id or not isinstance(points, list) or len(points) < 2:
            continue
        candidate = dict(raw)
        candidate["route_id"] = route_id
        candidate["points"] = list(points)
        candidate.setdefault(
            "minimum_clearance",
            candidate.get("minimum_static_clearance"),
        )
        candidates.append(candidate)
    return candidates


def _mission_route_candidates(
    database: Session,
    mission: NavigationMission,
) -> list[dict]:
    """Read the authoritative preview candidates already persisted in its receipt."""
    receipt = database.get(CommandReceipt, mission.request_id)
    if receipt is None or receipt.command_type != "navigation.compute_path":
        return []
    response = receipt.response if isinstance(receipt.response, dict) else {}
    return _normalized_route_candidates(
        response.get("route_candidates") or response.get("candidates")
    )


def _selected_candidate(
    candidates: list[dict],
    route_id: str | None,
    fallback_path: list[dict],
) -> tuple[str, list[dict]]:
    if route_id:
        for candidate in candidates:
            if candidate["route_id"] == route_id:
                return route_id, list(candidate["points"])
        # Compatibility for a READY mission previewed before candidate lists
        # were exposed.  The deterministic ID still binds exactly to the
        # server-persisted path; arbitrary browser IDs remain rejected.
        if not candidates and route_id == _route_id_for_path(fallback_path):
            return route_id, fallback_path
        raise ValueError("UNKNOWN_ROUTE_ID")
    for candidate in candidates:
        if list(candidate["points"]) == fallback_path:
            return str(candidate["route_id"]), fallback_path
    return _route_id_for_path(fallback_path), fallback_path


def _mission_view(
    mission: NavigationMission,
    *,
    candidates: list[dict] | None = None,
    selected_route_id: str | None = None,
) -> dict:
    candidate_views = list(candidates or [])
    selected_id, _ = _selected_candidate(
        candidate_views,
        selected_route_id,
        list(mission.path or []),
    )
    return {
        "route_id": selected_id,
        "mission_id": mission.mission_id,
        "destination_id": "CUSTOM-GOAL",
        "request_id": mission.request_id,
        "robot_id": mission.robot_id,
        "session_id": mission.control_session_id,
        "map_id": mission.map_id,
        "map_version": mission.map_version,
        "status": mission.status,
        "goal": mission.goal,
        "points": mission.path,
        "distance_m": mission.distance_m,
        "estimated_seconds": max(1, round(mission.distance_m / 0.15)),
        "error_code": mission.error_code,
        "error_message": mission.error_message,
        "candidates": candidate_views,
        "selected_route_id": selected_id,
        "created_at": mission.created_at.isoformat(),
        "updated_at": mission.updated_at.isoformat(),
    }


def _mission_start_rejection(mission: NavigationMission) -> dict | None:
    """Return a user-facing rejection before any navigation command is sent."""
    if mission.status == "READY" and mission.path:
        return None
    code = str(mission.error_code or ("NO_PATH" if not mission.path else "MISSION_NOT_READY"))
    message = PLAN_FAILURE_MESSAGES.get(
        code,
        str(mission.error_message or "Lộ trình chưa sẵn sàng để bắt đầu."),
    )
    return {"code": code, "message": message, "status": mission.status}


def _valid_lease(robot_id: str, session_id: str, user_id: str) -> None:
    session = hub.get_session(session_id, user_id)
    if session is None or session.robot_id != robot_id:
        raise HTTPException(status_code=403, detail="Phiên điều khiển không hợp lệ")


def _active_version(database: Session, map_id: str, version: int) -> MapVersion | None:
    record = database.get(MapRecord, map_id)
    if record is None or record.status != "ACTIVE" or record.active_version != version:
        return None
    return database.scalar(
        select(MapVersion).where(
            MapVersion.map_id == map_id,
            MapVersion.version == version,
            MapVersion.status == "ACTIVE",
        )
    )


def _robot_by_public_id(database: Session, robot_id: str) -> Robot | None:
    return database.scalar(select(Robot).where(Robot.robot_id == robot_id))


def _goal_in_map(version: MapVersion, goal: GoalPose) -> bool:
    origin = dict(version.origin or {})
    delta_x = goal.x - float(origin.get("x", 0))
    delta_y = goal.y - float(origin.get("y", 0))
    yaw = float(origin.get("yaw", 0))
    local_x = math.cos(yaw) * delta_x + math.sin(yaw) * delta_y
    local_y = -math.sin(yaw) * delta_x + math.cos(yaw) * delta_y
    return 0 <= local_x < version.width * version.resolution and 0 <= local_y < version.height * version.resolution


async def _command(
    database: Session,
    settings: Settings,
    *,
    request_id: str,
    robot_id: str,
    command_type: str,
    expected_state: str,
    payload: dict,
    timeout_seconds: float | None = None,
) -> dict:
    receipt = database.get(CommandReceipt, request_id)
    if receipt is not None:
        if receipt.robot_id != robot_id or receipt.command_type != command_type:
            raise HTTPException(status_code=409, detail="request_id đã được dùng cho lệnh khác")
        return receipt.response
    receipt = CommandReceipt(
        request_id=request_id,
        robot_id=robot_id,
        command_type=command_type,
        expected_state=expected_state,
        status="PENDING",
        response={"request_id": request_id, "status": "pending"},
    )
    database.add(receipt)
    database.commit()
    try:
        result = await hub.request_robot(
            robot_id,
            command_type,
            {**payload, "expected_state": expected_state},
            timeout_seconds=timeout_seconds or settings.robot_command_timeout_seconds,
            request_id=request_id,
        )
    except ConnectionError as exc:
        receipt.status = "REJECTED"
        receipt.response = {
            "request_id": request_id,
            "status": "rejected",
            "error_code": "ROBOT_OFFLINE",
            "error_message": "Robot đang ngoại tuyến",
        }
        database.commit()
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến") from exc
    except TimeoutError as exc:
        receipt.status = "REJECTED"
        receipt.response = {
            "request_id": request_id,
            "status": "rejected",
            "error_code": "ROBOT_TIMEOUT",
            "error_message": "Robot không ACK lệnh đúng hạn",
        }
        database.commit()
        raise HTTPException(status_code=504, detail="Robot không ACK lệnh đúng hạn") from exc
    receipt.status = str(result.get("status", "accepted")).upper()
    receipt.response = result
    database.commit()
    return result


def _navigation_preflight(robot_id: str) -> list[str]:
    robot = hub.robots.get(robot_id)
    if robot is None or robot.status != "online":
        return ["ROBOT_OFFLINE"]
    if robot.capabilities.get("source") == "simulator":
        return []
    health = robot.health
    checks = {
        "MAP_NOT_READY": health.get("map_state") == "READY",
        "NOT_LOCALIZED": bool(health.get("localized")) and health.get("localization_state", "READY") == "READY",
        "NAV2_NOT_READY": health.get("nav2") == "READY",
        "SAFETY_UNHEALTHY": health.get("safety") == "HEALTHY",
        "SCAN_STALE": bool(health.get("scan_fresh")),
        "ESTOP_ACTIVE": not bool(health.get("estop")),
        "COLLISION_FAULT": not bool(health.get("collision_fault")),
        "BATTERY_LOW": float(health.get("battery_percent", 0)) >= 15,
    }
    return [code for code, passed in checks.items() if not passed]


@router.post("/map/load")
async def load_map(
    body: MapLoadRequest,
    user_id: str = Depends(current_user),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    _valid_lease(body.robot_id, body.session_id, user_id)
    version = _active_version(database, body.map_id, body.version)
    if version is None:
        raise HTTPException(status_code=409, detail="Map/version chưa ACTIVE")
    runtime = hub.robots.get(body.robot_id)
    expected_state = body.expected_state
    runtime_state = str((runtime.health if runtime else {}).get("map_state") or expected_state).upper()
    if runtime_state in {"NAVIGATING", "PAUSED", "BLOCKED", "RECOVERY"}:
        canceled = await _command(
            database,
            settings,
            request_id=str(uuid4()),
            robot_id=body.robot_id,
            command_type="navigation.cancel",
            expected_state=runtime_state,
            payload={"reason": "activate_map"},
        )
        if canceled.get("status") not in {"accepted", "completed"}:
            raise HTTPException(status_code=409, detail="Không thể dừng Navigation cũ để kích hoạt map")
        expected_state = str(canceled.get("current_state") or "CANCELED")
        for mission in database.scalars(
            select(NavigationMission).where(
                NavigationMission.robot_id == body.robot_id,
                NavigationMission.status.not_in(
                    ("SUCCEEDED", "ARRIVED", "CANCELED", "PLAN_FAILED", "FAILED", "FAULT")
                ),
            )
        ):
            mission.status = "CANCELED"
    result = await _command(
        database,
        settings,
        request_id=body.request_id,
        robot_id=body.robot_id,
        command_type="map.load",
        expected_state=expected_state,
        payload={
            "map_id": body.map_id,
            "version": body.version,
            "checksum": version.checksum,
            "download_url": f"/api/maps/{body.map_id}/versions/{body.version}/download",
        },
        timeout_seconds=settings.mapping_command_timeout_seconds,
    )
    if result.get("status") not in {"accepted", "completed"}:
        raise HTTPException(
            status_code=409,
            detail={
                "code": result.get("error_code", "MAP_LOAD_REJECTED"),
                "message": result.get("error_message", "Robot từ chối nạp map"),
                "current_state": result.get("current_state"),
            },
        )
    cache = database.scalar(
        select(RobotMapCache).where(
            RobotMapCache.robot_id == body.robot_id,
            RobotMapCache.map_id == body.map_id,
            RobotMapCache.version == body.version,
        )
    )
    if cache is None:
        cache = RobotMapCache(
            robot_id=body.robot_id,
            map_id=body.map_id,
            version=body.version,
            checksum=version.checksum,
        )
        database.add(cache)
    cache.status = str(result.get("current_state", "LOADING_MAP"))
    cache.local_status = "AVAILABLE"
    cache.sync_status = "SYNCED"
    for previous in database.scalars(
        select(RobotMapCache).where(RobotMapCache.robot_id == body.robot_id)
    ):
        previous.active = previous.map_id == body.map_id and previous.version == body.version
    cache.progress_percent = float(result.get("progress_percent", 0))
    cache.error_message = result.get("error_message")
    # Robot.id is the internal UUID; navigation commands carry the public
    # robot_id. Looking it up through Session.get silently returned None and
    # left the dashboard on the previous map even after the edge loaded the
    # requested bundle successfully.
    robot_record = _robot_by_public_id(database, body.robot_id)
    if robot_record is not None:
        robot_record.map_id = body.map_id
        robot_record.active_map_version = body.version
    if runtime is not None:
        runtime.map_id = body.map_id
    database.commit()
    return {**result, "map_id": body.map_id, "version": body.version}


@router.post("/map/approximate-pose")
async def set_initial_pose(
    body: InitialPoseRequest,
    user_id: str = Depends(current_user),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    _valid_lease(body.robot_id, body.session_id, user_id)
    version = _active_version(database, body.map_id, body.version)
    if version is None:
        raise HTTPException(status_code=409, detail="Map/version chưa ACTIVE")
    if not _goal_in_map(version, body.pose):
        raise HTTPException(status_code=422, detail="Khu vực gợi ý nằm ngoài bản đồ")
    return await _command(
        database,
        settings,
        request_id=body.request_id,
        robot_id=body.robot_id,
        command_type="map.set_initial_pose",
        expected_state=body.expected_state,
        payload={
            "map_id": body.map_id,
            "version": body.version,
            # This endpoint conveys only a broad search center. The edge also
            # discards yaw and cannot become READY without strict LiDAR and
            # particle-cloud uniqueness verification.
            "pose": {"x": body.pose.x, "y": body.pose.y, "yaw": 0.0},
        },
        timeout_seconds=settings.mapping_command_timeout_seconds,
    )


@router.post("/map/relocalize")
async def relocalize(
    body: RelocalizeRequest,
    user_id: str = Depends(current_user),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    _valid_lease(body.robot_id, body.session_id, user_id)
    if _active_version(database, body.map_id, body.version) is None:
        raise HTTPException(status_code=409, detail="Map/version chưa ACTIVE")
    return await _command(
        database,
        settings,
        request_id=body.request_id,
        robot_id=body.robot_id,
        command_type="map.relocalize",
        expected_state=body.expected_state,
        payload={
            "map_id": body.map_id,
            "version": body.version,
            "allow_rotation": body.allow_rotation,
            "force_global": body.force_global,
        },
        # Relocalization may first perform the guarded MAPPING -> NAVIGATION
        # container handoff. Give that bounded switch the same ACK budget as
        # map loading instead of returning a false 504 after 15 seconds.
        timeout_seconds=settings.mapping_command_timeout_seconds,
    )


@router.get("/health/{robot_id}")
async def navigation_health(
    robot_id: str,
    _: str = Depends(current_user),
) -> dict:
    robot = hub.robots.get(robot_id)
    if robot is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    health = dict(robot.health)
    mode = str(health.get("mode") or (
        "MAPPING" if str(health.get("map_state", "")).startswith("MAPPING_") else "NAVIGATION"
    ))
    return {
        "mode": mode,
        "mapping": health.get("mapping") if mode == "MAPPING" else None,
        "navigation": {
            "state": health.get("map_state", "NO_ACTIVE_MAP"),
            "mapId": robot.map_id,
            "mapVersion": health.get("map_version", 0),
            "localized": bool(health.get("localized")),
            "localizationState": health.get("localization_state", "IDLE"),
            "localizationConfidence": float(health.get("localization_confidence", 0)),
            "localizationDiagnostics": health.get("localization_diagnostics", {}),
            "nav2Healthy": health.get("nav2") == "READY",
            "corridor": health.get("corridor"),
            "routeCandidates": health.get("route_candidates", []),
            "selectedRouteId": health.get("selected_route_id"),
            "manualHandoffReason": health.get("manual_handoff_reason"),
            "trajectory": health.get("trajectory", []),
        } if mode == "NAVIGATION" else None,
        "mapRegistry": health.get("map_registry", {"localCount": 0, "pendingSync": 0}),
    }


@router.get("/speed-mode/{robot_id}")
async def auto_navigation_speed_mode(
    robot_id: str,
    session_id: str,
    user_id: str = Depends(current_user),
) -> dict:
    _valid_lease(robot_id, session_id, user_id)
    runtime = hub.robots.get(robot_id)
    if runtime is None or runtime.status != "online":
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến")
    mode = str(runtime.health.get("auto_speed_mode") or "NORMAL").upper()
    if mode not in {"SLOW", "NORMAL", "FAST"}:
        mode = "NORMAL"
    return {
        "mode": mode,
        "profile": runtime.health.get("auto_speed_profile"),
    }


@router.post("/speed-mode")
async def set_auto_navigation_speed_mode(
    body: SpeedModeRequest,
    user_id: str = Depends(current_user),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    _valid_lease(body.robot_id, body.session_id, user_id)
    result = await _command(
        database,
        settings,
        request_id=body.request_id,
        robot_id=body.robot_id,
        command_type="navigation.speed_mode",
        expected_state=body.expected_state,
        payload={"mode": body.mode},
    )
    return {
        **result,
        "mode": str(result.get("mode") or body.mode).upper(),
    }


# Compatibility route for older clients. The handler deliberately rejects
# operator-supplied coordinates; a map click must never become robot pose.
router.add_api_route(
    "/map/initial-pose",
    set_initial_pose,
    methods=["POST"],
    include_in_schema=False,
)


@router.post("/compute-path")
async def compute_path(
    body: ComputePathRequest,
    user_id: str = Depends(current_user),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    _valid_lease(body.robot_id, body.session_id, user_id)
    version = _active_version(database, body.map_id, body.version)
    if version is None:
        raise HTTPException(status_code=409, detail="Map/version chưa ACTIVE")
    robot = _robot_by_public_id(database, body.robot_id)
    if robot is None or robot.map_id != body.map_id or robot.active_map_version != body.version:
        raise HTTPException(status_code=409, detail="Map hiển thị không khớp active map của robot")
    if not _goal_in_map(version, body.goal):
        raise HTTPException(status_code=422, detail="Điểm đến nằm ngoài bản đồ.")
    existing = database.scalar(
        select(NavigationMission).where(NavigationMission.request_id == body.request_id)
    )
    if existing is not None:
        return _mission_view(
            existing,
            candidates=_mission_route_candidates(database, existing),
        )
    result = await _command(
        database,
        settings,
        request_id=body.request_id,
        robot_id=body.robot_id,
        command_type="navigation.compute_path",
        expected_state=body.expected_state,
        payload={
            "map_id": body.map_id,
            "version": body.version,
            "goal": body.goal.model_dump(),
        },
        timeout_seconds=settings.navigation_planning_timeout_seconds,
    )
    points = list(result.get("points") or result.get("path") or [])
    # Planning happens before the chassis moves. A failed route is mission
    # status PLAN_FAILED while the robot runtime remains READY; BLOCKED is
    # reserved for an active navigation that exhausted recovery.
    status = (
        "READY"
        if result.get("status") in {"accepted", "completed"} and points
        else "PLAN_FAILED"
    )
    resolved_goal = dict(result.get("goal") or body.goal.model_dump())
    mission = NavigationMission(
        request_id=body.request_id,
        robot_id=body.robot_id,
        control_session_id=body.session_id,
        map_id=body.map_id,
        map_version=body.version,
        status=status,
        # Persist the bounded, safe snapped goal. navigation.start must send
        # exactly the destination that Nav2 successfully planned.
        goal=resolved_goal,
        path=points,
        distance_m=float(result.get("distance_m", 0)),
        error_code=result.get("error_code"),
        error_message=result.get("error_message"),
    )
    database.add(mission)
    database.commit()
    database.refresh(mission)
    return _mission_view(
        mission,
        candidates=_normalized_route_candidates(
            result.get("route_candidates") or result.get("candidates")
        ),
    )


@router.post("/start")
async def start_navigation(
    body: MissionCommandRequest,
    user_id: str = Depends(current_user),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    _valid_lease(body.robot_id, body.session_id, user_id)
    mission = database.get(NavigationMission, body.mission_id)
    if mission is None or mission.robot_id != body.robot_id:
        raise HTTPException(status_code=404, detail="Không tìm thấy mission")
    rejection = _mission_start_rejection(mission)
    if rejection is not None:
        raise HTTPException(status_code=409, detail=rejection)
    robot = _robot_by_public_id(database, body.robot_id)
    if robot is None or robot.map_id != mission.map_id or robot.active_map_version != mission.map_version:
        raise HTTPException(status_code=409, detail="Map mission không còn là active map của robot")
    failures = _navigation_preflight(body.robot_id)
    if failures:
        raise HTTPException(
            status_code=409,
            detail={
                "code": "PREFLIGHT_FAILED",
                "message": "Robot chưa đủ điều kiện an toàn để bắt đầu tự hành.",
                "failures": failures,
            },
        )
    candidates = _mission_route_candidates(database, mission)
    try:
        selected_route_id, selected_points = _selected_candidate(
            candidates,
            body.route_id,
            list(mission.path or []),
        )
    except ValueError as exc:
        raise HTTPException(
            status_code=409,
            detail="Tuyến đã chọn không thuộc kết quả preview hiện tại",
        ) from exc
    mission.path = selected_points
    mission.distance_m = sum(
        math.hypot(
            float(right["x"]) - float(left["x"]),
            float(right["y"]) - float(left["y"]),
        )
        for left, right in zip(selected_points, selected_points[1:])
    )
    try:
        navigation_transition(mission.status, "NAVIGATING")
    except InvalidTransition as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    result = await _command(
        database,
        settings,
        request_id=body.request_id,
        robot_id=body.robot_id,
        command_type="navigation.start",
        expected_state=body.expected_state,
        payload={
            "mission_id": mission.mission_id,
            "route_id": selected_route_id,
            "map_id": mission.map_id,
            "version": mission.map_version,
            "goal": mission.goal,
            # FollowPath executes the exact route that the user previewed.
            "points": mission.path,
        },
    )
    if result.get("status") in {"accepted", "completed"}:
        mission.status = "NAVIGATING"
        mission.error_code = None
        mission.error_message = None
    else:
        mission.status = "FAULT"
        mission.error_code = result.get("error_code")
        mission.error_message = result.get("error_message")
    database.commit()
    if result.get("status") not in {"accepted", "completed"}:
        # A robot-level rejection is an API failure. Returning HTTP 200 here
        # made the browser enter its "moving" state even though the adapter
        # remained READY and never published an autonomous velocity command.
        raise HTTPException(
            status_code=409,
            detail={
                "code": str(result.get("error_code") or "NAVIGATION_START_REJECTED"),
                "message": str(
                    result.get("error_message")
                    or "Robot từ chối bắt đầu hành trình."
                ),
                "status": str(result.get("status") or "rejected"),
            },
        )
    return _mission_view(
        mission,
        candidates=candidates,
        selected_route_id=selected_route_id,
    )


@router.post("/missions/{mission_id}/{action}")
async def mission_action(
    mission_id: str,
    action: str,
    body: MissionCommandRequest,
    user_id: str = Depends(current_user),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    actions = {
        "pause": ("navigation.pause", "PAUSED"),
        "resume": ("navigation.resume", "NAVIGATING"),
        "cancel": ("navigation.cancel", "CANCELED"),
        "manual": ("navigation.manual_handoff", "MANUAL_BYPASS"),
        "alternatives": ("navigation.alternatives", "COMPUTING_ALTERNATIVES"),
        "select-route": ("navigation.select_route", "NAVIGATING"),
        "back": ("navigation.route_selection_back", "NARROW_PATH_DECISION"),
    }
    selected = actions.get(action)
    if selected is None:
        raise HTTPException(status_code=404, detail="Thao tác navigation không hợp lệ")
    _valid_lease(body.robot_id, body.session_id, user_id)
    if body.mission_id != mission_id:
        raise HTTPException(status_code=409, detail="mission_id không khớp đường dẫn")
    mission = database.get(NavigationMission, mission_id)
    if mission is None or mission.robot_id != body.robot_id:
        raise HTTPException(status_code=404, detail="Không tìm thấy mission")
    command_type, target = selected
    if action == "select-route" and not body.route_id:
        raise HTTPException(status_code=422, detail="Chưa chọn tuyến đường")
    robot = _robot_by_public_id(database, body.robot_id)
    if action in {"pause", "resume", "manual", "alternatives", "select-route", "back"} and (
        robot is None
        or robot.map_id != mission.map_id
        or robot.active_map_version != mission.map_version
    ):
        raise HTTPException(status_code=409, detail="Map mission không còn là active map của robot")
    if database.get(CommandReceipt, body.request_id) is None:
        try:
            navigation_transition(mission.status, target)
        except InvalidTransition as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    result = await _command(
        database,
        settings,
        request_id=body.request_id,
        robot_id=body.robot_id,
        command_type=command_type,
        expected_state=body.expected_state,
        payload={
            "mission_id": mission.mission_id,
            "map_id": mission.map_id,
            "version": mission.map_version,
            "route_id": body.route_id,
            "reason": "NARROW_PATH",
        },
        timeout_seconds=(
            settings.mapping_command_timeout_seconds
            if action == "alternatives"
            else None
        ),
    )
    if result.get("status") in {"accepted", "completed"}:
        mission.status = str(result.get("current_state") or target).upper()
        points = list(result.get("points") or [])
        if points:
            mission.path = points
            mission.distance_m = sum(
                math.hypot(
                    float(right["x"]) - float(left["x"]),
                    float(right["y"]) - float(left["y"]),
                )
                for left, right in zip(points, points[1:])
            )
    database.commit()
    return {
        **_mission_view(mission),
        "candidates": list(result.get("candidates") or []),
        "destination_preserved": bool(result.get("destination_preserved", True)),
        "selected_route_id": result.get("route_id") or body.route_id,
    }


@router.get("/missions/{mission_id}")
async def mission_status(
    mission_id: str,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    mission = database.get(NavigationMission, mission_id)
    if mission is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy mission")
    return _mission_view(
        mission,
        candidates=_mission_route_candidates(database, mission),
    )


# Backward-compatible endpoints used by the current Center UI and simulator.
@router.post("/preview")
async def preview(
    body: NavigationPreviewRequest,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    robot = hub.robots.get(body.robot_id)
    destination = database.get(Destination, body.destination_id)
    if destination is None:
        destination = database.get(POI, body.destination_id)
    if robot is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    if destination is None or destination.map_id != robot.map_id:
        raise HTTPException(status_code=400, detail="Điểm đến không hợp lệ với bản đồ")
    # Simulator preview remains deterministic. Real robots use /compute-path,
    # which is backed by Nav2 ComputePathToPose and never by this fallback.
    return hub.create_route(
        body.robot_id,
        {
            "destination_id": body.destination_id,
            "x": destination.x,
            "y": destination.y,
        },
    )


@router.post("/goal")
async def goal(
    body: NavigationGoalRequest,
    user_id: str = Depends(current_user),
) -> dict:
    _valid_lease(body.robot_id, body.session_id, user_id)
    route = hub.routes.get(body.route_id)
    if route is None or route["robot_id"] != body.robot_id:
        raise HTTPException(status_code=404, detail="Không tìm thấy tuyến đường")
    message = {
        "message_id": str(uuid4()),
        "schema_version": "1.0",
        "message_type": "navigation.goal",
        "robot_id": body.robot_id,
        "session_id": body.session_id,
        "sequence": 0,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "ttl_ms": 5000,
        "payload": route,
    }
    if not await hub.forward_to_robot(body.robot_id, message):
        raise HTTPException(status_code=409, detail="Robot đang ngoại tuyến")
    return {"status": "accepted", **route}


@router.post("/cancel")
async def cancel_legacy(
    body: NavigationCancelRequest,
    user_id: str = Depends(current_user),
) -> dict:
    _valid_lease(body.robot_id, body.session_id, user_id)
    await hub.forward_to_robot(
        body.robot_id,
        {
            "message_id": str(uuid4()),
            "schema_version": "1.0",
            "message_type": "navigation.cancel",
            "robot_id": body.robot_id,
            "session_id": body.session_id,
            "sequence": 0,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "ttl_ms": 1000,
            "payload": {"reason": "user_cancelled"},
        },
    )
    return {"status": "cancelled"}
