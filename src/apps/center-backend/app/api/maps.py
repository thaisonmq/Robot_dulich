from __future__ import annotations

import io
import json
import re
import tarfile
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

from fastapi import (
    APIRouter,
    Depends,
    File,
    Form,
    HTTPException,
    Query,
    UploadFile,
)
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field
from sqlalchemy import delete, func, select
from sqlalchemy.orm import Session

from app.core.config import Settings, get_settings
from app.core.security import current_user, operator_user_id, user_or_robot
from app.models.database import get_db
from app.models.entities import (
    CommandReceipt,
    Destination,
    KeepoutZone,
    MapRecord,
    MappingSession,
    MapVersion,
    NavigationMission,
    NavigationRoute,
    POI,
    Robot,
    RobotMapCache,
    SpeedZone,
)
from app.services.hub import hub
from app.services.map_storage import InvalidMapBundle, MapBundleStore
from app.services.state_machines import InvalidTransition, mapping_transition

router = APIRouter(prefix="/api/maps", tags=["maps"])
SHA256_PATTERN = re.compile(r"^[0-9a-fA-F]{64}$")


class MapCreateRequest(BaseModel):
    name: str = Field(min_length=2, max_length=120)
    site_id: str = Field(min_length=1, max_length=64)
    floor_id: str = Field(min_length=1, max_length=64)
    notes: str = Field(default="", max_length=2000)


class MapUpdateRequest(BaseModel):
    name: str | None = Field(default=None, min_length=2, max_length=120)
    site_id: str | None = Field(default=None, min_length=1, max_length=64)
    floor_id: str | None = Field(default=None, min_length=1, max_length=64)
    notes: str | None = Field(default=None, max_length=2000)


class MappingStartRequest(MapCreateRequest):
    request_id: str = Field(min_length=8, max_length=64)
    robot_id: str = Field(min_length=3, max_length=64)
    expected_state: str = Field(default="IDLE", max_length=32)
    map_id: str | None = Field(default=None, min_length=3, max_length=64)
    source_version: int | None = Field(default=None, ge=1)


class MappingCommandRequest(BaseModel):
    request_id: str = Field(min_length=8, max_length=64)
    expected_state: str = Field(max_length=32)


class MapLifecycleRequest(BaseModel):
    version: int = Field(ge=1)


def _posegraph_basename(version: MapVersion) -> str | None:
    files = set(dict(version.metadata_json or {}).get("files") or {})
    for name in files:
        if name.endswith(".posegraph") and f"{name[:-10]}.data" in files:
            return name[:-10]
    return None


def _version_view(version: MapVersion) -> dict:
    return {
        "version": version.version,
        "status": version.status,
        "checksum": version.checksum,
        "resolution": version.resolution,
        "origin": version.origin,
        "width_pixels": version.width,
        "height_pixels": version.height,
        "created_by_robot": version.created_by_robot,
        "created_at": version.created_at.isoformat(),
        "download_url": f"/api/maps/{version.map_id}/versions/{version.version}/download",
        "preview_url": f"/api/maps/{version.map_id}/versions/{version.version}/preview",
        "can_continue": _posegraph_basename(version) is not None,
    }


def _map_view(database: Session, map_record: MapRecord, *, details: bool = False) -> dict:
    active = None
    if map_record.active_version is not None:
        active = database.scalar(
            select(MapVersion).where(
                MapVersion.map_id == map_record.map_id,
                MapVersion.version == map_record.active_version,
            )
        )
    value = {
        "map_id": map_record.map_id,
        "name": map_record.name,
        "site_id": map_record.site_id,
        "floor_id": map_record.floor_id,
        "notes": map_record.notes,
        "status": map_record.status,
        "active_version": map_record.active_version,
        "image_url": active and f"/api/maps/{map_record.map_id}/versions/{active.version}/preview" or map_record.image_url,
        "width_pixels": active.width if active else map_record.width_pixels,
        "height_pixels": active.height if active else map_record.height_pixels,
        "resolution_m_per_pixel": active.resolution if active else map_record.resolution_m_per_pixel,
        "origin": active.origin if active else map_record.origin,
        "checksum": active.checksum if active else "",
        "created_at": map_record.created_at.isoformat(),
        "updated_at": map_record.updated_at.isoformat(),
    }
    if details:
        versions = database.scalars(
            select(MapVersion)
            .where(MapVersion.map_id == map_record.map_id)
            .order_by(MapVersion.version.desc())
        ).all()
        value["versions"] = [_version_view(item) for item in versions]
        active_mapping = database.scalar(
            select(MappingSession)
            .where(
                MappingSession.map_id == map_record.map_id,
                MappingSession.status.not_in(("FINISHED", "CANCELED", "FAULT")),
            )
            .order_by(MappingSession.created_at.desc())
        )
        value["mapping_session"] = _session_view(active_mapping) if active_mapping else None
        value["pois"] = [
            {
                "destination_id": item.poi_id,
                "map_id": item.map_id,
                "name": item.name,
                "x": item.x,
                "y": item.y,
                "yaw": item.yaw,
                "enabled": item.enabled,
            }
            for item in database.scalars(
                select(POI).where(POI.map_id == map_record.map_id, POI.enabled.is_(True))
            )
        ]
        value["keepout_zones"] = [
            {"zone_id": item.zone_id, "name": item.name, "points": item.points}
            for item in database.scalars(
                select(KeepoutZone).where(KeepoutZone.map_id == map_record.map_id)
            )
        ]
        value["speed_zones"] = [
            {
                "zone_id": item.zone_id,
                "name": item.name,
                "points": item.points,
                "max_speed_mps": item.max_speed_mps,
            }
            for item in database.scalars(
                select(SpeedZone).where(SpeedZone.map_id == map_record.map_id)
            )
        ]
    return value


def _session_view(session: MappingSession) -> dict:
    return {
        "session_id": session.session_id,
        "map_id": session.map_id,
        "version": session.version,
        "robot_id": session.robot_id,
        "status": session.status,
        "metadata": session.metadata_json,
        "error_code": session.error_code,
        "error_message": session.error_message,
        "created_at": session.created_at.isoformat(),
        "updated_at": session.updated_at.isoformat(),
    }


async def _dispatch_idempotent(
    database: Session,
    *,
    request_id: str,
    robot_id: str,
    command_type: str,
    expected_state: str,
    payload: dict,
    timeout_seconds: float,
) -> dict:
    existing = database.get(CommandReceipt, request_id)
    if existing is not None:
        if existing.robot_id != robot_id or existing.command_type != command_type:
            raise HTTPException(status_code=409, detail="request_id đã được dùng cho lệnh khác")
        return existing.response
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
        response = await hub.request_robot(
            robot_id,
            command_type,
            {**payload, "expected_state": expected_state},
            timeout_seconds=timeout_seconds,
            request_id=request_id,
        )
    except ConnectionError as exc:
        response = {
            "request_id": request_id,
            "status": "rejected",
            "current_state": expected_state,
            "error_code": "ROBOT_OFFLINE",
            "error_message": "Robot đang ngoại tuyến",
        }
        receipt.status = "REJECTED"
        receipt.response = response
        database.commit()
        raise HTTPException(status_code=409, detail=response["error_message"]) from exc
    except TimeoutError as exc:
        response = {
            "request_id": request_id,
            "status": "rejected",
            "current_state": expected_state,
            "error_code": "ROBOT_TIMEOUT",
            "error_message": "Robot không ACK lệnh đúng hạn",
        }
        receipt.status = "REJECTED"
        receipt.response = response
        database.commit()
        raise HTTPException(status_code=504, detail=response["error_message"]) from exc
    receipt.status = str(response.get("status", "accepted")).upper()
    receipt.response = response
    database.commit()
    return response


@router.get("")
async def list_maps(
    status: str | None = Query(default=None),
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> list[dict]:
    statement = select(MapRecord).order_by(MapRecord.updated_at.desc())
    if status:
        statement = statement.where(MapRecord.status == status.upper())
    return [_map_view(database, item) for item in database.scalars(statement)]


@router.post("", status_code=201)
async def create_map(
    body: MapCreateRequest,
    _: str = Depends(operator_user_id),
    database: Session = Depends(get_db),
) -> dict:
    map_id = f"MAP-{uuid4().hex[:12].upper()}"
    record = MapRecord(
        map_id=map_id,
        name=body.name.strip(),
        site_id=body.site_id.strip(),
        floor_id=body.floor_id.strip(),
        notes=body.notes.strip(),
        status="DRAFT",
        image_url="",
        width_pixels=0,
        height_pixels=0,
        resolution_m_per_pixel=0.05,
        origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
    )
    database.add(record)
    database.commit()
    database.refresh(record)
    return _map_view(database, record)


@router.patch("/{map_id}")
async def update_map(
    map_id: str,
    body: MapUpdateRequest,
    _: str = Depends(operator_user_id),
    database: Session = Depends(get_db),
) -> dict:
    record = database.get(MapRecord, map_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy map")
    updates = body.model_dump(exclude_unset=True)
    if not updates:
        raise HTTPException(status_code=422, detail="Không có nội dung cần cập nhật")
    for field, value in updates.items():
        setattr(record, field, value.strip() if isinstance(value, str) else value)
    database.commit()
    database.refresh(record)
    return _map_view(database, record, details=True)


@router.delete("/{map_id}", status_code=204)
async def delete_map(
    map_id: str,
    _: str = Depends(operator_user_id),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> Response:
    record = database.get(MapRecord, map_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy map")
    if record.status == "ACTIVE" or record.active_version is not None:
        raise HTTPException(
            status_code=409,
            detail="Không thể xóa map ACTIVE; hãy chuyển robot sang map khác và Archive trước",
        )
    if database.scalar(select(Robot.robot_id).where(Robot.map_id == map_id)):
        raise HTTPException(status_code=409, detail="Map đang được gán cho robot")
    active_mapping = database.scalar(
        select(MappingSession.session_id).where(
            MappingSession.map_id == map_id,
            MappingSession.status.not_in(("FINISHED", "CANCELED", "FAULT")),
        )
    )
    if active_mapping:
        raise HTTPException(status_code=409, detail="Map đang có phiên mapping chưa kết thúc")
    active_mission = database.scalar(
        select(NavigationMission.mission_id).where(
            NavigationMission.map_id == map_id,
            NavigationMission.status.not_in(("ARRIVED", "CANCELED", "FAULT")),
        )
    )
    if active_mission:
        raise HTTPException(status_code=409, detail="Map đang có nhiệm vụ navigation chưa kết thúc")

    destination_ids = list(
        database.scalars(select(Destination.destination_id).where(Destination.map_id == map_id))
    )
    if destination_ids:
        database.execute(
            delete(NavigationRoute).where(NavigationRoute.destination_id.in_(destination_ids))
        )
    for entity in (
        NavigationMission,
        Destination,
        POI,
        KeepoutZone,
        SpeedZone,
        RobotMapCache,
        MappingSession,
        MapVersion,
    ):
        database.execute(delete(entity).where(entity.map_id == map_id))
    database.delete(record)
    database.commit()
    MapBundleStore(settings.map_storage_dir, settings.map_bundle_max_bytes).delete_map(map_id)
    return Response(status_code=204)


@router.post("/mapping-sessions", status_code=201)
async def start_mapping(
    body: MappingStartRequest,
    user_id: str = Depends(operator_user_id),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    duplicate = database.scalar(
        select(MappingSession).where(MappingSession.last_request_id == body.request_id)
    )
    if duplicate is not None:
        return _session_view(duplicate)
    robot = database.scalar(select(Robot).where(Robot.robot_id == body.robot_id))
    if robot is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy robot")
    runtime = hub.robots.get(body.robot_id)
    if runtime is None or runtime.status != "online":
        raise HTTPException(status_code=409, detail="Robot đang offline")
    if runtime.capabilities.get("mapping") is not True:
        blockers = runtime.capabilities.get("mapping_blockers") or ["ROS_NOT_READY"]
        raise HTTPException(
            status_code=409,
            detail="Robot chưa sẵn sàng mapping: " + ", ".join(map(str, blockers)),
        )
    active_session = database.scalar(
        select(MappingSession).where(
            MappingSession.robot_id == body.robot_id,
            MappingSession.status.not_in(("FINISHED", "CANCELED", "FAULT")),
        )
    )
    if active_session is not None:
        raise HTTPException(status_code=409, detail="Robot đang có phiên mapping chưa kết thúc")
    continuation: dict = {}
    if body.map_id:
        record = database.get(MapRecord, body.map_id)
        if record is None:
            raise HTTPException(status_code=404, detail="Không tìm thấy map cần tiếp tục")
        active_map_session = database.scalar(
            select(MappingSession).where(
                MappingSession.map_id == body.map_id,
                MappingSession.status.not_in(("FINISHED", "CANCELED", "FAULT")),
            )
        )
        if active_map_session is not None:
            raise HTTPException(
                status_code=409,
                detail=f"Map đang có phiên mapping chưa kết thúc: {active_map_session.session_id}",
            )
        source_statement = select(MapVersion).where(MapVersion.map_id == body.map_id)
        if body.source_version is not None:
            source_statement = source_statement.where(MapVersion.version == body.source_version)
        source = database.scalar(source_statement.order_by(MapVersion.version.desc()))
        if source is None:
            raise HTTPException(status_code=409, detail="Map chưa có version đã lưu để tiếp tục")
        posegraph_basename = _posegraph_basename(source)
        if posegraph_basename is None:
            raise HTTPException(status_code=409, detail="Version map không có pose-graph để tiếp tục SLAM")
        # Only immutable uploaded versions reserve a version number. A canceled
        # or faulted session may have advanced its in-memory draft counter but
        # did not create an artifact, so that number is safe to reuse.
        latest_version = (
            database.scalar(select(func.max(MapVersion.version)).where(MapVersion.map_id == body.map_id))
            or 0
        )
        map_id = record.map_id
        version = latest_version + 1
        metadata = {
            "name": record.name,
            "site_id": record.site_id,
            "floor_id": record.floor_id,
            "notes": record.notes,
            "continued_from_version": source.version,
        }
        continuation = {
            "source_version": source.version,
            "source_checksum": source.checksum,
            "source_download_url": f"/api/maps/{source.map_id}/versions/{source.version}/download",
            "posegraph_basename": posegraph_basename,
            "initial_pose": dict(source.metadata_json or {}).get("terminal_pose"),
        }
    else:
        map_id = f"MAP-{uuid4().hex[:12].upper()}"
        version = 1
        metadata = {
            "name": body.name,
            "site_id": body.site_id,
            "floor_id": body.floor_id,
            "notes": body.notes,
        }
        record = MapRecord(
            map_id=map_id,
            name=body.name.strip(),
            site_id=body.site_id.strip(),
            floor_id=body.floor_id.strip(),
            notes=body.notes.strip(),
            status="DRAFT",
            image_url="",
            width_pixels=0,
            height_pixels=0,
            resolution_m_per_pixel=0.05,
            origin={"x": 0.0, "y": 0.0, "yaw": 0.0},
        )
    mapping = MappingSession(
        map_id=map_id,
        version=version,
        robot_id=body.robot_id,
        user_id=user_id,
        status="STARTING",
        last_request_id=body.request_id,
        metadata_json=metadata,
    )
    if body.map_id:
        database.add(mapping)
    else:
        database.add_all((record, mapping))
    database.commit()
    try:
        response = await _dispatch_idempotent(
            database,
            request_id=body.request_id,
            robot_id=body.robot_id,
            command_type="mapping.start",
            expected_state=body.expected_state,
            payload={
                "mapping_session_id": mapping.session_id,
                "map_id": map_id,
                "version": version,
                "metadata": mapping.metadata_json,
                **continuation,
            },
            timeout_seconds=settings.mapping_command_timeout_seconds,
        )
    except HTTPException:
        receipt = database.get(CommandReceipt, body.request_id)
        receipt_response = dict(receipt.response or {}) if receipt else {}
        mapping.status = "FAULT"
        mapping.error_code = str(receipt_response.get("error_code") or "MAPPING_START_FAILED")
        mapping.error_message = str(
            receipt_response.get("error_message") or "Không thể khởi động phiên mapping"
        )
        database.commit()
        raise
    if response.get("status") in {"accepted", "completed"}:
        mapping.status = "MAPPING"
    else:
        mapping.status = "FAULT"
        mapping.error_code = str(response.get("error_code", "REJECTED"))
        mapping.error_message = str(response.get("error_message", "Robot từ chối lệnh"))
    database.commit()
    return _session_view(mapping)


@router.get("/mapping-sessions/{mapping_session_id}")
async def mapping_session(
    mapping_session_id: str,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    session = database.get(MappingSession, mapping_session_id)
    if session is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy phiên mapping")
    return _session_view(session)


_MAPPING_ACTIONS = {
    "pause": ("mapping.pause", "PAUSED"),
    "resume": ("mapping.resume", "MAPPING"),
    # Save Draft persists an immutable version but does not pause/finish SLAM.
    "save-draft": ("mapping.save_draft", None),
    "finish": ("mapping.finish", "FINISHED"),
    "cancel": ("mapping.cancel", "CANCELED"),
}


def _mapping_action_reached(response: dict, action: str, target: str) -> bool:
    if response.get("status") in {"accepted", "completed"}:
        return True
    return (
        action != "save-draft"
        and str(response.get("current_state", "")).upper() == target
    )


@router.post("/mapping-sessions/{mapping_session_id}/{action}")
async def mapping_action(
    mapping_session_id: str,
    action: str,
    body: MappingCommandRequest,
    _: str = Depends(operator_user_id),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    session = database.get(MappingSession, mapping_session_id)
    if session is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy phiên mapping")
    command = _MAPPING_ACTIONS.get(action)
    if command is None:
        raise HTTPException(status_code=404, detail="Thao tác mapping không hợp lệ")
    command_type, configured_target = command
    target = session.status if action == "save-draft" else configured_target
    assert target is not None
    existing = database.get(CommandReceipt, body.request_id)
    if existing is None:
        try:
            if action == "save-draft":
                if session.status not in {"MAPPING", "PAUSED"}:
                    raise InvalidTransition(
                        f"invalid transition {session.status} -> SAVING"
                    )
            elif action == "finish":
                mapping_transition(session.status, "FINISHING")
                mapping_transition("FINISHING", "FINISHED")
            else:
                mapping_transition(session.status, target)
        except InvalidTransition as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc
    response = await _dispatch_idempotent(
        database,
        request_id=body.request_id,
        robot_id=session.robot_id,
        command_type=command_type,
        expected_state=body.expected_state,
        payload={
            "mapping_session_id": session.session_id,
            "map_id": session.map_id,
            "version": session.version,
        },
        timeout_seconds=(
            settings.mapping_command_timeout_seconds
            if action in {"save-draft", "finish"}
            else settings.robot_command_timeout_seconds
        ),
    )
    robot_state = str(response.get("current_state", "")).upper()
    accepted = response.get("status") in {"accepted", "completed"}
    # A retry can arrive after the robot already reached the requested state
    # (for example Cancel ACK reached the edge but the browser retried). Treat
    # that terminal state as idempotent success instead of leaving Center in a
    # zombie MAPPING state.
    if _mapping_action_reached(response, action, target):
        session.status = target
        session.last_request_id = body.request_id
        session.error_code = None
        session.error_message = None
        if action == "save-draft" and existing is None and accepted:
            # The saved version is immutable; continue mapping into a new one.
            session.version += 1
        if target == "FINISHED":
            record = database.get(MapRecord, session.map_id)
            if record:
                record.status = "VALIDATING"
    else:
        if robot_state in {"MAPPING", "PAUSED", "FINISHED", "CANCELED", "FAULT"}:
            # The robot runtime is authoritative. Exposing its actual state
            # lets the UI recover instead of presenting buttons that can only
            # keep producing STATE_CONFLICT.
            session.status = robot_state
        session.error_code = str(response.get("error_code", "REJECTED"))
        session.error_message = str(response.get("error_message", "Robot từ chối lệnh"))
    database.commit()
    return _session_view(session)


@router.post("/{map_id}/versions", status_code=201)
async def upload_version(
    map_id: str,
    version: int = Form(ge=1),
    robot_id: str = Form(min_length=3, max_length=64),
    checksum: str = Form(min_length=64, max_length=64),
    bundle: UploadFile = File(),
    principal: tuple[str, str] = Depends(user_or_robot),
    database: Session = Depends(get_db),
    settings: Settings = Depends(get_settings),
) -> dict:
    if principal[0] == "robot" and principal[1] != robot_id:
        raise HTTPException(status_code=403, detail="Robot không được upload thay robot khác")
    if not SHA256_PATTERN.fullmatch(checksum):
        raise HTTPException(status_code=422, detail="Checksum SHA-256 không hợp lệ")
    record = database.get(MapRecord, map_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy bản đồ")
    existing = database.scalar(
        select(MapVersion).where(MapVersion.map_id == map_id, MapVersion.version == version)
    )
    if existing is not None:
        if existing.checksum == checksum.lower():
            return _version_view(existing)
        raise HTTPException(status_code=409, detail="Version đã tồn tại và không được ghi đè")
    store = MapBundleStore(settings.map_storage_dir, settings.map_bundle_max_bytes)
    try:
        destination, verified_checksum, metadata = store.save(
            bundle.file,
            map_id=map_id,
            version=version,
            expected_checksum=checksum,
        )
        resolution = float(metadata["resolution"])
        origin = dict(metadata["origin"])
        width = int(metadata["width"])
        height = int(metadata["height"])
        if metadata.get("map_id") not in {None, map_id} or int(metadata.get("version", version)) != version:
            raise InvalidMapBundle("metadata map/version does not match upload target")
        if resolution <= 0 or width <= 0 or height <= 0:
            raise InvalidMapBundle("invalid map dimensions")
    except (InvalidMapBundle, KeyError, TypeError, ValueError) as exc:
        raise HTTPException(status_code=422, detail=str(exc)) from exc
    map_version = MapVersion(
        map_id=map_id,
        version=version,
        status="VALIDATING",
        checksum=verified_checksum,
        storage_path=str(destination),
        resolution=resolution,
        origin=origin,
        width=width,
        height=height,
        metadata_json=metadata,
        created_by_robot=robot_id,
    )
    database.add(map_version)
    record.width_pixels = width
    record.height_pixels = height
    record.resolution_m_per_pixel = resolution
    record.origin = origin
    record.status = "VALIDATING"
    database.commit()
    database.refresh(map_version)
    return _version_view(map_version)


@router.get("/{map_id}/versions/{version}/download")
async def download_version(
    map_id: str,
    version: int,
    _: tuple[str, str] = Depends(user_or_robot),
    database: Session = Depends(get_db),
) -> FileResponse:
    item = database.scalar(
        select(MapVersion).where(MapVersion.map_id == map_id, MapVersion.version == version)
    )
    if item is None or not Path(item.storage_path).is_file():
        raise HTTPException(status_code=404, detail="Không tìm thấy bundle bản đồ")
    return FileResponse(
        item.storage_path,
        media_type="application/gzip",
        filename=f"{map_id}-v{version}.tar.gz",
        headers={"ETag": f'"{item.checksum}"', "X-Map-SHA256": item.checksum},
    )


@router.get("/{map_id}/versions/{version}/preview")
async def version_preview(
    map_id: str,
    version: int,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> Response:
    item = database.scalar(
        select(MapVersion).where(MapVersion.map_id == map_id, MapVersion.version == version)
    )
    if item is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy version bản đồ")
    try:
        with tarfile.open(item.storage_path, "r:*") as archive:
            member = next(
                (
                    candidate for candidate in archive.getmembers()
                    if candidate.isfile()
                    and candidate.name.lstrip("./") in {"preview.png", "preview.webp", "map.png", "map.pgm"}
                ),
                None,
            )
            if member is None:
                raise HTTPException(status_code=404, detail="Bundle chưa có preview")
            source = archive.extractfile(member)
            data = source.read() if source else b""
    except (OSError, tarfile.TarError) as exc:
        raise HTTPException(status_code=500, detail="Không đọc được preview") from exc
    suffix = Path(member.name).suffix.lower()
    media_type = {".png": "image/png", ".webp": "image/webp", ".pgm": "image/x-portable-graymap"}.get(suffix, "application/octet-stream")
    return Response(data, media_type=media_type, headers={"Cache-Control": "private, max-age=60"})


@router.post("/{map_id}/activate")
async def activate_map(
    map_id: str,
    body: MapLifecycleRequest,
    _: str = Depends(operator_user_id),
    database: Session = Depends(get_db),
) -> dict:
    record = database.get(MapRecord, map_id)
    version = database.scalar(
        select(MapVersion).where(MapVersion.map_id == map_id, MapVersion.version == body.version)
    )
    if record is None or version is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy map/version")
    if version.status not in {"DRAFT", "VALIDATING"}:
        if version.status == "ACTIVE":
            return _map_view(database, record, details=True)
        raise HTTPException(status_code=409, detail="Version không thể activate")
    for previous in database.scalars(
        select(MapVersion).where(MapVersion.map_id == map_id, MapVersion.status == "ACTIVE")
    ):
        previous.status = "ARCHIVED"
    version.status = "ACTIVE"
    version.activated_at = datetime.now(timezone.utc)
    record.active_version = version.version
    record.status = "ACTIVE"
    database.commit()
    return _map_view(database, record, details=True)


@router.post("/{map_id}/archive")
async def archive_map(
    map_id: str,
    _: str = Depends(operator_user_id),
    database: Session = Depends(get_db),
) -> dict:
    record = database.get(MapRecord, map_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy bản đồ")
    record.status = "ARCHIVED"
    for version in database.scalars(
        select(MapVersion).where(MapVersion.map_id == map_id, MapVersion.status == "ACTIVE")
    ):
        version.status = "ARCHIVED"
    database.commit()
    return _map_view(database, record, details=True)


@router.get("/{map_id}/cache")
async def map_cache(
    map_id: str,
    robot_id: str,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    items = database.scalars(
        select(RobotMapCache).where(
            RobotMapCache.map_id == map_id, RobotMapCache.robot_id == robot_id
        )
    ).all()
    return {
        "robot_id": robot_id,
        "map_id": map_id,
        "versions": [
            {
                "version": item.version,
                "checksum": item.checksum,
                "status": item.status,
                "progress_percent": item.progress_percent,
                "error_message": item.error_message,
            }
            for item in items
        ],
    }


@router.get("/{map_id}")
async def get_map(
    map_id: str,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> dict:
    record = database.get(MapRecord, map_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy bản đồ")
    return _map_view(database, record, details=True)


@router.get("/{map_id}/destinations")
async def get_destinations(
    map_id: str,
    _: str = Depends(current_user),
    database: Session = Depends(get_db),
) -> list[dict]:
    record = database.get(MapRecord, map_id)
    if record is None:
        raise HTTPException(status_code=404, detail="Không tìm thấy bản đồ")
    destinations = [
        {
            "destination_id": item.destination_id,
            "map_id": item.map_id,
            "name": item.name,
            "x": item.x,
            "y": item.y,
            "yaw": item.yaw,
            "enabled": item.enabled,
        }
        for item in database.scalars(
            select(Destination).where(Destination.map_id == map_id, Destination.enabled.is_(True))
        )
    ]
    destinations.extend(
        {
            "destination_id": item.poi_id,
            "map_id": item.map_id,
            "name": item.name,
            "x": item.x,
            "y": item.y,
            "yaw": item.yaw,
            "enabled": item.enabled,
        }
        for item in database.scalars(
            select(POI).where(POI.map_id == map_id, POI.enabled.is_(True))
        )
    )
    return destinations
