import asyncio
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from app.api import auth, maps, navigation, robot_auth, robots, sessions, users, websockets
from app.core.config import get_settings
from app.models.database import Base, engine
from app.models.database import SessionLocal
from app.models.entities import ControlSession, Robot
from app.services.hub import SessionRuntime, hub
from app.services.seed import seed_database

settings = get_settings()


async def presence_monitor() -> None:
    while True:
        await asyncio.sleep(2)
        await hub.renew_session_media_leases(
            settings.media_lease_ttl_seconds,
            settings.media_lease_renew_seconds,
        )
        abandoned_sessions = await hub.expire_unconnected_sessions(
            settings.session_connect_timeout_seconds
        )
        disconnected_sessions = await hub.expire_disconnected_sessions(
            settings.session_reconnect_timeout_seconds
        )
        ended_sessions = [*abandoned_sessions, *disconnected_sessions]
        if ended_sessions:
            with SessionLocal.begin() as database:
                for session in ended_sessions:
                    record = database.get(ControlSession, session.session_id)
                    if record:
                        record.status = "ended"
                        record.ended_at = session.ended_at
                        record.end_reason = session.end_reason
        hub.expire_preview_leases()
        now = datetime.now(timezone.utc)
        for robot_id, robot in hub.robots.items():
            if (
                robot.status == "online"
                and robot.last_seen_at
                and (now - robot.last_seen_at).total_seconds() > settings.heartbeat_timeout_seconds
            ):
                socket = hub.robot_sockets.get(robot_id)
                if socket:
                    await websockets.ws_error(
                        socket, 4002, "heartbeat timeout"
                    )


@asynccontextmanager
async def lifespan(_: FastAPI):
    Base.metadata.create_all(bind=engine)
    seed_database()
    with SessionLocal.begin() as database:
        now = datetime.now(timezone.utc)
        for active in (
            database.query(ControlSession)
            .filter(ControlSession.status == "active")
            .all()
        ):
            session = hub.sessions.get(active.session_id)
            if session is None:
                session = SessionRuntime(
                    session_id=active.session_id,
                    robot_id=active.robot_id,
                    user_id=active.user_id,
                    status="active",
                    started_at=(
                        active.started_at.replace(tzinfo=timezone.utc)
                        if active.started_at.tzinfo is None
                        else active.started_at
                    ),
                    expires_at=None,
                    control_ever_connected=True,
                    control_disconnected_at=now,
                    robot_disconnected_at=now,
                )
                hub.sessions[session.session_id] = session
                hub.robot_session[session.robot_id] = session.session_id
        for robot in database.query(Robot).all():
            hub.sync_registry_robot(
                robot.robot_id,
                robot.name,
                robot.site_id,
                robot.map_id,
                enabled=robot.enabled,
                enrolled=robot.credential_hash is not None,
                battery_percent=robot.battery_percent,
                last_seen_at=robot.last_seen_at,
            )
    monitor = asyncio.create_task(presence_monitor())
    yield
    monitor.cancel()
    await asyncio.gather(monitor, return_exceptions=True)


app = FastAPI(title=settings.app_name, version="1.0.0", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origin_list,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
app.include_router(auth.router)
app.include_router(users.router)
app.include_router(robot_auth.router)
app.include_router(robots.router)
app.include_router(sessions.router)
app.include_router(maps.router)
app.include_router(navigation.router)
app.include_router(websockets.router)

sample_dir = Path(settings.sample_data_dir)
if not sample_dir.exists():
    sample_dir = Path(__file__).resolve().parents[4] / "sample-data"
maps_dir = sample_dir / "maps"
maps_dir.mkdir(parents=True, exist_ok=True)
app.mount("/maps", StaticFiles(directory=maps_dir), name="maps")


@app.get("/health", tags=["system"])
async def health() -> dict:
    return {
        "status": "healthy",
        "service": "center-backend",
        "robots_online": sum(robot.status == "online" for robot in hub.robots.values()),
    }
