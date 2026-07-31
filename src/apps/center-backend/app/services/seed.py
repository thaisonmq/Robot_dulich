import hashlib
from datetime import datetime, timezone
from sqlalchemy import or_, select

from app.core.config import get_settings
from app.core.security import hash_password
from app.models.database import SessionLocal
from app.models.entities import Destination, MapRecord, Robot, User
from app.services.maps import DESTINATIONS, MAP


def _seed_demo_robot(database, settings) -> None:
    robot = database.scalar(
        select(Robot).where(Robot.robot_id == settings.robot_id)
    )
    if robot is None:
        database.add(
            Robot(
                robot_id=settings.robot_id,
                name="Robot Bảo tàng 01",
                site_id="SITE-HCM-01",
                map_id=MAP["map_id"],
                capabilities={
                    "video": True,
                    "audio": True,
                    "navigation": True,
                    "teleoperation": True,
                },
                credential_hash=hashlib.sha256(
                    settings.robot_credential.encode()
                ).hexdigest(),
                enrolled_at=datetime.now(timezone.utc),
            )
        )
    elif robot.credential_hash is None:
        robot.credential_hash = hashlib.sha256(
            settings.robot_credential.encode()
        ).hexdigest()
        robot.enrolled_at = datetime.now(timezone.utc)


def seed_database() -> None:
    """Insert the deterministic demo records without overwriting operator data."""
    settings = get_settings()
    with SessionLocal.begin() as database:
        admin = database.scalar(
            select(User).where(
                or_(
                    User.username == settings.bootstrap_admin_username.casefold(),
                    User.email == settings.bootstrap_admin_email.casefold(),
                )
            )
        )
        if admin is None:
            database.add(
                User(
                    username=settings.bootstrap_admin_username.casefold(),
                    email=settings.bootstrap_admin_email.casefold(),
                    password_hash=hash_password(settings.bootstrap_admin_password),
                    full_name=settings.bootstrap_admin_name,
                    role="admin",
                    active=True,
                    email_verified=True,
                    must_change_password=True,
                )
            )

        demo = database.scalar(
            select(User).where(User.email == settings.demo_email.casefold())
        )
        if demo is None:
            database.add(
                User(
                    username="demo",
                    email=settings.demo_email.casefold(),
                    password_hash=hash_password(settings.demo_password),
                    full_name="Nguyễn Minh",
                    role="operator",
                    active=True,
                    email_verified=True,
                )
            )
        elif demo.password_hash == "managed-by-demo-environment":
            demo.password_hash = hash_password(settings.demo_password)
            demo.full_name = demo.full_name or "Nguyễn Minh"
            demo.username = demo.username or "demo"

        if settings.seed_demo_robot:
            _seed_demo_robot(database, settings)

        if database.get(MapRecord, MAP["map_id"]) is None:
            database.add(
                MapRecord(
                    map_id=MAP["map_id"],
                    name=MAP["name"],
                    image_url=MAP["image_url"],
                    width_pixels=MAP["width_pixels"],
                    height_pixels=MAP["height_pixels"],
                    resolution_m_per_pixel=MAP["resolution_m_per_pixel"],
                    origin=MAP["origin"],
                )
            )

        for item in DESTINATIONS:
            if database.get(Destination, item["destination_id"]) is None:
                database.add(Destination(**item))
