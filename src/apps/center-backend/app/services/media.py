from datetime import timedelta

from livekit import api

from app.core.config import Settings


def create_media_token(
    settings: Settings, robot_id: str, user_id: str, session_id: str
) -> str:
    identity = f"user:{user_id}:session:{session_id}"
    grant = api.VideoGrants(
        room_join=True,
        room=f"robot-{robot_id}",
        can_publish=True,
        can_subscribe=True,
    )
    return (
        api.AccessToken(settings.livekit_api_key, settings.livekit_api_secret)
        .with_identity(identity)
        .with_name("ROVERA operator")
        .with_grants(grant)
        .with_ttl(timedelta(minutes=30))
        .to_jwt()
    )


def create_spectator_media_token(
    settings: Settings, robot_id: str, user_id: str, session_id: str
) -> str:
    grant = api.VideoGrants(
        room_join=True,
        room=f"robot-{robot_id}",
        can_publish=False,
        can_subscribe=True,
    )
    return (
        api.AccessToken(settings.livekit_api_key, settings.livekit_api_secret)
        .with_identity(f"spectator:{user_id}:session:{session_id}")
        .with_name("ROVERA supervisor")
        .with_grants(grant)
        .with_ttl(timedelta(minutes=30))
        .to_jwt()
    )


def create_robot_media_token(
    settings: Settings, robot_id: str, purpose: str = "main"
) -> str:
    identity = f"robot:{robot_id}"
    name = robot_id
    if purpose == "video":
        # The optimized H.264 publisher is a separate LiveKit participant from
        # the Python audio/control adapter. Reusing one identity would make the
        # two authenticated connections continually replace each other.
        identity = f"{identity}:video"
        name = f"{robot_id} camera"
    grant = api.VideoGrants(
        room_join=True,
        room=f"robot-{robot_id}",
        can_publish=True,
        can_subscribe=purpose != "video",
    )
    return (
        api.AccessToken(settings.livekit_api_key, settings.livekit_api_secret)
        .with_identity(identity)
        .with_name(name)
        .with_grants(grant)
        .with_ttl(timedelta(minutes=30))
        .to_jwt()
    )


def create_preview_media_token(
    settings: Settings, robot_id: str, user_id: str
) -> str:
    grant = api.VideoGrants(
        room_join=True,
        room=f"robot-{robot_id}",
        can_publish=False,
        can_subscribe=True,
    )
    return (
        api.AccessToken(settings.livekit_api_key, settings.livekit_api_secret)
        .with_identity(f"preview:{user_id}:{robot_id}")
        .with_name("ROVERA camera preview")
        .with_grants(grant)
        .with_ttl(timedelta(minutes=10))
        .to_jwt()
    )
