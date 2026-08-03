from functools import lru_cache

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    app_name: str = "ROVERA Center"
    environment: str = "development"
    database_url: str = "sqlite:///./rovera.db"
    redis_url: str = "redis://localhost:6379/0"
    jwt_secret: str = "change-me-in-production"
    jwt_algorithm: str = "HS256"
    jwt_expire_minutes: int = 480
    demo_email: str = "demo@rovera.local"
    demo_password: str = "demo123"
    bootstrap_admin_username: str = "admin"
    bootstrap_admin_password: str = "admin123"
    bootstrap_admin_email: str = "admin@rovera.local"
    bootstrap_admin_name: str = "Quản trị hệ thống"
    frontend_public_url: str = "http://localhost:8080"
    google_client_id: str = ""
    google_client_secret: str = ""
    google_redirect_uri: str = ""
    oauth_state_expire_minutes: int = 10
    oauth_login_code_expire_seconds: int = 120
    seed_demo_robot: bool = False
    robot_id: str = "ROBOT-001"
    robot_credential: str = "robot-001-change-me"
    robot_token_expire_minutes: int = 15
    # A value <= 0 disables the old absolute control-session limit.
    session_timeout_seconds: int = 0
    session_connect_timeout_seconds: int = 15
    session_reconnect_timeout_seconds: int = 300
    heartbeat_timeout_seconds: int = 8
    media_lease_ttl_seconds: int = 30
    media_lease_renew_seconds: int = 10
    livekit_url: str = "ws://localhost:7880"
    livekit_robot_url: str = ""
    livekit_api_key: str = "devkey"
    livekit_api_secret: str = "dev-secret-at-least-32-characters-long"
    simulator_media_source: str = ""
    simulator_rtsp_path: str = ""
    cors_origins: str = "http://localhost:5173,http://localhost:8080"
    sample_data_dir: str = "/sample-data"

    model_config = SettingsConfigDict(env_file=".env", extra="ignore")

    @property
    def cors_origin_list(self) -> list[str]:
        return [value.strip() for value in self.cors_origins.split(",") if value.strip()]

    @property
    def robot_livekit_url(self) -> str:
        return self.livekit_robot_url.strip() or self.livekit_url

    @property
    def google_oauth_enabled(self) -> bool:
        return bool(self.google_client_id.strip() and self.google_client_secret.strip())


@lru_cache
def get_settings() -> Settings:
    return Settings()
