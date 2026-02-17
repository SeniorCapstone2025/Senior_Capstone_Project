from pydantic_settings import BaseSettings, SettingsConfigDict
from functools import lru_cache
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    model_config = SettingsConfigDict(
        env_file="app/.env",
        env_file_encoding="utf-8",
        case_sensitive=False,
    )

    # Supabase Configuration
    supabase_url: str
    supabase_service_key: str

    # Application Settings
    app_name: str = "ROS2 Rover Control"
    debug: bool = False

    # CORS Settings
    cors_origins: str = "http://localhost:3000"

    # Model Settings
    yolo_model_path: str = "model/yolo11n.pt"

    # Status Caching Settings
    status_cache_battery_threshold: float = 5.0
    status_cache_heartbeat_seconds: int = 300

    # Rosbridge Configuration
    rosbridge_url: str = "ws://172.20.10.2:9090"
    rosbridge_reconnect_interval: int = 5
    rosbridge_connect_timeout: int = 10

    @property
    def cors_origins_list(self) -> List[str]:
        return [origin.strip() for origin in self.cors_origins.split(",")]


@lru_cache()
def get_settings() -> Settings:
    return Settings()
