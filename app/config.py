from pydantic_settings import BaseSettings
from functools import lru_cache
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

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

    class Config:
        env_file = ".env"
        case_sensitive = False

    @property
    def cors_origins_list(self) -> List[str]:
        """Convert comma-separated CORS origins to list"""
        return [origin.strip() for origin in self.cors_origins.split(",")]


@lru_cache()
def get_settings() -> Settings:
    """Get cached settings instance"""
    return Settings()
