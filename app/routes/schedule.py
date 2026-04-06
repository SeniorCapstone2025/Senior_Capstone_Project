from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import List

from app.scheduler import enable_schedule, disable_schedule, get_schedule_config

router = APIRouter(prefix="/schedule", tags=["schedule"])


class ScheduleConfig(BaseModel):
    interval_minutes: int = Field(10, ge=1, le=1440, description="Interval between scans (1-1440 minutes)")
    shelf_ids: List[str] = Field(default=["shelf_1", "shelf_2", "shelf_3"])


@router.get("/")
async def get_schedule():
    """Return the current auto-scan schedule."""
    return get_schedule_config()


@router.post("/")
async def set_schedule(config: ScheduleConfig):
    """Enable automatic scanning on an interval."""
    enable_schedule(config.interval_minutes, config.shelf_ids)
    return {
        "message": f"Auto-scan enabled every {config.interval_minutes} minute(s)",
        **get_schedule_config(),
    }


@router.delete("/")
async def clear_schedule():
    """Disable automatic scanning."""
    disable_schedule()
    return {"message": "Auto-scan disabled", **get_schedule_config()}