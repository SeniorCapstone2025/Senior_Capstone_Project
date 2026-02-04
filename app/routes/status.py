from fastapi import APIRouter
from datetime import datetime
import logging

from app.database import save_status
from app.utils.state_manager import rover_state
from app.config import get_settings

router = APIRouter(prefix="/status", tags=["status"])
logger = logging.getLogger(__name__)


@router.get("/")
async def status():
    # Get current state from state manager
    current_state = rover_state.get_state()
    battery_level = rover_state.get_battery_level()
    current_task = rover_state.get_current_task()

    status_data = {
        "state": current_state,
        "battery_level": battery_level,
        "last_update": datetime.now().isoformat(),
        "current_task": current_task,
        "detected_objects": [
            {"name": "rock", "confidence": 0.95, "position": [12.4, 7.9]},
            {"name": "bottle", "confidence": 0.88, "position": [3.1, 4.5]}
        ]
    }

    # Only save if state has changed significantly or heartbeat elapsed
    settings = get_settings()
    if rover_state.should_save_to_database(
        battery_threshold=settings.status_cache_battery_threshold,
        heartbeat_seconds=settings.status_cache_heartbeat_seconds
    ):
        try:
            save_status(
                state=status_data["state"],
                battery=status_data["battery_level"],
                current_task=status_data["current_task"],
            )
            rover_state.mark_as_saved()
            logger.info("Status saved to database (change detected or heartbeat)")
        except Exception as e:
            logger.warning(f"Failed to save status to database: {e}")
    else:
        logger.debug("Status not saved (no significant changes)")

    return status_data