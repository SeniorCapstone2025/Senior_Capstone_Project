from fastapi import APIRouter
from datetime import datetime
import logging

from app.database import save_status
from app.utils.state_manager import rover_state
from app.rosbridge import rosbridge_client
from app.config import get_settings

router = APIRouter(prefix="/status", tags=["status"])
logger = logging.getLogger(__name__)


@router.get("/")
async def status():
    # Get full status from state manager (includes FSM state from ROS2)
    status_data = rover_state.get_status_dict()
    status_data["rosbridge_connected"] = rosbridge_client.is_connected
    status_data["last_update"] = datetime.now().isoformat()

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
                current_task=status_data.get("current_task"),
            )
            rover_state.mark_as_saved()
            logger.info("Status saved to database")
        except Exception as e:
            logger.warning(f"Failed to save status to database: {e}")

    return status_data