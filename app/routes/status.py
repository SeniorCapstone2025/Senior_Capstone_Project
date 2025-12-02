from fastapi import APIRouter
from datetime import datetime

from app.database import save_status
from app.utils.state_manager import rover_state

router = APIRouter(prefix="/status", tags=["status"])


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

    # Save status to database
    save_status(
        state=status_data["state"],
        battery=status_data["battery_level"],
        current_task=status_data["current_task"],
    )

    return status_data