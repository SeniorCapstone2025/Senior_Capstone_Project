from fastapi import APIRouter
from datetime import datetime

from app.database import save_status

router = APIRouter(prefix="/status",tags=["status"])

@router.get("/")
async def status():
        status_data = {
            "state": "idle",
            "battery_level": 83,
            "last_update": datetime.now().isoformat(),
            "current_task": None,
            "detected_objects": [
                {"name": "rock", "confidence": 0.95, "position": [12.4, 7.9]},
                {"name": "bottle", "confidence": 0.88, "position": [3.1, 4.5]}
            ]
        }

        save_status(
            state = status_data["state"],
            battery = status_data["battery_level"],
            current_task= status_data["current_task"],
        )
        return status_data