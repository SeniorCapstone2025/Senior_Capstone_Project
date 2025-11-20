from fastapi import APIRouter
from datetime import datetime

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
        return status_data