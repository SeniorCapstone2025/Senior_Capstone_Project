from fastapi import APIRouter,HTTPException
from pydantic import BaseModel
from datetime import datetime

from app.database import save_detected_object
import logging

logger = logging.getLogger(__name__)

router = APIRouter(
    prefix="/detections",
    tags=["detections"],
)

class Detection(BaseModel):
    object_name: str
    confidence: float
    x: float
    y: float

@router.post("/")
async def detections(data: Detection):
    try:
        save_detected_object(
            object_name=data.object_name,
            confidence=data.confidence,
            x=data.x,
            y=data.y
        )
    except Exception as e:
        logger.warning(f"Failed to save detection to database: {e}")

    return {
       "message":"Detection saved",
       "timestamp": datetime.now().isoformat(),
       "objects":data
   }