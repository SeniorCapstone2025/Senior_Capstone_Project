from fastapi import APIRouter,HTTPException
from pydantic import BaseModel
from datetime import datetime

from app.database import save_detected_object

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
    save_detected_object(
        object_name=  data.object_name,
        confidence=data.confidence,
        x=data.x,
        y = data.y
    )

    return {
       "message":"Detection saved",
       "timestamp": datetime.now().isoformat(),
       "objects":data
   }