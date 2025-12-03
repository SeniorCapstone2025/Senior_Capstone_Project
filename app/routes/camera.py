from fastapi import APIRouter
from app.websocket.camera_stream import camera_service

router = APIRouter(prefix="/camera", tags=["camera"])


@router.get("/status")
async def get_camera_status():
    """Get current camera streaming status"""
    return camera_service.get_status()