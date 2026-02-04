from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from app.websocket.connection_manager import manager
from app.websocket.camera_stream import camera_service
import logging
import json

logger = logging.getLogger(__name__)

router = APIRouter(tags=["websocket"])


@router.websocket("/ws/camera")
async def websocket_camera_endpoint(websocket: WebSocket):
    """WebSocket endpoint for camera streaming"""
    await manager.connect(websocket)
    try:
        # Send initial connection confirmation
        await manager.send_personal_message({
            "type": "connected",
            "message": "Connected to camera stream",
            "status": camera_service.get_status()
        }, websocket)

        while True:
            # Receive messages from client
            data = await websocket.receive_text()
            message = json.loads(data)

            action = message.get("action")
            logger.info(f"Received WebSocket action: {action}")

            if action == "start_stream":
                result = await camera_service.start_stream()
                await manager.send_personal_message({
                    "type": "response",
                    "action": "start_stream",
                    "result": result
                }, websocket)

            elif action == "stop_stream":
                result = await camera_service.stop_stream()
                await manager.send_personal_message({
                    "type": "response",
                    "action": "stop_stream",
                    "result": result
                }, websocket)

            elif action == "get_status":
                status = camera_service.get_status()
                await manager.send_personal_message({
                    "type": "response",
                    "action": "get_status",
                    "result": status
                }, websocket)

            else:
                await manager.send_personal_message({
                    "type": "error",
                    "message": f"Unknown action: {action}"
                }, websocket)

    except WebSocketDisconnect:
        manager.disconnect(websocket)
        logger.info("WebSocket client disconnected")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
        manager.disconnect(websocket)