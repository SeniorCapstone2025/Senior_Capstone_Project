from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List
import asyncio
import uuid
import logging

from app.database import save_command, create_scan
from app.rosbridge import rosbridge_client
from app.ros_handlers import start_scan

logger = logging.getLogger(__name__)
router = APIRouter(
    prefix="/command", tags=["commands"]
)


class Command(BaseModel):
    command: str
    shelf_ids: Optional[List[str]] = None  # For start command


valid_commands = {
    "start": "Scan initiated",
    "pause": "Scan paused",
    "cancel": "Scan cancelled",
    "return": "Returning to base",
}


@router.post("/")
async def send_command(data: Command):
    command = data.command.lower()

    if command not in valid_commands:
        raise HTTPException(status_code=400, detail="Invalid command")

    ros_result = {"sent": False, "error": None}

    # Forward to ROS2 via rosbridge
    if rosbridge_client.is_connected:
        try:
            if command == "start":
                # Generate scan ID and fire DB write in background (non-blocking)
                scan_id = f"scan_{uuid.uuid4().hex[:8]}"
                shelf_ids = data.shelf_ids or ["shelf_1"]

                asyncio.get_event_loop().run_in_executor(
                    None, lambda: _safe_create_scan(scan_id, shelf_ids)
                )

                # Publish to ROS2 immediately
                await start_scan(scan_id, shelf_ids)
                ros_result = {"sent": True, "scan_id": scan_id, "shelf_ids": shelf_ids}

            elif command == "cancel":
                await rosbridge_client.call_service(
                    "/inventory_fsm/emergency_stop",
                    timeout=5.0
                )
                ros_result = {"sent": True}

            elif command == "pause":
                await rosbridge_client.call_service(
                    "/inventory_fsm/stop_scan",
                    timeout=5.0
                )
                ros_result = {"sent": True}

            elif command == "return":
                await rosbridge_client.publish(
                    "/waypoint_navigator/goal",
                    "std_msgs/msg/String",
                    {"data": "home"}
                )
                ros_result = {"sent": True}

        except Exception as e:
            logger.warning(f"ROS2 command failed: {e}")
            ros_result = {"sent": False, "error": str(e)}
    else:
        ros_result = {"sent": False, "error": "rosbridge not connected"}

    # Save command to log in background (non-blocking)
    cmd_status = "OK" if ros_result.get("sent") else "FAILED"
    asyncio.get_event_loop().run_in_executor(
        None, lambda: _safe_save_command(command, cmd_status)
    )

    return {
        "command": command,
        "timestamp": datetime.now().isoformat(),
        "message": valid_commands[command],
        "ros_result": ros_result,
        "rosbridge_connected": rosbridge_client.is_connected
    }


def _safe_create_scan(scan_id: str, shelf_ids: list):
    """Background wrapper — never raises."""
    try:
        create_scan(scan_id, shelf_ids)
    except Exception as e:
        logger.warning(f"Failed to create scan record: {e}")


def _safe_save_command(command: str, status: str):
    """Background wrapper — never raises."""
    try:
        save_command(command, status)
    except Exception as e:
        logger.warning(f"Failed to save command: {e}")
