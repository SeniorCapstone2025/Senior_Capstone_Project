from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from datetime import datetime
from typing import Optional, List
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
                # Generate scan ID and create database record
                scan_id = f"scan_{uuid.uuid4().hex[:8]}"
                shelf_ids = data.shelf_ids or ["shelf_1"]

                try:
                    create_scan(scan_id, shelf_ids)
                except Exception as e:
                    logger.warning(f"Failed to create scan record: {e}")

                # Publish to ROS2
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

    # Save command to log
    try:
        save_command(command, "OK" if ros_result.get("sent") else "FAILED")
    except Exception as e:
        logger.warning(f"Failed to save command: {e}")

    return {
        "command": command,
        "timestamp": datetime.now().isoformat(),
        "message": valid_commands[command],
        "ros_result": ros_result,
        "rosbridge_connected": rosbridge_client.is_connected
    }
