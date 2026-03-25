"""
ROS2 communication handlers via rosbridge.

Handles subscriptions from FSM and publishes commands/data to FSM.
"""

import asyncio
import json
import logging
import time
from datetime import datetime

from app.rosbridge import rosbridge_client
from app.utils.state_manager import rover_state
from app.websocket.connection_manager import manager
from app.database import (
    get_shelf_inventory,
    save_scan_results,
    update_scan_status,
)

logger = logging.getLogger(__name__)


async def setup_ros_handlers():
    """Register all ROS2 topic subscriptions."""

    # =========================================================================
    # /fsm_status — Primary state source (SUBSCRIBE)
    # =========================================================================

    async def on_fsm_status(msg):
        try:
            data = json.loads(msg.get("data", "{}"))
        except json.JSONDecodeError:
            data = {"state": msg.get("data", "UNKNOWN")}

        state = data.get("state", "UNKNOWN")
        scan_id = data.get("scan_id")
        shelf_id = data.get("shelf_id")
        message = data.get("message", "")
        timestamp = data.get("timestamp", datetime.now().isoformat())

        logger.info(f"[FSM] {state}: {message}")

        # Get previous state before updating
        previous_state = rover_state.get_fsm_state()

        # Update state manager
        rover_state.set_fsm_state(state, scan_id, shelf_id)

        # Update scan status in database in background (non-blocking)
        if scan_id:
            if state == "NAVIGATE_TO_SHELF" and previous_state == "IDLE":
                asyncio.get_event_loop().run_in_executor(
                    None, lambda sid=scan_id: _safe_update_scan(sid, "in_progress")
                )
            elif state == "IDLE" and previous_state not in ("IDLE", "UNKNOWN"):
                asyncio.get_event_loop().run_in_executor(
                    None, lambda sid=scan_id: _safe_update_scan(sid, "completed")
                )
            elif state == "ERROR":
                asyncio.get_event_loop().run_in_executor(
                    None, lambda sid=scan_id: _safe_update_scan(sid, "failed")
                )

        # If FSM needs expected inventory, send it (uses DB read — run in thread)
        if state == "FETCH_EXPECTED_INVENTORY" and shelf_id and scan_id:
            await send_expected_inventory(scan_id, shelf_id)

        # Broadcast to dashboard
        await manager.broadcast({
            "type": "fsm_status",
            "state": state,
            "scan_id": scan_id,
            "shelf_id": shelf_id,
            "message": message,
            "timestamp": timestamp
        })

    await rosbridge_client.subscribe(
        "/fsm_status",
        "std_msgs/msg/String",
        on_fsm_status
    )

    # =========================================================================
    # /scan_results — Per-shelf scan completion (SUBSCRIBE)
    # =========================================================================

    async def on_scan_results(msg):
        try:
            data = json.loads(msg.get("data", "{}"))
        except json.JSONDecodeError:
            logger.error("Invalid scan results JSON")
            return

        scan_id = data.get("scan_id")
        shelf_id = data.get("shelf_id")

        logger.info(f"[RESULTS] Scan {scan_id} shelf {shelf_id}: match={data.get('match')}")

        # Save to database in background (non-blocking)
        asyncio.get_event_loop().run_in_executor(
            None, lambda d=data: _safe_save_results(d)
        )

        # Broadcast to dashboard
        await manager.broadcast({
            "type": "scan_results",
            "data": data
        })

    await rosbridge_client.subscribe(
        "/scan_results",
        "std_msgs/msg/String",
        on_scan_results
    )

    # =========================================================================
    # /waypoint_navigator/status — Navigation updates (SUBSCRIBE)
    # =========================================================================

    async def on_nav_status(msg):
        status = msg.get("data", "")
        logger.info(f"[NAV] {status}")

        await manager.broadcast({
            "type": "nav_status",
            "status": status
        })

    await rosbridge_client.subscribe(
        "/waypoint_navigator/status",
        "std_msgs/msg/String",
        on_nav_status
    )

    # =========================================================================
    # /yolov11_ros2/object_detect — Live YOLO bounding boxes (SUBSCRIBE)
    # =========================================================================

    _last_detection_broadcast = 0.0  # throttle timestamp

    async def on_yolo_detections(msg):
        nonlocal _last_detection_broadcast
        now = time.time()
        if now - _last_detection_broadcast < 0.1:  # 100ms throttle
            return
        _last_detection_broadcast = now

        objects = msg.get("objects", [])
        detections = []
        for obj in objects:
            detections.append({
                "class_name": obj.get("class_name", ""),
                "box": obj.get("box", []),
                "score": obj.get("score", 0.0),
                "width": obj.get("width", 0),
                "height": obj.get("height", 0),
            })

        await manager.broadcast({
            "type": "yolo_detections",
            "detections": detections
        })

    await rosbridge_client.subscribe(
        "/yolov11_ros2/object_detect",
        "interfaces/msg/ObjectsInfo",
        on_yolo_detections
    )

    logger.info("ROS2 handlers registered")


# =============================================================================
# Publishers
# =============================================================================

async def send_expected_inventory(scan_id: str, shelf_id: str):
    """Send expected inventory when FSM requests it."""
    try:
        # DB read in thread pool so it doesn't block the event loop
        expected_items = await asyncio.get_event_loop().run_in_executor(
            None, get_shelf_inventory, shelf_id
        )

        payload = {
            "scan_id": scan_id,
            "shelf_id": shelf_id,
            "expected_items": expected_items
        }

        await rosbridge_client.publish(
            "/scan/expected_inventory",
            "std_msgs/msg/String",
            {"data": json.dumps(payload)}
        )

        logger.info(f"Sent expected inventory for {shelf_id}: {expected_items}")

    except Exception as e:
        logger.error(f"Failed to send expected inventory: {e}")


async def start_scan(scan_id: str, shelf_ids: list):
    """Publish scan start command to FSM."""
    payload = {
        "scan_id": scan_id,
        "shelf_ids": shelf_ids,
        "timestamp": datetime.now().isoformat()
    }

    await rosbridge_client.publish(
        "/scan/start",
        "std_msgs/msg/String",
        {"data": json.dumps(payload)}
    )

    logger.info(f"Published scan start: {scan_id} -> {shelf_ids}")


# =============================================================================
# Background DB wrappers (never raise, never block event loop)
# =============================================================================

def _safe_update_scan(scan_id: str, status: str):
    try:
        update_scan_status(scan_id, status)
    except Exception as e:
        logger.error(f"Failed to update scan {scan_id} to {status}: {e}")


def _safe_save_results(data: dict):
    try:
        save_scan_results(data)
    except Exception as e:
        logger.error(f"Failed to save scan results: {e}")
