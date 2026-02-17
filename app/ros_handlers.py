"""
ROS2 communication handlers via rosbridge.

Handles subscriptions from FSM and publishes commands/data to FSM.
"""

import json
import logging
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

        # Update scan status in database based on FSM state
        if scan_id:
            if state == "NAVIGATE_TO_SHELF" and previous_state == "IDLE":
                try:
                    update_scan_status(scan_id, "in_progress")
                except Exception as e:
                    logger.error(f"Failed to update scan status: {e}")
            elif state == "IDLE" and previous_state not in ("IDLE", "UNKNOWN"):
                try:
                    update_scan_status(scan_id, "completed")
                except Exception as e:
                    logger.error(f"Failed to update scan status: {e}")
            elif state == "ERROR":
                try:
                    update_scan_status(scan_id, "failed")
                except Exception as e:
                    logger.error(f"Failed to update scan status: {e}")

        # If FSM needs expected inventory, send it
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

        # Save to database
        try:
            save_scan_results(data)
        except Exception as e:
            logger.error(f"Failed to save scan results: {e}")

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

    logger.info("ROS2 handlers registered")


# =============================================================================
# Publishers
# =============================================================================

async def send_expected_inventory(scan_id: str, shelf_id: str):
    """Send expected inventory when FSM requests it."""
    try:
        expected_items = get_shelf_inventory(shelf_id)

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
