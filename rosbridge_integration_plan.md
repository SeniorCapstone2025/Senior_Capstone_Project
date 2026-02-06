# Rosbridge Integration Plan — Pure Pub/Sub Architecture

## Overview

This plan connects the FastAPI backend to the ROS2 rover using `rosbridge_server`. The architecture uses **pure pub/sub** communication — no HTTP requests from ROS2 nodes to the backend. This eliminates sync complexity and makes the FSM state the single source of truth.

### Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              rosbridge_server                               │
│                              ws://robot:9090                                │
└─────────────────────────────────────────────────────────────────────────────┘
                    ▲                                     │
                    │ subscribe                           │ publish
                    │                                     ▼
┌───────────────────┴─────────────────┐   ┌──────────────────────────────────┐
│          FastAPI Backend            │   │           ROS2 FSM Node          │
│                                     │   │                                  │
│  PUBLISH:                           │   │  PUBLISH:                        │
│    /scan/start                      │   │    /fsm_status                   │
│    /scan/expected_inventory         │   │    /scan_results                 │
│                                     │   │                                  │
│  SUBSCRIBE:                         │   │  SUBSCRIBE:                      │
│    /fsm_status ◄────────────────────┼───┤    /scan/start                   │
│    /scan_results ◄──────────────────┼───┤    /scan/expected_inventory      │
│                                     │   │                                  │
│  STATE:                             │   │  STATE:                          │
│    rover_state (from /fsm_status)   │   │    ScanState FSM                 │
└─────────────────────────────────────┘   └──────────────────────────────────┘
         │                                              ▲
         │ WebSocket                                    │ ROS2 topics
         ▼                                              │
┌─────────────────────┐                    ┌────────────┴───────────────────┐
│  Web Dashboard      │                    │  waypoint_navigator            │
│                     │                    │  qr_scanner                    │
│  Real-time updates  │                    │  yolo_detector                 │
└─────────────────────┘                    └────────────────────────────────┘
```

### Key Principles

1. **Single source of truth**: FSM state comes only from `/fsm_status` subscription
2. **No HTTP in ROS2 nodes**: All communication via rosbridge topics
3. **Reactive backend**: Responds to FSM state changes, sends data when FSM needs it
4. **Real-time dashboard**: WebSocket broadcasts happen immediately on ROS2 updates

---

## Prerequisites

### On the robot / ROS2 machine

```bash
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

This opens a WebSocket endpoint at `ws://0.0.0.0:9090`.

### On the backend machine

Add `roslibpy` to requirements:

```bash
pip install roslibpy
```

Or add to `requirements.txt`:
```
roslibpy
```

We use `roslibpy` instead of raw websockets because it's a battle-tested library that handles rosbridge protocol, reconnection, and message serialization automatically.

---

## Database Schema

### `scans` table — Tracks scan sessions

```sql
CREATE TABLE scans (
    scan_id TEXT PRIMARY KEY,
    status TEXT NOT NULL DEFAULT 'pending',  -- pending, in_progress, completed, failed
    shelf_ids TEXT[] NOT NULL,               -- Array of shelves to scan
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    completed_at TIMESTAMP WITH TIME ZONE
);
```

### `scan_results` table — Per-shelf results within a scan

```sql
CREATE TABLE scan_results (
    id SERIAL PRIMARY KEY,
    scan_id TEXT REFERENCES scans(scan_id),
    shelf_id TEXT NOT NULL,
    expected_items TEXT[] DEFAULT '{}',
    detected_items TEXT[] DEFAULT '{}',
    missing_items TEXT[] DEFAULT '{}',
    unexpected_items TEXT[] DEFAULT '{}',
    match BOOLEAN DEFAULT FALSE,
    scanned_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

### `shelf_inventory` table — Expected items per shelf

```sql
CREATE TABLE shelf_inventory (
    shelf_id TEXT PRIMARY KEY,              -- Matches QR code on shelf
    expected_items TEXT[] DEFAULT '{}',
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);
```

### Data Flow

1. Dashboard clicks "Start Scan" → Backend creates `scans` row with `status='pending'`
2. Backend publishes `/scan/start` → Updates `status='in_progress'`
3. FSM scans each shelf → Backend inserts into `scan_results`
4. FSM returns to IDLE → Backend updates `status='completed'`

---

## Topic Definitions

### Topics Published by FSM (Backend Subscribes)

#### `/fsm_status` — FSM State Updates

Published on every state transition. This is the **authoritative robot state**.

```json
{
  "state": "NAVIGATE_TO_SHELF",
  "scan_id": "scan_abc123",
  "shelf_id": "shelf_1",
  "timestamp": "2026-02-05T14:30:00Z",
  "message": "Navigating to shelf_1"
}
```

**Possible states**: `IDLE`, `NAVIGATE_TO_SHELF`, `ALIGN_WITH_SHELF`, `FETCH_EXPECTED_INVENTORY`, `RUN_YOLO_DETECTION`, `COMPARE_INVENTORY`, `SEND_RESULTS`, `RETURN_HOME`, `ERROR`

#### `/scan_results` — Scan Completion Results

Published when FSM completes inventory comparison.

```json
{
  "scan_id": "scan_abc123",
  "shelf_id": "shelf_1",
  "timestamp": "2026-02-05T14:32:15Z",
  "expected_items": ["bottle", "cup", "book"],
  "detected_items": ["bottle", "cup"],
  "missing_items": ["book"],
  "unexpected_items": [],
  "match": false
}
```

### Topics Published by Backend (FSM Subscribes)

#### `/scan/start` — Start Scan Command

```json
{
  "scan_id": "scan_abc123",
  "shelf_ids": ["shelf_1", "shelf_2"],
  "timestamp": "2026-02-05T14:30:00Z"
}
```

#### `/scan/expected_inventory` — Expected Items for Shelf

Sent when backend sees FSM enter `FETCH_EXPECTED_INVENTORY` state.

```json
{
  "scan_id": "scan_abc123",
  "shelf_id": "shelf_1",
  "expected_items": ["bottle", "cup", "book"]
}
```

---

## Files to Create / Modify

```
app/
├── rosbridge.py              # RosBridgeClient class (currently empty)
├── ros_handlers.py           # NEW: ROS2 pub/sub handlers
├── config.py                 # Add rosbridge settings
├── main.py                   # Add lifespan hooks
├── routes/
│   ├── commands.py           # Publish to /scan/start
│   └── status.py             # Return live FSM state
└── utils/
    └── state_manager.py      # Add FSM state tracking

src/app/app/
└── inventory_fsm_node.py     # Add publishers, remove HTTP calls, remove QR state
```

### QR Scanner Removal

The QR scanner is no longer needed. Its only purpose was to identify the shelf to fetch expected inventory, but now:
- `shelf_id` is sent with the scan start command
- Backend sends expected inventory when FSM enters `FETCH_EXPECTED_INVENTORY` state

**TODO after integration:**
- Remove `SCAN_QR` state from `inventory_fsm_node.py`
- Remove `qr_scanner_node.py` or repurpose for other uses
- Update FSM flow: `NAVIGATE_TO_SHELF` → `ALIGN_WITH_SHELF` → `FETCH_EXPECTED_INVENTORY`

---

## Implementation Details

### 1. Configuration — `app/config.py`

Add to `Settings` class:

```python
# Rosbridge Configuration
rosbridge_url: str = "ws://localhost:9090"
rosbridge_reconnect_interval: int = 5
rosbridge_connect_timeout: int = 10
```

---

### 2. Rosbridge Client — `app/rosbridge.py`

Uses `roslibpy` for robust rosbridge communication. roslibpy runs in a background thread, and we bridge to FastAPI's async loop via a message queue.

```python
"""
Rosbridge client using roslibpy.

roslibpy handles the rosbridge protocol, reconnection, and message
serialization. We bridge its threaded model to FastAPI's async loop.
"""

import roslibpy
import asyncio
import logging
from queue import Queue, Empty
from typing import Callable, Dict, Optional
from urllib.parse import urlparse

logger = logging.getLogger(__name__)


class RosBridgeClient:
    """roslibpy-based client for rosbridge communication."""

    def __init__(self, url: str = "ws://localhost:9090"):
        self.url = url
        self._ros: Optional[roslibpy.Ros] = None
        self._publishers: Dict[str, roslibpy.Topic] = {}
        self._subscribers: Dict[str, roslibpy.Topic] = {}
        self._message_queue: Queue = Queue()
        self._callbacks: Dict[str, Callable] = {}
        self._consumer_task: Optional[asyncio.Task] = None

    @property
    def is_connected(self) -> bool:
        return self._ros is not None and self._ros.is_connected

    async def connect(self):
        """Connect to rosbridge and start message consumer."""
        parsed = urlparse(self.url)
        host = parsed.hostname or "localhost"
        port = parsed.port or 9090

        self._ros = roslibpy.Ros(host=host, port=port)
        self._ros.on_ready(lambda: logger.info(f"Connected to rosbridge at {host}:{port}"))

        # Run roslibpy in background thread
        self._ros.run_in_thread()

        # Wait for connection (up to 5 seconds)
        for _ in range(50):
            if self._ros.is_connected:
                break
            await asyncio.sleep(0.1)

        if self._ros.is_connected:
            # Start async message consumer
            self._consumer_task = asyncio.create_task(self._consume_messages())
        else:
            logger.warning(f"Failed to connect to rosbridge at {host}:{port}")

    async def disconnect(self):
        """Disconnect and cleanup."""
        if self._consumer_task:
            self._consumer_task.cancel()
            try:
                await self._consumer_task
            except asyncio.CancelledError:
                pass

        # Unadvertise publishers
        for topic in self._publishers.values():
            try:
                topic.unadvertise()
            except Exception:
                pass

        # Unsubscribe subscribers
        for topic in self._subscribers.values():
            try:
                topic.unsubscribe()
            except Exception:
                pass

        if self._ros:
            self._ros.terminate()

        self._publishers.clear()
        self._subscribers.clear()
        self._callbacks.clear()

        logger.info("Disconnected from rosbridge")

    async def _consume_messages(self):
        """Async consumer that processes queued messages from roslibpy thread."""
        while True:
            try:
                # Process all pending messages
                while True:
                    try:
                        topic, msg = self._message_queue.get_nowait()
                        if topic in self._callbacks:
                            try:
                                await self._callbacks[topic](msg)
                            except Exception as e:
                                logger.error(f"Callback error for {topic}: {e}")
                    except Empty:
                        break

                await asyncio.sleep(0.01)  # 10ms poll interval

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Message consumer error: {e}")
                await asyncio.sleep(0.1)

    async def subscribe(self, topic: str, msg_type: str, callback: Callable):
        """Subscribe to ROS2 topic with async callback."""
        if not self._ros:
            raise ConnectionError("Not connected to rosbridge")

        self._callbacks[topic] = callback

        def _on_message(msg):
            # Called from roslibpy thread - queue for async processing
            self._message_queue.put((topic, msg))

        subscriber = roslibpy.Topic(self._ros, topic, msg_type)
        subscriber.subscribe(_on_message)
        self._subscribers[topic] = subscriber
        logger.info(f"Subscribed to {topic}")

    async def unsubscribe(self, topic: str):
        """Unsubscribe from ROS2 topic."""
        if topic in self._subscribers:
            self._subscribers[topic].unsubscribe()
            del self._subscribers[topic]
        if topic in self._callbacks:
            del self._callbacks[topic]
        logger.info(f"Unsubscribed from {topic}")

    async def publish(self, topic: str, msg_type: str, msg: dict):
        """Publish message to ROS2 topic."""
        if not self._ros or not self._ros.is_connected:
            raise ConnectionError("Not connected to rosbridge")

        if topic not in self._publishers:
            publisher = roslibpy.Topic(self._ros, topic, msg_type)
            publisher.advertise()
            self._publishers[topic] = publisher
            # Brief delay for advertise to propagate
            await asyncio.sleep(0.1)

        self._publishers[topic].publish(roslibpy.Message(msg))

    async def call_service(
        self, service: str, service_type: str = "std_srvs/srv/Trigger",
        args: dict = None, timeout: float = 10.0
    ) -> dict:
        """Call ROS2 service and wait for response."""
        if not self._ros or not self._ros.is_connected:
            raise ConnectionError("Not connected to rosbridge")

        srv = roslibpy.Service(self._ros, service, service_type)
        request = roslibpy.ServiceRequest(args or {})

        # Run blocking call in thread pool
        result = await asyncio.wait_for(
            asyncio.to_thread(srv.call, request),
            timeout=timeout
        )

        return dict(result)


# Singleton instance
rosbridge_client = RosBridgeClient()
```

---

### 3. ROS Handlers — `app/ros_handlers.py`

Handles both subscriptions (receiving from ROS2) and publishing (sending to ROS2).

```python
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
```

---

### 4. State Manager Updates — `app/utils/state_manager.py`

Add FSM state tracking:

```python
class RoverStateManager:
    def __init__(self):
        # ... existing fields ...

        # FSM state (from /fsm_status subscription)
        self._fsm_state = "IDLE"
        self._fsm_scan_id: Optional[str] = None
        self._fsm_shelf_id: Optional[str] = None

    def set_fsm_state(self, state: str, scan_id: str = None, shelf_id: str = None):
        """Update state from FSM subscription (single source of truth)."""
        self._fsm_state = state
        self._fsm_scan_id = scan_id
        self._fsm_shelf_id = shelf_id
        self._last_update = datetime.now()

        # Map FSM state to simple state for backwards compatibility
        fsm_to_simple = {
            "IDLE": "idle",
            "NAVIGATE_TO_SHELF": "navigating",
            "ALIGN_WITH_SHELF": "navigating",
            "SCAN_QR": "scanning",
            "FETCH_EXPECTED_INVENTORY": "scanning",
            "RUN_YOLO_DETECTION": "scanning",
            "COMPARE_INVENTORY": "scanning",
            "SEND_RESULTS": "scanning",
            "RETURN_HOME": "returning",
            "ERROR": "error",
        }
        self._state = fsm_to_simple.get(state, "idle")

    def get_fsm_state(self) -> str:
        return self._fsm_state

    def get_status_dict(self) -> dict:
        return {
            "state": self._state,
            "fsm_state": self._fsm_state,
            "scan_id": self._fsm_scan_id,
            "shelf_id": self._fsm_shelf_id,
            "battery_level": self._battery_level,
            "current_task": self._current_task,
            "last_update": self._last_update.isoformat()
        }
```

---

### 5. Main App Lifespan — `app/main.py`

```python
from contextlib import asynccontextmanager
from fastapi import FastAPI

from app.rosbridge import rosbridge_client
from app.ros_handlers import setup_ros_handlers
from app.config import get_settings

settings = get_settings()


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    rosbridge_client.url = settings.rosbridge_url
    await rosbridge_client.connect()
    await setup_ros_handlers()

    yield

    # Shutdown
    await rosbridge_client.disconnect()


app = FastAPI(title=settings.app_name, lifespan=lifespan)

# ... rest of existing setup ...
```

---

### 6. Commands Route — `app/routes/commands.py`

```python
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from datetime import datetime
from typing import Optional
import uuid
import logging

from app.database import save_command, create_scan
from app.rosbridge import rosbridge_client
from app.ros_handlers import start_scan

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/command", tags=["commands"])


class Command(BaseModel):
    command: str
    shelf_ids: Optional[list[str]] = None  # For start command


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
```

---

### 7. Status Route — `app/routes/status.py`

```python
from fastapi import APIRouter
from datetime import datetime
import logging

from app.database import save_status
from app.utils.state_manager import rover_state
from app.rosbridge import rosbridge_client
from app.config import get_settings

router = APIRouter(prefix="/status", tags=["status"])
logger = logging.getLogger(__name__)


@router.get("/")
async def status():
    status_data = rover_state.get_status_dict()
    status_data["rosbridge_connected"] = rosbridge_client.is_connected
    status_data["last_update"] = datetime.now().isoformat()

    # Database caching logic
    settings = get_settings()
    if rover_state.should_save_to_database(
        battery_threshold=settings.status_cache_battery_threshold,
        heartbeat_seconds=settings.status_cache_heartbeat_seconds
    ):
        try:
            save_status(
                state=status_data["state"],
                battery=status_data["battery_level"],
                current_task=status_data.get("current_task"),
            )
            rover_state.mark_as_saved()
        except Exception as e:
            logger.warning(f"Failed to save status: {e}")

    return status_data
```

---

## ROS2 FSM Node Changes

### Required modifications to `src/app/app/inventory_fsm_node.py`:

#### 1. Add Publishers

```python
def __init__(self, name='inventory_fsm'):
    # ... existing init ...

    # NEW: Publishers for backend
    self.fsm_status_pub = self.create_publisher(
        String, '/fsm_status', 10)
    self.scan_results_pub = self.create_publisher(
        String, '/scan_results', 10)

    # NEW: Subscribers for backend commands
    self.scan_start_sub = self.create_subscription(
        String, '/scan/start',
        self.scan_start_callback, 10)
    self.expected_inventory_sub = self.create_subscription(
        String, '/scan/expected_inventory',
        self.expected_inventory_callback, 10)
```

#### 2. Publish Status on State Transitions

```python
def transition_to(self, new_state: ScanState):
    old_state = self.state
    self.state = new_state
    self.get_logger().info(f'State: {old_state.name} -> {new_state.name}')

    # NEW: Publish to backend
    self.publish_fsm_status()

    # Execute state entry action
    handler = self.state_handlers.get(new_state)
    if handler:
        handler()

def publish_fsm_status(self):
    """Publish current state for backend subscription."""
    import json
    from datetime import datetime

    status = {
        "state": self.state.name,
        "scan_id": self.current_scan_id,
        "shelf_id": self.current_shelf_id,
        "timestamp": datetime.now().isoformat(),
        "message": self.get_state_message()
    }

    msg = String()
    msg.data = json.dumps(status)
    self.fsm_status_pub.publish(msg)
```

#### 3. Handle Commands from Backend

```python
def scan_start_callback(self, msg):
    """Handle scan start from backend."""
    import json

    try:
        data = json.loads(msg.data)
        self.current_scan_id = data.get("scan_id")
        self.shelf_queue = data.get("shelf_ids", [])

        with self.lock:
            if self.is_active and self.state == ScanState.IDLE:
                self.transition_to(ScanState.NAVIGATE_TO_SHELF)

    except Exception as e:
        self.get_logger().error(f'Invalid scan start: {e}')

def expected_inventory_callback(self, msg):
    """Handle expected inventory from backend."""
    import json

    try:
        data = json.loads(msg.data)

        if data.get("scan_id") != self.current_scan_id:
            return
        if data.get("shelf_id") != self.current_shelf_id:
            return

        with self.lock:
            if self.state == ScanState.FETCH_EXPECTED_INVENTORY:
                self.expected_items = data.get("expected_items", [])
                self.transition_to(ScanState.RUN_YOLO_DETECTION)

    except Exception as e:
        self.get_logger().error(f'Invalid expected inventory: {e}')
```

#### 4. Publish Results via Topic (Remove HTTP)

```python
def on_enter_send_results(self):
    """Publish results to backend via topic."""
    import json
    from datetime import datetime

    results = {
        "scan_id": self.current_scan_id,
        "shelf_id": self.current_shelf_id,
        "timestamp": datetime.now().isoformat(),
        "expected_items": self.expected_items,
        "detected_items": self.detected_items,
        "missing_items": self.missing_items,
        "unexpected_items": self.unexpected_items,
        "match": self.match
    }

    msg = String()
    msg.data = json.dumps(results)
    self.scan_results_pub.publish(msg)

    # Continue to next shelf or return home
    if self.shelf_queue:
        self.transition_to(ScanState.NAVIGATE_TO_SHELF)
    else:
        self.transition_to(ScanState.RETURN_HOME)
```

#### 5. Wait for Inventory (Remove HTTP GET)

```python
def on_enter_fetch_expected(self):
    """Wait for expected inventory from backend."""
    self.get_logger().info(f'Waiting for inventory for {self.current_shelf_id}')

    # Start timeout
    self.fetch_timeout_timer = self.create_timer(
        10.0, self.fetch_timeout_callback)

def fetch_timeout_callback(self):
    self.fetch_timeout_timer.cancel()
    self.destroy_timer(self.fetch_timeout_timer)

    if self.state == ScanState.FETCH_EXPECTED_INVENTORY:
        self.get_logger().error('Timeout waiting for expected inventory')
        self.transition_to(ScanState.ERROR)
```

---

## Sequence Diagram

```
Dashboard          Backend              rosbridge           FSM
    │                  │                    │                │
    │ POST /command    │                    │                │
    │ {command:"start"}│                    │                │
    │─────────────────►│                    │                │
    │                  │                    │                │
    │                  │ pub /scan/start    │                │
    │                  │───────────────────►│───────────────►│
    │                  │                    │                │
    │ {status:"pending"}                    │                │ transition to
    │◄─────────────────│                    │                │ NAVIGATE_TO_SHELF
    │                  │                    │                │
    │                  │ sub /fsm_status    │                │
    │                  │◄───────────────────│◄───────────────│
    │                  │                    │                │
    │ WS: fsm_status   │                    │                │
    │◄ ─ ─ ─ ─ ─ ─ ─ ─ │                    │                │
    │                  │                    │                │
    │                  │ sub /fsm_status    │                │ FETCH_EXPECTED
    │                  │◄───────────────────│◄───────────────│
    │                  │                    │                │
    │                  │ pub /scan/expected_inventory        │
    │                  │───────────────────►│───────────────►│
    │                  │                    │                │
    │                  │                    │                │ RUN_YOLO
    │                  │                    │                │ COMPARE
    │                  │                    │                │
    │                  │ sub /scan_results  │                │
    │                  │◄───────────────────│◄───────────────│
    │                  │                    │                │
    │                  │ save to DB         │                │
    │                  │                    │                │
    │ WS: scan_results │                    │                │
    │◄ ─ ─ ─ ─ ─ ─ ─ ─ │                    │                │
```

---

## Implementation Order

1. **`app/config.py`** — Add rosbridge settings
2. **`app/rosbridge.py`** — Implement RosBridgeClient (roslibpy)
3. **`app/ros_handlers.py`** — Create pub/sub handlers
4. **`app/utils/state_manager.py`** — Add FSM state tracking
5. **`app/main.py`** — Add lifespan hooks
6. **`app/routes/commands.py`** — Update to publish scan start
7. **`app/routes/status.py`** — Return live FSM state
8. **`inventory_fsm_node.py`** — Add publishers/subscribers, remove HTTP, remove SCAN_QR state

---

## Testing

```bash
# Terminal 1: Launch rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Start backend
uvicorn app.main:app --reload

# Terminal 3: Simulate FSM status
ros2 topic pub /fsm_status std_msgs/msg/String '{data: "{\"state\":\"IDLE\"}"}' --once

# Terminal 4: Check backend received it
curl http://localhost:8000/status/

# Terminal 5: Send start command
curl -X POST http://localhost:8000/command/ \
  -H "Content-Type: application/json" \
  -d '{"command":"start","shelf_ids":["shelf_1"]}'

# Terminal 6: Verify scan/start was published
ros2 topic echo /scan/start
```
