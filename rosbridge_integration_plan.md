# Rosbridge WebSocket Client — Implementation Plan

## Overview

The FastAPI backend needs a WebSocket client that connects to `rosbridge_server` (the standard ROS2 WebSocket bridge, typically at `ws://<robot_ip>:9090`). This allows the backend to **publish commands** to ROS2 topics and **subscribe** to ROS2 topics for results — without requiring `rclpy` or a ROS2 installation on the backend host.

### Current Architecture Gap

```
Frontend  --(HTTP/WS)-->  FastAPI Backend  --(nothing)-->  ROS2 Nodes
                                            ^
                                            |
                              This gap needs rosbridge
```

Right now the ROS2 `inventory_fsm_node` makes HTTP requests _to_ the backend, but the backend has **no way to push commands into ROS2**. The status route returns hardcoded mock data. Adding a rosbridge client closes the loop:

```
Frontend  --(HTTP/WS)-->  FastAPI Backend  --(rosbridge WS)-->  rosbridge_server  --(ROS2 topics)-->  Nodes
                                           <--(subscriptions)--
```

---

## Prerequisites

### On the robot / ROS2 machine

Install and launch `rosbridge_server`:

```bash
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

This opens a WebSocket endpoint at `ws://0.0.0.0:9090` by default.

### On the backend machine

No new pip dependencies are needed. The project already has `websockets==15.0` in `requirements.txt`, which is sufficient. If you prefer a higher-level library, `roslibpy` is an option (`pip install roslibpy`), but this plan uses raw `websockets` to avoid adding dependencies and to keep full control over the async integration with FastAPI.

---

## Rosbridge Protocol Reference

Rosbridge uses a JSON protocol over WebSocket. The three operations we need:

### Publish a message

```json
{
  "op": "publish",
  "topic": "/waypoint_navigator/goal",
  "msg": {
    "data": "shelf_1"
  }
}
```

### Subscribe to a topic

```json
{
  "op": "subscribe",
  "topic": "/waypoint_navigator/status",
  "type": "std_msgs/msg/String"
}
```

After subscribing, rosbridge pushes messages to us:

```json
{
  "op": "publish",
  "topic": "/waypoint_navigator/status",
  "msg": {
    "data": "reached"
  }
}
```

### Call a service

```json
{
  "op": "call_service",
  "service": "/inventory_fsm/start_scan",
  "args": {}
}
```

Response:

```json
{
  "op": "service_response",
  "service": "/inventory_fsm/start_scan",
  "values": { "success": true, "message": "Scan started" }
}
```

---

## Files to Create / Modify

```
app/
├── rosbridge.py            # (exists, currently empty) — RosBridgeClient class
├── config.py               # Add rosbridge_url setting
├── main.py                 # Wire up startup/shutdown lifecycle
├── .env.example            # Add rosbridge_url
├── routes/
│   └── commands.py         # Publish commands to ROS2 via rosbridge
│   └── status.py           # Return live data from ROS2 subscriptions
```

---

## Implementation Details

### 1. Configuration — `app/config.py`

Add these fields to the `Settings` class:

```python
# Rosbridge Configuration
rosbridge_url: str = "ws://localhost:9090"
rosbridge_reconnect_interval: int = 5    # seconds between reconnect attempts
rosbridge_connect_timeout: int = 10      # seconds to wait for initial connection
```

Update `app/.env.example`:

```
rosbridge_url="ws://localhost:9090"
```

---

### 2. Rosbridge Client — `app/rosbridge.py`

This is the core module. It should implement a singleton `RosBridgeClient` class with these responsibilities:

- Maintain a persistent WebSocket connection to rosbridge_server
- Auto-reconnect on disconnect with backoff
- Publish messages to ROS2 topics
- Subscribe to ROS2 topics with callback handlers
- Call ROS2 services and return results
- Thread-safe and fully async (integrates with FastAPI's event loop)

#### Class Skeleton

```python
import asyncio
import json
import logging
from typing import Any, Callable, Coroutine, Dict, Optional
import websockets
from websockets.asyncio.client import ClientConnection

logger = logging.getLogger(__name__)


class RosBridgeClient:
    """WebSocket client for rosbridge_server communication."""

    def __init__(self, url: str = "ws://localhost:9090"):
        self.url = url
        self._ws: Optional[ClientConnection] = None
        self._connected = False
        self._subscriptions: Dict[str, Callable] = {}  # topic -> async callback
        self._service_futures: Dict[str, asyncio.Future] = {}  # service_id -> future
        self._listen_task: Optional[asyncio.Task] = None
        self._reconnect_task: Optional[asyncio.Task] = None
        self._should_run = False
        self._id_counter = 0

    # -- Lifecycle --

    async def connect(self):
        """Open WebSocket connection and start listener loop."""
        ...

    async def disconnect(self):
        """Gracefully close connection."""
        ...

    async def _reconnect_loop(self):
        """Background task: reconnect on disconnect with exponential backoff."""
        ...

    async def _listen(self):
        """Background task: read incoming messages and dispatch to handlers."""
        ...

    # -- Publishing --

    async def publish(self, topic: str, msg_type: str, msg: dict):
        """Publish a message to a ROS2 topic."""
        ...

    # -- Subscribing --

    async def subscribe(self, topic: str, msg_type: str, callback: Callable):
        """Subscribe to a ROS2 topic. callback receives the msg dict."""
        ...

    async def unsubscribe(self, topic: str):
        """Unsubscribe from a ROS2 topic."""
        ...

    # -- Services --

    async def call_service(self, service: str, args: dict = None, timeout: float = 10.0) -> dict:
        """Call a ROS2 service and return the response."""
        ...

    # -- Internal --

    async def _send(self, message: dict):
        """Send a JSON message over the WebSocket."""
        ...

    def _next_id(self) -> str:
        """Generate unique message ID."""
        ...


# Singleton instance
rosbridge_client = RosBridgeClient()
```

#### Key Implementation Notes

**Connection management:**
```python
async def connect(self):
    self._should_run = True
    try:
        self._ws = await websockets.connect(self.url)
        self._connected = True
        logger.info(f"Connected to rosbridge at {self.url}")

        # Start listener
        self._listen_task = asyncio.create_task(self._listen())

        # Re-subscribe to any topics we had before reconnecting
        for topic, (msg_type, callback) in self._subscriptions.items():
            await self._send({
                "op": "subscribe",
                "topic": topic,
                "type": msg_type
            })

    except Exception as e:
        logger.error(f"Failed to connect to rosbridge: {e}")
        self._connected = False
        # Start reconnect loop
        if not self._reconnect_task or self._reconnect_task.done():
            self._reconnect_task = asyncio.create_task(self._reconnect_loop())
```

**Listener loop** (dispatches incoming messages to subscription callbacks):
```python
async def _listen(self):
    try:
        async for raw_msg in self._ws:
            msg = json.loads(raw_msg)
            op = msg.get("op")

            if op == "publish":
                # Incoming subscription data
                topic = msg.get("topic")
                if topic in self._subscriptions:
                    msg_type, callback = self._subscriptions[topic]
                    asyncio.create_task(callback(msg.get("msg", {})))

            elif op == "service_response":
                # Service call response
                service_id = msg.get("id")
                if service_id in self._service_futures:
                    self._service_futures[service_id].set_result(msg.get("values", {}))

    except websockets.ConnectionClosed:
        logger.warning("Rosbridge connection closed")
        self._connected = False
        if self._should_run:
            self._reconnect_task = asyncio.create_task(self._reconnect_loop())
```

**Publishing** (what the commands route will call):
```python
async def publish(self, topic: str, msg_type: str, msg: dict):
    if not self._connected:
        raise ConnectionError("Not connected to rosbridge")
    await self._send({
        "op": "publish",
        "topic": topic,
        "type": msg_type,
        "msg": msg
    })
```

**Service calls** (for triggering FSM services like `start_scan`):
```python
async def call_service(self, service: str, args: dict = None, timeout: float = 10.0) -> dict:
    if not self._connected:
        raise ConnectionError("Not connected to rosbridge")

    call_id = self._next_id()
    future = asyncio.get_event_loop().create_future()
    self._service_futures[call_id] = future

    await self._send({
        "op": "call_service",
        "id": call_id,
        "service": service,
        "args": args or {}
    })

    try:
        result = await asyncio.wait_for(future, timeout=timeout)
        return result
    finally:
        self._service_futures.pop(call_id, None)
```

**Reconnect loop:**
```python
async def _reconnect_loop(self):
    backoff = 1
    max_backoff = 30

    while self._should_run and not self._connected:
        logger.info(f"Reconnecting to rosbridge in {backoff}s...")
        await asyncio.sleep(backoff)
        try:
            await self.connect()
            if self._connected:
                return
        except Exception:
            pass
        backoff = min(backoff * 2, max_backoff)
```

---

### 3. Startup / Shutdown — `app/main.py`

Wire the client into FastAPI's lifespan:

```python
from contextlib import asynccontextmanager
from app.rosbridge import rosbridge_client
from app.config import get_settings

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    settings = get_settings()
    rosbridge_client.url = settings.rosbridge_url
    await rosbridge_client.connect()

    # Register default subscriptions
    await setup_ros_subscriptions()

    yield

    # Shutdown
    await rosbridge_client.disconnect()

app = FastAPI(title=settings.app_name, lifespan=lifespan)
```

The `setup_ros_subscriptions()` function registers all topic subscriptions the backend cares about (see Section 5).

---

### 4. Publishing Commands — `app/routes/commands.py`

Modify the `send_command` endpoint to publish to ROS2 via rosbridge in addition to updating local state. Map each dashboard command to the appropriate ROS2 action:

```python
from app.rosbridge import rosbridge_client

COMMAND_TO_ROS_ACTION = {
    "start":  {"method": "service", "target": "/inventory_fsm/start_scan"},
    "pause":  {"method": "service", "target": "/inventory_fsm/stop_scan"},
    "cancel": {"method": "service", "target": "/inventory_fsm/emergency_stop"},
    "search": {"method": "publish", "target": "/waypoint_navigator/goal",
               "type": "std_msgs/msg/String"},
    "return": {"method": "publish", "target": "/waypoint_navigator/goal",
               "type": "std_msgs/msg/String", "msg": {"data": "home"}},
    "reboot": None,  # handled locally, no ROS2 equivalent
}

@router.post("/")
async def send_command(data: Command):
    command = data.command.lower()

    if command not in valid_commands:
        raise HTTPException(status_code=400, detail="invalid command")

    # Update local state
    rover_state.update_state(command)

    # Forward to ROS2 via rosbridge
    ros_action = COMMAND_TO_ROS_ACTION.get(command)
    ros_result = None

    if ros_action and rosbridge_client.is_connected:
        try:
            if ros_action["method"] == "service":
                ros_result = await rosbridge_client.call_service(ros_action["target"])
            elif ros_action["method"] == "publish":
                msg = ros_action.get("msg", {"data": command})
                await rosbridge_client.publish(
                    ros_action["target"], ros_action["type"], msg
                )
                ros_result = {"published": True}
        except Exception as e:
            logger.warning(f"Failed to send command to ROS2: {e}")
            ros_result = {"error": str(e)}

    # ... rest of existing response logic ...
    response["ros_result"] = ros_result
    return response
```

---

### 5. Subscribing to ROS2 Topics — `app/main.py` (or new file `app/ros_subscriptions.py`)

Create a setup function that subscribes to the topics the backend needs. Each subscription callback updates the shared state so the REST/WS endpoints can serve live data.

```python
from app.rosbridge import rosbridge_client
from app.utils.state_manager import rover_state

async def setup_ros_subscriptions():
    """Register all ROS2 topic subscriptions via rosbridge."""

    # Navigation status — update rover state when nav completes/fails
    async def on_nav_status(msg):
        status = msg.get("data", "")
        logger.info(f"[ROS2] Nav status: {status}")
        if status == "reached":
            rover_state.update_state("idle")  # or more granular
        elif status == "failed":
            rover_state.update_state("error")

    await rosbridge_client.subscribe(
        "/waypoint_navigator/status",
        "std_msgs/msg/String",
        on_nav_status
    )

    # QR scanner results
    async def on_qr_detected(msg):
        qr_data = msg.get("data", "")
        logger.info(f"[ROS2] QR detected: {qr_data}")
        # Forward to dashboard WebSocket clients
        from app.websocket.connection_manager import manager
        await manager.broadcast({
            "type": "qr_detected",
            "item_id": qr_data,
            "source": "ros2"
        })

    await rosbridge_client.subscribe(
        "/qr_scanner/qr_code",
        "std_msgs/msg/String",
        on_qr_detected
    )

    # YOLO detection results
    async def on_detections(msg):
        detections_json = msg.get("data", "")
        logger.info(f"[ROS2] Detections received")
        # Parse and forward to dashboard
        from app.websocket.connection_manager import manager
        await manager.broadcast({
            "type": "ros2_detections",
            "data": detections_json,
            "source": "ros2"
        })

    await rosbridge_client.subscribe(
        "/yolo_detector/detections",
        "std_msgs/msg/String",
        on_detections
    )

    # FSM state — keep backend state_manager in sync with actual robot state
    async def on_fsm_status(msg):
        status = msg.get("data", "")
        logger.info(f"[ROS2] FSM status: {status}")
        # Map FSM states to rover_state states as appropriate

    await rosbridge_client.subscribe(
        "/fsm_status",
        "std_msgs/msg/String",
        on_fsm_status
    )

    # Battery / odometry (if available)
    # await rosbridge_client.subscribe("/battery_state", "sensor_msgs/msg/BatteryState", on_battery)

    logger.info("ROS2 subscriptions registered via rosbridge")
```

---

### 6. Status Route — `app/routes/status.py`

Replace the hardcoded mock data with live values from the state manager (which is now updated by rosbridge subscription callbacks):

```python
@router.get("/")
async def status():
    status_data = rover_state.get_status_dict()

    # Add rosbridge connection status
    from app.rosbridge import rosbridge_client
    status_data["rosbridge_connected"] = rosbridge_client.is_connected

    # ... existing database caching logic ...
    return status_data
```

---

## ROS2 Topic / Service Map

Summary of what the backend will interact with through rosbridge:

| Direction | Type | Name | ROS2 Msg Type | Purpose |
|-----------|------|------|---------------|---------|
| **Publish** | Topic | `/waypoint_navigator/goal` | `std_msgs/msg/String` | Send navigation target |
| **Publish** | Topic | `/qr_scanner/trigger` | `std_msgs/msg/Bool` | Enable/disable QR scanning |
| **Publish** | Topic | `/yolo_detector/trigger` | `std_msgs/msg/Bool` | Enable/disable YOLO detection |
| **Subscribe** | Topic | `/waypoint_navigator/status` | `std_msgs/msg/String` | Nav status: navigating, reached, blocked, failed |
| **Subscribe** | Topic | `/qr_scanner/qr_code` | `std_msgs/msg/String` | Decoded QR code data |
| **Subscribe** | Topic | `/yolo_detector/detections` | `std_msgs/msg/String` | Detection results (JSON string) |
| **Subscribe** | Topic | `/fsm_status` | `std_msgs/msg/String` | FSM state updates |
| **Service** | Service | `/inventory_fsm/start_scan` | `std_srvs/srv/Trigger` | Start scan sequence |
| **Service** | Service | `/inventory_fsm/stop_scan` | `std_srvs/srv/Trigger` | Gracefully stop scan |
| **Service** | Service | `/inventory_fsm/emergency_stop` | `std_srvs/srv/Trigger` | Emergency halt |

---

## Suggested Implementation Order

1. **`app/config.py`** — Add `rosbridge_url` setting + update `.env.example`
2. **`app/rosbridge.py`** — Implement `RosBridgeClient` (connect, publish, subscribe, call_service, reconnect)
3. **`app/main.py`** — Add lifespan hook to connect/disconnect and register subscriptions
4. **`app/ros_subscriptions.py`** — Subscription callbacks that update `rover_state` and broadcast to dashboard WS clients
5. **`app/routes/commands.py`** — Forward commands to ROS2 via rosbridge publish/service calls
6. **`app/routes/status.py`** — Remove mock data, serve live state + rosbridge connection status
7. **Test** with `rosbridge_server` running and verify pub/sub round-trip

---

## Testing Without a Robot

You can test the integration without a physical robot or ROS2 nodes:

```bash
# Terminal 1: Launch rosbridge_server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Start the FastAPI backend
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload

# Terminal 3: Publish a fake nav status to verify subscription works
ros2 topic pub /waypoint_navigator/status std_msgs/msg/String "{data: 'reached'}" --once

# Terminal 4: Verify the backend received it
curl http://localhost:8000/status/
# Should show updated state from the subscription callback

# Terminal 5: Send a command from the API and verify it reaches ROS2
curl -X POST http://localhost:8000/command/ -H "Content-Type: application/json" -d '{"command": "start"}'
# Then check:
ros2 topic echo /inventory_fsm/start_scan
```

---

## Error Handling Considerations

- **rosbridge not available at startup**: The backend should start normally and keep retrying the connection in the background. All REST endpoints should remain functional — commands just won't reach ROS2 until connected.
- **Connection drops mid-session**: The reconnect loop handles this. Subscriptions are re-registered after reconnect.
- **Slow/unresponsive services**: The `call_service` method has a configurable timeout (default 10s). If a service call times out, the endpoint returns an error to the caller but doesn't block.
- **`is_connected` property**: Expose this on the client so routes can include connection status in responses and the frontend can display it.
