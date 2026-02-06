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
