import cv2
import base64
import asyncio
import logging
from ultralytics import YOLO
from app.config import get_settings
from app.database import save_detected_object
from app.websocket.connection_manager import manager
from typing import Optional
import numpy as np

logger = logging.getLogger(__name__)


class CameraStreamService:
    """Handles webcam streaming with YOLO object detection"""

    def __init__(self):
        self.model: Optional[YOLO] = None
        self.capture: Optional[cv2.VideoCapture] = None
        self.is_streaming = False
        self.stream_task: Optional[asyncio.Task] = None
        self.fps = 30
        self.frame_delay = 1.0 / self.fps

    def load_model(self):
        """Load the YOLO model"""
        if self.model is None:
            try:
                settings = get_settings()
                logger.info(f"Loading YOLO model from {settings.yolo_model_path}")
                self.model = YOLO(settings.yolo_model_path)
                logger.info("YOLO model loaded successfully")
            except Exception as e:
                logger.error(f"Error loading YOLO model: {e}")
                raise

    def start_camera(self):
        """Initialize the webcam"""
        if self.capture is None or not self.capture.isOpened():
            try:
                self.capture = cv2.VideoCapture(0)
                if not self.capture.isOpened():
                    raise Exception("Failed to open webcam")

                # Set camera properties for better performance
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.capture.set(cv2.CAP_PROP_FPS, self.fps)

                logger.info("Webcam started successfully")
            except Exception as e:
                logger.error(f"Error starting webcam: {e}")
                raise

    def stop_camera(self):
        """Release the webcam"""
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            logger.info("Webcam released")

    async def process_frame(self, frame: np.ndarray) -> dict:
        """Process a frame with YOLO detection"""
        try:
            # Run YOLO detection
            results = self.model(frame, verbose=False)

            # Extract detections
            detections = []
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    confidence = float(box.conf[0])
                    class_id = int(box.cls[0])
                    class_name = result.names[class_id]

                    # Calculate center point
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2

                    detection = {
                        "object_name": class_name,
                        "confidence": round(confidence, 2),
                        "bbox": {
                            "x1": round(x1, 2),
                            "y1": round(y1, 2),
                            "x2": round(x2, 2),
                            "y2": round(y2, 2)
                        },
                        "center": {
                            "x": round(center_x, 2),
                            "y": round(center_y, 2)
                        }
                    }
                    detections.append(detection)

                    # Save to database asynchronously (fire and forget)
                    if confidence > 0.25:  # Only save high-confidence detections
                        asyncio.create_task(
                            asyncio.to_thread(
                                save_detected_object,
                                class_name,
                                confidence,
                                center_x,
                                center_y
                            )
                        )

                # Draw bounding boxes on frame
                annotated_frame = result.plot()
                return annotated_frame, detections

            return frame, detections

        except Exception as e:
            logger.error(f"Error processing frame: {e}")
            return frame, []

    def encode_frame(self, frame: np.ndarray) -> str:
        """Encode frame as base64 JPEG"""
        try:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            # Convert to base64
            frame_base64 = base64.b64encode(buffer).decode('utf-8')
            return frame_base64
        except Exception as e:
            logger.error(f"Error encoding frame: {e}")
            return ""

    async def stream_loop(self):
        """Main streaming loop"""
        logger.info("Starting camera stream loop")
        frame_count = 0

        try:
            while self.is_streaming:
                if self.capture is None or not self.capture.isOpened():
                    logger.error("Camera not available")
                    break

                # Capture frame
                ret, frame = self.capture.read()
                if not ret:
                    logger.warning("Failed to capture frame")
                    await asyncio.sleep(0.1)
                    continue

                # Process frame with YOLO
                annotated_frame, detections = await self.process_frame(frame)

                # Encode frame
                frame_base64 = self.encode_frame(annotated_frame)

                if frame_base64:
                    # Prepare message
                    message = {
                        "type": "frame",
                        "frame": frame_base64,
                        "detections": detections,
                        "frame_number": frame_count,
                        "timestamp": asyncio.get_event_loop().time()
                    }

                    # Broadcast to all connected clients
                    await manager.broadcast(message)

                    frame_count += 1
                    if frame_count % 30 == 0:  # Log every 30 frames (1 second at 30 fps)
                        logger.debug(f"Streamed {frame_count} frames, {len(detections)} detections")

                # Control frame rate
                await asyncio.sleep(self.frame_delay)

        except Exception as e:
            logger.error(f"Error in stream loop: {e}")
        finally:
            logger.info("Camera stream loop stopped")

    async def start_stream(self):
        """Start the camera stream"""
        if self.is_streaming:
            logger.warning("Stream is already running")
            return {"status": "already_streaming"}

        try:
            # Load model and start camera
            self.load_model()
            self.start_camera()

            # Start streaming
            self.is_streaming = True
            self.stream_task = asyncio.create_task(self.stream_loop())

            logger.info("Camera stream started")
            await manager.broadcast({
                "type": "status",
                "message": "stream_started",
                "streaming": True
            })

            return {"status": "stream_started"}

        except Exception as e:
            logger.error(f"Error starting stream: {e}")
            self.is_streaming = False
            return {"status": "error", "message": str(e)}

    async def stop_stream(self):
        """Stop the camera stream"""
        if not self.is_streaming:
            logger.warning("Stream is not running")
            return {"status": "not_streaming"}

        try:
            # Stop streaming
            self.is_streaming = False

            # Wait for stream task to complete
            if self.stream_task:
                await self.stream_task
                self.stream_task = None

            # Stop camera
            self.stop_camera()

            logger.info("Camera stream stopped")
            await manager.broadcast({
                "type": "status",
                "message": "stream_stopped",
                "streaming": False
            })

            return {"status": "stream_stopped"}

        except Exception as e:
            logger.error(f"Error stopping stream: {e}")
            return {"status": "error", "message": str(e)}

    def get_status(self) -> dict:
        """Get current streaming status"""
        return {
            "streaming": self.is_streaming,
            "camera_open": self.capture is not None and self.capture.isOpened(),
            "model_loaded": self.model is not None,
            "active_connections": manager.get_connection_count()
        }


# Global camera service instance
camera_service = CameraStreamService()