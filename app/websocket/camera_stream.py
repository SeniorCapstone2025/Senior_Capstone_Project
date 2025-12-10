import cv2
import base64
import asyncio
import logging
from ultralytics import YOLO
from qreader import QReader
from enum import Enum
from app.config import get_settings
from app.database import save_detected_object, get_inventory_item
from app.websocket.connection_manager import manager
from typing import Optional, Dict, Any
import numpy as np
import time

logger = logging.getLogger(__name__)


class CameraMode(Enum):
    """Camera operation modes"""
    QR_SCAN = "qr_scan"
    OBJECT_DETECTION = "object_detection"


class CameraStreamService:
    """Handles webcam streaming with YOLO object detection"""

    def __init__(self):
        self.model: Optional[YOLO] = None
        self.qr_reader: Optional[QReader] = None
        self.capture: Optional[cv2.VideoCapture] = None
        self.is_streaming = False
        self.stream_task: Optional[asyncio.Task] = None
        self.fps = 30
        self.frame_delay = 1.0 / self.fps

        # QR scanning workflow state
        self.current_mode: CameraMode = CameraMode.QR_SCAN
        self.expected_item: Optional[Dict[str, Any]] = None
        self.detection_start_time: Optional[float] = None
        self.detection_timeout: float = 20.0  # 20 seconds timeout

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

    def load_qr_reader(self):
        """Load the QR code reader"""
        if self.qr_reader is None:
            try:
                logger.info("Loading QR code reader")
                self.qr_reader = QReader()
                logger.info("QR reader loaded successfully")
            except Exception as e:
                logger.error(f"Error loading QR reader: {e}")
                raise

    async def scan_qr_code(self, frame: np.ndarray) -> Optional[str]:
        """Scan QR code from frame and return item_id"""
        try:
            # Detect and decode QR codes in the frame
            decoded_text = self.qr_reader.detect_and_decode(image=frame)

            if decoded_text and len(decoded_text) > 0:
                # Get the first decoded QR code
                item_id = decoded_text[0]
                if item_id:  # Check if not None or empty
                    logger.info(f"QR code detected: {item_id}")
                    return item_id
            return None
        except Exception as e:
            logger.error(f"Error scanning QR code: {e}")
            return None

    async def fetch_inventory_item(self, item_id: str) -> Optional[Dict[str, Any]]:
        """Fetch inventory item from database"""
        try:
            result = await asyncio.to_thread(get_inventory_item, item_id)

            if result and result.data and len(result.data) > 0:
                item = result.data[0]
                self.expected_item = item
                logger.info(f"Fetched inventory item: {item}")
                return item
            else:
                logger.warning(f"Item {item_id} not found in inventory")
                return None
        except Exception as e:
            logger.error(f"Error fetching inventory item {item_id}: {e}")
            return None

    async def check_for_expected_item(self, detections: list) -> Optional[Dict[str, Any]]:
        """Check if expected item is in the detections"""
        if not self.expected_item or not detections:
            return None

        expected_name = self.expected_item.get("name", "").lower()

        for detection in detections:
            object_name = detection.get("object_name", "").lower()
            confidence = detection.get("confidence", 0)

            # Check if the detected object matches the expected item
            if expected_name in object_name or object_name in expected_name:
                if confidence > 0.3:  # Require higher confidence for matches
                    logger.info(f"Expected item '{expected_name}' found! Confidence: {confidence}")
                    return detection

        return None

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
        """Main streaming loop with QR scan → object detection workflow"""
        logger.info("Starting camera stream loop in QR_SCAN mode")
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

                annotated_frame = frame
                detections = []

                # State machine logic
                if self.current_mode == CameraMode.QR_SCAN:
                    # QR SCAN MODE: Look for QR codes
                    item_id = await self.scan_qr_code(frame)

                    if item_id:
                        # QR code found! Fetch inventory item
                        logger.info(f"QR code scanned: {item_id}. Fetching inventory item...")
                        await manager.broadcast({
                            "type": "qr_detected",
                            "item_id": item_id,
                            "message": f"QR code detected: {item_id}"
                        })

                        item = await self.fetch_inventory_item(item_id)

                        if item:
                            # Successfully fetched item, switch to object detection mode
                            self.current_mode = CameraMode.OBJECT_DETECTION
                            self.detection_start_time = time.time()
                            logger.info(f"Switching to OBJECT_DETECTION mode. Looking for: {item.get('name')}")

                            await manager.broadcast({
                                "type": "mode_changed",
                                "mode": "object_detection",
                                "expected_item": item,
                                "message": f"Now searching for: {item.get('name')}"
                            })
                        else:
                            # Item not found in inventory
                            await manager.broadcast({
                                "type": "error",
                                "message": f"Item {item_id} not found in inventory"
                            })

                    # Draw "QR SCAN MODE" text on frame
                    cv2.putText(annotated_frame, "QR SCAN MODE", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                elif self.current_mode == CameraMode.OBJECT_DETECTION:
                    # OBJECT DETECTION MODE: Look for expected item
                    annotated_frame, detections = await self.process_frame(frame)

                    # Check if expected item is detected
                    matched_detection = await self.check_for_expected_item(detections)

                    if matched_detection:
                        # Expected item found!
                        logger.info(f"✓ Expected item found: {self.expected_item.get('name')}")

                        await manager.broadcast({
                            "type": "item_found",
                            "expected_item": self.expected_item,
                            "detection": matched_detection,
                            "message": f"✓ Found: {self.expected_item.get('name')}"
                        })

                        # TODO: Save to database that item was successfully found
                        # Example: save_item_search_result(
                        #     item_id=self.expected_item['item_id'],
                        #     status='found',
                        #     detection=matched_detection
                        # )

                        # Reset to QR scan mode
                        self.current_mode = CameraMode.QR_SCAN
                        self.expected_item = None
                        self.detection_start_time = None
                        logger.info("Returning to QR_SCAN mode")

                        await manager.broadcast({
                            "type": "mode_changed",
                            "mode": "qr_scan",
                            "message": "Returning to QR scan mode"
                        })

                    else:
                        # Check for timeout
                        elapsed_time = time.time() - self.detection_start_time
                        remaining_time = self.detection_timeout - elapsed_time

                        if elapsed_time >= self.detection_timeout:
                            # Timeout! Item not found
                            logger.warning(f"✗ Timeout: Item '{self.expected_item.get('name')}' not found after {self.detection_timeout}s")

                            await manager.broadcast({
                                "type": "item_not_found",
                                "expected_item": self.expected_item,
                                "elapsed_time": elapsed_time,
                                "message": f"✗ Not found: {self.expected_item.get('name')} (timeout)"
                            })

                            # TODO: Save to database that item was not found
                            # Example: save_item_search_result(
                            #     item_id=self.expected_item['item_id'],
                            #     status='not_found',
                            #     timeout=True
                            # )

                            # Reset to QR scan mode
                            self.current_mode = CameraMode.QR_SCAN
                            self.expected_item = None
                            self.detection_start_time = None
                            logger.info("Returning to QR_SCAN mode")

                            await manager.broadcast({
                                "type": "mode_changed",
                                "mode": "qr_scan",
                                "message": "Returning to QR scan mode"
                            })

                        else:
                            # Still searching, draw countdown on frame
                            cv2.putText(annotated_frame, f"SEARCHING: {self.expected_item.get('name')}", (10, 30),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                            cv2.putText(annotated_frame, f"Time remaining: {int(remaining_time)}s", (10, 60),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

                # Encode and broadcast frame
                frame_base64 = self.encode_frame(annotated_frame)

                if frame_base64:
                    message = {
                        "type": "frame",
                        "frame": frame_base64,
                        "detections": detections,
                        "mode": self.current_mode.value,
                        "expected_item": self.expected_item.get('name') if self.expected_item else None,
                        "frame_number": frame_count,
                        "timestamp": time.time()
                    }

                    await manager.broadcast(message)

                    frame_count += 1
                    if frame_count % 30 == 0:
                        logger.debug(f"Streamed {frame_count} frames, mode: {self.current_mode.value}")

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
            # Load YOLO model, QR reader, and start camera
            self.load_model()
            self.load_qr_reader()
            self.start_camera()

            # Reset state to QR_SCAN mode
            self.current_mode = CameraMode.QR_SCAN
            self.expected_item = None
            self.detection_start_time = None

            # Start streaming
            self.is_streaming = True
            self.stream_task = asyncio.create_task(self.stream_loop())

            logger.info("Camera stream started in QR_SCAN mode")
            await manager.broadcast({
                "type": "status",
                "message": "stream_started",
                "streaming": True,
                "mode": self.current_mode.value
            })

            return {"status": "stream_started", "mode": self.current_mode.value}

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