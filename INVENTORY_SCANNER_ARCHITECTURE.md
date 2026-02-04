# Autonomous Inventory Scanner - Technical Architecture

## System Overview

An autonomous robot system that scans inventory on shelves using QR codes and computer vision, communicating with a FastAPI backend and web dashboard.

## Architecture Diagram

```
Web Dashboard → FastAPI Backend → rosbridge_websocket → ROS2 Nodes
     ↓                ↑                                        ↓
  User clicks    Results flow                          Robot operates
  "Start Scan"   back here                            autonomously
```

## ROS2 Node Architecture

### 1. inventory_fsm_node (Main Controller)
**File**: `src/app/app/inventory_fsm_node.py`
**Purpose**: Finite State Machine coordinating entire scan process
**Services**:
- `~/enter`, `~/exit`, `~/start`, `~/stop`, `~/init_finish` (standard lifecycle)
- `~/start_scan` - Trigger complete scan sequence from dashboard
- `~/emergency_stop` - Immediate halt

**Subscriptions**:
- `/qr_scanner/qr_code` (String) - QR code data (shelf ID)
- `/yolo_detector/detections` (custom msg) - Object detection results
- `/waypoint_navigator/status` (String) - Navigation status

**Publishers**:
- `/waypoint_navigator/goal` (String) - Target waypoint ID
- `/qr_scanner/trigger` (Bool) - Start/stop QR scanning
- `/yolo_detector/trigger` (Bool) - Start/stop object detection

**HTTP Communication**:
- POST `/api/scan/start` - Receive scan request from dashboard
- GET `/api/inventory/shelf/{id}` - Get expected inventory for shelf
- POST `/api/scan/results` - Send comparison results back

**States**:
```
IDLE
  ↓ (start_scan request)
NAVIGATE_TO_SHELF
  ↓ (waypoint reached)
ALIGN_WITH_SHELF
  ↓ (alignment complete)
SCAN_QR
  ↓ (QR detected - returns shelf_id)
FETCH_EXPECTED_INVENTORY (GET from backend)
  ↓ (data received)
RUN_YOLO_DETECTION
  ↓ (detection complete)
COMPARE_INVENTORY (presence/absence only, no counting)
  ↓ (comparison done)
SEND_RESULTS_TO_BACKEND
  ↓ (upload complete)
RETURN_HOME
  ↓ (home reached)
IDLE
```

---

### 2. waypoint_navigator_node (Simple Navigation)
**File**: `src/app/app/waypoint_navigator_node.py`
**Purpose**: Navigate robot to predefined waypoint coordinates
**Approach**: Simple lidar-based navigation with obstacle avoidance

**Subscriptions**:
- `/scan_raw` (LaserScan) - Lidar data for obstacle detection
- `/waypoint_navigator/goal` (String) - Target waypoint ID from FSM

**Publishers**:
- `/controller/cmd_vel` (Twist) - Motor commands
- `/waypoint_navigator/status` (String) - "navigating", "reached", "blocked", "failed"

**Parameters**:
- `waypoints_file` - YAML file with waypoint definitions
- `speed_linear` - Forward speed (default: 0.2 m/s)
- `speed_angular` - Turn speed (default: 0.5 rad/s)
- `goal_tolerance` - Distance to consider goal reached (default: 0.15 m)
- `obstacle_distance` - Minimum safe distance (default: 0.3 m)

**Algorithm**:
1. Load waypoint from YAML config
2. Calculate heading to target using simple geometry
3. Turn toward target
4. Move forward while checking lidar for obstacles
5. Stop when within tolerance of goal
6. Publish "reached" status

---

### 3. qr_scanner_node (QR Code Detection)
**File**: `src/app/app/qr_scanner_node.py`
**Purpose**: Detect and decode QR codes from camera feed
**Library**: `pyzbar`

**Subscriptions**:
- `/ascamera/camera_publisher/rgb0/image` (Image) - Camera feed
- `/qr_scanner/trigger` (Bool) - Enable/disable scanning

**Publishers**:
- `/qr_scanner/qr_code` (String) - Detected QR code data (shelf ID)
- `/qr_scanner/debug_image` (Image) - Annotated image with QR highlighted

**Services**:
- `~/enter`, `~/exit`, `~/start`, `~/stop`, `~/init_finish`

**Behavior**:
- Only process images when triggered by FSM
- Publish first valid QR code detected (this is the shelf ID)
- Stop scanning after successful detection
- Timeout after 10 seconds if no QR found

---

### 4. yolo_detector_node (Object Detection)
**File**: `src/app/app/yolo_detector_node.py`
**Purpose**: Detect objects on shelf using YOLOv8
**Library**: `ultralytics`
**Model**: YOLOv8n (nano) with pre-trained COCO weights

**Subscriptions**:
- `/ascamera/camera_publisher/rgb0/image` (Image) - Camera feed
- `/yolo_detector/trigger` (Bool) - Enable/disable detection

**Publishers**:
- `/yolo_detector/detections` (DetectionArray) - List of detected objects
- `/yolo_detector/debug_image` (Image) - Annotated image with bounding boxes

**Services**:
- `~/enter`, `~/exit`, `~/start`, `~/stop`, `~/init_finish`
- `~/set_confidence` (SetFloat64) - Adjust confidence threshold

**Behavior**:
- Capture and process single frame when triggered
- Run YOLOv8 inference
- Filter detections by confidence threshold (default: 0.5)
- Publish list of detected class names (presence only, no counting for MVP)
- Return debug image with bounding boxes

**Detection Message Format**:
```python
# DetectionArray.msg
std_msgs/Header header
Detection[] detections

# Detection.msg
string class_name
float32 confidence
int32 x
int32 y
int32 width
int32 height
```

---

## Configuration Files

### waypoints.yaml
**Location**: `src/app/config/waypoints.yaml`

```yaml
waypoints:
  home:
    x: 0.0
    y: 0.0
    theta: 0.0

  shelf_1:
    x: 2.5
    y: 1.0
    theta: 1.57  # Face shelf (90 degrees)

  shelf_2:
    x: 2.5
    y: 3.0
    theta: 1.57

  shelf_3:
    x: 5.0
    y: 1.0
    theta: 1.57
```

### inventory_config.yaml
**Location**: `src/app/config/inventory_config.yaml`

```yaml
backend:
  url: "http://localhost:8000"  # FastAPI backend URL
  timeout: 10  # seconds

navigation:
  linear_speed: 0.2  # m/s
  angular_speed: 0.5  # rad/s
  goal_tolerance: 0.15  # meters
  obstacle_threshold: 0.3  # meters

qr_scanner:
  timeout: 10.0  # seconds
  retry_count: 3

yolo_detector:
  model: "yolov8n.pt"  # Nano model for speed
  confidence_threshold: 0.5
  device: "cpu"  # Use "cuda" if GPU available
  image_size: 640

inventory_comparison:
  match_tolerance: 0.8  # Fuzzy matching threshold for class names
```

---

## FastAPI Backend Integration

### Required Endpoints

#### 1. Start Scan (Dashboard → Backend → ROS)
**Endpoint**: `POST /api/scan/start`
**Request Body**:
```json
{
  "scan_id": "unique-scan-id",
  "shelf_ids": ["shelf_1"]
}
```

**Backend Action**:
- Store scan request in database
- Forward to ROS2 via rosbridge_websocket:
```javascript
ros.callService({
  name: '/inventory_fsm/start_scan',
  serviceType: 'std_srvs/Trigger'
});
```

**Response**:
```json
{
  "success": true,
  "scan_id": "unique-scan-id",
  "status": "initiated"
}
```

---

#### 2. Get Shelf Inventory (ROS → Backend)
**Endpoint**: `GET /api/inventory/shelf/{id}`
**Example**: `GET /api/inventory/shelf/shelf_1`

**Response**:
```json
{
  "success": true,
  "shelf_id": "shelf_1",
  "expected_items": ["bottle", "cup", "book"]
}
```

**Note**: QR code scanned by robot IS the shelf_id. Simple direct lookup.

---

#### 3. Submit Results (ROS → Backend)
**Endpoint**: `POST /api/scan/results`
**Request Body**:
```json
{
  "scan_id": "unique-scan-id",
  "shelf_id": "shelf_1",
  "timestamp": "2026-01-19T10:30:00Z",
  "expected_items": ["bottle", "cup", "book"],
  "detected_items": ["bottle", "cup"],
  "missing_items": ["book"],
  "unexpected_items": [],
  "match": false,
  "image_base64": "iVBORw0KGgoAAAANS..."
}
```

**Response**:
```json
{
  "success": true,
  "message": "Results stored successfully"
}
```

**Note**: MVP only checks presence/absence of items, not counts.

---

## Data Flow Sequence

### Complete Scan Sequence

1. **User Action**: Click "Start Scan" on web dashboard
2. **Dashboard → Backend**: `POST /api/scan/start {"shelf_ids": ["shelf_1"]}`
3. **Backend → ROS2**: Call `/inventory_fsm/start_scan` via rosbridge
4. **FSM**: State = NAVIGATE_TO_SHELF
5. **FSM → Navigator**: Publish `"shelf_1"` to `/waypoint_navigator/goal`
6. **Navigator**: Drive to waypoint, publish "reached" when done
7. **FSM**: State = ALIGN_WITH_SHELF (fine positioning)
8. **FSM**: State = SCAN_QR
9. **FSM → QR Scanner**: Publish `True` to `/qr_scanner/trigger`
10. **QR Scanner**: Detect QR, publish `"shelf_1"` to `/qr_scanner/qr_code`
11. **FSM**: State = FETCH_EXPECTED_INVENTORY
12. **FSM → Backend**: `GET /api/inventory/shelf/shelf_1`
13. **Backend → FSM**: Return `{"expected_items": ["bottle", "cup", "book"]}`
14. **FSM**: State = RUN_YOLO_DETECTION
15. **FSM → YOLO**: Publish `True` to `/yolo_detector/trigger`
16. **YOLO**: Run detection, publish results to `/yolo_detector/detections`
17. **FSM**: State = COMPARE_INVENTORY
18. **FSM**: Compare expected vs detected items (presence/absence only)
    - Expected: ["bottle", "cup", "book"]
    - Detected: ["bottle", "cup"]
    - Missing: ["book"]
    - Unexpected: []
19. **FSM**: State = SEND_RESULTS_TO_BACKEND
20. **FSM → Backend**: `POST /api/scan/results` with comparison data
21. **Backend → Dashboard**: Update UI with results
22. **FSM**: State = RETURN_HOME
23. **FSM → Navigator**: Publish `"home"` to `/waypoint_navigator/goal`
24. **Navigator**: Drive home, publish "reached"
25. **FSM**: State = IDLE

---

## Implementation Timeline (4-6 Weeks)

### Week 1: Foundation
- [ ] Create all node skeletons with lifecycle services
- [ ] Setup config files (waypoints.yaml, inventory_config.yaml)
- [ ] Test basic FSM state transitions
- [ ] Setup FastAPI endpoints (stubs)

### Week 2: Navigation
- [ ] Implement waypoint_navigator_node
- [ ] Test manual navigation to shelf positions
- [ ] Calibrate waypoint coordinates in physical space
- [ ] Add obstacle avoidance

### Week 3: QR Scanning
- [ ] Implement qr_scanner_node
- [ ] Test QR detection reliability
- [ ] Integrate with FSM
- [ ] Connect to backend GET endpoint

### Week 4: Object Detection
- [ ] Implement yolo_detector_node
- [ ] Test YOLOv8 on shelf items
- [ ] Tune confidence thresholds
- [ ] Integrate with FSM

### Week 5: Integration
- [ ] Complete FSM inventory comparison logic (presence/absence)
- [ ] End-to-end testing of full workflow
- [ ] Backend integration (results upload)
- [ ] Error handling and recovery

### Week 6: Testing & Refinement
- [ ] Real-world testing in target environment
- [ ] Detection accuracy tuning
- [ ] Performance optimization
- [ ] Documentation and demo preparation

---

## Dependencies

### Python Packages
```txt
rclpy
cv_bridge
sensor_msgs
geometry_msgs
std_srvs
pyzbar
opencv-python
ultralytics
torch
torchvision
numpy
requests
pyyaml
```

### ROS2 Packages
- `ros_robot_controller` (existing)
- `controller` (existing)
- `peripherals` (camera drivers, existing)
- `interfaces` (custom messages - need to add DetectionArray)

---

## Risk Mitigation

### Technical Risks

**Risk**: QR codes not detected due to lighting/angle
**Mitigation**: Add shelf alignment step, test multiple QR sizes, add retry logic

**Risk**: YOLO misidentifies objects
**Mitigation**: Use high confidence threshold (0.5+), take multiple frames, manual review on dashboard

**Risk**: Navigation gets stuck
**Mitigation**: Timeout on each state, emergency stop service, simple waypoints reduce complexity

**Risk**: Backend communication fails
**Mitigation**: Retry logic, store results locally, upload when connection restored

### Timeline Risks

**Risk**: ROS2 learning curve
**Mitigation**: Copy existing patterns, start simple, incremental testing

**Risk**: Hardware issues (camera/lidar failures)
**Mitigation**: Test early, have backup hardware, focus on single shelf for MVP

---

## Success Metrics (MVP)

- [ ] Robot navigates to shelf position autonomously
- [ ] QR code detected with >90% success rate (QR = shelf_id)
- [ ] YOLOv8 detects common objects (bottles, cups, books)
- [ ] Inventory comparison identifies missing/unexpected items (presence only, no counting)
- [ ] Results successfully sent to backend
- [ ] Robot returns home after scan
- [ ] Complete cycle time < 2 minutes for single shelf
- [ ] Dashboard displays results in real-time

---

## Future Enhancements (Post-MVP)

- Item counting (quantity verification)
- Multi-shelf scanning in sequence
- 3D depth estimation for item positioning
- Custom YOLO model trained on specific inventory
- Automatic map generation and shelf discovery
- Battery level monitoring and auto-charging
- Scheduling system for regular scans
- Real-time video streaming to dashboard
- Voice feedback for operator
