# Inventory Metrics Integration Guide

## Overview

The Inventory Metrics screen is now connected to the backend database and API. This guide explains how to seed data, run the system, and use the endpoints.

## Architecture

### Backend Flow
1. **API Endpoints** (`app/routes/inventory.py`)
   - `POST /inventory/scans` - Create a new scan session
   - `GET /inventory/scans/{scan_id}` - Fetch scan data with results
   - `PATCH /inventory/scans/{scan_id}/status` - Update scan status
   - `POST /inventory/scans/{scan_id}/results` - Save shelf scan results
   - `GET /inventory/scans/{scan_id}/results` - Get all results for a scan
   - `GET /inventory/shelves/{shelf_id}/expected-items` - Get shelf inventory

2. **Database Layer** (`app/database.py`)
   - Stores scan sessions in `scans` table
   - Stores per-shelf results in `scan_results` table
   - Manages inventory data in related tables

### Frontend Flow
1. **InventoryMetrics Component** (`frontend/src/components/InventoryMetrics.js`)
   - Fetches scan data on component mount
   - Transforms API response to component state
   - Displays real-time inventory metrics
   - Supports refresh to update data

## Setup and Running

### 1. Start the Backend

```bash
# Navigate to project root
cd Senior_Capstone_Project

# Install dependencies (if not already done)
pip install -r requirements.txt

# Run the backend
python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

### 2. Seed Demo Data (Optional)

To populate the database with demo inventory data:

```bash
# From the project root
python -c "from app.utils.seed_data import seed_demo_scan_data; seed_demo_scan_data()"
```

This creates:
- A scan session: `SCN-20240318-004`
- 6 shelves with varies statuses (OK, Warning, Missing, Unexpected)
- Full inventory results with expected/detected/missing/unexpected items

### 3. Start the Frontend

```bash
cd frontend/ros2-rover-dashboard

# Install dependencies (if not already done)
npm install

# Start development server
npm run dev
```

The frontend will be available at `http://localhost:3000`

### 4. Configure API URL (if needed)

Edit `frontend/ros2-rover-dashboard/.env.local`:

```env
NEXT_PUBLIC_API_URL=http://localhost:8000
```

## API Endpoints Reference

### Create a Scan

```bash
curl -X POST http://localhost:8000/inventory/scans \
  -H "Content-Type: application/json" \
  -d '{
    "scan_id": "SCN-20240318-004",
    "shelf_ids": ["A3-S1", "A3-S2", "A4-S1"],
    "operator": "M.A.R.S. Unit 01",
    "aisle": "A3–A7"
  }'
```

### Fetch Scan Data

```bash
curl http://localhost:8000/inventory/scans/SCN-20240318-004
```

Returns:
```json
{
  "status": "success",
  "scan": {
    "scan_id": "SCN-20240318-004",
    "status": "completed",
    "shelf_ids": ["A3-S1", "A3-S2", "A4-S1"],
    "created_at": "2024-03-18T14:22:11",
    "completed_at": "2024-03-18T14:47:03"
  },
  "results": [
    {
      "scan_id": "SCN-20240318-004",
      "shelf_id": "A3-S1",
      "expected_items": [...],
      "detected_items": [...],
      "missing_items": [],
      "unexpected_items": [],
      "match": true
    }
  ],
  "metrics": {
    "total_shelves": 3,
    "total_expected": 50,
    "total_found": 48,
    "total_missing": 2,
    "total_unexpected": 0,
    "accuracy": 96.0,
    "ok_shelves": 2,
    "warning_shelves": 1,
    "missing_shelves": 0,
    "unexpected_shelves": 0
  }
}
```

### Save Scan Results for a Shelf

```bash
curl -X POST http://localhost:8000/inventory/scans/SCN-20240318-004/results \
  -H "Content-Type: application/json" \
  -d '{
    "shelf_id": "A3-S1",
    "expected_items": [
      {"sku": "SKU-10041", "name": "Wireless Keyboard MK-7", "qty": 4},
      {"sku": "SKU-10042", "name": "Monitor Stand 27\"", "qty": 2}
    ],
    "detected_items": [
      {"sku": "SKU-10041", "name": "Wireless Keyboard MK-7", "qty": 4},
      {"sku": "SKU-10042", "name": "Monitor Stand 27\"", "qty": 2}
    ],
    "missing_items": [],
    "unexpected_items": [],
    "match": true
  }'
```

### Update Scan Status

```bash
curl -X PATCH http://localhost:8000/inventory/scans/SCN-20240318-004/status \
  -H "Content-Type: application/json" \
  -d '{"status": "completed"}'
```

## Database Schema

### scans Table
```sql
CREATE TABLE scans (
    id INTEGER PRIMARY KEY,
    scan_id TEXT UNIQUE NOT NULL,
    status TEXT DEFAULT 'pending',  -- pending, in_progress, completed, failed
    shelf_ids TEXT NOT NULL,        -- JSON array
    created_at TEXT DEFAULT datetime('now'),
    completed_at TEXT
);
```

### scan_results Table
```sql
CREATE TABLE scan_results (
    id INTEGER PRIMARY KEY,
    scan_id TEXT NOT NULL,
    shelf_id TEXT NOT NULL,
    expected_items TEXT DEFAULT '[]',      -- JSON array
    detected_items TEXT DEFAULT '[]',      -- JSON array
    missing_items TEXT DEFAULT '[]',       -- JSON array
    unexpected_items TEXT DEFAULT '[]',    -- JSON array
    match INTEGER DEFAULT 0,               -- Boolean
    scanned_at TEXT DEFAULT datetime('now')
);
```

## Component Usage

### In a Next.js Page

```javascript
import InventoryMetrics from '@/components/InventoryMetrics';

export default function InventoryPage() {
  return <InventoryMetrics />;
}
```

### Passing a Custom Scan ID

Edit the component to support URL parameters:

```javascript
'use client';

import { useSearchParams } from 'next/navigation';
import InventoryMetrics from '@/components/InventoryMetrics';

export default function InventoryPage() {
  const searchParams = useSearchParams();
  const scanId = searchParams.get('scanId') || 'SCN-20240318-004';

  return <InventoryMetrics scanId={scanId} />;
}
```

Then update the component to accept `scanId` as a prop.

## Troubleshooting

### "Failed to load inventory data" Error

1. **Check backend is running**: `curl http://localhost:8000/health`
2. **Check API URL**: Verify `NEXT_PUBLIC_API_URL` in `.env.local`
3. **Check database**: Verify scan data exists with correct scan_id
4. **Check CORS**: Backend allows all origins by default (see `app/main.py`)

### Empty Shelves List

1. **Seed data**: Run `python -c "from app.utils.seed_data import seed_demo_scan_data; seed_demo_scan_data()"`
2. **Verify scan_id**: Make sure the scan_id being fetched exists in the database
3. **Check logs**: Review backend logs for errors

### CORS Issues

If frontend can't reach backend:

1. Verify `allow_origins=["*"]` in `app/main.py` (already configured)
2. Check network tab in browser developer tools
3. Ensure backend is running on `http://localhost:8000`

## Real-Time Updates (Future Enhancement)

For live updates as scans progress:

1. **WebSocket Integration**: Use the existing WebSocket route in `app/routes/websocket_route.py`
2. **Polling**: Frontend can call refresh endpoint periodically
3. **Server-Sent Events**: Implement SSE endpoint in backend

## Integration with ROS2

The inventory scan data can be populated by:

1. **detect_and_scan_node**: Python ROS2 node that performs QR/barcode scanning
2. **Sends detections** to `/inventory/scans/{scan_id}/results` endpoint
3. **Component automatically displays** updated results

Example flow:
```
ROS2 Node (detect_and_scan)
    ↓
    POST to /inventory/scans/{scan_id}/results
    ↓
Database (scan_results)
    ↓
GET /inventory/scans/{scan_id}
    ↓
Frontend (InventoryMetrics.js)
    ↓
Live Display
```

## Testing Checklist

- [ ] Backend running on port 8000
- [ ] Frontend running on port 3000
- [ ] Demo data seeded: `SCN-20240318-004`
- [ ] Pages load without "Failed to load" error
- [ ] Metrics show accurate totals
- [ ] Shelves list displays 6 items
- [ ] Clicking shelves updates detail view
- [ ] Refresh button updates last refresh time
- [ ] Filter and search work correctly
- [ ] Error handling displays gracefully

## Next Steps

1. **Integrate with ROS2 scanning**: Create nodes to populate scan_results
2. **Add real-time WebSocket updates**: For live scan progress
3. **Implement authentication**: Secure API endpoints
4. **Add PDF export**: Generate scan reports
5. **Historical analysis**: Store and compare past scans
6. **Barcode/QR integration**: Auto-detect items via camera

---

For questions or issues, check the backend logs and browser console for debugging information.
