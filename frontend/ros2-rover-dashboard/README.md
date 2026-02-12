# M.A.R.S. ROS2 Rover Dashboard

Mobile Autonomous Rover System - Inventory Scanning & Logging System

## Features

- **M.A.R.S. Branding** - Custom logo and branding throughout the interface
- **Real-time Rover Control** - Start, pause, cancel, and return commands
- **Live Camera Feed** - WebSocket-based video streaming with object detection
- **Backend Monitor Page** - Comprehensive view of all backend systems
- **Status Monitoring** - Battery level, connection status, and scan progress
- **Inventory Tracking** - Recent scans display with database sync status

## Project Structure

```
ros2-rover-dashboard/
├── src/
│   ├── app/
│   │   ├── layout.js          # Root layout with metadata
│   │   ├── page.js             # Main dashboard page
│   │   ├── globals.css         # Global styles
│   │   └── backend/
│   │       └── page.js         # Backend monitoring page
│   ├── components/
│   │   ├── RoverDashboard.js   # Main dashboard component
│   │   ├── CameraFeed.js       # Camera feed with WebSocket
│   │   └── BackendInfo.js      # Backend monitoring component
│   ├── hooks/
│   │   ├── useRoverCommands.js # Hook for sending commands
│   │   ├── useRoverStatus.js   # Hook for status polling
│   │   └── useCameraStream.js  # Hook for camera WebSocket
│   └── services/
│       └── api.js              # API service layer
├── package.json
├── next.config.js
├── tailwind.config.js
└── postcss.config.js
```

## Setup

1. **Install dependencies:**
   ```bash
   npm install
   ```

2. **Set environment variables (optional):**
   Create a `.env.local` file:
   ```
   NEXT_PUBLIC_API_URL=http://localhost:8000
   ```

3. **Run the development server:**
   ```bash
   npm run dev
   ```

4. **Open in browser:**
   Navigate to `http://localhost:3000`

## Backend API Requirements

The dashboard expects the following backend endpoints:

- `GET /status/` - Rover status (state, battery_level, etc.)
- `POST /command/` - Send commands (start, pause, cancel, return)
- `GET /health` - API health check
- `GET /command/test-db` - Database connection test
- `WS /ws/camera` - WebSocket for camera feed

## Navigation

- **Main Dashboard** - Default page with rover controls and camera feed
- **Backend Monitor** - Click the database icon (bottom of sidebar) to view backend status
- **Return to Dashboard** - Click the back arrow from the backend monitor page

## Features in Backend Monitor

- **API Health** - Service status, version, uptime
- **Database Status** - Connection status, table count, row count
- **Live Rover Status** - Real-time status data from the rover
- **API Endpoints** - Documentation of available endpoints
- **Connection Configuration** - Current API and WebSocket URLs

## Development

Build for production:
```bash
npm run build
npm start
```

## Technologies

- **Next.js 14** - React framework with App Router
- **Tailwind CSS** - Utility-first styling
- **Lucide React** - Icon library
- **WebSocket** - Real-time camera streaming
