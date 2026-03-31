'use client';

import React, { useRef, useEffect, useCallback } from 'react';
import { Camera, Wifi, WifiOff, AlertCircle } from 'lucide-react';
import { useCameraStream } from '../hooks/useCameraStream';

const VIDEO_URL = process.env.NEXT_PUBLIC_VIDEO_URL || '';
const MJPEG_SRC = VIDEO_URL
  ? `${VIDEO_URL}/stream?topic=/ascamera/camera_publisher/rgb0/image`
  : '';

const FSM_BANNER = {
  IDLE:                     { text: 'IDLE',              color: '#9ca3af' },
  NAVIGATE_TO_SHELF:        { text: 'NAVIGATING TO',     color: '#60a5fa' },
  ALIGN_WITH_SHELF:         { text: 'ALIGNING',          color: '#60a5fa' },
  FETCH_EXPECTED_INVENTORY: { text: 'LOADING INVENTORY', color: '#facc15' },
  RUN_YOLO_DETECTION:       { text: 'DETECTING',         color: '#4ade80' },
  COMPARE_INVENTORY:        { text: 'COMPARING',         color: '#facc15' },
  SEND_RESULTS:             { text: 'SENDING RESULTS',   color: '#facc15' },
  RETURN_HOME:              { text: 'RETURNING HOME',    color: '#60a5fa' },
  ERROR:                    { text: 'ERROR',             color: '#f87171' },
};

export default function CameraFeed({ isActive = true }) {
  const {
    frame,
    detections,
    liveDetections,
    fsmState,
    fsmShelfId,
    connected,
    streaming,
    error,
    frameNumber,
    startStream,
    stopStream
  } = useCameraStream();

  const useMjpeg = !!MJPEG_SRC;
  const canvasRef = useRef(null);
  const imgRef = useRef(null);

  // Auto-start stream when rover becomes active (fallback mode only)
  useEffect(() => {
    if (!useMjpeg && isActive && connected && !streaming) {
      startStream();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isActive, connected, streaming, useMjpeg]);

  // ---- Canvas overlay drawing ----
  const drawOverlay = useCallback(() => {
    const canvas = canvasRef.current;
    const img = imgRef.current;
    if (!canvas || !img) return;

    // Match canvas resolution to displayed image size
    const rect = img.getBoundingClientRect();
    canvas.width = rect.width;
    canvas.height = rect.height;

    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Draw bounding boxes
    if (liveDetections.length > 0) {
      const sample = liveDetections[0];
      const srcW = sample.width || 640;
      const srcH = sample.height || 480;
      const scaleX = canvas.width / srcW;
      const scaleY = canvas.height / srcH;

      liveDetections.forEach((det) => {
        const [x1, y1, x2, y2] = det.box;
        const dx = x1 * scaleX;
        const dy = y1 * scaleY;
        const dw = (x2 - x1) * scaleX;
        const dh = (y2 - y1) * scaleY;

        // Box
        ctx.strokeStyle = '#4ade80';
        ctx.lineWidth = 2;
        ctx.strokeRect(dx, dy, dw, dh);

        // Label background
        const label = `${det.class_name} ${(det.score * 100).toFixed(0)}%`;
        ctx.font = 'bold 14px monospace';
        const textW = ctx.measureText(label).width;
        ctx.fillStyle = 'rgba(0,0,0,0.7)';
        ctx.fillRect(dx, dy - 20, textW + 8, 20);

        // Label text
        ctx.fillStyle = '#4ade80';
        ctx.fillText(label, dx + 4, dy - 5);
      });
    }

    // Draw FSM state banner
    const banner = FSM_BANNER[fsmState] || FSM_BANNER.IDLE;
    let bannerText = banner.text;
    if (fsmState === 'NAVIGATE_TO_SHELF' && fsmShelfId) {
      bannerText += ` ${fsmShelfId}`;
    }

    ctx.font = 'bold 18px monospace';
    const tw = ctx.measureText(bannerText).width;
    ctx.fillStyle = 'rgba(0,0,0,0.7)';
    ctx.fillRect(8, 8, tw + 16, 30);
    ctx.fillStyle = banner.color;
    ctx.fillText(bannerText, 16, 29);
  }, [liveDetections, fsmState, fsmShelfId]);

  // Redraw overlay when detections or FSM state change
  useEffect(() => {
    if (useMjpeg) {
      drawOverlay();
    }
  }, [useMjpeg, drawOverlay]);

  // Also redraw on window resize so the canvas stays aligned
  useEffect(() => {
    if (!useMjpeg) return;
    const handleResize = () => drawOverlay();
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [useMjpeg, drawOverlay]);

  // Pick which detections list to show in the stats panel
  const activeDetections = useMjpeg ? liveDetections : detections;

  return (
    <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
      {/* Header with Status */}
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center space-x-2">
          <Camera size={20} className="text-gray-400" />
          <h3 className="text-white font-semibold">Live Camera Feed</h3>
        </div>
        <div className="flex items-center space-x-3">
          {/* Connection Status */}
          <div className="flex items-center space-x-2">
            {connected ? (
              <>
                <Wifi size={16} className="text-green-400" />
                <span className="text-xs text-green-400">Connected</span>
              </>
            ) : (
              <>
                <WifiOff size={16} className="text-red-400" />
                <span className="text-xs text-red-400">Disconnected</span>
              </>
            )}
          </div>

          {/* Streaming Status (fallback mode) */}
          {!useMjpeg && streaming && (
            <div className="flex items-center bg-red-600 text-white px-3 py-1 rounded text-xs font-semibold">
              <div className="w-2 h-2 bg-white rounded-full mr-2 animate-pulse"></div>
              REC
            </div>
          )}

          {/* MJPEG live indicator */}
          {useMjpeg && (
            <div className="flex items-center bg-green-600 text-white px-3 py-1 rounded text-xs font-semibold">
              <div className="w-2 h-2 bg-white rounded-full mr-2 animate-pulse"></div>
              LIVE
            </div>
          )}

          {/* Frame Counter (fallback mode) */}
          {!useMjpeg && streaming && (
            <span className="text-xs text-gray-500">
              Frame #{frameNumber}
            </span>
          )}
        </div>
      </div>

      {/* Video Feed Area */}
      <div className="bg-gradient-to-br from-blue-950 to-gray-950 rounded-lg aspect-video flex items-center justify-center border-2 border-gray-800 relative overflow-hidden">
        {error && (
          <div className="absolute top-4 left-4 right-4 bg-red-900/90 text-red-200 px-4 py-2 rounded flex items-center space-x-2 z-20">
            <AlertCircle size={16} />
            <span className="text-sm">{error}</span>
          </div>
        )}

        {useMjpeg ? (
          /* -------- MJPEG MODE -------- */
          <>
            <img
              ref={imgRef}
              src={MJPEG_SRC}
              alt="Camera Feed"
              className="w-full h-full object-contain"
              onLoad={drawOverlay}
            />
            <canvas
              ref={canvasRef}
              className="absolute inset-0 w-full h-full pointer-events-none"
            />
          </>
        ) : (
          /* -------- FALLBACK BASE64 MODE -------- */
          <>
            {!connected ? (
              <div className="text-center text-gray-600">
                <WifiOff size={64} className="mx-auto mb-4 opacity-50" />
                <p className="font-semibold">Connecting to camera...</p>
                <p className="text-sm">Establishing WebSocket connection</p>
              </div>
            ) : !streaming || !frame ? (
              <div className="text-center text-gray-600">
                <Camera size={64} className="mx-auto mb-4 opacity-50" />
                <p className="font-semibold">Camera feed inactive</p>
                <p className="text-sm">
                  {isActive ? 'Starting camera stream...' : 'Start rover to activate camera'}
                </p>
              </div>
            ) : (
              <>
                <img
                  src={`data:image/jpeg;base64,${frame}`}
                  alt="Camera Feed"
                  className="w-full h-full object-contain"
                />
                {detections.length > 0 && (
                  <div className="absolute bottom-4 left-4 bg-black/70 backdrop-blur-sm px-4 py-2 rounded-lg">
                    <p className="text-green-400 text-sm font-semibold mb-1">
                      Detections: {detections.length}
                    </p>
                    <div className="space-y-1 max-h-32 overflow-y-auto">
                      {detections.slice(0, 5).map((detection, idx) => (
                        <div key={idx} className="text-xs text-white flex items-center justify-between space-x-3">
                          <span className="font-medium">{detection.object_name}</span>
                          <span className="text-green-400">{(detection.confidence * 100).toFixed(0)}%</span>
                        </div>
                      ))}
                      {detections.length > 5 && (
                        <p className="text-xs text-gray-400 mt-1">
                          +{detections.length - 5} more
                        </p>
                      )}
                    </div>
                  </div>
                )}
                <div className="absolute inset-0 pointer-events-none">
                  <div className="absolute top-0 left-0 right-0 h-1 bg-gradient-to-r from-transparent via-red-500 to-transparent animate-pulse"></div>
                </div>
              </>
            )}
          </>
        )}
      </div>

      {/* Manual Controls (for debugging, fallback mode only) */}
      {!useMjpeg && process.env.NODE_ENV === 'development' && (
        <div className="mt-4 flex items-center justify-center space-x-2">
          <button
            onClick={startStream}
            disabled={!connected || streaming}
            className="px-3 py-1 bg-green-600 hover:bg-green-700 disabled:bg-gray-700 disabled:opacity-50 rounded text-sm transition-all"
          >
            Start Stream
          </button>
          <button
            onClick={stopStream}
            disabled={!connected || !streaming}
            className="px-3 py-1 bg-red-600 hover:bg-red-700 disabled:bg-gray-700 disabled:opacity-50 rounded text-sm transition-all"
          >
            Stop Stream
          </button>
        </div>
      )}

      {/* Detection Statistics */}
      {activeDetections.length > 0 && (
        <div className="mt-4 grid grid-cols-3 gap-2">
          {Object.entries(
            activeDetections.reduce((acc, det) => {
              const name = det.class_name || det.object_name;
              acc[name] = (acc[name] || 0) + 1;
              return acc;
            }, {})
          ).slice(0, 3).map(([name, count]) => (
            <div key={name} className="bg-gray-800 rounded px-3 py-2 text-center">
              <p className="text-xs text-gray-400">{name}</p>
              <p className="text-lg font-bold text-blue-400">{count}</p>
            </div>
          ))}
        </div>
      )}
    </div>
  );
}
