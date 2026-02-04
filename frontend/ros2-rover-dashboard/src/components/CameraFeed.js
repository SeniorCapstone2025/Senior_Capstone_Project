'use client';

import React from 'react';
import { Camera, Wifi, WifiOff, AlertCircle } from 'lucide-react';
import { useCameraStream } from '../hooks/useCameraStream';

export default function CameraFeed({ isActive = true }) {
  const {
    frame,
    detections,
    connected,
    streaming,
    error,
    frameNumber,
    startStream,
    stopStream
  } = useCameraStream();

  // Auto-start stream when rover becomes active
  React.useEffect(() => {
    if (isActive && connected && !streaming) {
      console.log('Starting stream because isActive=true, connected=true, streaming=false');
      startStream();
    }
    // Note: We don't auto-stop the stream when isActive becomes false
    // This prevents the stream from stopping when status updates to 'idle'
    // User can manually stop using the stop button if needed
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [isActive, connected, streaming]);

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

          {/* Streaming Status */}
          {streaming && (
            <div className="flex items-center bg-red-600 text-white px-3 py-1 rounded text-xs font-semibold">
              <div className="w-2 h-2 bg-white rounded-full mr-2 animate-pulse"></div>
              REC
            </div>
          )}

          {/* Frame Counter */}
          {streaming && (
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
            {/* Video Frame */}
            <img
              src={`data:image/jpeg;base64,${frame}`}
              alt="Camera Feed"
              className="w-full h-full object-contain"
            />

            {/* Detection Overlay Info */}
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

            {/* Recording Indicator Pulse */}
            <div className="absolute inset-0 pointer-events-none">
              <div className="absolute top-0 left-0 right-0 h-1 bg-gradient-to-r from-transparent via-red-500 to-transparent animate-pulse"></div>
            </div>
          </>
        )}
      </div>

      {/* Manual Controls (for debugging) */}
      {process.env.NODE_ENV === 'development' && (
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
      {streaming && detections.length > 0 && (
        <div className="mt-4 grid grid-cols-3 gap-2">
          {Object.entries(
            detections.reduce((acc, det) => {
              acc[det.object_name] = (acc[det.object_name] || 0) + 1;
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