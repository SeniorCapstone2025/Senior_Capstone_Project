import { useState, useEffect, useCallback, useRef } from 'react';

const WS_URL = 'ws://localhost:8001/ws/camera';

export function useCameraStream() {
  const [frame, setFrame] = useState(null);
  const [detections, setDetections] = useState([]);
  const [connected, setConnected] = useState(false);
  const [streaming, setStreaming] = useState(false);
  const [error, setError] = useState(null);
  const [frameNumber, setFrameNumber] = useState(0);

  const wsRef = useRef(null);
  const reconnectTimeoutRef = useRef(null);
  const reconnectAttemptsRef = useRef(0);
  const maxReconnectAttempts = 5;

  const connect = useCallback(() => {
    // Clear any existing reconnect timeout
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }

    // Don't reconnect if we've exceeded max attempts
    if (reconnectAttemptsRef.current >= maxReconnectAttempts) {
      setError('Max reconnection attempts reached');
      return;
    }

    try {
      const ws = new WebSocket(WS_URL);

      ws.onopen = () => {
        console.log('WebSocket connected');
        setConnected(true);
        setError(null);
        reconnectAttemptsRef.current = 0;
      };

      ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);

          switch (data.type) {
            case 'connected':
              console.log('Connected to camera stream:', data);
              if (data.status?.streaming) {
                setStreaming(true);
              }
              break;

            case 'frame':
              setFrame(data.frame);
              setDetections(data.detections || []);
              setFrameNumber(data.frame_number || 0);
              break;

            case 'status':
              console.log('Status update:', data);
              if (data.message === 'stream_started') {
                setStreaming(true);
              } else if (data.message === 'stream_stopped') {
                setStreaming(false);
                setFrame(null);
                setDetections([]);
              }
              break;

            case 'response':
              console.log('Response:', data);
              if (data.action === 'start_stream' && data.result?.status === 'stream_started') {
                setStreaming(true);
              } else if (data.action === 'stop_stream' && data.result?.status === 'stream_stopped') {
                setStreaming(false);
                setFrame(null);
                setDetections([]);
              }
              break;

            case 'error':
              console.error('WebSocket error message:', data.message);
              setError(data.message);
              break;

            default:
              console.log('Unknown message type:', data.type);
          }
        } catch (err) {
          console.error('Error parsing WebSocket message:', err);
        }
      };

      ws.onerror = (event) => {
        console.error('WebSocket error:', event);
        setError('WebSocket connection error');
      };

      ws.onclose = () => {
        console.log('WebSocket closed');
        setConnected(false);
        setStreaming(false);

        // Attempt to reconnect
        reconnectAttemptsRef.current += 1;
        if (reconnectAttemptsRef.current < maxReconnectAttempts) {
          const delay = Math.min(1000 * Math.pow(2, reconnectAttemptsRef.current), 10000);
          console.log(`Reconnecting in ${delay}ms (attempt ${reconnectAttemptsRef.current})`);
          reconnectTimeoutRef.current = setTimeout(connect, delay);
        }
      };

      wsRef.current = ws;
    } catch (err) {
      console.error('Error creating WebSocket:', err);
      setError(err.message);
    }
  }, []);

  const disconnect = useCallback(() => {
    // Clear reconnect timeout
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }

    // Reset reconnect attempts
    reconnectAttemptsRef.current = maxReconnectAttempts;

    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }

    setConnected(false);
    setStreaming(false);
    setFrame(null);
    setDetections([]);
  }, []);

  const sendMessage = useCallback((message) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(message));
    } else {
      console.error('WebSocket is not connected');
      setError('WebSocket is not connected');
    }
  }, []);

  const startStream = useCallback(() => {
    sendMessage({ action: 'start_stream' });
  }, [sendMessage]);

  const stopStream = useCallback(() => {
    sendMessage({ action: 'stop_stream' });
  }, [sendMessage]);

  const getStatus = useCallback(() => {
    sendMessage({ action: 'get_status' });
  }, [sendMessage]);

  // Auto-connect on mount
  useEffect(() => {
    connect();

    // Cleanup on unmount
    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, [connect]);

  return {
    frame,
    detections,
    connected,
    streaming,
    error,
    frameNumber,
    connect,
    disconnect,
    startStream,
    stopStream,
    getStatus
  };
}