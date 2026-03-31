'use client';

import React, { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import {
  Database,
  Activity,
  Server,
  HardDrive,
  Wifi,
  AlertCircle,
  CheckCircle,
  Clock,
  ArrowLeft,
  RefreshCw,
  Zap,
  Calendar,
  Hash
} from 'lucide-react';
import { useRoverStatus } from '../hooks/useRoverStatus';
import apiService from '../services/api';

export default function BackendInfo() {
  const router = useRouter();
  const { status, loading: statusLoading, error: statusError } = useRoverStatus(5000);
  
  const [healthData, setHealthData] = useState(null);
  const [dbTestResult, setDbTestResult] = useState(null);
  const [loading, setLoading] = useState(true);
  const [lastRefresh, setLastRefresh] = useState(new Date());

  const fetchBackendInfo = async () => {
    setLoading(true);
    try {
      // Fetch health check
      const health = await apiService.healthCheck();
      setHealthData(health);

      // Fetch database test
      const dbTest = await apiService.testDatabase();
      setDbTestResult(dbTest);
    } catch (error) {
      console.error('Failed to fetch backend info:', error);
    } finally {
      setLoading(false);
      setLastRefresh(new Date());
    }
  };

  useEffect(() => {
    fetchBackendInfo();
  }, []);

  const getStatusIcon = (isHealthy) => {
    return isHealthy ? (
      <CheckCircle size={20} className="text-green-400" />
    ) : (
      <AlertCircle size={20} className="text-red-400" />
    );
  };

  const getStatusColor = (isHealthy) => {
    return isHealthy ? 'text-green-400' : 'text-red-400';
  };

  return (
    <div className="flex min-h-screen bg-gray-950">
      {/* Sidebar with Logo */}
      <div className="w-24 bg-gray-900 flex flex-col items-center py-6 space-y-4 border-r border-gray-800">
        <div className="mb-8">
          <div className="w-16 h-16 bg-gradient-to-br from-red-600 to-orange-600 rounded-lg flex items-center justify-center text-xs text-white font-black shadow-lg shadow-red-900/50">
            <div className="text-center leading-tight">
              <div className="text-[10px] tracking-wider">M.A.R.S.</div>
              <div className="text-[6px] text-red-100 mt-0.5">ROVER</div>
            </div>
          </div>
        </div>

        <div className="flex-1"></div>

        <button
          onClick={() => router.push('/')}
          className="w-16 h-16 bg-gray-800 hover:bg-gray-700 rounded-lg flex items-center justify-center transition-all"
          title="Back to Dashboard"
        >
          <ArrowLeft size={24} />
        </button>
      </div>

      {/* Main Content */}
      <div className="flex-1 p-8">
        {/* Header */}
        <div className="mb-8 flex items-center justify-between">
          <div>
            <h1 className="text-4xl font-bold text-white mb-2">Backend System Monitor</h1>
            <p className="text-gray-500">Real-time backend status and diagnostics</p>
          </div>
          <button
            onClick={fetchBackendInfo}
            disabled={loading}
            className="flex items-center space-x-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:opacity-50 text-white px-4 py-2 rounded-lg transition-all"
          >
            <RefreshCw size={18} className={loading ? 'animate-spin' : ''} />
            <span>Refresh</span>
          </button>
        </div>

        {/* Last Refresh Time */}
        <div className="mb-6 text-sm text-gray-500 flex items-center space-x-2">
          <Clock size={16} />
          <span>Last updated: {lastRefresh.toLocaleTimeString()}</span>
        </div>

        <div className="grid grid-cols-12 gap-6">
          {/* API Health Status */}
          <div className="col-span-6">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-4">
                <Server size={24} className="text-blue-400" />
                <h2 className="text-xl font-semibold text-white">API Health</h2>
              </div>

              {loading && !healthData ? (
                <div className="text-gray-500">Loading...</div>
              ) : healthData ? (
                <div className="space-y-4">
                  <div className="flex items-center justify-between">
                    <span className="text-gray-400">Status</span>
                    <div className="flex items-center space-x-2">
                      {getStatusIcon(healthData.status === 'ok')}
                      <span className={`font-semibold ${getStatusColor(healthData.status === 'ok')}`}>
                        {healthData.status?.toUpperCase() || 'UNKNOWN'}
                      </span>
                    </div>
                  </div>

                  {healthData.timestamp && (
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Timestamp</span>
                      <span className="text-white font-mono text-sm">
                        {new Date(healthData.timestamp).toLocaleString()}
                      </span>
                    </div>
                  )}

                  {healthData.version && (
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">API Version</span>
                      <span className="text-white font-mono">{healthData.version}</span>
                    </div>
                  )}

                  {healthData.uptime !== undefined && (
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Uptime</span>
                      <span className="text-white font-mono">
                        {Math.floor(healthData.uptime / 3600)}h {Math.floor((healthData.uptime % 3600) / 60)}m
                      </span>
                    </div>
                  )}

                  {/* Additional health data */}
                  {Object.entries(healthData).map(([key, value]) => {
                    if (['status', 'timestamp', 'version', 'uptime'].includes(key)) return null;
                    return (
                      <div key={key} className="flex items-center justify-between">
                        <span className="text-gray-400 capitalize">{key.replace(/_/g, ' ')}</span>
                        <span className="text-white font-mono text-sm">
                          {typeof value === 'object' ? JSON.stringify(value) : String(value)}
                        </span>
                      </div>
                    );
                  })}
                </div>
              ) : (
                <div className="text-red-400 flex items-center space-x-2">
                  <AlertCircle size={18} />
                  <span>Failed to fetch health data</span>
                </div>
              )}
            </div>
          </div>

          {/* Database Status */}
          <div className="col-span-6">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-4">
                <Database size={24} className="text-purple-400" />
                <h2 className="text-xl font-semibold text-white">Database Status</h2>
              </div>

              {loading && !dbTestResult ? (
                <div className="text-gray-500">Loading...</div>
              ) : dbTestResult ? (
                <div className="space-y-4">
                  <div className="flex items-center justify-between">
                    <span className="text-gray-400">Connection</span>
                    <div className="flex items-center space-x-2">
                      {getStatusIcon(dbTestResult.status === 'ok' || dbTestResult.status === 'connected')}
                      <span className={`font-semibold ${getStatusColor(dbTestResult.status === 'ok' || dbTestResult.status === 'connected')}`}>
                        {(dbTestResult.status || 'UNKNOWN').toUpperCase()}
                      </span>
                    </div>
                  </div>

                  {dbTestResult.database && (
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Database Name</span>
                      <span className="text-white font-mono">{dbTestResult.database}</span>
                    </div>
                  )}

                  {dbTestResult.message && (
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Message</span>
                      <span className="text-white text-sm">{dbTestResult.message}</span>
                    </div>
                  )}

                  {dbTestResult.tables_count !== undefined && (
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Tables</span>
                      <span className="text-white font-mono">{dbTestResult.tables_count}</span>
                    </div>
                  )}

                  {dbTestResult.rows_count !== undefined && (
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Total Rows</span>
                      <span className="text-white font-mono">{dbTestResult.rows_count}</span>
                    </div>
                  )}

                  {/* Additional database data */}
                  {Object.entries(dbTestResult).map(([key, value]) => {
                    if (['status', 'database', 'message', 'tables_count', 'rows_count'].includes(key)) return null;
                    return (
                      <div key={key} className="flex items-center justify-between">
                        <span className="text-gray-400 capitalize">{key.replace(/_/g, ' ')}</span>
                        <span className="text-white font-mono text-sm">
                          {typeof value === 'object' ? JSON.stringify(value) : String(value)}
                        </span>
                      </div>
                    );
                  })}
                </div>
              ) : (
                <div className="text-red-400 flex items-center space-x-2">
                  <AlertCircle size={18} />
                  <span>Failed to fetch database info</span>
                </div>
              )}
            </div>
          </div>

          {/* Rover Status */}
          <div className="col-span-12">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
              <div className="flex items-center space-x-2 mb-4">
                <Activity size={24} className="text-green-400" />
                <h2 className="text-xl font-semibold text-white">Live Rover Status</h2>
              </div>

              {statusLoading && !status ? (
                <div className="text-gray-500">Loading rover status...</div>
              ) : statusError ? (
                <div className="text-red-400 flex items-center space-x-2">
                  <AlertCircle size={18} />
                  <span>Failed to connect to rover: {statusError}</span>
                </div>
              ) : status ? (
                <div className="grid grid-cols-4 gap-4">
                  {Object.entries(status).map(([key, value]) => (
                    <div key={key} className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                      <div className="flex items-center space-x-2 mb-2">
                        <Hash size={16} className="text-gray-400" />
                        <span className="text-xs text-gray-500 uppercase">{key.replace(/_/g, ' ')}</span>
                      </div>
                      <p className="text-lg font-bold text-white break-words">
                        {typeof value === 'object' ? JSON.stringify(value, null, 2) : String(value)}
                      </p>
                    </div>
                  ))}
                </div>
              ) : (
                <div className="text-gray-500">No rover status data available</div>
              )}
            </div>
          </div>

          {/* API Endpoints */}
          <div className="col-span-12">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
              <div className="flex items-center space-x-2 mb-4">
                <Zap size={24} className="text-yellow-400" />
                <h2 className="text-xl font-semibold text-white">API Endpoints</h2>
              </div>

              <div className="grid grid-cols-2 gap-4">
                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm font-semibold text-white">POST /command/</span>
                    <span className="text-xs bg-blue-900/40 text-blue-400 px-2 py-1 rounded">POST</span>
                  </div>
                  <p className="text-xs text-gray-400">Send commands to the rover (start, pause, cancel, return)</p>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm font-semibold text-white">GET /status/</span>
                    <span className="text-xs bg-green-900/40 text-green-400 px-2 py-1 rounded">GET</span>
                  </div>
                  <p className="text-xs text-gray-400">Get current rover status and battery level</p>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm font-semibold text-white">GET /health</span>
                    <span className="text-xs bg-green-900/40 text-green-400 px-2 py-1 rounded">GET</span>
                  </div>
                  <p className="text-xs text-gray-400">API health check and system information</p>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm font-semibold text-white">POST /detections/</span>
                    <span className="text-xs bg-blue-900/40 text-blue-400 px-2 py-1 rounded">POST</span>
                  </div>
                  <p className="text-xs text-gray-400">Submit object detection results to database</p>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm font-semibold text-white">WS /ws/camera</span>
                    <span className="text-xs bg-purple-900/40 text-purple-400 px-2 py-1 rounded">WS</span>
                  </div>
                  <p className="text-xs text-gray-400">WebSocket for live camera feed and detections</p>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <div className="flex items-center justify-between mb-2">
                    <span className="text-sm font-semibold text-white">GET /command/test-db</span>
                    <span className="text-xs bg-green-900/40 text-green-400 px-2 py-1 rounded">GET</span>
                  </div>
                  <p className="text-xs text-gray-400">Test database connection and retrieve info</p>
                </div>
              </div>
            </div>
          </div>

          {/* Connection Info */}
          <div className="col-span-12">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
              <div className="flex items-center space-x-2 mb-4">
                <Wifi size={24} className="text-cyan-400" />
                <h2 className="text-xl font-semibold text-white">Connection Configuration</h2>
              </div>

              <div className="grid grid-cols-3 gap-4">
                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <span className="text-xs text-gray-500 uppercase mb-2 block">API Base URL</span>
                  <p className="text-white font-mono text-sm break-all">
                    {process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000'}
                  </p>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <span className="text-xs text-gray-500 uppercase mb-2 block">WebSocket URL</span>
                  <p className="text-white font-mono text-sm break-all">
                    ws://localhost:8000/ws/camera
                  </p>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <span className="text-xs text-gray-500 uppercase mb-2 block">Polling Interval</span>
                  <p className="text-white font-mono text-sm">
                    5000ms (5s)
                  </p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
