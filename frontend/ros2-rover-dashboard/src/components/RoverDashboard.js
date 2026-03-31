'use client';

import React, { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import {
  Power,
  Pause,
  XCircle,
  Home,
  Battery,
  Wifi,
  Camera,
  Package,
  CheckCircle,
  Activity,
  Settings,
  Database,
  AlertCircle,
  BarChart3,
  LogOut,
  User
} from 'lucide-react';
import { useRoverStatus } from '../hooks/useRoverStatus';
import { useRoverCommands } from '../hooks/useRoverCommands';
import CameraFeed from './CameraFeed';

export default function RoverDashboard({ currentUser, onLogout }) {
  const router = useRouter();
  
  // Use real API hooks
  const { status, loading: statusLoading, error: statusError } = useRoverStatus(2000);
  const { sendCommand, loading: commandLoading, error: commandError } = useRoverCommands();

  // Local state for UI-specific data
  const [scannedItems, setScannedItems] = useState(16);
  const [currentScan, setCurrentScan] = useState('');
  const [uptime, setUptime] = useState('00:15:32');
  const [notification, setNotification] = useState(null);
  const [showSettings, setShowSettings] = useState(false);

  // Extract data from API response or use defaults
  const roverStatus = status?.state || 'idle';
  const batteryLevel = status?.battery_level || 0;
  const connectionStatus = statusError ? 'Disconnected' : 'Connected';

  useEffect(() => {
    if (roverStatus === 'scanning' || roverStatus === 'moving') {
      const items = [
        'Box #A-1234',
        'Pallet #B-5678',
        'Container #C-9012',
        'Shelf Unit #D-3456',
        'Crate #E-7890'
      ];
      const interval = setInterval(() => {
        setCurrentScan(items[Math.floor(Math.random() * items.length)]);
        setScannedItems(prev => prev + 1);
      }, 3000);
      return () => clearInterval(interval);
    }
  }, [roverStatus]);

  const handleCommand = async (command) => {
    try {
      setNotification({ type: 'loading', message: `Sending ${command} command...` });
      await sendCommand(command);
      setNotification({ type: 'success', message: `Command "${command}" sent successfully!` });
      setTimeout(() => setNotification(null), 3000);
    } catch (error) {
      console.error('Command failed:', error);
      setNotification({ type: 'error', message: `Failed to send ${command} command: ${error.message}` });
      setTimeout(() => setNotification(null), 5000);
    }
  };

  const getStatusColor = () => {
    const status = roverStatus.toLowerCase();
    if (status.includes('scan')) return 'text-green-400';
    if (status.includes('pause')) return 'text-yellow-400';
    if (status.includes('return')) return 'text-blue-400';
    return 'text-gray-400';
  };

  const getBatteryColor = () => {
    if (batteryLevel > 50) return 'text-green-400';
    if (batteryLevel > 20) return 'text-yellow-400';
    return 'text-red-400';
  };

  return (
    <div className="flex min-h-screen bg-gray-950">
      {/* Left Sidebar */}
      <div className="w-24 bg-gray-900 flex flex-col items-center py-6 space-y-4 border-r border-gray-800">
        <div className="mb-8">
          <div className="w-16 h-16 bg-gradient-to-br from-red-600 to-orange-600 rounded-lg flex items-center justify-center text-xs text-white font-black shadow-lg shadow-red-900/50">
            <div className="text-center leading-tight">
              <div className="text-[10px] tracking-wider">M.A.R.S.</div>
              <div className="text-[6px] text-red-100 mt-0.5">ROVER</div>
            </div>
          </div>
        </div>

        <div className="flex-1 flex flex-col space-y-2">
          <div className="text-xs text-gray-500 px-4 mb-2">Controls</div>
          
          <button
            onClick={() => handleCommand('start')}
            disabled={roverStatus !== 'idle' || commandLoading}
            className="w-16 h-16 bg-green-600 hover:bg-green-700 disabled:bg-gray-700 disabled:opacity-50 rounded-lg flex items-center justify-center transition-all mx-auto relative group"
            title="Start Mission"
          >
            {commandLoading ? (
              <Activity size={24} className="animate-spin" />
            ) : (
              <Power size={24} />
            )}
            <span className="absolute left-20 bg-gray-800 text-white text-xs px-2 py-1 rounded opacity-0 group-hover:opacity-100 transition-opacity whitespace-nowrap pointer-events-none">
              Start Mission
            </span>
          </button>

          <button
            onClick={() => handleCommand('pause')}
            disabled={roverStatus === 'idle' || commandLoading}
            className="w-16 h-16 bg-yellow-600 hover:bg-yellow-700 disabled:bg-gray-700 disabled:opacity-50 rounded-lg flex items-center justify-center transition-all mx-auto relative group"
            title="Pause Mission"
          >
            {commandLoading ? (
              <Activity size={24} className="animate-spin" />
            ) : (
              <Pause size={24} />
            )}
            <span className="absolute left-20 bg-gray-800 text-white text-xs px-2 py-1 rounded opacity-0 group-hover:opacity-100 transition-opacity whitespace-nowrap pointer-events-none">
              Pause Mission
            </span>
          </button>

          <button
            onClick={() => handleCommand('cancel')}
            disabled={roverStatus === 'idle' || commandLoading}
            className="w-16 h-16 bg-red-600 hover:bg-red-700 disabled:bg-gray-700 disabled:opacity-50 rounded-lg flex items-center justify-center transition-all mx-auto relative group"
            title="Cancel Mission"
          >
            {commandLoading ? (
              <Activity size={24} className="animate-spin" />
            ) : (
              <XCircle size={24} />
            )}
            <span className="absolute left-20 bg-gray-800 text-white text-xs px-2 py-1 rounded opacity-0 group-hover:opacity-100 transition-opacity whitespace-nowrap pointer-events-none">
              Cancel Mission
            </span>
          </button>
        </div>

        <div className="flex flex-col space-y-2">
          <button
            onClick={() => handleCommand('return')}
            disabled={commandLoading}
            className="w-16 h-16 bg-gray-700 hover:bg-gray-600 disabled:bg-gray-800 disabled:opacity-50 rounded-lg flex items-center justify-center transition-all mx-auto"
            title="Return to Base"
          >
            <Home size={24} />
          </button>
          
          <button 
            onClick={() => setShowSettings(!showSettings)}
            className="w-16 h-16 bg-gray-800 hover:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto"
            title="Settings"
          >
            <Settings size={24} />
          </button>
          
          {/* Inventory Metrics — same pattern as Backend Info button */}
          <button 
            onClick={() => router.push('/inventory')}
            className="w-16 h-16 bg-gray-800 hover:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto"
            title="Inventory Metrics"
          >
            <BarChart3 size={24} />
          </button>

          <button 
            onClick={() => router.push('/backend')}
            className="w-16 h-16 bg-gray-800 hover:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto"
            title="Backend Info"
          >
            <Database size={24} />
          </button>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 p-8">
        {/* Notification Banner */}
        {notification && (
          <div className={`mb-4 p-4 rounded-lg flex items-center space-x-3 ${
            notification.type === 'success' ? 'bg-green-900/50 border border-green-700' :
            notification.type === 'error' ? 'bg-red-900/50 border border-red-700' :
            'bg-blue-900/50 border border-blue-700'
          }`}>
            {notification.type === 'success' && <CheckCircle size={20} className="text-green-400" />}
            {notification.type === 'error' && <AlertCircle size={20} className="text-red-400" />}
            {notification.type === 'loading' && <Activity size={20} className="text-blue-400 animate-spin" />}
            <span className="text-white">{notification.message}</span>
            <button 
              onClick={() => setNotification(null)}
              className="ml-auto text-gray-400 hover:text-white"
            >
              <XCircle size={18} />
            </button>
          </div>
        )}

        {/* Settings Modal */}
        {showSettings && (
          <div className="fixed inset-0 bg-black/50 backdrop-blur-sm z-50 flex items-center justify-center" onClick={() => setShowSettings(false)}>
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 max-w-md w-full mx-4" onClick={(e) => e.stopPropagation()}>
              <div className="flex items-center justify-between mb-4">
                <h2 className="text-xl font-semibold text-white flex items-center space-x-2">
                  <Settings size={24} className="text-blue-400" />
                  <span>Settings</span>
                </h2>
                <button onClick={() => setShowSettings(false)} className="text-gray-400 hover:text-white">
                  <XCircle size={24} />
                </button>
              </div>
              
              <div className="space-y-4">
                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <h3 className="text-sm font-semibold text-white mb-2">API Configuration</h3>
                  <p className="text-xs text-gray-400 mb-2">Backend URL:</p>
                  <p className="text-sm text-white font-mono bg-gray-900 px-3 py-2 rounded">
                    {process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000'}
                  </p>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <h3 className="text-sm font-semibold text-white mb-2">Connection Status</h3>
                  <div className="flex items-center justify-between text-sm">
                    <span className="text-gray-400">API Status:</span>
                    <span className={connectionStatus === 'Connected' ? 'text-green-400' : 'text-red-400'}>
                      {connectionStatus}
                    </span>
                  </div>
                </div>

                <div className="bg-gray-800 rounded-lg p-4 border border-gray-700">
                  <h3 className="text-sm font-semibold text-white mb-2">Rover Information</h3>
                  <div className="space-y-2 text-sm">
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Current State:</span>
                      <span className="text-white font-semibold">{roverStatus}</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Battery Level:</span>
                      <span className="text-white font-semibold">{batteryLevel.toFixed(1)}%</span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400">Items Scanned:</span>
                      <span className="text-white font-semibold">{scannedItems}</span>
                    </div>
                  </div>
                </div>

                <button
                  onClick={() => setShowSettings(false)}
                  className="w-full bg-blue-600 hover:bg-blue-700 text-white py-2 rounded-lg transition-all"
                >
                  Close
                </button>
              </div>
            </div>
          </div>
        )}

        {/* Header */}
        <div className="mb-8 flex items-center justify-between">
          <div>
            <h1 className="text-4xl font-bold text-white mb-2">M.A.R.S. Control Center</h1>
            <p className="text-gray-500">Mobile Autonomous Rover System - Inventory Scanning & Logging</p>
          </div>
          <div className="flex items-center space-x-3">
            {/* User Info */}
            {currentUser && (
              <div className="flex items-center space-x-2 bg-gray-800 px-3 py-2 rounded-lg border border-gray-700">
                <User size={18} className="text-orange-400" />
                <span className="text-sm text-gray-300">{currentUser.username}</span>
                <span className="text-xs text-gray-500">({currentUser.role})</span>
              </div>
            )}

            {/* Inventory Metrics button — same style as Backend Monitor */}
            <button
              onClick={() => router.push('/inventory')}
              className="flex items-center space-x-2 bg-gray-800 hover:bg-gray-700 text-white px-4 py-2 rounded-lg transition-all border border-gray-700"
            >
              <BarChart3 size={20} />
              <span>Inventory Metrics</span>
            </button>
            <button
              onClick={() => router.push('/backend')}
              className="flex items-center space-x-2 bg-gray-800 hover:bg-gray-700 text-white px-4 py-2 rounded-lg transition-all border border-gray-700"
            >
              <Database size={20} />
              <span>Backend Monitor</span>
            </button>

            {/* Logout Button */}
            {onLogout && (
              <button
                onClick={onLogout}
                className="flex items-center space-x-2 bg-red-900 hover:bg-red-800 text-red-100 px-4 py-2 rounded-lg transition-all border border-red-700"
                title="Log out"
              >
                <LogOut size={20} />
                <span>Logout</span>
              </button>
            )}
          </div>
        </div>

        <div className="grid grid-cols-12 gap-6">
          {/* Left Column - Command & Camera */}
          <div className="col-span-8 space-y-6">
            {/* Command Controls Header */}
            <div className="bg-gray-900 rounded-lg p-4">
              <div className="flex items-center space-x-6">
                <div className="flex items-center space-x-2">
                  <Activity size={20} className="text-gray-400" />
                  <span className="text-gray-400 text-sm">Command Controls</span>
                </div>
                <div className="flex items-center space-x-2">
                  <Camera size={20} className="text-gray-400" />
                  <span className="text-white font-semibold">Live Camera Feed</span>
                </div>
              </div>
            </div>

            {/* Status Bar */}
            <div className="grid grid-cols-4 gap-4">
              <div className="bg-gray-900 rounded-lg p-4 border border-gray-800">
                <div className="flex items-center space-x-2 mb-2">
                  <Activity size={18} className="text-blue-400" />
                  <span className="text-xs text-gray-500 uppercase">Status</span>
                </div>
                <p className={`text-lg font-bold ${getStatusColor()}`}>{roverStatus}</p>
              </div>

              <div className="bg-gray-900 rounded-lg p-4 border border-gray-800">
                <div className="flex items-center space-x-2 mb-2">
                  <Battery size={18} className={getBatteryColor()} />
                  <span className="text-xs text-gray-500 uppercase">Battery</span>
                </div>
                <p className={`text-lg font-bold ${getBatteryColor()}`}>{batteryLevel.toFixed(1)}%</p>
              </div>

              <div className="bg-gray-900 rounded-lg p-4 border border-gray-800">
                <div className="flex items-center space-x-2 mb-2">
                  <Wifi size={18} className="text-green-400" />
                  <span className="text-xs text-gray-500 uppercase">Connection</span>
                </div>
                <p className="text-lg font-bold text-green-400">{connectionStatus}</p>
              </div>

              <div className="bg-gray-900 rounded-lg p-4 border border-gray-800">
                <div className="flex items-center space-x-2 mb-2">
                  <Package size={18} className="text-purple-400" />
                  <span className="text-xs text-gray-500 uppercase">Scanned</span>
                </div>
                <p className="text-lg font-bold text-purple-400">{scannedItems} items</p>
              </div>
            </div>

            {/* Camera Feed */}
            <CameraFeed isActive={roverStatus !== 'idle'} />
          </div>

          {/* Right Column - Recent Scans */}
          <div className="col-span-4">
            <div className="bg-gray-900 rounded-lg p-5 border border-gray-800">
              <div className="flex items-center space-x-2 mb-4">
                <Package size={20} className="text-gray-400" />
                <h2 className="text-white font-semibold">Recent Inventory Scans</h2>
              </div>
              
              <div className="space-y-2 max-h-[calc(100vh-200px)] overflow-y-auto">
                {Array(9).fill(null).map((_, idx) => (
                  <div key={idx} className="bg-gray-800 p-3 rounded-lg hover:bg-gray-750 transition-all border border-gray-700">
                    <div className="flex items-start justify-between">
                      <div className="flex items-start space-x-3">
                        <Package size={18} className="text-blue-400 mt-1" />
                        <div>
                          <p className="text-white font-semibold text-sm">Pallet #B-5678</p>
                          <p className="text-xs text-gray-500">INV-2024-001</p>
                        </div>
                      </div>
                      <div className="flex flex-col items-end space-y-1">
                        <span className="text-xs text-gray-500">2 min ago</span>
                        <div className="flex items-center space-x-1">
                          <CheckCircle size={14} className="text-green-400" />
                          <span className="text-xs bg-green-900/40 text-green-400 px-2 py-0.5 rounded">
                            DB Synced
                          </span>
                        </div>
                      </div>
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
