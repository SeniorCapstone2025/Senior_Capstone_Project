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
  Database
} from 'lucide-react';
import { useRoverStatus } from '../hooks/useRoverStatus';
import { useRoverCommands } from '../hooks/useRoverCommands';
import CameraFeed from './CameraFeed';

export default function RoverDashboard() {
  const router = useRouter();
  
  // Use real API hooks
  const { status, loading: statusLoading, error: statusError } = useRoverStatus(2000);
  const { sendCommand, loading: commandLoading, error: commandError } = useRoverCommands();

  // Local state for UI-specific data
  const [scannedItems, setScannedItems] = useState(16);
  const [currentScan, setCurrentScan] = useState('');
  const [uptime, setUptime] = useState('00:15:32');

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
      await sendCommand(command);
      // Status will update automatically via polling
    } catch (error) {
      console.error('Command failed:', error);
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
            disabled={roverStatus !== 'idle'}
            className="w-16 h-16 bg-green-600 hover:bg-green-700 disabled:bg-gray-700 disabled:opacity-50 rounded-lg flex items-center justify-center transition-all mx-auto"
          >
            <Power size={24} />
          </button>

          <button
            onClick={() => handleCommand('pause')}
            disabled={roverStatus === 'idle'}
            className="w-16 h-16 bg-yellow-600 hover:bg-yellow-700 disabled:bg-gray-700 disabled:opacity-50 rounded-lg flex items-center justify-center transition-all mx-auto"
          >
            <Pause size={24} />
          </button>

          <button
            onClick={() => handleCommand('cancel')}
            disabled={roverStatus === 'idle'}
            className="w-16 h-16 bg-red-600 hover:bg-red-700 disabled:bg-gray-700 disabled:opacity-50 rounded-lg flex items-center justify-center transition-all mx-auto"
          >
            <XCircle size={24} />
          </button>
        </div>

        <div className="flex flex-col space-y-2">
          <button
            onClick={() => handleCommand('return')}
            className="w-16 h-16 bg-gray-700 hover:bg-gray-600 rounded-lg flex items-center justify-center transition-all mx-auto"
          >
            <Home size={24} />
          </button>
          
          <button className="w-16 h-16 bg-gray-800 hover:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto">
            <Settings size={24} />
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
        {/* Header */}
        <div className="mb-8 flex items-center justify-between">
          <div>
            <h1 className="text-4xl font-bold text-white mb-2">M.A.R.S. Control Center</h1>
            <p className="text-gray-500">Mobile Autonomous Rover System - Inventory Scanning & Logging</p>
          </div>
          <button
            onClick={() => router.push('/backend')}
            className="flex items-center space-x-2 bg-gray-800 hover:bg-gray-700 text-white px-4 py-2 rounded-lg transition-all border border-gray-700"
          >
            <Database size={20} />
            <span>Backend Monitor</span>
          </button>
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
