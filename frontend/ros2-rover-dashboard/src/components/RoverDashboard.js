'use client';

import React, { useState, useEffect } from 'react';
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
  Folder
} from 'lucide-react';

export default function RoverDashboard() {
  const [roverStatus, setRoverStatus] = useState('Returning to Base');
  const [batteryLevel, setBatteryLevel] = useState(86.8);
  const [connectionStatus, setConnectionStatus] = useState('Connected');
  const [scannedItems, setScannedItems] = useState(16);
  const [currentScan, setCurrentScan] = useState('');
  const [uptime, setUptime] = useState('00:15:32');

  useEffect(() => {
    if (roverStatus === 'Scanning' || roverStatus === 'Moving') {
      const interval = setInterval(() => {
        setBatteryLevel(prev => Math.max(0, prev - 0.1));
      }, 5000);
      return () => clearInterval(interval);
    }
  }, [roverStatus]);

  useEffect(() => {
    if (roverStatus === 'Scanning') {
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

  const handleCommand = (command) => {
    switch(command) {
      case 'start':
        setRoverStatus('Scanning');
        break;
      case 'pause':
        setRoverStatus('Paused');
        break;
      case 'terminate':
        setRoverStatus('Idle');
        setCurrentScan('');
        break;
      case 'return':
        setRoverStatus('Returning to Base');
        setCurrentScan('');
        break;
    }
  };

  const getStatusColor = () => {
    switch(roverStatus) {
      case 'Scanning': return 'text-green-400';
      case 'Paused': return 'text-yellow-400';
      case 'Returning to Base': return 'text-blue-400';
      case 'Idle': return 'text-gray-400';
      default: return 'text-gray-400';
    }
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
          <div className="w-16 h-16 bg-gray-800 rounded-lg flex items-center justify-center text-xs text-gray-400 font-bold">
            YOUR<br/>LOGO
          </div>
        </div>

        <div className="flex-1 flex flex-col space-y-2">
          <div className="text-xs text-gray-500 px-4 mb-2">Controls</div>
          
          <button
            onClick={() => handleCommand('start')}
            disabled={roverStatus === 'Scanning'}
            className="w-16 h-16 bg-green-600 hover:bg-green-700 disabled:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto"
          >
            <Power size={24} />
          </button>
          
          <button
            onClick={() => handleCommand('pause')}
            disabled={roverStatus !== 'Scanning'}
            className="w-16 h-16 bg-gray-700 hover:bg-gray-600 disabled:bg-gray-800 rounded-lg flex items-center justify-center transition-all mx-auto"
          >
            <Pause size={24} />
          </button>
          
          <button
            onClick={() => handleCommand('terminate')}
            disabled={roverStatus === 'Idle'}
            className="w-16 h-16 bg-red-600 hover:bg-red-700 disabled:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto"
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
          
          <button className="w-16 h-16 bg-gray-800 hover:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto">
            <Folder size={24} />
          </button>
        </div>
      </div>

      {/* Main Content */}
      <div className="flex-1 p-8">
        {/* Header */}
        <div className="mb-8">
          <h1 className="text-4xl font-bold text-white mb-2">ROS 2 Rover Control Center</h1>
          <p className="text-gray-500">Inventory Scanning & Logging System</p>
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
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
              <div className="bg-gradient-to-br from-blue-950 to-gray-950 rounded-lg aspect-video flex items-center justify-center border-2 border-gray-800 relative">
                {roverStatus === 'Idle' ? (
                  <div className="text-center text-gray-600">
                    <Camera size={64} className="mx-auto mb-4 opacity-50" />
                    <p>Camera feed inactive</p>
                    <p className="text-sm">Start scan to view live feed</p>
                  </div>
                ) : (
                  <>
                    <div className="absolute inset-0 bg-gradient-to-br from-blue-900/20 to-gray-900/40"></div>
                    <div className="relative text-center z-10">
                      <div className="animate-pulse mb-4">
                        <Camera size={64} className="mx-auto text-blue-400" />
                      </div>
                      <p className="text-blue-400 font-semibold text-lg">LIVE FEED ACTIVE</p>
                      {currentScan && (
                        <div className="mt-4 bg-black/50 px-4 py-2 rounded">
                          <p className="text-green-400 text-sm">Currently Scanning:</p>
                          <p className="text-white font-bold">{currentScan}</p>
                        </div>
                      )}
                    </div>
                    <div className="absolute top-4 left-4 bg-red-600 text-white px-3 py-1 rounded text-sm font-semibold flex items-center">
                      <div className="w-2 h-2 bg-white rounded-full mr-2 animate-pulse"></div>
                      REC
                    </div>
                  </>
                )}
              </div>
            </div>
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