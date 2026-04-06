'use client';

import React, { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import {
  BarChart3,
  Package,
  CheckCircle,
  XCircle,
  AlertTriangle,
  Home,
  RefreshCw,
  Clock,
  Search,
  Filter,
  Layers,
  Hash,
  Database,
  LogOut,
  User
} from 'lucide-react';

// ─────────────────────────────────────────────────────────────────────────────
// API Configuration
// ─────────────────────────────────────────────────────────────────────────────
const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

// // Mock/default scan session for demo purposes
// const DEFAULT_SCAN_SESSION = {
//   id: 'SCN-20240318-004',
//   started: '2024-03-18 14:22:11',
//   completed: '2024-03-18 14:47:03',
//   duration: '24m 52s',
//   aisle: 'A3–A7',
//   operator: 'M.A.R.S. Unit 01',
//   totalShelves: 0,
//   totalExpected: 0,
//   totalFound: 0,
//   totalMissing: 0,
//   totalUnexpected: 0,
// };

const DEFAULT_SCAN_SESSION = {
  id: '—',
  started: '—',
  completed: '—',
  duration: '—',
  aisle: '—',
  operator: 'M.A.R.S. Unit 01',
  totalShelves: 0,
  totalExpected: 0,
  totalFound: 0,
  totalMissing: 0,
  totalUnexpected: 0,
};

// ─────────────────────────────────────────────────────────────────────────────
// Utility Functions
// ─────────────────────────────────────────────────────────────────────────────

/**
 * Fetch all scans from the backend API (most recent first)
 * @returns {Promise<array>} Array of scan objects
 */
async function fetchAllScans() {
  try {
    const response = await fetch(`${API_BASE_URL}/inventory/scans`);
    if (!response.ok) {
      throw new Error(`API error: ${response.status}`);
    }
    const data = await response.json();
    return data.scans || [];
  } catch (error) {
    console.error('Error fetching scans:', error);
    throw error;
  }
}

/**
 * Fetch scan data from the backend API
 * @param {string} scanId - The scan ID to fetch
 * @returns {Promise<object>} Scan data with results and metrics
 */
async function fetchScanData(scanId) {
  try {
    const response = await fetch(`${API_BASE_URL}/inventory/scans/${scanId}`);
    if (!response.ok) {
      throw new Error(`API error: ${response.status}`);
    }
    return await response.json();
  } catch (error) {
    console.error('Error fetching scan data:', error);
    throw error;
  }
}

// /**
//  * Convert API scan results to shelf list format (original — expects {sku, name, qty} objects)
//  */
// function transformResultsToShelves(results) {
//   return results.map((result) => {
//     const shelf = {
//       id: result.shelf_id,
//       label: `Shelf ${result.shelf_id}`,
//       expected: result.expected_items || [],
//       unexpected: result.unexpected_items || [],
//       status: getShelfStatus(result),
//     };
//     const detectedMap = new Map(
//       (result.detected_items || []).map((item) => [item.sku, item.qty || 1])
//     );
//     shelf.expected = shelf.expected.map((item) => ({
//       ...item,
//       found: detectedMap.get(item.sku) || 0,
//     }));
//     return shelf;
//   });
// }

/**
 * Generate a deterministic SKU from a name using ASCII values.
 * e.g. "bottle" -> "SKU-98111116116108101" (ascii of each char)
 */
function generateSku(name) {
  const ascii = name.split('').map((c) => c.charCodeAt(0)).join('');
  return `SKU-${ascii}`;
}

/**
 * Convert API scan results to shelf list format.
 * Items are simple strings (e.g. "bottle", "mouse"), not objects.
 */
function transformResultsToShelves(results) {
  return results.map((result) => {
    const expected = result.expected_items || [];
    const detected = result.detected_items || [];
    const unexpected = result.unexpected_items || [];

    const detectedSet = new Set(detected.map((i) => typeof i === 'string' ? i.toLowerCase() : i));

    const shelf = {
      id: result.shelf_id,
      label: `Shelf ${result.shelf_id}`,
      expected: expected.map((item) => {
        const name = typeof item === 'string' ? item : item.name;
        return {
          sku: generateSku(name),
          name: name.charAt(0).toUpperCase() + name.slice(1),
          qty: 1,
          found: detectedSet.has(name.toLowerCase()) ? 1 : 0,
        };
      }),
      unexpected: unexpected.map((item) => {
        const name = typeof item === 'string' ? item : item.name;
        return { sku: generateSku(name), name: name.charAt(0).toUpperCase() + name.slice(1), qty: 1 };
      }),
      status: getShelfStatus(result),
    };

    return shelf;
  });
}

/**
 * Determine shelf status based on results
 * @param {object} result - Scan result for a shelf
 * @returns {string} Status: 'ok', 'warning', 'missing', 'unexpected'
 */
function getShelfStatus(result) {
  const missingCount = (result.missing_items || []).length;
  const unexpectedCount = (result.unexpected_items || []).length;
  const expectedCount = (result.expected_items || []).length;
  const detectedCount = (result.detected_items || []).length;

  if (missingCount > 0) return 'missing';
  if (unexpectedCount > 0) return 'unexpected';
  if (detectedCount < expectedCount) return 'warning';
  return 'ok';
}

/**
 * Transform API response to display format
 * @param {object} apiData - Data from API
 * @returns {object} Formatted data for component
 */
function transformApiData(apiData) {
  const { scan, results, metrics } = apiData;

  const shelves = transformResultsToShelves(results || []);

  const scanSession = {
    id: scan.scan_id,
    started: new Date(scan.created_at).toLocaleString(),
    completed: scan.completed_at
      ? new Date(scan.completed_at).toLocaleString()
      : 'In Progress',
    duration: '—',
    aisle: '—',
    operator: 'M.A.R.S. Unit 01',
    totalShelves: metrics.total_shelves || 0,
    totalExpected: metrics.total_expected || 0,
    totalFound: metrics.total_found || 0,
    totalMissing: metrics.total_missing || 0,
    totalUnexpected: metrics.total_unexpected || 0,
  };

  return { scanSession, shelves };
}

const SHELVES = [];

const STATUS_CONFIG = {
  ok:         { label: 'OK',         textColor: 'text-green-400',  bgBadge: 'bg-green-900/40 text-green-400',   border: 'border-gray-700'   },
  warning:    { label: 'Shortage',   textColor: 'text-yellow-400', bgBadge: 'bg-yellow-900/40 text-yellow-400', border: 'border-yellow-700' },
  missing:    { label: 'Missing',    textColor: 'text-red-400',    bgBadge: 'bg-red-900/40 text-red-400',       border: 'border-red-700'    },
  unexpected: { label: 'Unexpected', textColor: 'text-purple-400', bgBadge: 'bg-purple-900/40 text-purple-400', border: 'border-purple-700' },
};

function StatusBadge({ status }) {
  const cfg = STATUS_CONFIG[status];
  return (
    <span className={`text-xs font-semibold px-2 py-0.5 rounded ${cfg.bgBadge}`}>
      {cfg.label}
    </span>
  );
}

export default function InventoryMetrics({ currentUser, onLogout }) {
  const router = useRouter();

  // State management
  const [scanSession, setScanSession] = useState(DEFAULT_SCAN_SESSION);
  const [shelves, setShelves] = useState([]);
  const [selectedShelf, setSelectedShelf] = useState(null);
  const [filter, setFilter] = useState('all');
  const [search, setSearch] = useState('');
  const [lastRefresh, setLastRefresh] = useState(new Date());
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  // const [scanId, setScanId] = useState('SCN-20240318-004'); // Default scan ID - can be parameterized
  const [scanId, setScanId] = useState(null); // Will be set from latest scan
  const [availableScans, setAvailableScans] = useState([]);

  // Fetch scan data from API
  const fetchData = async (id) => {
    const targetId = id || scanId;
    if (!targetId) return;

    try {
      setLoading(true);
      setError(null);

      const apiData = await fetchScanData(targetId);

      if (apiData.status === 'success') {
        const { scanSession: session, shelves: shelvesData } =
          transformApiData(apiData);

        setScanSession(session);
        setShelves(shelvesData);

        // Auto-select first shelf
        if (shelvesData.length > 0 && !selectedShelf) {
          setSelectedShelf(shelvesData[0]);
        }
      }
    } catch (err) {
      console.error('Failed to fetch inventory data:', err);
      setError(
        `Failed to load inventory data: ${err.message}. Please check that the backend is running.`
      );
      // Fallback to empty data
      setScanSession(DEFAULT_SCAN_SESSION);
      setShelves([]);
    } finally {
      setLoading(false);
      setLastRefresh(new Date());
    }
  };

  // Load available scans on mount, auto-select the latest
  useEffect(() => {
    async function loadScans() {
      try {
        const scans = await fetchAllScans();
        setAvailableScans(scans);
        if (scans.length > 0) {
          const latestId = scans[0].scan_id;
          setScanId(latestId);
          fetchData(latestId);
        } else {
          setLoading(false);
          setError('No scans found. Run a scan from the robot first.');
        }
      } catch (err) {
        setLoading(false);
        setError(`Failed to load scans: ${err.message}`);
      }
    }
    loadScans();
  }, []);

  // Fetch data when scanId changes (from dropdown)
  useEffect(() => {
    if (scanId) {
      fetchData(scanId);
    }
  }, [scanId]);

  // Update selected shelf when shelves change
  useEffect(() => {
    if (shelves.length > 0 && !selectedShelf) {
      setSelectedShelf(shelves[0]);
    }
  }, [shelves, selectedShelf]);

  const accuracy = Math.round(
    (scanSession.totalFound / Math.max(scanSession.totalExpected, 1)) * 100
  );

  const handleRefresh = () => {
    fetchData();
  };

  const filteredShelves = shelves.filter((s) => {
    const matchesFilter = filter === 'all' || s.status === filter;
    const matchesSearch =
      search === '' ||
      s.label.toLowerCase().includes(search.toLowerCase()) ||
      s.id.toLowerCase().includes(search.toLowerCase()) ||
      s.expected.some(
        (i) =>
          i.name?.toLowerCase().includes(search.toLowerCase()) ||
          i.sku?.toLowerCase().includes(search.toLowerCase())
      );
    return matchesFilter && matchesSearch;
  });

  return (
    <div className="flex min-h-screen bg-gray-950">
      {/* Sidebar — identical to BackendInfo */}
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
          <button
            onClick={() => router.push('/')}
            className="w-16 h-16 bg-gray-800 hover:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto"
            title="Dashboard"
          >
            <Home size={24} />
          </button>

          <button
            onClick={() => router.push('/inventory')}
            className="w-16 h-16 bg-blue-700 hover:bg-blue-600 rounded-lg flex items-center justify-center transition-all mx-auto"
            title="Inventory Metrics (Current)"
          >
            <BarChart3 size={24} />
          </button>

          <button
            onClick={() => router.push('/backend')}
            className="w-16 h-16 bg-gray-800 hover:bg-gray-700 rounded-lg flex items-center justify-center transition-all mx-auto"
            title="Backend Monitor"
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
            <h1 className="text-4xl font-bold text-white mb-2">Inventory Metrics</h1>
            <p className="text-gray-500">Shelf-level scan results — expected vs. found</p>
          </div>
          <div className="flex items-center space-x-2">
            {/* Scan Selector */}
            {availableScans.length > 0 && (
              <select
                value={scanId || ''}
                onChange={(e) => setScanId(e.target.value)}
                className="bg-gray-800 border border-gray-700 rounded-lg px-3 py-2 text-sm text-gray-300 focus:outline-none focus:border-blue-500"
              >
                {availableScans.map((scan) => (
                  <option key={scan.scan_id} value={scan.scan_id}>
                    {scan.scan_id} — {scan.status}
                  </option>
                ))}
              </select>
            )}

            {/* User Info */}
            {currentUser && (
              <div className="flex items-center space-x-2 bg-gray-800 px-3 py-2 rounded-lg border border-gray-700">
                <User size={18} className="text-orange-400" />
                <span className="text-sm text-gray-300">{currentUser.username}</span>
                <span className="text-xs text-gray-500">({currentUser.role})</span>
              </div>
            )}

            <button
              onClick={() => router.push('/')}
              className="flex items-center space-x-2 bg-gray-800 hover:bg-gray-700 text-white px-4 py-2 rounded-lg transition-all border border-gray-700"
            >
              <Home size={20} />
              <span>Dashboard</span>
            </button>
            <button
              onClick={() => router.push('/backend')}
              className="flex items-center space-x-2 bg-gray-800 hover:bg-gray-700 text-white px-4 py-2 rounded-lg transition-all border border-gray-700"
            >
              <Database size={20} />
              <span>Backend Monitor</span>
            </button>
            <button
              onClick={handleRefresh}
              disabled={loading}
              className="flex items-center space-x-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:opacity-50 text-white px-4 py-2 rounded-lg transition-all"
            >
              <RefreshCw size={18} className={loading ? 'animate-spin' : ''} />
              <span>Refresh</span>
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

        {/* Error Alert */}
        {error && (
          <div className="mb-6 bg-red-900/30 border border-red-700 rounded-lg p-4 text-red-300 text-sm flex items-start space-x-2">
            <AlertTriangle size={18} className="flex-shrink-0 mt-0.5" />
            <div>
              <p className="font-semibold">Error Loading Data</p>
              <p className="text-xs mt-1">{error}</p>
            </div>
          </div>
        )}

        {/* Loading State */}
        {loading && shelves.length === 0 && (
          <div className="flex items-center justify-center py-20">
            <div className="text-center">
              <div className="inline-block">
                <RefreshCw size={32} className="animate-spin text-blue-400 mb-4" />
              </div>
              <p className="text-gray-400">Loading inventory data...</p>
            </div>
          </div>
        )}

        {/* Last Refresh Info */}
        {!loading && shelves.length > 0 && (
          <div className="mb-6 text-sm text-gray-500 flex items-center space-x-2">
            <Clock size={16} />
            <span>Last updated: {lastRefresh.toLocaleTimeString()}</span>
            <span className="text-gray-700">·</span>
            <span>Session: {scanSession.id}</span>
            <span className="text-gray-700">·</span>
            <span>Operator: {scanSession.operator}</span>
          </div>
        )}

        {/* Content Grid */}
        {!loading && shelves.length > 0 && (
          <div className="grid grid-cols-12 gap-4">
            {/* Stat Cards */}
            <div className="col-span-3">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-2">
                <Layers size={20} className="text-blue-400" />
                <span className="text-xs text-gray-500 uppercase">Shelves Scanned</span>
              </div>
              <p className="text-3xl font-bold text-white">{scanSession.totalShelves}</p>
              <p className="text-xs text-gray-500 mt-1">Aisle {scanSession.aisle}</p>
            </div>
          </div>

          <div className="col-span-3">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-2">
                <CheckCircle size={20} className="text-green-400" />
                <span className="text-xs text-gray-500 uppercase">Accuracy</span>
              </div>
              <p className="text-3xl font-bold text-green-400">{accuracy}%</p>
              <p className="text-xs text-gray-500 mt-1">{scanSession.totalFound} / {scanSession.totalExpected} items found</p>
            </div>
          </div>

          <div className="col-span-3">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-2">
                <AlertTriangle size={20} className="text-yellow-400" />
                <span className="text-xs text-gray-500 uppercase">Missing / Short</span>
              </div>
              <p className="text-3xl font-bold text-yellow-400">{scanSession.totalMissing}</p>
              <p className="text-xs text-gray-500 mt-1">Across all shelves</p>
            </div>
          </div>

          <div className="col-span-3">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-2">
                <Package size={20} className="text-purple-400" />
                <span className="text-xs text-gray-500 uppercase">Unexpected Items</span>
              </div>
              <p className="text-3xl font-bold text-purple-400">{scanSession.totalUnexpected}</p>
              <p className="text-xs text-gray-500 mt-1">Not in manifest</p>
            </div>
          </div>

          {/* Accuracy Bar */}
          <div className="col-span-12">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
              <div className="flex items-center justify-between mb-3">
                <div className="flex items-center space-x-2">
                  <BarChart3 size={20} className="text-blue-400" />
                  <h2 className="text-lg font-semibold text-white">Overall Inventory Accuracy</h2>
                </div>
                <span className="text-green-400 font-bold">{accuracy}%</span>
              </div>
              <div className="h-3 bg-gray-800 rounded-full overflow-hidden">
                <div
                  className="h-full bg-gradient-to-r from-blue-500 to-green-400 rounded-full"
                  style={{ width: `${accuracy}%` }}
                />
              </div>
            </div>
          </div>

          {/* Shelf List */}
          <div className="col-span-4">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
              <div className="flex items-center space-x-2 mb-4">
                <Layers size={24} className="text-blue-400" />
                <h2 className="text-xl font-semibold text-white">Shelves</h2>
              </div>

              <div className="space-y-2 mb-4">
                <div className="relative">
                  <Search size={14} className="absolute left-3 top-1/2 -translate-y-1/2 text-gray-500" />
                  <input
                    type="text"
                    placeholder="Search shelves or SKUs..."
                    value={search}
                    onChange={(e) => setSearch(e.target.value)}
                    className="w-full bg-gray-800 border border-gray-700 rounded-lg pl-9 pr-3 py-2 text-sm text-gray-200 placeholder-gray-600 focus:outline-none focus:border-blue-500"
                  />
                </div>
                <select
                  value={filter}
                  onChange={(e) => setFilter(e.target.value)}
                  className="w-full bg-gray-800 border border-gray-700 rounded-lg px-3 py-2 text-sm text-gray-300 focus:outline-none focus:border-blue-500"
                >
                  <option value="all">All Statuses</option>
                  <option value="ok">OK</option>
                  <option value="warning">Shortage</option>
                  <option value="missing">Missing</option>
                  <option value="unexpected">Unexpected</option>
                </select>
              </div>

              <div className="space-y-2 max-h-[500px] overflow-y-auto">
                {filteredShelves.length === 0 && (
                  <p className="text-gray-600 text-sm text-center py-6">No shelves match your filter.</p>
                )}
                {filteredShelves.map((shelf) => {
                  const isSelected = selectedShelf?.id === shelf.id;
                  return (
                    <button
                      key={shelf.id}
                      onClick={() => setSelectedShelf(shelf)}
                      className={`w-full text-left p-3 rounded-lg border transition-all ${
                        isSelected
                          ? 'bg-blue-900/30 border-blue-600'
                          : 'bg-gray-800 border-gray-700 hover:border-gray-500'
                      }`}
                    >
                      <div className="flex items-center justify-between">
                        <div>
                          <p className="text-xs text-gray-500">{shelf.id}</p>
                          <p className="text-sm font-semibold text-white">{shelf.label}</p>
                        </div>
                        <StatusBadge status={shelf.status} />
                      </div>
                    </button>
                  );
                })}
              </div>
            </div>
          </div>

          {/* Shelf Detail */}
          <div className="col-span-8">
            {selectedShelf ? (
              <div className="space-y-6">
                {/* Expected Items */}
                <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
                  <div className="flex items-center justify-between mb-4">
                    <div className="flex items-center space-x-2">
                      <Filter size={24} className="text-green-400" />
                      <h2 className="text-xl font-semibold text-white">
                        {selectedShelf.label} — Expected Items
                      </h2>
                    </div>
                    <StatusBadge status={selectedShelf.status} />
                  </div>

                  <div className="grid grid-cols-1 gap-3">
                    {selectedShelf.expected.map((item) => {
                      const isMissing = item.found === 0;
                      const isShort = item.found < item.qty && !isMissing;
                      return (
                        <div
                          key={item.sku}
                          className={`bg-gray-800 rounded-lg p-4 border ${
                            isMissing ? 'border-red-700' : isShort ? 'border-yellow-700' : 'border-gray-700'
                          }`}
                        >
                          <div className="flex items-center justify-between">
                            <div className="flex items-center space-x-3">
                              <Hash size={16} className="text-gray-400" />
                              <div>
                                <p className="text-xs text-gray-500">{item.sku}</p>
                                <p className="text-sm font-semibold text-white">{item.name}</p>
                              </div>
                            </div>
                            <div className="text-right">
                              <p className={`text-xl font-bold ${
                                isMissing ? 'text-red-400' : isShort ? 'text-yellow-400' : 'text-green-400'
                              }`}>
                                {item.found} / {item.qty}
                              </p>
                              <p className="text-xs text-gray-500">found / expected</p>
                            </div>
                          </div>
                          {(isMissing || isShort) && (
                            <div className={`mt-2 text-xs px-2 py-1 rounded inline-block ${
                              isMissing ? 'bg-red-900/40 text-red-400' : 'bg-yellow-900/40 text-yellow-400'
                            }`}>
                              {isMissing
                                ? `⚠ All ${item.qty} units missing`
                                : `⚠ ${item.qty - item.found} unit(s) short`}
                            </div>
                          )}
                        </div>
                      );
                    })}
                  </div>
                </div>

                {/* Unexpected Items */}
                {selectedShelf.unexpected.length > 0 && (
                  <div className="bg-gray-900 rounded-lg p-6 border border-gray-800">
                    <div className="flex items-center space-x-2 mb-4">
                      <Package size={24} className="text-purple-400" />
                      <h2 className="text-xl font-semibold text-white">Unexpected Items Detected</h2>
                    </div>
                    <div className="grid grid-cols-1 gap-3">
                      {selectedShelf.unexpected.map((item) => (
                        <div key={item.sku} className="bg-gray-800 rounded-lg p-4 border border-purple-700">
                          <div className="flex items-center justify-between">
                            <div className="flex items-center space-x-3">
                              <Hash size={16} className="text-gray-400" />
                              <div>
                                <p className="text-xs text-gray-500">{item.sku}</p>
                                <p className="text-sm font-semibold text-white">{item.name}</p>
                              </div>
                            </div>
                            <div className="text-right">
                              <p className="text-xl font-bold text-purple-400">+{item.qty}</p>
                              <p className="text-xs text-gray-500">not in manifest</p>
                            </div>
                          </div>
                        </div>
                      ))}
                    </div>
                  </div>
                )}

                {/* Supabase placeholder */}
                <div className="bg-gray-900 rounded-lg p-4 border border-dashed border-gray-700 text-center text-sm text-gray-600">
                  📡 Live data sync via Backend API — Connected
                </div>
              </div>
            ) : (
              <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 flex items-center justify-center h-64 text-gray-600">
                Select a shelf from the list to view details
              </div>
            )}
          </div>
          </div>
        )}
      </div>
    </div>
  );
}
