'use client';

import React, { useState } from 'react';
import { useRouter } from 'next/navigation';
import {
  BarChart3,
  Package,
  CheckCircle,
  XCircle,
  AlertTriangle,
  ArrowLeft,
  RefreshCw,
  Clock,
  Search,
  Filter,
  Layers,
  Hash
} from 'lucide-react';

// ─── Dummy Data (replace with Supabase fetch when ready) ──────────────────────
const SCAN_SESSION = {
  id: 'SCN-20240318-004',
  started: '2024-03-18 14:22:11',
  completed: '2024-03-18 14:47:03',
  duration: '24m 52s',
  aisle: 'A3–A7',
  operator: 'M.A.R.S. Unit 01',
  totalShelves: 18,
  totalExpected: 214,
  totalFound: 197,
  totalMissing: 17,
  totalUnexpected: 4,
};

const SHELVES = [
  {
    id: 'A3-S1',
    label: 'Aisle A3 · Shelf 1',
    status: 'ok',
    expected: [
      { sku: 'SKU-10041', name: 'Wireless Keyboard MK-7', qty: 4, found: 4 },
      { sku: 'SKU-10042', name: 'Monitor Stand 27"', qty: 2, found: 2 },
      { sku: 'SKU-10043', name: 'USB-C Hub 7-Port', qty: 6, found: 6 },
    ],
    unexpected: [],
  },
  {
    id: 'A3-S2',
    label: 'Aisle A3 · Shelf 2',
    status: 'warning',
    expected: [
      { sku: 'SKU-10051', name: 'Ergonomic Mouse M500', qty: 8, found: 8 },
      { sku: 'SKU-10052', name: 'Mouse Pad XL', qty: 5, found: 3 },
      { sku: 'SKU-10053', name: 'Wrist Rest Gel', qty: 4, found: 4 },
    ],
    unexpected: [],
  },
  {
    id: 'A4-S1',
    label: 'Aisle A4 · Shelf 1',
    status: 'missing',
    expected: [
      { sku: 'SKU-10061', name: 'Webcam HD 1080p', qty: 3, found: 0 },
      { sku: 'SKU-10062', name: 'Ring Light 10in', qty: 2, found: 2 },
      { sku: 'SKU-10063', name: 'Desk Microphone USB', qty: 3, found: 3 },
    ],
    unexpected: [],
  },
  {
    id: 'A4-S2',
    label: 'Aisle A4 · Shelf 2',
    status: 'unexpected',
    expected: [
      { sku: 'SKU-10071', name: 'HDMI Cable 6ft', qty: 10, found: 10 },
      { sku: 'SKU-10072', name: 'Display Port Cable', qty: 6, found: 6 },
    ],
    unexpected: [
      { sku: 'SKU-99901', name: 'VGA Adapter (unregistered)', qty: 2 },
    ],
  },
  {
    id: 'A5-S1',
    label: 'Aisle A5 · Shelf 1',
    status: 'warning',
    expected: [
      { sku: 'SKU-10081', name: 'Laptop Stand Aluminum', qty: 5, found: 5 },
      { sku: 'SKU-10082', name: 'Portable SSD 1TB', qty: 4, found: 2 },
      { sku: 'SKU-10083', name: 'USB Flash Drive 64GB', qty: 12, found: 10 },
    ],
    unexpected: [],
  },
  {
    id: 'A5-S2',
    label: 'Aisle A5 · Shelf 2',
    status: 'ok',
    expected: [
      { sku: 'SKU-10091', name: 'Power Strip 6-Outlet', qty: 4, found: 4 },
      { sku: 'SKU-10092', name: 'Cable Management Kit', qty: 6, found: 6 },
      { sku: 'SKU-10093', name: 'Surge Protector', qty: 3, found: 3 },
    ],
    unexpected: [],
  },
  {
    id: 'A6-S1',
    label: 'Aisle A6 · Shelf 1',
    status: 'unexpected',
    expected: [
      { sku: 'SKU-10101', name: 'Bluetooth Speaker', qty: 3, found: 3 },
      { sku: 'SKU-10102', name: 'Headphone Stand', qty: 4, found: 4 },
    ],
    unexpected: [
      { sku: 'SKU-88801', name: 'Legacy Radio Unit (obsolete)', qty: 2 },
    ],
  },
  {
    id: 'A6-S2',
    label: 'Aisle A6 · Shelf 2',
    status: 'ok',
    expected: [
      { sku: 'SKU-10111', name: 'AA Batteries 20-Pack', qty: 8, found: 8 },
      { sku: 'SKU-10112', name: 'AAA Batteries 20-Pack', qty: 6, found: 6 },
      { sku: 'SKU-10113', name: 'Wireless Charger Pad', qty: 4, found: 4 },
    ],
    unexpected: [],
  },
];

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

export default function InventoryMetrics() {
  const router = useRouter();
  const [selectedShelf, setSelectedShelf] = useState(SHELVES[0]);
  const [filter, setFilter] = useState('all');
  const [search, setSearch] = useState('');
  const [lastRefresh, setLastRefresh] = useState(new Date());
  const [loading, setLoading] = useState(false);

  const accuracy = Math.round((SCAN_SESSION.totalFound / SCAN_SESSION.totalExpected) * 100);

  const handleRefresh = () => {
    setLoading(true);
    // TODO: fetch from Supabase
    setTimeout(() => {
      setLoading(false);
      setLastRefresh(new Date());
    }, 800);
  };

  const filteredShelves = SHELVES.filter((s) => {
    const matchesFilter = filter === 'all' || s.status === filter;
    const matchesSearch =
      search === '' ||
      s.label.toLowerCase().includes(search.toLowerCase()) ||
      s.id.toLowerCase().includes(search.toLowerCase()) ||
      s.expected.some(
        (i) =>
          i.name.toLowerCase().includes(search.toLowerCase()) ||
          i.sku.toLowerCase().includes(search.toLowerCase())
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
            <h1 className="text-4xl font-bold text-white mb-2">Inventory Metrics</h1>
            <p className="text-gray-500">Shelf-level scan results — expected vs. found</p>
          </div>
          <button
            onClick={handleRefresh}
            disabled={loading}
            className="flex items-center space-x-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-700 disabled:opacity-50 text-white px-4 py-2 rounded-lg transition-all"
          >
            <RefreshCw size={18} className={loading ? 'animate-spin' : ''} />
            <span>Refresh</span>
          </button>
        </div>

        {/* Last Refresh */}
        <div className="mb-6 text-sm text-gray-500 flex items-center space-x-2">
          <Clock size={16} />
          <span>Last updated: {lastRefresh.toLocaleTimeString()}</span>
          <span className="text-gray-700">·</span>
          <span>Session: {SCAN_SESSION.id}</span>
          <span className="text-gray-700">·</span>
          <span>Operator: {SCAN_SESSION.operator}</span>
          <span className="text-gray-700">·</span>
          <span className="text-xs bg-gray-800 text-gray-500 border border-gray-700 px-2 py-0.5 rounded">
            Supabase sync pending
          </span>
        </div>

        <div className="grid grid-cols-12 gap-6">

          {/* Stat Cards */}
          <div className="col-span-3">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-2">
                <Layers size={20} className="text-blue-400" />
                <span className="text-xs text-gray-500 uppercase">Shelves Scanned</span>
              </div>
              <p className="text-3xl font-bold text-white">{SCAN_SESSION.totalShelves}</p>
              <p className="text-xs text-gray-500 mt-1">Aisle {SCAN_SESSION.aisle}</p>
            </div>
          </div>

          <div className="col-span-3">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-2">
                <CheckCircle size={20} className="text-green-400" />
                <span className="text-xs text-gray-500 uppercase">Accuracy</span>
              </div>
              <p className="text-3xl font-bold text-green-400">{accuracy}%</p>
              <p className="text-xs text-gray-500 mt-1">{SCAN_SESSION.totalFound} / {SCAN_SESSION.totalExpected} items found</p>
            </div>
          </div>

          <div className="col-span-3">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-2">
                <AlertTriangle size={20} className="text-yellow-400" />
                <span className="text-xs text-gray-500 uppercase">Missing / Short</span>
              </div>
              <p className="text-3xl font-bold text-yellow-400">{SCAN_SESSION.totalMissing}</p>
              <p className="text-xs text-gray-500 mt-1">Across all shelves</p>
            </div>
          </div>

          <div className="col-span-3">
            <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 h-full">
              <div className="flex items-center space-x-2 mb-2">
                <Package size={20} className="text-purple-400" />
                <span className="text-xs text-gray-500 uppercase">Unexpected Items</span>
              </div>
              <p className="text-3xl font-bold text-purple-400">{SCAN_SESSION.totalUnexpected}</p>
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
                  📡 Live data sync via Supabase — not yet configured
                </div>
              </div>
            ) : (
              <div className="bg-gray-900 rounded-lg p-6 border border-gray-800 flex items-center justify-center h-64 text-gray-600">
                Select a shelf from the list to view details
              </div>
            )}
          </div>

        </div>
      </div>
    </div>
  );
}
