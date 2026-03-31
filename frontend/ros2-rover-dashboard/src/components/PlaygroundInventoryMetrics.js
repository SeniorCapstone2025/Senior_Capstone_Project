'use client';

import { useState, useEffect, useRef } from 'react';
import { useRouter } from 'next/navigation';
import { motion, AnimatePresence } from 'framer-motion';
import {
  BarChart3,
  Database,
  Navigation,
  LogOut,
  RefreshCw,
  ChevronDown,
  Package,
  AlertTriangle,
  CheckCircle,
  Zap,
  Menu,
  X,
} from 'lucide-react';
import AnimatedBackground from './AnimatedBackground';
import { fadeInUpVariants, containerVariants, glowVariants, scaleInVariants } from '@/utils/animations';

const navigationItems = [
  { icon: Navigation, label: 'Dashboard', href: '/' },
  { icon: BarChart3, label: 'Inventory Metrics', href: '/inventory', current: true },
  { icon: Database, label: 'Backend Monitor', href: '/backend' },
];

export default function PlaygroundInventoryMetrics() {
  const [data, setData] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState('');
  const [selectedShelf, setSelectedShelf] = useState(null);
  const [expandedShelf, setExpandedShelf] = useState(null);
  const [currentUser, setCurrentUser] = useState(null);
  const [mobileMenuOpen, setMobileMenuOpen] = useState(false);
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });
  const router = useRouter();
  const containerRef = useRef(null);

  useEffect(() => {
    const user = localStorage.getItem('mars_user');
    const token = localStorage.getItem('mars_token');

    if (!token) {
      router.push('/login');
      return;
    }

    if (user) {
      setCurrentUser(JSON.parse(user));
    }

    fetchData();
  }, [router]);

  useEffect(() => {
    const handleMouseMove = (e) => {
      if (containerRef.current) {
        const rect = containerRef.current.getBoundingClientRect();
        const x = (e.clientX - rect.left) / rect.width;
        const y = (e.clientY - rect.top) / rect.height;
        setMousePosition({ x, y });
      }
    };

    window.addEventListener('mousemove', handleMouseMove);
    return () => window.removeEventListener('mousemove', handleMouseMove);
  }, []);

  const fetchData = async () => {
    setLoading(true);
    setError('');

    try {
      const token = localStorage.getItem('mars_token');
      const response = await fetch('http://localhost:8000/inventory/scans/SCN-20240318-004', {
        headers: { Authorization: `Bearer ${token}` },
      });

      if (!response.ok) throw new Error('Failed to fetch inventory data');

      const scanData = await response.json();
      setData(scanData);
    } catch (err) {
      setError(err.message || 'Error fetching inventory data');
    } finally {
      setLoading(false);
    }
  };

  const handleLogout = () => {
    localStorage.removeItem('mars_token');
    localStorage.removeItem('mars_user');
    router.push('/login');
  };

  const handleNavigate = (href) => {
    router.push(href);
    setMobileMenuOpen(false);
  };

  const getStatusColor = (status) => {
    switch (status?.toLowerCase()) {
      case 'ok':
        return 'from-green-600 to-green-500';
      case 'warning':
        return 'from-yellow-600 to-yellow-500';
      case 'missing':
        return 'from-red-600 to-red-500';
      case 'unexpected':
        return 'from-orange-600 to-orange-500';
      default:
        return 'from-gray-600 to-gray-500';
    }
  };

  const getStatusIcon = (status) => {
    switch (status?.toLowerCase()) {
      case 'ok':
        return <CheckCircle className="w-5 h-5 text-green-400" />;
      case 'warning':
        return <AlertTriangle className="w-5 h-5 text-yellow-400" />;
      case 'missing':
        return <Package className="w-5 h-5 text-red-400" />;
      default:
        return <Zap className="w-5 h-5 text-orange-400" />;
    }
  };

  return (
    <div ref={containerRef} className="min-h-screen w-full bg-gray-900 text-white overflow-hidden">
      <AnimatedBackground variant="grid" />

      {/* Header */}
      <motion.header
        className="relative z-40 border-b border-gray-800 backdrop-blur-md bg-gray-900/50"
        initial={{ y: -100 }}
        animate={{ y: 0 }}
        transition={{ duration: 0.5 }}
      >
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-4 flex items-center justify-between">
          <motion.div className="text-2xl font-bold bg-gradient-to-r from-red-600 to-orange-600 bg-clip-text text-transparent">
            M.A.R.S
          </motion.div>

          {/* Desktop Navigation */}
          <div className="hidden md:flex items-center gap-4">
            {navigationItems.map((item) => (
              <motion.button
                key={item.label}
                onClick={() => handleNavigate(item.href)}
                className={`px-4 py-2 rounded-lg transition flex items-center gap-2 ${
                  item.current
                    ? 'bg-orange-600/50 border border-orange-500'
                    : 'bg-gray-800/50 hover:bg-gray-800'
                }`}
                whileHover={{ scale: 1.05 }}
              >
                <item.icon className="w-5 h-5" />
                <span className="text-sm">{item.label}</span>
              </motion.button>
            ))}
          </div>

          {/* User Menu */}
          <div className="flex items-center gap-4">
            {currentUser && (
              <motion.div className="px-4 py-2 bg-gray-800/50 rounded-lg text-sm hidden sm:block">
                Welcome, <span className="font-semibold text-orange-400">{currentUser.username}</span>
              </motion.div>
            )}
            <motion.button
              onClick={handleLogout}
              className="px-4 py-2 rounded-lg bg-red-600/50 hover:bg-red-600 transition flex items-center gap-2"
              whileHover={{ scale: 1.05 }}
            >
              <LogOut className="w-5 h-5" />
              <span className="hidden sm:inline text-sm">Logout</span>
            </motion.button>

            <button
              className="md:hidden p-2"
              onClick={() => setMobileMenuOpen(!mobileMenuOpen)}
            >
              {mobileMenuOpen ? <X className="w-6 h-6" /> : <Menu className="w-6 h-6" />}
            </button>
          </div>
        </div>

        {mobileMenuOpen && (
          <motion.div
            className="md:hidden border-t border-gray-800 bg-gray-900 p-4 space-y-2"
            initial={{ opacity: 0, y: -10 }}
            animate={{ opacity: 1, y: 0 }}
          >
            {navigationItems.map((item) => (
              <motion.button
                key={item.label}
                onClick={() => handleNavigate(item.href)}
                className="w-full px-4 py-2 rounded-lg bg-gray-800/50 hover:bg-gray-800 transition flex items-center gap-2"
              >
                <item.icon className="w-5 h-5" />
                <span>{item.label}</span>
              </motion.button>
            ))}
          </motion.div>
        )}
      </motion.header>

      {/* Main Content */}
      <motion.main
        className="relative z-10 max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8"
        variants={containerVariants}
        initial="hidden"
        animate="visible"
      >
        {/* Header */}
        <motion.div custom={0} variants={fadeInUpVariants} className="mb-8 flex items-center justify-between">
          <div>
            <h1 className="text-4xl font-bold mb-2">
              <span className="bg-gradient-to-r from-red-600 to-orange-600 bg-clip-text text-transparent">
                Inventory Metrics
              </span>
            </h1>
            <p className="text-gray-400">Real-time scan data and shelf status monitoring</p>
          </div>

          <motion.button
            onClick={fetchData}
            disabled={loading}
            className="px-6 py-3 bg-gradient-to-r from-red-600 to-orange-600 hover:from-red-700 hover:to-orange-700 disabled:opacity-50 rounded-lg flex items-center gap-2 font-semibold"
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
          >
            <RefreshCw className={`w-5 h-5 ${loading ? 'animate-spin' : ''}`} />
            Refresh
          </motion.button>
        </motion.div>

        {error && (
          <motion.div
            className="mb-6 p-4 bg-red-900/30 border border-red-500/50 rounded-lg"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
          >
            <p className="text-red-300">{error}</p>
          </motion.div>
        )}

        {loading ? (
          <motion.div className="flex items-center justify-center h-64" custom={1} variants={fadeInUpVariants}>
            <motion.div
              animate={{ rotate: 360 }}
              transition={{ duration: 2, repeat: Infinity }}
              className="w-12 h-12 border-4 border-orange-600/30 border-t-orange-600 rounded-full"
            />
          </motion.div>
        ) : data ? (
          <>
            {/* Stats Grid */}
            <motion.div
              className="grid grid-cols-1 md:grid-cols-4 gap-6 mb-8"
              variants={containerVariants}
              custom={1}
            >
              {[
                { label: 'Total Accuracy', value: `${Math.round(data.metrics?.accuracy || 0)}%`, icon: '✅' },
                { label: 'Shelves Scanned', value: data.shelves?.length || 0, icon: '📊' },
                { label: 'Items Found', value: data.metrics?.total_items_found || 0, icon: '📦' },
                { label: 'Issues Detected', value: data.metrics?.missing_items || 0, icon: '⚠️' },
              ].map((stat, index) => (
                <motion.div
                  key={stat.label}
                  className="bg-gray-800/40 backdrop-blur-xl border border-gray-700/50 rounded-xl p-6 relative overflow-hidden group"
                  custom={index}
                  variants={scaleInVariants}
                  whileHover={{ y: -10, borderColor: 'rgba(249, 115, 22, 0.8)' }}
                >
                  <div className="absolute inset-0 bg-gradient-to-br from-red-600 to-orange-600 opacity-0 group-hover:opacity-10 transition" />
                  <div className="relative">
                    <div className="text-4xl mb-2">{stat.icon}</div>
                    <p className="text-gray-400 text-sm mb-2">{stat.label}</p>
                    <p className="text-4xl font-bold text-transparent bg-gradient-to-r from-red-400 to-orange-400 bg-clip-text">
                      {stat.value}
                    </p>
                  </div>
                </motion.div>
              ))}
            </motion.div>

            {/* Shelves List */}
            <motion.div
              className="space-y-4"
              variants={containerVariants}
              custom={2}
            >
              <h2 className="text-2xl font-bold mb-6">Shelf Status</h2>

              {data.shelves?.map((shelf, index) => (
                <motion.div
                  key={shelf.shelf_id}
                  className="group"
                  custom={index}
                  variants={fadeInUpVariants}
                >
                  <motion.button
                    onClick={() => setExpandedShelf(expandedShelf === shelf.shelf_id ? null : shelf.shelf_id)}
                    className="w-full"
                    whileHover={{ x: 5 }}
                  >
                    <div className="bg-gray-800/40 backdrop-blur-xl border border-gray-700/50 hover:border-orange-500/50 rounded-xl p-6 transition cursor-pointer">
                      <div className="flex items-center justify-between">
                        <div className="flex items-center gap-4">
                          <div className={`w-3 h-3 rounded-full bg-gradient-to-r ${getStatusColor(shelf.status)}`} />
                          <div className="text-left">
                            <h3 className="text-xl font-semibold">{shelf.shelf_id}</h3>
                            <p className="text-gray-400 text-sm">
                              {shelf.items?.length || 0} items • Status: <span className="text-orange-400 font-semibold">{shelf.status}</span>
                            </p>
                          </div>
                        </div>

                        <div className="flex items-center gap-6">
                          <div className="text-right">
                            <p className="text-2xl font-bold text-orange-400">{Math.round((shelf.accuracy || 0))}%</p>
                            <p className="text-xs text-gray-400">Accuracy</p>
                          </div>

                          <motion.div
                            animate={{ rotate: expandedShelf === shelf.shelf_id ? 180 : 0 }}
                            transition={{ duration: 0.3 }}
                          >
                            <ChevronDown className="w-6 h-6 text-orange-500" />
                          </motion.div>
                        </div>
                      </div>
                    </div>
                  </motion.button>

                  {/* Expanded Details */}
                  <AnimatePresence>
                    {expandedShelf === shelf.shelf_id && (
                      <motion.div
                        initial={{ opacity: 0, height: 0 }}
                        animate={{ opacity: 1, height: 'auto' }}
                        exit={{ opacity: 0, height: 0 }}
                        transition={{ duration: 0.3 }}
                        className="mt-3 overflow-hidden"
                      >
                        <div className="bg-gray-800/20 backdrop-blur-xl border border-gray-700/30 rounded-xl p-6 space-y-4">
                          {shelf.items?.map((item, i) => (
                            <motion.div
                              key={i}
                              className="flex items-center justify-between p-4 bg-gray-800/40 rounded-lg"
                              initial={{ opacity: 0, x: -20 }}
                              animate={{ opacity: 1, x: 0 }}
                              transition={{ delay: i * 0.05 }}
                            >
                              <div className="flex-1">
                                <p className="font-semibold text-white">{item.product_name}</p>
                                <p className="text-sm text-gray-400">SKU: {item.sku}</p>
                              </div>
                              <div className="text-right flex items-center gap-4">
                                <div>
                                  <p className="text-orange-400 font-bold">{item.quantity_found}</p>
                                  <p className="text-xs text-gray-400">Expected: {item.quantity_expected}</p>
                                </div>
                                {getStatusIcon(item.status)}
                              </div>
                            </motion.div>
                          ))}
                        </div>
                      </motion.div>
                    )}
                  </AnimatePresence>
                </motion.div>
              ))}
            </motion.div>
          </>
        ) : (
          <motion.div className="text-center py-16" custom={1} variants={fadeInUpVariants}>
            <p className="text-gray-400">No data available</p>
          </motion.div>
        )}
      </motion.main>

      {/* Floating Background Elements */}
      <motion.div
        className="fixed top-20 right-20 w-72 h-72 bg-red-600/10 rounded-full blur-3xl pointer-events-none"
        animate={{ y: [0, -50, 0], scale: [0.8, 1.1, 0.8] }}
        transition={{ duration: 15, repeat: Infinity }}
      />
    </div>
  );
}
