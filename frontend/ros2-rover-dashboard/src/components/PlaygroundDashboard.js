'use client';

import { useState, useEffect, useRef } from 'react';
import { useRouter } from 'next/navigation';
import { motion } from 'framer-motion';
import {
  BarChart3,
  Database,
  Zap,
  Navigation,
  LogOut,
  Menu,
  X,
} from 'lucide-react';
import AnimatedBackground from './AnimatedBackground';
import { fadeInUpVariants, containerVariants, glowVariants, scaleInVariants } from '@/utils/animations';

const navigationItems = [
  {
    icon: Navigation,
    label: 'Dashboard',
    href: '/',
    color: 'from-red-600 to-red-500',
  },
  {
    icon: BarChart3,
    label: 'Inventory Metrics',
    href: '/inventory',
    color: 'from-orange-600 to-orange-500',
  },
  {
    icon: Database,
    label: 'Backend Monitor',
    href: '/backend',
    color: 'from-red-600 to-orange-600',
  },
];

export default function PlaygroundDashboard() {
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

  const handleLogout = () => {
    localStorage.removeItem('mars_token');
    localStorage.removeItem('mars_user');
    router.push('/login');
  };

  const handleNavigate = (href) => {
    router.push(href);
    setMobileMenuOpen(false);
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
            {navigationItems.map((item, index) => (
              <motion.button
                key={item.label}
                onClick={() => handleNavigate(item.href)}
                className="px-4 py-2 rounded-lg bg-gray-800/50 hover:bg-gray-800 transition flex items-center gap-2 group"
                whileHover={{ scale: 1.05 }}
                whileTap={{ scale: 0.95 }}
              >
                <item.icon className="w-5 h-5 group-hover:text-orange-500 transition" />
                <span className="text-sm">{item.label}</span>
              </motion.button>
            ))}
          </div>

          {/* User Menu */}
          <div className="flex items-center gap-4">
            {currentUser && (
              <motion.div
                className="px-4 py-2 bg-gray-800/50 rounded-lg text-sm"
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
              >
                Welcome, <span className="font-semibold text-orange-400">{currentUser.username}</span>
              </motion.div>
            )}
            <motion.button
              onClick={handleLogout}
              className="px-4 py-2 rounded-lg bg-red-600/50 hover:bg-red-600 transition flex items-center gap-2"
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
            >
              <LogOut className="w-5 h-5" />
              <span className="hidden sm:inline text-sm">Logout</span>
            </motion.button>

            {/* Mobile Menu Toggle */}
            <button
              className="md:hidden p-2"
              onClick={() => setMobileMenuOpen(!mobileMenuOpen)}
            >
              {mobileMenuOpen ? (
                <X className="w-6 h-6" />
              ) : (
                <Menu className="w-6 h-6" />
              )}
            </button>
          </div>
        </div>

        {/* Mobile Menu */}
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
                whileHover={{ x: 10 }}
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
        className="relative z-10 max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-12"
        variants={containerVariants}
        initial="hidden"
        animate="visible"
      >
        {/* Welcome Section */}
        <motion.div custom={0} variants={fadeInUpVariants} className="mb-16">
          <h1 className="text-5xl font-bold mb-4">
            Welcome to <span className="bg-gradient-to-r from-red-600 to-orange-600 bg-clip-text text-transparent">M.A.R.S</span>
          </h1>
          <p className="text-gray-400 text-lg max-w-2xl">
            Multi-Agent Rover System Dashboard. Monitor your rover, track inventory, and manage backend operations in real-time.
          </p>
        </motion.div>

        {/* Interactive Cards Grid */}
        <motion.div
          className="grid grid-cols-1 md:grid-cols-3 gap-8 mb-12"
          variants={containerVariants}
        >
          {navigationItems.map((item, index) => (
            <motion.button
              key={item.label}
              onClick={() => handleNavigate(item.href)}
              className="group relative h-64 rounded-2xl overflow-hidden cursor-pointer"
              custom={index + 1}
              variants={scaleInVariants}
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
            >
              {/* Gradient Background */}
              <div className={`absolute inset-0 bg-gradient-to-br ${item.color} opacity-0 group-hover:opacity-20 transition duration-300`} />

              {/* Glass Morphism Card */}
              <div className="absolute inset-0 bg-gray-800/40 backdrop-blur-xl border border-gray-700/50 group-hover:border-orange-500/50 transition" />

              {/* Content */}
              <div className="relative h-full flex flex-col items-center justify-center p-6 text-center">
                <motion.div
                  animate={glowVariants.animate}
                  className="mb-4 p-4 bg-gradient-to-br from-red-600 to-orange-600 rounded-2xl"
                >
                  <item.icon className="w-8 h-8 text-white" />
                </motion.div>

                <h3 className="text-2xl font-bold mb-2">{item.label}</h3>

                <div className="text-4xl font-bold bg-gradient-to-r from-red-400 to-orange-400 bg-clip-text text-transparent opacity-0 group-hover:opacity-100 transition">
                  {index === 0 ? '📊' : index === 1 ? '📦' : '🗄️'}
                </div>

                <motion.div
                  className="mt-4 text-orange-400 opacity-0 group-hover:opacity-100 transition"
                  animate={{ x: [0, 5, 0] }}
                  transition={{ duration: 2, repeat: Infinity }}
                >
                  Click to explore →
                </motion.div>
              </div>

              {/* Animated Border */}
              <motion.div
                className="absolute inset-0 rounded-2xl pointer-events-none"
                style={{
                  border: '2px solid transparent',
                  borderImage: 'linear-gradient(90deg, #dc2626, #ea580c, #dc2626) 1',
                }}
                animate={{
                  borderImage: [
                    'linear-gradient(90deg, #dc2626, #ea580c, #dc2626) 1',
                    'linear-gradient(180deg, #dc2626, #ea580c, #dc2626) 1',
                    'linear-gradient(270deg, #dc2626, #ea580c, #dc2626) 1',
                    'linear-gradient(360deg, #dc2626, #ea580c, #dc2626) 1',
                  ],
                }}
                transition={{ duration: 4, repeat: Infinity }}
              />
            </motion.button>
          ))}
        </motion.div>

        {/* Stats Section */}
        <motion.div
          className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6"
          custom={4}
          variants={fadeInUpVariants}
        >
          {[
            { label: 'Active Scans', value: '12', icon: '📊' },
            { label: 'DB Records', value: '1.2K', icon: '💾' },
            { label: 'System Status', value: 'Online', icon: '✅' },
            { label: 'API Health', value: '100%', icon: '⚡' },
          ].map((stat, index) => (
            <motion.div
              key={stat.label}
              className="bg-gray-800/40 backdrop-blur-xl border border-gray-700/50 rounded-xl p-6"
              whileHover={{ y: -5, borderColor: 'rgba(249, 115, 22, 0.8)' }}
            >
              <div className="text-3xl mb-2">{stat.icon}</div>
              <p className="text-gray-400 text-sm mb-2">{stat.label}</p>
              <p className="text-3xl font-bold text-orange-400">{stat.value}</p>
            </motion.div>
          ))}
        </motion.div>
      </motion.main>

      {/* Floating Background Elements */}
      <motion.div
        className="fixed top-10 right-10 w-96 h-96 bg-red-600/10 rounded-full blur-3xl pointer-events-none"
        animate={{
          x: [0, 100, 0],
          y: [0, -100, 0],
          scale: [0.8, 1.2, 0.8],
        }}
        transition={{
          duration: 20,
          repeat: Infinity,
          ease: 'easeInOut',
        }}
      />
      <motion.div
        className="fixed bottom-10 left-10 w-72 h-72 bg-orange-600/10 rounded-full blur-3xl pointer-events-none"
        animate={{
          x: [0, -100, 0],
          y: [0, 100, 0],
          scale: [0.8, 1.1, 0.8],
        }}
        transition={{
          duration: 25,
          repeat: Infinity,
          ease: 'easeInOut',
          delay: 2,
        }}
      />
    </div>
  );
}
