'use client';

import { useState, useRef, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import { motion } from 'framer-motion';
import { ShieldCheck, AlertCircle, Loader } from 'lucide-react';
import AnimatedBackground from './AnimatedBackground';
import { fadeInUpVariants, containerVariants, glowVariants } from '@/utils/animations';

export default function PlaygroundLogin() {
  const [username, setUsername] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const [mousePosition, setMousePosition] = useState({ x: 0, y: 0 });
  const containerRef = useRef(null);
  const router = useRouter();

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

  const handleLogin = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const response = await fetch('http://localhost:8000/api/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ username, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        setError(data.detail || 'Login failed');
        setLoading(false);
        return;
      }

      if (data.access_token) {
        localStorage.setItem('mars_token', data.access_token);
        localStorage.setItem('mars_user', JSON.stringify(data.user || { username }));
        router.push('/');
      }
    } catch (err) {
      setError('Failed to connect to server');
      setLoading(false);
    }
  };

  return (
    <div
      ref={containerRef}
      className="min-h-screen w-full relative overflow-hidden bg-gray-900"
    >
      <AnimatedBackground variant="grid" />

      {/* Content Container */}
      <div className="relative h-screen flex items-center justify-center z-10 p-4">
        <motion.div
          className="w-full max-w-md"
          variants={containerVariants}
          initial="hidden"
          animate="visible"
        >
          {/* Logo Section */}
          <motion.div
            className="mb-12 text-center"
            custom={0}
            variants={fadeInUpVariants}
          >
            <motion.div
              className="inline-block relative mb-6"
              animate={{ rotateY: mousePosition.x * 20 - 10 }}
              style={{ perspective: 1000 }}
            >
              <motion.div
                className="w-20 h-20 rounded-2xl bg-gradient-to-br from-red-600 to-orange-600 flex items-center justify-center"
                animate={glowVariants.animate}
                whileHover={{ scale: 1.1 }}
              >
                <ShieldCheck className="w-10 h-10 text-white" />
              </motion.div>
            </motion.div>

            <h1 className="text-4xl font-bold text-white mb-2">M.A.R.S</h1>
            <p className="text-gray-400 text-sm">Multi-Agent Rover System</p>
          </motion.div>

          {/* Form Container */}
          <motion.div
            className="relative group"
            custom={1}
            variants={fadeInUpVariants}
          >
            {/* 3D effect border */}
            <div className="absolute -inset-1 bg-gradient-to-r from-red-600 to-orange-600 rounded-2xl blur opacity-0 group-hover:opacity-100 transition duration-500" />
            <div className="absolute -inset-1 bg-gradient-to-r from-red-600 to-orange-600 rounded-2xl blur-2xl opacity-0 group-hover:opacity-50 transition duration-500" />

            {/* Form */}
            <motion.form
              onSubmit={handleLogin}
              className="relative bg-gray-800/80 backdrop-blur-xl rounded-2xl p-8 border border-gray-700 hover:border-red-500/50 transition"
            >
              {/* Username Input */}
              <motion.div
                className="mb-6"
                custom={2}
                variants={fadeInUpVariants}
              >
                <label className="block text-sm font-semibold text-gray-300 mb-3">
                  Username
                </label>
                <motion.input
                  type="text"
                  value={username}
                  onChange={(e) => setUsername(e.target.value)}
                  placeholder="Enter your username"
                  className="w-full px-4 py-3 bg-gray-700/50 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:outline-none focus:border-red-500 focus:ring-2 focus:ring-red-500/50 transition"
                  whileFocus={{ scale: 1.02, boxShadow: '0 0 20px rgba(220,38,38,0.3)' }}
                  disabled={loading}
                />
              </motion.div>

              {/* Password Input */}
              <motion.div
                className="mb-6"
                custom={3}
                variants={fadeInUpVariants}
              >
                <label className="block text-sm font-semibold text-gray-300 mb-3">
                  Password
                </label>
                <motion.input
                  type="password"
                  value={password}
                  onChange={(e) => setPassword(e.target.value)}
                  placeholder="Enter your password"
                  className="w-full px-4 py-3 bg-gray-700/50 border border-gray-600 rounded-lg text-white placeholder-gray-400 focus:outline-none focus:border-red-500 focus:ring-2 focus:ring-red-500/50 transition"
                  whileFocus={{ scale: 1.02, boxShadow: '0 0 20px rgba(220,38,38,0.3)' }}
                  disabled={loading}
                  onKeyPress={(e) => e.key === 'Enter' && handleLogin(e)}
                />
              </motion.div>

              {/* Error Message */}
              {error && (
                <motion.div
                  className="mb-6 p-4 bg-red-900/30 border border-red-500/50 rounded-lg flex items-start gap-3"
                  initial={{ opacity: 0, y: -10 }}
                  animate={{ opacity: 1, y: 0 }}
                  exit={{ opacity: 0, y: -10 }}
                >
                  <AlertCircle className="w-5 h-5 text-red-400 flex-shrink-0 mt-0.5" />
                  <p className="text-red-300 text-sm">{error}</p>
                </motion.div>
              )}

              {/* Submit Button */}
              <motion.button
                type="submit"
                disabled={loading}
                className="w-full bg-gradient-to-r from-red-600 to-orange-600 hover:from-red-700 hover:to-orange-700 disabled:from-gray-600 disabled:to-gray-600 text-white font-semibold py-3 px-4 rounded-lg transition duration-200 flex items-center justify-center gap-2"
                whileHover={{ scale: 1.02 }}
                whileTap={{ scale: 0.98 }}
                custom={4}
                variants={fadeInUpVariants}
              >
                {loading ? (
                  <>
                    <Loader className="w-5 h-5 animate-spin" />
                    Logging in...
                  </>
                ) : (
                  'Login to Dashboard'
                )}
              </motion.button>

              {/* Demo Credentials Hint */}
              <motion.p
                className="text-center text-gray-400 text-xs mt-6"
                custom={5}
                variants={fadeInUpVariants}
              >
                Demo: admin / admin1234
              </motion.p>
            </motion.form>
          </motion.div>

          {/* Floating Elements */}
          <motion.div
            className="absolute -top-20 right-10 w-32 h-32 bg-red-600/20 rounded-full blur-3xl"
            animate={{
              y: [0, -30, 0],
              x: [0, 20, 0],
              scale: [0.8, 1.2, 0.8],
            }}
            transition={{
              duration: 8,
              repeat: Infinity,
              ease: 'easeInOut',
            }}
          />
          <motion.div
            className="absolute -bottom-20 left-10 w-40 h-40 bg-orange-600/10 rounded-full blur-3xl"
            animate={{
              y: [0, 30, 0],
              x: [0, -20, 0],
              scale: [0.8, 1.1, 0.8],
            }}
            transition={{
              duration: 10,
              repeat: Infinity,
              ease: 'easeInOut',
              delay: 1,
            }}
          />
        </motion.div>
      </div>
    </div>
  );
}
