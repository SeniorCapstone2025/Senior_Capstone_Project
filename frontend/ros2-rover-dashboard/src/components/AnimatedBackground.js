'use client';

import { useRef, useEffect } from 'react';
import { motion } from 'framer-motion';

export default function AnimatedBackground({ variant = 'grid' }) {
  const containerRef = useRef(null);

  useEffect(() => {
    if (variant === 'grid' && containerRef.current) {
      const handleMouseMove = (e) => {
        const rect = containerRef.current.getBoundingClientRect();
        const x = (e.clientX - rect.left) / rect.width;
        const y = (e.clientY - rect.top) / rect.height;

        containerRef.current.style.setProperty('--mouse-x', x);
        containerRef.current.style.setProperty('--mouse-y', y);
      };

      window.addEventListener('mousemove', handleMouseMove);
      return () => window.removeEventListener('mousemove', handleMouseMove);
    }
  }, [variant]);

  if (variant === 'grid') {
    return (
      <div
        ref={containerRef}
        className="absolute inset-0 overflow-hidden bg-gray-900"
      >
        {/* Gradient background */}
        <div className="absolute inset-0 bg-gradient-to-br from-gray-900 via-red-900/20 to-gray-900" />

        {/* Animated grid */}
        <div className="absolute inset-0 opacity-30">
          <div className="absolute inset-0" style={{
            backgroundImage: `
              linear-gradient(rgba(220, 38, 38, 0.3) 1px, transparent 1px),
              linear-gradient(90deg, rgba(220, 38, 38, 0.3) 1px, transparent 1px)
            `,
            backgroundSize: '50px 50px',
            animation: 'moveGrid 20s linear infinite'
          }} />
        </div>

        {/* Floating particles */}
        {[...Array(20)].map((_, i) => (
          <motion.div
            key={i}
            className="absolute w-1 h-1 bg-red-500 rounded-full"
            style={{
              left: `${Math.random() * 100}%`,
              top: `${Math.random() * 100}%`,
            }}
            animate={{
              y: [0, -100, 0],
              x: [0, Math.random() * 100 - 50, 0],
              opacity: [0.3, 0.8, 0.3],
            }}
            transition={{
              duration: 5 + Math.random() * 3,
              repeat: Infinity,
              ease: 'easeInOut',
            }}
          />
        ))}

        {/* Glow orbs */}
        {[...Array(3)].map((_, i) => (
          <motion.div
            key={`orb-${i}`}
            className="absolute rounded-full blur-3xl"
            style={{
              width: 300 + i * 50,
              height: 300 + i * 50,
              left: `${-50 + i * 50}%`,
              top: `${-30 + i * 40}%`,
              background: i === 0 ? 'rgba(220, 38, 38, 0.1)' : i === 1 ? 'rgba(249, 115, 22, 0.08)' : 'rgba(220, 38, 38, 0.05)',
            }}
            animate={{
              scale: [0.8, 1.2, 0.8],
              x: [0, 50, 0],
              y: [0, -50, 0],
            }}
            transition={{
              duration: 8 + i * 2,
              repeat: Infinity,
              ease: 'easeInOut',
            }}
          />
        ))}

        <style jsx>{`
          @keyframes moveGrid {
            0% {
              transform: translate(0, 0);
            }
            100% {
              transform: translate(50px, 50px);
            }
          }
        `}</style>
      </div>
    );
  }

  if (variant === 'gradient') {
    return (
      <div className="absolute inset-0 bg-gradient-to-br from-gray-900 via-red-900/30 to-orange-900/20" />
    );
  }

  return null;
}
