'use client';

import { useState, useEffect } from 'react';
import { useRouter } from 'next/navigation';
import BackendInfo from '../../components/BackendInfo';

export default function BackendPage() {
  const router = useRouter();
  const [currentUser, setCurrentUser] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  // Restore authentication from localStorage on mount
  useEffect(() => {
    const restoreSession = async () => {
      try {
        const token = localStorage.getItem("mars_token");
        const userJSON = localStorage.getItem("mars_user");

        if (!token || !userJSON) {
          // Not logged in, redirect to home (login)
          router.push('/');
          return;
        }

        const user = JSON.parse(userJSON);
        setCurrentUser(user);
      } catch (error) {
        console.error("Failed to restore session:", error);
        // Clear corrupted data and redirect to login
        localStorage.removeItem("mars_token");
        localStorage.removeItem("mars_user");
        router.push('/');
      } finally {
        setIsLoading(false);
      }
    };

    restoreSession();
  }, [router]);

  const handleLogout = () => {
    // Clear session
    localStorage.removeItem("mars_token");
    localStorage.removeItem("mars_user");
    setCurrentUser(null);
    router.push('/');
  };

  if (isLoading) {
    return (
      <div className="min-h-screen bg-gray-950 flex items-center justify-center">
        <div className="text-center">
          <div className="w-12 h-12 border-2 border-orange-500/20 border-t-orange-500 rounded-full animate-spin mx-auto mb-4"></div>
          <p className="text-gray-400 font-mono text-sm">Restoring session...</p>
        </div>
      </div>
    );
  }

  return <BackendInfo currentUser={currentUser} onLogout={handleLogout} />;
}
