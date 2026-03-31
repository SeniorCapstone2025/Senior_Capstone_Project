"use client";

import { useState, useEffect } from "react";
import LoginPage from "../components/LoginPage";
import RoverDashboard from "../components/RoverDashboard";

/**
 * Root page — shows LoginPage until the user authenticates,
 * then renders RoverDashboard.
 *
 * Now with persistent session — restores authentication from localStorage
 * so users don't have to log in again when navigating between pages.
 */
export default function Page() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [currentUser, setCurrentUser] = useState(null);
  const [isLoading, setIsLoading] = useState(true);

  // Restore authentication from localStorage on mount
  useEffect(() => {
    const restoreSession = async () => {
      try {
        const token = localStorage.getItem("mars_token");
        const userJSON = localStorage.getItem("mars_user");

        if (token && userJSON) {
          const user = JSON.parse(userJSON);
          setCurrentUser(user);
          setIsAuthenticated(true);
        }
      } catch (error) {
        console.error("Failed to restore session:", error);
        // Clear corrupted data
        localStorage.removeItem("mars_token");
        localStorage.removeItem("mars_user");
      } finally {
        setIsLoading(false);
      }
    };

    restoreSession();
  }, []);

  const handleLoginSuccess = (result) => {
    // result = { token, user } — already saved to localStorage by LoginPage
    setCurrentUser(result.user);
    setIsAuthenticated(true);
  };

  const handleLogout = () => {
    // Clear session
    localStorage.removeItem("mars_token");
    localStorage.removeItem("mars_user");
    setCurrentUser(null);
    setIsAuthenticated(false);
  };

  if (isLoading) {
    // Show loading state while restoring session
    return (
      <div className="min-h-screen bg-gray-950 flex items-center justify-center">
        <div className="text-center">
          <div className="w-12 h-12 border-2 border-orange-500/20 border-t-orange-500 rounded-full animate-spin mx-auto mb-4"></div>
          <p className="text-gray-400 font-mono text-sm">Restoring session...</p>
        </div>
      </div>
    );
  }

  if (!isAuthenticated) {
    return <LoginPage onLoginSuccess={handleLoginSuccess} />;
  }

  // Pass currentUser and logout handler down to RoverDashboard
  return <RoverDashboard currentUser={currentUser} onLogout={handleLogout} />;
}
