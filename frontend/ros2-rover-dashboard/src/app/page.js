"use client";

import { useState } from "react";
import LoginPage from "../components/LoginPage";
import RoverDashboard from "../components/RoverDashboard";

/**
 * Root page — shows LoginPage until the user authenticates,
 * then renders RoverDashboard (unchanged).
 *
 * When you're ready to wire up real session persistence (cookies,
 * localStorage, a React context, etc.) just replace the `isAuthenticated`
 * state here — nothing else in the component tree needs to change.
 */
export default function Page() {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [currentUser, setCurrentUser] = useState(null);

  const handleLoginSuccess = (result) => {
    // result = { token, user } — store however you need:
    // localStorage.setItem("mars_token", result.token);
    setCurrentUser(result.user);
    setIsAuthenticated(true);
  };

  if (!isAuthenticated) {
    return <LoginPage onLoginSuccess={handleLoginSuccess} />;
  }

  // Pass currentUser down if RoverDashboard ever needs it
  return <RoverDashboard currentUser={currentUser} />;
}
