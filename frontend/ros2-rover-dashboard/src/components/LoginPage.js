"use client";

import { useState } from "react";
import { Shield, Eye, EyeOff, Wifi, WifiOff, AlertCircle, Loader } from "lucide-react";

// ─────────────────────────────────────────────
//  CONFIG — swap these for real env vars when ready
// ─────────────────────────────────────────────
const API_URL = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000";

// ─────────────────────────────────────────────
//  AUTHENTICATION SERVICE  (stub — wire up later)
// ─────────────────────────────────────────────
const authService = {
  /**
   * Replace the body of this function with your real auth call.
   * Expected response shape:
   *   { token: string, user: { id, username, role } }
   *
   * Throw an Error (with a user-readable .message) on failure.
   */
  async login(username, password) {
    // TODO: replace with real API call, e.g.:
    // const res = await fetch(`${API_URL}/auth/login`, {
    //   method: "POST",
    //   headers: { "Content-Type": "application/json" },
    //   body: JSON.stringify({ username, password }),
    // });
    // if (!res.ok) {
    //   const err = await res.json();
    //   throw new Error(err.detail || "Authentication failed");
    // }
    // return res.json();   // { token, user }

    // ── STUB: always resolves after 1 s ──
    await new Promise((r) => setTimeout(r, 1000));
    if (username === "admin" && password === "mars") {
      return { token: "stub-token", user: { id: 1, username, role: "operator" } };
    }
    throw new Error("Invalid credentials");
  },
};

// ─────────────────────────────────────────────
//  SMALL COMPONENTS
// ─────────────────────────────────────────────

function MarsLogo() {
  return (
    <div className="flex flex-col items-center gap-3">
      {/* Animated radar ring */}
      <div className="relative flex items-center justify-center w-20 h-20">
        <div className="absolute inset-0 rounded-full border-2 border-orange-500/20 animate-ping" />
        <div className="absolute inset-2 rounded-full border border-orange-500/30" />
        <div className="relative z-10 flex items-center justify-center w-14 h-14 rounded-full bg-gray-900 border-2 border-orange-500 shadow-[0_0_20px_rgba(249,115,22,0.4)]">
          <Shield className="w-7 h-7 text-orange-400" />
        </div>
      </div>

      <div className="text-center">
        <h1 className="text-2xl font-bold tracking-[0.2em] text-white font-mono">
          M.A.R.S.
        </h1>
        <p className="text-xs tracking-widest text-gray-400 uppercase mt-0.5">
          Mobile Autonomous Rover System
        </p>
      </div>
    </div>
  );
}

function ConnectionIndicator({ status }) {
  const map = {
    checking: { icon: Wifi,    color: "text-yellow-400", label: "Checking API…" },
    online:   { icon: Wifi,    color: "text-green-400",  label: "API Online"    },
    offline:  { icon: WifiOff, color: "text-red-400",    label: "API Offline"   },
  };
  const { icon: Icon, color, label } = map[status] || map.checking;

  return (
    <div className={`flex items-center gap-1.5 text-xs font-mono ${color}`}>
      <Icon className="w-3 h-3" />
      <span>{label}</span>
      <span className="text-gray-600">— {API_URL}</span>
    </div>
  );
}

// ─────────────────────────────────────────────
//  MAIN LOGIN COMPONENT
// ─────────────────────────────────────────────

export default function LoginPage({ onLoginSuccess }) {
  const [username, setUsername]         = useState("");
  const [password, setPassword]         = useState("");
  const [showPassword, setShowPassword] = useState(false);
  const [isLoading, setIsLoading]       = useState(false);
  const [error, setError]               = useState("");

  // Connection status — wire this up to a real health-check if desired
  // e.g. poll GET /health on mount and update state accordingly
  const [connectionStatus] = useState("online"); // "checking" | "online" | "offline"

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!username.trim() || !password.trim()) {
      setError("Username and password are required.");
      return;
    }

    setIsLoading(true);
    setError("");

    try {
      const result = await authService.login(username.trim(), password);

      // ── Store token however your app needs it ──
      // localStorage.setItem("mars_token", result.token);
      // or use a context / cookie / etc.

      // Notify parent — e.g. redirect to dashboard
      if (typeof onLoginSuccess === "function") {
        onLoginSuccess(result);
      }
    } catch (err) {
      setError(err.message || "Authentication failed. Please try again.");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="min-h-screen bg-gray-950 flex flex-col items-center justify-center p-4">
      {/* Subtle grid overlay */}
      <div
        className="fixed inset-0 pointer-events-none opacity-[0.03]"
        style={{
          backgroundImage:
            "linear-gradient(rgba(249,115,22,0.8) 1px, transparent 1px), linear-gradient(90deg, rgba(249,115,22,0.8) 1px, transparent 1px)",
          backgroundSize: "40px 40px",
        }}
      />

      <div className="relative w-full max-w-md">
        {/* Card */}
        <div className="bg-gray-900 border border-gray-800 rounded-2xl shadow-2xl overflow-hidden">

          {/* Top accent bar */}
          <div className="h-1 w-full bg-gradient-to-r from-orange-600 via-orange-400 to-orange-600" />

          <div className="p-8 flex flex-col gap-8">

            {/* Logo */}
            <MarsLogo />

            {/* Divider */}
            <div className="flex items-center gap-3">
              <div className="flex-1 h-px bg-gray-800" />
              <span className="text-xs font-mono text-gray-600 tracking-widest uppercase">
                Operator Access
              </span>
              <div className="flex-1 h-px bg-gray-800" />
            </div>

            {/* Form */}
            <form onSubmit={handleSubmit} className="flex flex-col gap-5">

              {/* Username */}
              <div className="flex flex-col gap-1.5">
                <label
                  htmlFor="username"
                  className="text-xs font-mono text-gray-400 tracking-widest uppercase"
                >
                  Username
                </label>
                <input
                  id="username"
                  type="text"
                  autoComplete="username"
                  autoFocus
                  value={username}
                  onChange={(e) => {
                    setUsername(e.target.value);
                    if (error) setError("");
                  }}
                  disabled={isLoading}
                  placeholder="Enter username"
                  className="w-full bg-gray-800 border border-gray-700 rounded-lg px-4 py-3
                             text-white font-mono text-sm placeholder-gray-600
                             focus:outline-none focus:ring-2 focus:ring-orange-500 focus:border-transparent
                             disabled:opacity-50 disabled:cursor-not-allowed
                             transition-all duration-200"
                />
              </div>

              {/* Password */}
              <div className="flex flex-col gap-1.5">
                <label
                  htmlFor="password"
                  className="text-xs font-mono text-gray-400 tracking-widest uppercase"
                >
                  Password
                </label>
                <div className="relative">
                  <input
                    id="password"
                    type={showPassword ? "text" : "password"}
                    autoComplete="current-password"
                    value={password}
                    onChange={(e) => {
                      setPassword(e.target.value);
                      if (error) setError("");
                    }}
                    disabled={isLoading}
                    placeholder="Enter password"
                    className="w-full bg-gray-800 border border-gray-700 rounded-lg px-4 py-3 pr-12
                               text-white font-mono text-sm placeholder-gray-600
                               focus:outline-none focus:ring-2 focus:ring-orange-500 focus:border-transparent
                               disabled:opacity-50 disabled:cursor-not-allowed
                               transition-all duration-200"
                  />
                  <button
                    type="button"
                    onClick={() => setShowPassword((v) => !v)}
                    disabled={isLoading}
                    className="absolute right-3 top-1/2 -translate-y-1/2 text-gray-500
                               hover:text-gray-300 disabled:opacity-50 transition-colors duration-150"
                    aria-label={showPassword ? "Hide password" : "Show password"}
                  >
                    {showPassword ? <EyeOff className="w-4 h-4" /> : <Eye className="w-4 h-4" />}
                  </button>
                </div>
              </div>

              {/* Error message */}
              {error && (
                <div className="flex items-center gap-2 bg-red-900/30 border border-red-700/50 rounded-lg px-4 py-3">
                  <AlertCircle className="w-4 h-4 text-red-400 flex-shrink-0" />
                  <p className="text-sm font-mono text-red-300">{error}</p>
                </div>
              )}

              {/* Submit */}
              <button
                type="submit"
                disabled={isLoading}
                className="w-full flex items-center justify-center gap-2
                           bg-orange-600 hover:bg-orange-500 active:bg-orange-700
                           disabled:bg-gray-700 disabled:cursor-not-allowed
                           text-white font-mono text-sm font-semibold tracking-widest uppercase
                           py-3 rounded-lg
                           shadow-[0_0_20px_rgba(249,115,22,0.25)] hover:shadow-[0_0_28px_rgba(249,115,22,0.4)]
                           transition-all duration-200"
              >
                {isLoading ? (
                  <>
                    <Loader className="w-4 h-4 animate-spin" />
                    Authenticating…
                  </>
                ) : (
                  "Access System"
                )}
              </button>
            </form>

            {/* Footer */}
            <div className="flex flex-col items-center gap-2">
              <ConnectionIndicator status={connectionStatus} />
              <p className="text-xs text-gray-700 font-mono">
                v1.0.0 — Authorized Personnel Only
              </p>
            </div>
          </div>
        </div>

        {/* Outer glow */}
        <div className="absolute -inset-px rounded-2xl bg-orange-500/5 blur-xl -z-10" />
      </div>
    </div>
  );
}
