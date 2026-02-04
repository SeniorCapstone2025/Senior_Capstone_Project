import { useState, useEffect } from 'react';
import apiService from '../services/api';

export function useRoverStatus(pollingInterval = 2000) {
  const [status, setStatus] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const data = await apiService.getStatus();
        setStatus(data);
        setError(null);
      } catch (err) {
        setError(err.message);
        console.error('Failed to fetch rover status:', err);
      } finally {
        setLoading(false);
      }
    };

    // Initial fetch
    fetchStatus();

    // Set up polling
    const interval = setInterval(fetchStatus, pollingInterval);

    return () => clearInterval(interval);
  }, [pollingInterval]);

  return { status, loading, error };
}