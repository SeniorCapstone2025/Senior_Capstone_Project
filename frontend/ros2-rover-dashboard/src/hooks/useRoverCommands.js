import { useState } from 'react';
import apiService from '../services/api';

export function useRoverCommands() {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState(null);

  const sendCommand = async (command) => {
    setLoading(true);
    setError(null);

    try {
      const response = await apiService.sendCommand(command);
      console.log('Command sent successfully:', response);
      return response;
    } catch (err) {
      setError(err.message);
      console.error('Failed to send command:', err);
      throw err;
    } finally {
      setLoading(false);
    }
  };

  return { sendCommand, loading, error };
}