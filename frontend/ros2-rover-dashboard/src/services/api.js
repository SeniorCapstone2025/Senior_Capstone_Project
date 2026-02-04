const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000';

class ApiService {
  async request(endpoint, options = {}) {
    const url = `${API_BASE_URL}${endpoint}`;
    const config = {
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
      ...options,
    };

    try {
      const response = await fetch(url, config);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error(`API Error (${endpoint}):`, error);
      throw error;
    }
  }

  // Command endpoints
  async sendCommand(command) {
    return this.request('/command/', {
      method: 'POST',
      body: JSON.stringify({ command }),
    });
  }

  // Status endpoints
  async getStatus() {
    return this.request('/status/');
  }

  // Detections endpoint
  async submitDetection(detection) {
    return this.request('/detections/', {
      method: 'POST',
      body: JSON.stringify(detection),
    });
  }

  // Inventory endpoints
  async getInventoryItem(itemId) {
    return this.request(`/inventory/${itemId}`);
  }

  async createInventoryItem(item) {
    return this.request('/inventory/', {
      method: 'POST',
      body: JSON.stringify(item),
    });
  }

  // Health check
  async healthCheck() {
    return this.request('/health');
  }

  // Test database connection
  async testDatabase() {
    return this.request('/command/test-db');
  }

}

export default new ApiService();