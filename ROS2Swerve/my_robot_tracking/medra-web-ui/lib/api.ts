import axios from 'axios';

const API_BASE_URL = process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8080';

const api = axios.create({
  baseURL: API_BASE_URL,
  headers: {
    'Content-Type': 'application/json',
  },
});

export interface RobotStatus {
  tracking_active: boolean;
  safety_stop: boolean;
}

export const robotAPI = {
  getStatus: async (): Promise<RobotStatus> => {
    const response = await api.get('/api/status');
    return response.data;
  },

  moveRobot: async (direction: 'up' | 'down' | 'left' | 'right', speed: number = 50) => {
    return api.post('/api/move', { direction, speed });
  },

  stopRobot: async () => {
    return api.post('/api/stop');
  },

  steerRobot: async (angle: number) => {
    return api.post('/api/steer', { angle });
  },

  controlTopHalf: async (action: string) => {
    return api.post('/api/top_half', { action });
  },

  getCameraFrame: async (): Promise<string> => {
    const response = await api.get('/api/camera_frame');
    return response.data.frame;
  },
};

