'use client';

import { useRouter } from 'next/navigation';
import { useState, useEffect } from 'react';
import { robotAPI } from '@/lib/api';

export default function CameraPage() {
  const router = useRouter();
  const [cameraFrame, setCameraFrame] = useState<string>('');
  const [trackingActive, setTrackingActive] = useState(false);
  const [safetyStop, setSafetyStop] = useState(false);

  useEffect(() => {
    // Update camera feed
    const updateCamera = async () => {
      try {
        const frame = await robotAPI.getCameraFrame();
        setCameraFrame(frame);
      } catch (error) {
        console.error('Camera error:', error);
      }
    };

    // Update status
    const updateStatus = async () => {
      try {
        const status = await robotAPI.getStatus();
        setTrackingActive(status.tracking_active);
        setSafetyStop(status.safety_stop);
      } catch (error) {
        console.error('Status error:', error);
      }
    };

    // Update camera every 100ms (10 FPS)
    const cameraInterval = setInterval(updateCamera, 100);
    updateCamera();

    // Update status every second
    const statusInterval = setInterval(updateStatus, 1000);
    updateStatus();

    return () => {
      clearInterval(cameraInterval);
      clearInterval(statusInterval);
    };
  }, []);

  const handleImageClick = (e: React.MouseEvent<HTMLImageElement>) => {
    const rect = e.currentTarget.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    console.log('Selected point:', x, y);
    // TODO: Send point selection to ROS2
  };

  return (
    <div className="min-h-screen bg-gray-900 p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="flex justify-between items-center mb-6 pb-4 border-b-2 border-gray-700">
          <button
            onClick={() => router.push('/dashboard')}
            className="btn-primary"
          >
            ← Back
          </button>
          <h1 className="text-3xl font-bold text-white">Camera Tracking</h1>
          <div className="flex items-center space-x-4">
            <div className="flex items-center space-x-2">
              <div
                className={`w-4 h-4 rounded-full ${
                  trackingActive ? 'bg-green-500' : 'bg-red-500'
                } animate-pulse`}
              ></div>
              <span className="text-white font-medium">
                {trackingActive ? 'Tracking: Active' : 'Tracking: Inactive'}
              </span>
            </div>
            {safetyStop && (
              <span className="bg-red-600 text-white px-4 py-2 rounded-lg font-semibold">
                STOPPED
              </span>
            )}
          </div>
        </div>

        {/* Camera Viewport */}
        <div className="flex justify-center items-center min-h-[calc(100vh-200px)]">
          <div className="relative bg-black rounded-2xl overflow-hidden shadow-2xl border-4 border-gray-700 max-w-full max-h-[90vh]">
            {cameraFrame ? (
              <img
                src={cameraFrame}
                alt="Camera Feed"
                onClick={handleImageClick}
                className="max-w-full max-h-[90vh] cursor-crosshair"
              />
            ) : (
              <div className="w-full h-96 flex items-center justify-center text-white text-xl">
                Loading camera feed...
              </div>
            )}
            <div className="absolute top-4 left-4 bg-black/70 text-white px-4 py-2 rounded-lg text-sm">
              Click on the image to select tracking target
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

