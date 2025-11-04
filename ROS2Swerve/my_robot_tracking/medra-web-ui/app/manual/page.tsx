'use client';

import { useRouter } from 'next/navigation';
import { useState, useEffect } from 'react';
import { robotAPI } from '@/lib/api';

export default function ManualPage() {
  const router = useRouter();
  const [isMoving, setIsMoving] = useState<string | null>(null);
  let moveInterval: NodeJS.Timeout | null = null;

  const startMove = async (direction: 'up' | 'down' | 'left' | 'right') => {
    if (moveInterval) return;
    setIsMoving(direction);
    
    await robotAPI.moveRobot(direction, 50);
    
    moveInterval = setInterval(() => {
      robotAPI.moveRobot(direction, 50).catch(console.error);
    }, 100);
  };

  const stopMove = async () => {
    if (moveInterval) {
      clearInterval(moveInterval);
      moveInterval = null;
    }
    setIsMoving(null);
    await robotAPI.stopRobot();
  };

  useEffect(() => {
    return () => {
      if (moveInterval) {
        clearInterval(moveInterval);
      }
    };
  }, []);

  const handleTopHalf = async (action: string) => {
    await robotAPI.controlTopHalf(action);
  };

  return (
    <div className="min-h-screen bg-white p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="flex justify-between items-center mb-8 pb-4 border-b-2 border-gray-200">
          <button
            onClick={() => router.push('/dashboard')}
            className="btn-primary"
          >
            ← Back
          </button>
          <h1 className="text-3xl font-bold text-gray-800">Manual Control</h1>
          <div className="w-24"></div>
        </div>

        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 mt-8">
          {/* Robot Movement Controls */}
          <div className="space-y-6">
            <h2 className="text-2xl font-semibold text-primary-600 text-center">
              Robot Movement
            </h2>
            <div className="flex flex-col items-center space-y-4">
              {/* Up */}
              <button
                onMouseDown={() => startMove('up')}
                onMouseUp={stopMove}
                onTouchStart={() => startMove('up')}
                onTouchEnd={stopMove}
                className={`w-32 h-32 text-5xl font-bold rounded-2xl border-4 transition-all duration-200 ${
                  isMoving === 'up'
                    ? 'bg-primary-500 text-white border-primary-600 scale-95'
                    : 'bg-white text-primary-600 border-primary-500 hover:bg-primary-50 hover:scale-105'
                } shadow-lg`}
              >
                ↑
              </button>

              {/* Left and Right */}
              <div className="flex space-x-4">
                <button
                  onMouseDown={() => startMove('left')}
                  onMouseUp={stopMove}
                  onTouchStart={() => startMove('left')}
                  onTouchEnd={stopMove}
                  className={`w-32 h-32 text-5xl font-bold rounded-2xl border-4 transition-all duration-200 ${
                    isMoving === 'left'
                      ? 'bg-primary-500 text-white border-primary-600 scale-95'
                      : 'bg-white text-primary-600 border-primary-500 hover:bg-primary-50 hover:scale-105'
                  } shadow-lg`}
                >
                  ←
                </button>
                <button
                  onMouseDown={() => startMove('right')}
                  onMouseUp={stopMove}
                  onTouchStart={() => startMove('right')}
                  onTouchEnd={stopMove}
                  className={`w-32 h-32 text-5xl font-bold rounded-2xl border-4 transition-all duration-200 ${
                    isMoving === 'right'
                      ? 'bg-primary-500 text-white border-primary-600 scale-95'
                      : 'bg-white text-primary-600 border-primary-500 hover:bg-primary-50 hover:scale-105'
                  } shadow-lg`}
                >
                  →
                </button>
              </div>

              {/* Down */}
              <button
                onMouseDown={() => startMove('down')}
                onMouseUp={stopMove}
                onTouchStart={() => startMove('down')}
                onTouchEnd={stopMove}
                className={`w-32 h-32 text-5xl font-bold rounded-2xl border-4 transition-all duration-200 ${
                  isMoving === 'down'
                    ? 'bg-primary-500 text-white border-primary-600 scale-95'
                    : 'bg-white text-primary-600 border-primary-500 hover:bg-primary-50 hover:scale-105'
                } shadow-lg`}
              >
                ↓
              </button>
            </div>
          </div>

          {/* Top Half Controls */}
          <div className="space-y-6">
            <h2 className="text-2xl font-semibold text-secondary-600 text-center">
              Top Half Control
            </h2>
            <div className="flex flex-col items-center space-y-4">
              {/* Up Controls */}
              <div className="flex items-center space-x-4">
                <div className="flex flex-col items-center space-y-2">
                  <span className="text-4xl text-secondary-600">↑</span>
                  <div className="flex space-x-2">
                    <button
                      onClick={() => handleTopHalf('up_plus')}
                      className="w-12 h-12 bg-secondary-500 text-white rounded-lg font-bold hover:bg-secondary-600 active:scale-95 transition-all shadow-lg"
                    >
                      +
                    </button>
                    <button
                      onClick={() => handleTopHalf('up_minus')}
                      className="w-12 h-12 bg-secondary-500 text-white rounded-lg font-bold hover:bg-secondary-600 active:scale-95 transition-all shadow-lg"
                    >
                      -
                    </button>
                  </div>
                </div>
              </div>

              {/* Two Up Buttons */}
              <div className="flex space-x-4">
                <button
                  onMouseDown={() => handleTopHalf('up')}
                  onMouseUp={() => handleTopHalf('stop')}
                  className="w-40 h-24 text-3xl font-bold rounded-xl border-4 border-secondary-500 bg-white text-secondary-600 hover:bg-secondary-50 active:bg-secondary-500 active:text-white active:scale-95 transition-all shadow-lg"
                >
                  ↑
                </button>
                <button
                  onMouseDown={() => handleTopHalf('up')}
                  onMouseUp={() => handleTopHalf('stop')}
                  className="w-40 h-24 text-3xl font-bold rounded-xl border-4 border-secondary-500 bg-white text-secondary-600 hover:bg-secondary-50 active:bg-secondary-500 active:text-white active:scale-95 transition-all shadow-lg"
                >
                  ↑
                </button>
              </div>

              {/* Two Down Buttons */}
              <div className="flex space-x-4">
                <button
                  onMouseDown={() => handleTopHalf('down')}
                  onMouseUp={() => handleTopHalf('stop')}
                  className="w-40 h-24 text-3xl font-bold rounded-xl border-4 border-secondary-500 bg-white text-secondary-600 hover:bg-secondary-50 active:bg-secondary-500 active:text-white active:scale-95 transition-all shadow-lg"
                >
                  ↓
                </button>
                <button
                  onMouseDown={() => handleTopHalf('down')}
                  onMouseUp={() => handleTopHalf('stop')}
                  className="w-40 h-24 text-3xl font-bold rounded-xl border-4 border-secondary-500 bg-white text-secondary-600 hover:bg-secondary-50 active:bg-secondary-500 active:text-white active:scale-95 transition-all shadow-lg"
                >
                  ↓
                </button>
              </div>

              {/* Down Controls */}
              <div className="flex items-center space-x-4">
                <div className="flex flex-col items-center space-y-2">
                  <span className="text-4xl text-secondary-600">↓</span>
                  <div className="flex space-x-2">
                    <button
                      onClick={() => handleTopHalf('down_plus')}
                      className="w-12 h-12 bg-secondary-500 text-white rounded-lg font-bold hover:bg-secondary-600 active:scale-95 transition-all shadow-lg"
                    >
                      +
                    </button>
                    <button
                      onClick={() => handleTopHalf('down_minus')}
                      className="w-12 h-12 bg-secondary-500 text-white rounded-lg font-bold hover:bg-secondary-600 active:scale-95 transition-all shadow-lg"
                    >
                      -
                    </button>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}

