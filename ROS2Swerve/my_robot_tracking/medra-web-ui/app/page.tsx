'use client';

import { useRouter } from 'next/navigation';
import Link from 'next/link';
import { useState } from 'react';

export default function LoginPage() {
  const router = useRouter();
  const [isNavigating, setIsNavigating] = useState(false);

  const handleLogin = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    if (isNavigating) return;
    setIsNavigating(true);
    console.log('Navigating to dashboard...');
    try {
      router.push('/dashboard');
    } catch (error) {
      console.error('Navigation error:', error);
      // Fallback to window.location
      window.location.href = '/dashboard';
    }
  };

  const handleRegister = (e: React.MouseEvent<HTMLButtonElement>) => {
    e.preventDefault();
    if (isNavigating) return;
    setIsNavigating(true);
    console.log('Navigating to dashboard...');
    try {
      router.push('/dashboard');
    } catch (error) {
      console.error('Navigation error:', error);
      // Fallback to window.location
      window.location.href = '/dashboard';
    }
  };

  return (
    <div className="min-h-screen flex items-center justify-center p-4">
      <div className="card max-w-md w-full text-center space-y-8">
        <div className="space-y-4">
          <h1 className="text-6xl font-bold text-primary-600 mb-8 tracking-wide">
            MedRa
          </h1>
          <p className="text-gray-600 text-lg">Medical Robot Control System</p>
        </div>
        
        <div className="space-y-4 pt-8">
          <button
            onClick={handleLogin}
            disabled={isNavigating}
            className="w-full text-lg bg-primary-500 hover:bg-primary-600 text-white font-semibold py-4 px-8 rounded-xl transition-all duration-200 transform hover:scale-105 active:scale-95 shadow-lg hover:shadow-xl disabled:opacity-50 disabled:cursor-not-allowed cursor-pointer"
          >
            {isNavigating ? 'Loading...' : 'Login'}
          </button>
          <button
            onClick={handleRegister}
            disabled={isNavigating}
            className="w-full text-lg bg-secondary-500 hover:bg-secondary-600 text-white font-semibold py-4 px-8 rounded-xl transition-all duration-200 transform hover:scale-105 active:scale-95 shadow-lg hover:shadow-xl disabled:opacity-50 disabled:cursor-not-allowed cursor-pointer"
          >
            {isNavigating ? 'Loading...' : 'Register'}
          </button>
        </div>
      </div>
    </div>
  );
}

