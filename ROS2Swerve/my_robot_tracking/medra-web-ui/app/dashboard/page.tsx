'use client';

import { useRouter } from 'next/navigation';

export default function DashboardPage() {
  const router = useRouter();
  const doctorName = 'Dr. Smith'; // TODO: Get from auth

  return (
    <div className="min-h-screen p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <div className="flex justify-end mb-8">
          <div className="bg-white/20 backdrop-blur-md rounded-full px-6 py-3 shadow-lg">
            <p className="text-white text-lg font-semibold">{doctorName}</p>
          </div>
        </div>

        {/* Main Content */}
        <div className="grid grid-cols-1 md:grid-cols-2 gap-8 max-w-4xl mx-auto mt-20">
          {/* Manual Mode Button */}
          <button
            onClick={() => router.push('/manual')}
            className="group relative h-96 bg-white rounded-3xl shadow-2xl hover:shadow-3xl transition-all duration-300 transform hover:scale-105 hover:-translate-y-2 overflow-hidden"
          >
            <div className="absolute inset-0 bg-gradient-to-br from-primary-500 to-primary-700 opacity-0 group-hover:opacity-10 transition-opacity duration-300"></div>
            <div className="flex flex-col items-center justify-center h-full space-y-6">
              <div className="text-7xl mb-4">🎮</div>
              <h2 className="text-4xl font-bold text-gray-800">Manual Mode</h2>
              <p className="text-gray-600 text-lg">Direct robot control</p>
            </div>
          </button>

          {/* Camera Button */}
          <button
            onClick={() => router.push('/camera')}
            className="group relative h-96 bg-white rounded-3xl shadow-2xl hover:shadow-3xl transition-all duration-300 transform hover:scale-105 hover:-translate-y-2 overflow-hidden"
          >
            <div className="absolute inset-0 bg-gradient-to-br from-secondary-500 to-secondary-700 opacity-0 group-hover:opacity-10 transition-opacity duration-300"></div>
            <div className="flex flex-col items-center justify-center h-full space-y-6">
              <div className="text-7xl mb-4">📷</div>
              <h2 className="text-4xl font-bold text-gray-800">Camera</h2>
              <p className="text-gray-600 text-lg">Live tracking view</p>
            </div>
          </button>
        </div>
      </div>
    </div>
  );
}

