# MedRa Web UI - Next.js Version

A modern, clean web interface for controlling the MedRa medical robot, built with Next.js, TypeScript, and Tailwind CSS.

## Features

- 🎨 **Modern UI**: Clean, professional design with smooth animations
- 📱 **Responsive**: Works on desktop, tablet, and mobile devices
- ⚡ **Fast**: Built with Next.js for optimal performance
- 🔄 **Real-time**: Live camera feed and status updates
- 🎮 **Intuitive Controls**: Large, touch-friendly buttons for robot control

## Tech Stack

- **Next.js 14** - React framework
- **TypeScript** - Type safety
- **Tailwind CSS** - Modern styling
- **Axios** - API client

## Setup

### Prerequisites

- Node.js 18+ and npm
- Flask backend running on port 8080 (or configure `NEXT_PUBLIC_API_URL`)

### Installation

1. **Install dependencies:**
   ```bash
   cd medra-web-ui
   npm install
   ```

2. **Configure API URL (optional):**
   Create a `.env.local` file:
   ```env
   NEXT_PUBLIC_API_URL=http://localhost:8080
   ```
   Or set it when running (defaults to `http://localhost:8080`)

3. **Run development server:**
   ```bash
   npm run dev
   ```

4. **Open browser:**
   Navigate to `http://localhost:3000`

## Building for Production

```bash
npm run build
npm start
```

## Project Structure

```
medra-web-ui/
├── app/
│   ├── page.tsx          # Login page
│   ├── dashboard/        # Dashboard page
│   ├── manual/           # Manual control page
│   ├── camera/           # Camera/tracking page
│   ├── layout.tsx        # Root layout
│   └── globals.css       # Global styles
├── components/           # Reusable components (future)
├── lib/
│   └── api.ts           # API client for Flask backend
└── public/              # Static assets
```

## Pages

### Login Page (`/`)
- MedRa branding
- Login and Register buttons (auth not implemented yet)

### Dashboard (`/dashboard`)
- Main menu with Manual Mode and Camera buttons
- Doctor name display in header

### Manual Control (`/manual`)
- Arrow buttons for robot movement (↑↓←→)
- Top half controls with +/- buttons
- Touch-friendly large buttons

### Camera View (`/camera`)
- Live camera feed from ROS2
- Tracking status indicator
- Click to select tracking target

## API Integration

The frontend connects to the Flask backend at `http://localhost:8080` (or `NEXT_PUBLIC_API_URL`).

**API Endpoints:**
- `GET /api/status` - Get robot status
- `POST /api/move` - Move robot
- `POST /api/stop` - Stop robot
- `POST /api/steer` - Steer robot
- `POST /api/top_half` - Control top half
- `GET /api/camera_frame` - Get camera frame

## Development

### Running in Development Mode

```bash
npm run dev
```

### Building

```bash
npm run build
npm start
```

### Linting

```bash
npm run lint
```

## Customization

### Colors

Edit `tailwind.config.ts` to customize colors:
- Primary colors: Used for main actions
- Secondary colors: Used for secondary actions

### Styling

- Global styles: `app/globals.css`
- Component styles: Use Tailwind classes or create custom components

## Troubleshooting

### API Connection Issues

- Make sure Flask backend is running on port 8080
- Check `NEXT_PUBLIC_API_URL` environment variable
- Check browser console for CORS errors

### Camera Feed Not Loading

- Ensure camera_node is running in ROS2
- Check Flask backend is receiving camera frames
- Check browser console for errors

## Future Enhancements

- [ ] Supabase authentication
- [ ] Real-time tracking visualization
- [ ] Settings/preferences page
- [ ] Battery status and telemetry
- [ ] Error handling and retry logic
- [ ] WebSocket for real-time updates

