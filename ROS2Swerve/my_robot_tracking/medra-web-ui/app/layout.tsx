import type { Metadata } from 'next'
import './globals.css'

export const metadata: Metadata = {
  title: 'MedRa - Medical Robot Control',
  description: 'Control interface for MedRa medical robot',
}

export default function RootLayout({
  children,
}: {
  children: React.ReactNode
}) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  )
}

