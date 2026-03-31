import './globals.css'

export const metadata = {
  title: 'M.A.R.S. - ROS 2 Rover Dashboard',
  description: 'Mobile Autonomous Rover System - Inventory Scanning & Logging',
}

export default function RootLayout({ children }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  )
}
