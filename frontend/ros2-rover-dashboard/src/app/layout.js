import './globals.css'

export const metadata = {
  title: 'ROS 2 Rover Dashboard',
  description: 'Inventory Scanning & Logging System',
}

export default function RootLayout({ children }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  )
}
