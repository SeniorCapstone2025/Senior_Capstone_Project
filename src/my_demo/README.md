# My Demo Package

A simple ROS 2 demo package for learning MentorPi development workflow.

## What's Inside

This package contains:
- **publisher_node.py**: Publishes a "Hello from MentorPi!" message with a counter every second to the `demo_topic`
- **subscriber_node.py**: Subscribes to `demo_topic` and logs received messages
- **demo.launch.py**: Launch file that starts both nodes together

## Building the Package

From your workspace root (C:\╘┤┬δ):

```bash
# Build only this package
colcon build --packages-select my_demo

# Or build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Running the Demo

### Option 1: Using the Launch File (Recommended)

```bash
ros2 launch my_demo demo.launch.py
```

This will start both the publisher and subscriber together. You should see output like:
```
[simple_publisher]: Publishing: "Hello from MentorPi! Count: 0"
[simple_subscriber]: Received: "Hello from MentorPi! Count: 0"
[simple_publisher]: Publishing: "Hello from MentorPi! Count: 1"
[simple_subscriber]: Received: "Hello from MentorPi! Count: 1"
...
```

Press `Ctrl+C` to stop.

### Option 2: Running Nodes Individually

In one terminal:
```bash
ros2 run my_demo publisher_node
```

In another terminal:
```bash
ros2 run my_demo subscriber_node
```

## Inspecting Topics

While the nodes are running, open a new terminal and try:

```bash
# List active topics
ros2 topic list

# See messages being published
ros2 topic echo /demo_topic

# See topic info
ros2 topic info /demo_topic
```

## Package Structure

```
my_demo/
├── my_demo/                    # Python module
│   ├── __init__.py
│   ├── publisher_node.py       # Publisher node
│   └── subscriber_node.py      # Subscriber node
├── launch/                     # Launch files
│   └── demo.launch.py
├── resource/                   # Resource marker
│   └── my_demo
├── package.xml                 # Package manifest
├── setup.py                    # Python package setup
├── setup.cfg                   # Install configuration
└── README.md                   # This file
```

## Next Steps

Use this package as a template for your own development:
1. Copy the structure to create new packages
2. Add your own nodes in the `my_demo/` directory
3. Register them in `setup.py` entry_points
4. Add dependencies to `package.xml`
5. Create launch files in `launch/`
6. Build and run!
