# Drive System ROS

Version: 0.1.0

Code name: N/A

## Introduction

Interop between the ControlSystem and the DriveSystem.

## Dependencies

-   Python dependencies: pyserial

## Building

```bash
# Make sure you have the ROS2 environment sourced

# Build the packages
colcon build --symlink-install

# Source the overlay workspace (every time you open a new terminal)
source install/local_setup.bash
```

## Running

Make sure the microcontroller is connected and running.

To start the node, run `ros2 run drive_system drive_system`.
