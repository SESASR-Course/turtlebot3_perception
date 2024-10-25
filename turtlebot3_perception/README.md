# Description

# Installation
1. Install dependencies with rosdep 
```bash
rosdep install --from-path src --ignore-src -y
```
2. Build the package

# Usage
1. Set the environment variable CAMERA_MODEL to "realsense" or "oakd"
2. Launch the camera driver with 
```bash 
ros2 launch turtlebot3_perception camera.launch.py
```
3. Launch the AprilTag detection using
```bash 
ros2 launch turtlebot3_perception apriltag.launch.py
```