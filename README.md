
# Description
Collection of packages, nodes and algorithms to perform perception tasks using LiDAR and cameras.

# Installation
1. Install dependencies with rosdep 
```bash
rosdep install --from-path src --ignore-src -y
```
2. Build the package using colcon
3. In using Oak D Pro, install udev rules with the script [./turtlebot3_perception/debian/udev/install_udev](./turtlebot3_perception/debian/udev/install_udev)
```bash 
sudo ./install_udev
```
# Usage
## Drivers
### Start Camera Driver

1. Set the environment variable CAMERA_MODEL to "realsense" or "oakd"
2. Launch the camera driver with 
```bash 
ros2 launch turtlebot3_perception camera.launch.py
```
3. Launch the AprilTag detection using
```bash 
ros2 launch turtlebot3_perception apriltag.launch.py
```
## Features extraction
### Lines from LaserScan
Run the example node with
```bash 
ros2 run turtlebot3_perception laserscan2lines
```

You can use the function directly in your nodes putting this import in your file
```python
from turtlebot3_perception.laserscan2lines import laserscan2lines
```

### Start AprilTag landmark detection

1. Launch the required nodes with
```bash 
ros2 launch turtlebot3_perception apriltag.launch.py
```
