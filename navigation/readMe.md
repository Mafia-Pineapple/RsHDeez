
## Prerequisites

- ROS2 Humble
- Ignition Gazebo Fortress
- Required ROS2 packages:
  - `rclcpp`
  - `geometry_msgs`
  - `nav_msgs`
  - `sensor_msgs`
  - `std_msgs`
  - `std_srvs`
  - `tf2`
  - `ros_ign_gazebo`
  - `ros_ign_bridge`

## Installation

1. Build the package:
```bash
colcon build --packages-select navigation
source install/setup.bash
```

## Quick Start

### 1. Launch the Simulation

```bash
ros2 launch navigation scout.launch.py
```

This will start:
- Ignition Gazebo with mountainous terrain
- Scout drone model
- All necessary bridges and sensor nodes
- Quadcopter controller node
- AGL parser node

### 1.5. Start Bridging for Cameras

```bash
ros2 run ros_gz_bridge parameter_bridge "/model/scout/thermal/image@sensor_msgs/msg/Image@gz.msgs.Image" "/model/scout/thermal/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
```


### 2. Enable Wandering Mode

In a new terminal:
```bash
source ~/41068_ws/install/setup.bash
ros2 service call /wander_mode std_srvs/srv/SetBool "data: true"
```

The drone will now autonomously explore, maintaining 1.5m above ground level.

### 3. Disable Wandering Mode

```bash
ros2 service call /wander_mode std_srvs/srv/SetBool "data: false"
```

## Usage

### Wandering Mode (Autonomous Exploration)

**Enable:**
```bash
ros2 service call /wander_mode std_srvs/srv/SetBool "data: true"
```

**Behavior:**
- Maintains 1.5m altitude above ground (terrain-following)
- Random forward speed: 0.3 to 1.2 m/s
- Random gentle turning: -0.3 to 0.3 rad/s
- Changes direction every 3-8 seconds
- Works on mountainous/varying terrain

**Disable:**
```bash
ros2 service call /wander_mode std_srvs/srv/SetBool "data: false"
```

### Goal-Based Navigation

**Set a goal:**
```bash
ros2 topic pub /drone/goal_stamped geometry_msgs/msg/PointStamped "{
  header: {frame_id: 'map'},
  point: {x: 10.0, y: 5.0, z: 570.0}
}"
```

**Start flying to goal:**
```bash
ros2 service call /reach_goal std_srvs/srv/SetBool "data: true"
```

**Stop/Land:**
```bash
ros2 service call /reach_goal std_srvs/srv/SetBool "data: false"
```

### Monitor Drone Status

**Check altitude above ground:**
```bash
ros2 topic echo /drone/agl_distance
```

**Check position:**
```bash
ros2 topic echo /odometry
```

**Check velocity commands:**
```bash
ros2 topic echo /cmd_vel
```

**Check all available topics:**
```bash
ros2 topic list
```

**Check all available services:**
```bash
ros2 service list
```

## Available Topics

### Subscribed Topics
- `/odometry` (nav_msgs/Odometry) - Drone position and velocity
- `/drone/agl_distance` (std_msgs/Float64) - Altitude above ground (filtered)
- `/drone/agl_raw` (std_msgs/Float64) - Raw AGL sensor data
- `/drone/goal_stamped` (geometry_msgs/PointStamped) - Target waypoint

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to drone
- `/drone/agl_distance` (std_msgs/Float64) - Filtered altitude above ground
- `/drone/agl_raw` (std_msgs/Float64) - Raw AGL readings

### Services
- `/wander_mode` (std_srvs/SetBool) - Enable/disable autonomous wandering
- `/reach_goal` (std_srvs/SetBool) - Start/stop goal-seeking behavior


### Calebs ReadMe --- Live Camera view

To make "it send the camera information" 
```bash 
ros2 launch realsense2_camera rs_launch.py
```

To view the gazebo camera:

```bash
ros2 run rqt_image_view rqt_image_view
```