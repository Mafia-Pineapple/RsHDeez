# Autonomous Drone Navigation System

A complete ROS2 + Gazebo autonomous drone system with 3D SLAM, collision avoidance, and intelligent exploration capabilities for forest/terrain navigation.

---

## Table of Contents
- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage Modes](#usage-modes)
- [SLAM & Mapping](#slam--mapping)
- [Autonomous Exploration](#autonomous-exploration)
- [Navigation (Nav2)](#navigation-nav2)
- [Available Topics & Services](#available-topics--services)
- [Troubleshooting](#troubleshooting)

---

## Features

### Core Capabilities
- ✅ **3D SLAM** with RTAB-Map (LIDAR-based)
- ✅ **Autonomous exploration** with collision avoidance
- ✅ **Terrain following** using downward AGL sensor
- ✅ **Real-time 3D mapping**
- ✅ **Goal-based navigation** (Nav2 integration)
- ✅ **Multiple flight patterns** (spiral, figure-8, circle)
- ✅ **Obstacle avoidance** using 360° LIDAR
- ✅ **RGB-D camera** for visualization

### Sensor Suite
- 360° horizontal LIDAR (40m range)
- Downward-facing AGL sensor (altitude above ground)
- IMU (orientation and acceleration)
- RGB-D camera (optional, for visualization)
- Odometry with drift correction

---

## Prerequisites

### Required Software
- **Ubuntu 22.04**
- **ROS2 Humble**
- **Ignition Gazebo Fortress**

### Install ROS2 Humble
```bash
# Follow official ROS2 Humble installation
sudo apt install ros-humble-desktop
```

### Install Gazebo Fortress
```bash
sudo apt install ignition-fortress
```

### Required ROS2 Packages
```bash
sudo apt install \
  ros-humble-ros-ign-gazebo \
  ros-humble-ros-ign-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rtabmap-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup
```

---

## Installation

### 1. Clone the Repository
```bash
mkdir -p ~/41068_ws/src
cd ~/41068_ws/src
# Clone your navigation package here
```

### 2. Build the Workspace
```bash
cd ~/41068_ws
colcon build --packages-select navigation
source install/setup.bash
```

### 3. Verify Installation
```bash
# Check package is installed
ros2 pkg list | grep navigation

# Should show: navigation
```

---

## Quick Start

### Complete System Launch (3 Terminals)

**Terminal 1: Launch Simulation & Drone**
```bash
cd ~/41068_ws
source install/setup.bash
ros2 launch navigation scout_launch.py
```
*This starts Gazebo with terrain, spawns the drone, and initializes all sensors.*

**Terminal 2: Launch RTAB-Map SLAM**
```bash
cd ~/41068_ws
source install/setup.bash
ros2 launch navigation rtabmap_launch.py
```
*This starts 3D SLAM mapping with LIDAR. The RTAB-Map viewer window will open showing the map building in real-time.*

**Terminal 3: Launch Autonomous Explorer**
```bash
cd ~/41068_ws
source install/setup.bash

# 2-minute exploration
ros2 run navigation smooth_explorer_node --ros-args \
  -p exploration_time:=120.0 \
  -p cruise_speed:=0.8 \
  -p survey_pattern:=spiral
```
*The drone will autonomously explore, building a 3D map while avoiding obstacles.*

---

## Usage Modes

### 1. Manual Goal Navigation

Set a specific waypoint:
```bash
ros2 topic pub --once /drone/goal_stamped geometry_msgs/msg/PointStamped "{
  header: {frame_id: 'map'},
  point: {x: 10.0, y: 5.0, z: 5.0}
}"
```

Enable autonomous flight to goal:
```bash
ros2 service call /reach_goal std_srvs/srv/SetBool "{data: true}"
```

Stop/hover:
```bash
ros2 service call /reach_goal std_srvs/srv/SetBool "{data: false}"
```

### 2. Autonomous Exploration

Explore for 3 minutes with spiral pattern:
```bash
ros2 run navigation smooth_explorer_node --ros-args \
  -p exploration_time:=180.0 \
  -p cruise_speed:=1.0 \
  -p cruise_height:=5.0 \
  -p exploration_radius:=25.0 \
  -p survey_pattern:=spiral
```

**Parameters:**
- `exploration_time`: Duration in seconds (default: 180)
- `cruise_speed`: Flight speed in m/s (default: 0.8)
- `cruise_height`: Flight altitude in meters (default: 5.0)
- `exploration_radius`: Max distance from start (default: 25.0)
- `survey_pattern`: `spiral`, `figure8`, or `circle`
- `obstacle_distance`: Emergency stop distance (default: 3.0m)
- `safe_distance`: Slow-down distance (default: 5.0m)

**Available Patterns:**

**Spiral** (recommended for systematic coverage):
```bash
-p survey_pattern:=spiral
```

**Figure-8** (good for loop closure):
```bash
-p survey_pattern:=figure8
```

**Circle** (expanding circular pattern):
```bash
-p survey_pattern:=circle
```

### 3. Wandering Mode (Legacy)

Random exploration with terrain following:
```bash
ros2 service call /wander_mode std_srvs/srv/SetBool "data: true"
```

Behavior:
- Maintains 1.5m above ground
- Random speed: 0.3-1.2 m/s
- Random turns: -0.3 to 0.3 rad/s
- Changes direction every 3-8 seconds

Disable:
```bash
ros2 service call /wander_mode std_srvs/srv/SetBool "data: false"
```

---

## SLAM & Mapping

### Starting RTAB-Map

RTAB-Map provides 3D SLAM using LIDAR data:
```bash
ros2 launch navigation rtabmap_launch.py
```

**What you'll see:**
- RTAB-Map viewer window opens
- 3D point cloud builds up as drone flies
- Blue graph nodes show drone trajectory
- Green lines indicate loop closures

### Monitoring SLAM Performance

Check map is building:
```bash
ros2 topic hz /rtabmap/mapData
# Should show ~1 Hz

ros2 topic echo /rtabmap/grid_map --once
# Should show occupancy grid data
```

Check localization:
```bash
ros2 topic echo /rtabmap/localization_pose
# Shows drone's position in map frame
```

View TF tree:
```bash
ros2 run tf2_tools view_frames
# Creates frames.pdf showing: map → odom → base_link
```

### Saving Maps

Save current map:
```bash
ros2 service call /rtabmap/set_mode_mapping rtabmap_ros/srv/SetGoal "{}"
```

The map database is automatically saved to: `~/.ros/rtabmap.db`

---

## Autonomous Exploration

### Conservative (Dense Forest)
```bash
ros2 run navigation smooth_explorer_node --ros-args \
  -p cruise_speed:=0.5 \
  -p obstacle_distance:=4.0 \
  -p safe_distance:=7.0 \
  -p avoidance_gain:=2.0 \
  -p exploration_time:=180.0
```

### Balanced (Mixed Terrain)
```bash
ros2 run navigation smooth_explorer_node --ros-args \
  -p cruise_speed:=0.8 \
  -p obstacle_distance:=3.0 \
  -p safe_distance:=5.0 \
  -p avoidance_gain:=1.5 \
  -p exploration_time:=180.0
```

### Aggressive (Open Areas)
```bash
ros2 run navigation smooth_explorer_node --ros-args \
  -p cruise_speed:=1.2 \
  -p obstacle_distance:=2.0 \
  -p safe_distance:=4.0 \
  -p avoidance_gain:=1.0 \
  -p exploration_time:=120.0
```

### Collision Avoidance Features

The explorer includes:
- **8-sector LIDAR analysis** (360° awareness)
- **3-tier response system:**
  - Critical (<3m): Emergency stop + escape
  - Warning (3-5m): Slow down + adjust
  - Safe (>5m): Normal operation
- **Smart direction selection** (turns toward open space)
- **Lateral avoidance** (uses sideways movement)
- **Automatic return home** after exploration

---

## Navigation (Nav2)

### Setup Nav2 (One-time)

1. Ensure SLAM is running and map is built
2. Launch Nav2:
```bash
ros2 launch navigation nav2_launch.py
```

### Sending Navigation Goals

**Via Command Line:**
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 10.0, y: 5.0, z: 5.0},
    orientation: {w: 1.0}
  }
}"
```

**Via RViz:**
1. Open RViz: `ros2 run rviz2 rviz2`
2. Set Fixed Frame to `map`
3. Add displays: Map, TF, LaserScan, Path
4. Use "2D Goal Pose" tool to click destination

### Nav2 Features

- **Global path planning** using SMAC Planner
- **Local obstacle avoidance** with MPPI controller
- **Dynamic replanning** on obstacle detection
- **Costmap layers** for safe navigation
- **Recovery behaviors** (spin, backup, wait)

---

## Available Topics & Services

### Key Topics

**Subscribed:**
- `/odometry` - Drone position/velocity (nav_msgs/Odometry)
- `/scan` - LIDAR data (sensor_msgs/LaserScan)
- `/drone/agl_distance` - Altitude above ground (std_msgs/Float64)
- `/imu` - IMU data (sensor_msgs/Imu)
- `/camera/image` - RGB image (sensor_msgs/Image)
- `/camera/depth/image` - Depth image (sensor_msgs/Image)

**Published:**
- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/rtabmap/grid_map` - 3D occupancy grid (nav_msgs/OccupancyGrid)
- `/rtabmap/cloud_map` - Point cloud map (sensor_msgs/PointCloud2)
- `/rtabmap/localization_pose` - Localization (geometry_msgs/PoseStamped)
- `/map` - 2D map projection (nav_msgs/OccupancyGrid)

### Services

- `/reach_goal` - Enable goal-seeking (std_srvs/SetBool)
- `/wander_mode` - Enable wandering (std_srvs/SetBool)
- `/rtabmap/reset` - Reset SLAM map
- `/rtabmap/pause` - Pause mapping
- `/rtabmap/resume` - Resume mapping

### TF Frames
```
map (global reference)
 └─ odom (odometry frame)
     └─ base_link (drone body)
         ├─ base_scan (LIDAR)
         ├─ imu_link (IMU)
         └─ camera_link (camera)
```

---

## Monitoring & Debugging

### Check System Status

**SLAM Health:**
```bash
ros2 topic hz /rtabmap/mapData        # Should be ~1 Hz
ros2 topic hz /rtabmap/grid_map       # Should be ~1 Hz
ros2 run tf2_ros tf2_echo map odom    # Should show updating transform
```

**Sensor Health:**
```bash
ros2 topic hz /scan        # Should be ~3 Hz
ros2 topic hz /odometry    # Should be ~20 Hz
ros2 topic hz /imu         # Should be ~100 Hz
```

**Drone Position:**
```bash
ros2 topic echo /odometry | grep -A 3 "position:"
```

**Altitude Above Ground:**
```bash
ros2 topic echo /drone/agl_distance
```

**Closest Obstacle:**
```bash
ros2 topic echo /scan | head -n 20
```

### Visualization

**Launch RViz:**
```bash
ros2 run rviz2 rviz2
```

**Recommended displays:**
- Map (`/map`)
- TF (all frames)
- LaserScan (`/scan`)
- Path (`/plan` - global path)
- Path (`/local_plan` - local path)
- Costmap (`/global_costmap/costmap`)
- Costmap (`/local_costmap/costmap`)
- PointCloud2 (`/rtabmap/cloud_map`)
- RobotModel (URDF visualization)

Set **Fixed Frame** to `map`.

**View Camera Feed:**
```bash
ros2 run rqt_image_view rqt_image_view
```
Select topic: `/camera/image` or `/camera/depth/image`

---

## Troubleshooting

### Drone Won't Take Off

**Check odometry is publishing:**
```bash
ros2 topic hz /odometry
# Should be ~20 Hz
```

**Check controller is running:**
```bash
ros2 node list | grep quadcopter
# Should show: /quadcopter_controller
```

### SLAM Not Building Map

**Check RTAB-Map is running:**
```bash
ros2 node list | grep rtabmap
# Should show: /rtabmap
```

**Check LIDAR is publishing:**
```bash
ros2 topic hz /scan
# Should be ~3 Hz
```

**Check TF tree:**
```bash
ros2 run tf2_ros tf2_echo map base_link
# Should show transform
```

### Collision Avoidance Not Working

**Check LIDAR data:**
```bash
ros2 topic echo /scan --once
# Should show ranges array with values
```

**Verify obstacle detection:**
```bash
# Run explorer with debug output
ros2 run navigation smooth_explorer_node --ros-args --log-level debug
```

### Drone Flies Away

**Issue:** Explorer starts before odometry is ready

**Solution:** Wait 5 seconds after launching before starting explorer:
```bash
ros2 launch navigation scout_launch.py
sleep 5
ros2 run navigation smooth_explorer_node
```

### Nav2 Can't Plan Path

**Check map exists:**
```bash
ros2 topic echo /map --once
# Should show occupancy grid
```

**Check localization:**
```bash
ros2 topic echo /rtabmap/localization_pose
# Should show current pose
```

**Check costmaps:**
```bash
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```

---

## Advanced Configuration

### Adjusting SLAM Parameters

Edit `launch/rtabmap_launch.py`:
```python
parameters=[{
    'RGBD/LinearUpdate': '0.1',    # Update every 10cm
    'RGBD/AngularUpdate': '0.1',   # Update every 6 degrees
    'Icp/VoxelSize': '0.1',        # ICP resolution
    'Grid/RangeMax': '15.0',       # Max LIDAR range to use
    # ... more parameters
}]
```

### Adjusting Explorer Behavior

Edit `src/smooth_explorer.cpp`:
```cpp
// Line ~15: Adjust safety distances
this->declare_parameter("obstacle_distance", 3.0);  // Stop distance
this->declare_parameter("safe_distance", 5.0);      // Slow down distance

// Line ~250: Adjust spiral expansion rate
spiral_angle_ += 0.6;   // Rotation per goal
spiral_radius_ += 1.0;  // Expansion per goal
```

### Custom Nav2 Costmaps

Edit `config/nav2_params.yaml`:
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      width: 10              # Local costmap size (meters)
      resolution: 0.1        # Cell size (meters)
      robot_radius: 0.5      # Drone safety radius
      inflation_radius: 1.0  # Obstacle inflation
```

---

## System Architecture
```
┌─────────────────────────────────────────────┐
│           Gazebo Simulation                 │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐ │
│  │  Terrain │  │   Drone  │  │ Sensors  │ │
│  └──────────┘  └──────────┘  └──────────┘ │
└─────────────────┬───────────────────────────┘
                  │ ROS-IGN Bridge
┌─────────────────┴───────────────────────────┐
│              ROS2 Layer                     │
│  ┌─────────────┐  ┌──────────────────────┐ │
│  │ RTAB-Map    │  │  Smooth Explorer     │ │
│  │ (SLAM)      │  │  (Collision Avoid)   │ │
│  └─────────────┘  └──────────────────────┘ │
│  ┌─────────────┐  ┌──────────────────────┐ │
│  │ Nav2        │  │  Quadcopter Control  │ │
│  │ (Planning)  │  │  (Low-level)         │ │
│  └─────────────┘  └──────────────────────┘ │
└─────────────────────────────────────────────┘
```

---

## Performance Tips

1. **For better SLAM:** Fly slowly (0.5-0.8 m/s) with smooth turns
2. **For faster exploration:** Use spiral pattern with higher speed
3. **In dense forests:** Reduce obstacle_distance to 2.5m, increase avoidance_gain
4. **For loop closure:** Use figure-8 pattern to revisit areas
5. **Save CPU:** Disable RTAB-Map visualization if not needed

---

## Credits & License

Developed for autonomous drone navigation research.

**Key Technologies:**
- ROS2 Humble
- Ignition Gazebo Fortress
- RTAB-Map
- Nav2

**Contributors:**
- [Your name/team]

**License:** MIT

---

## Contact & Support

For issues, questions, or contributions:
- GitHub Issues: [Your repo]
- Documentation: [Your docs]
- Email: [Your email]

---

**Happy Flying! 🚁🗺️**