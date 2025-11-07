# Autonomous Drone Grid Explorer

A ROS2-based autonomous exploration system for drones that systematically maps terrain while avoiding obstacles. Built for educational purposes and tested in Gazebo simulation.

---

## What Does This Do?

This system makes a drone autonomously explore an area by flying to waypoints in a grid pattern. Think of it like a lawn mower pattern, but smarter - it can avoid obstacles, detect when it's stuck, and build a 3D map of the environment as it goes.

The drone uses a 360° LIDAR sensor to detect obstacles and avoid them in real-time. If it can't reach a particular grid point (maybe there's a tree in the way), it marks it as unreachable and moves on. Once it's visited all the points it can, it automatically flies back home and lands.

While exploring, RTAB-Map runs in the background building a 3D point cloud map from the LIDAR data. You get a nice visualization window showing the map being built in real-time.

---

## How It Works (High Level)

### The Grid Pattern
When the drone starts, it generates a square spiral pattern of waypoints centered on its starting position. For example, with default settings you get a 5x5 grid (25 points) spaced 20 meters apart. The drone visits them in order: center → right → up → left → left → down → down, spiraling outward.

### Navigation
For each grid point:
1. Calculate the direction and distance to the target
2. Command the drone to fly that direction at cruise speed (1.5 m/s)
3. Use the downward laser sensor to maintain 15 meters above ground
4. Check LIDAR every 50ms (20Hz loop) for obstacles

### Collision Avoidance
This is where it gets interesting. The LIDAR gives us 360 rays in a circle. I divide them into 8 sectors (front, front-left, left, back-left, etc.) and keep track of the closest obstacle in each direction.

There are three zones:
- **Critical (<1.5m)**: Emergency stop! Kill forward motion, turn toward open space, back up if necessary
- **Warning (1.5-3m)**: Slow down proportionally, start turning away from obstacles, slide sideways
- **Safe (>3m)**: Full speed ahead, no worries

The cool part is that drones can move sideways, so when there's an obstacle on the left, it doesn't just turn - it actively slides right while still heading toward the goal. Makes for much smoother navigation.

### Getting Stuck
Sometimes the drone can't make progress - maybe the path is blocked, or it's navigating around a large obstacle. Every frame (50ms), the code checks:
- Are we moving at all? (distance > 10cm)
- Are we getting closer to the goal? (at least 20cm closer)

If neither is true for 12 seconds (240 frames), it marks that grid point as unreachable and moves to the next one. Before giving up, if there's an obstacle directly in front, it tries climbing 5 meters higher to go over it. Sometimes this works, sometimes it doesn't.

### SLAM Integration
RTAB-Map runs in parallel doing its own thing. It takes the same LIDAR data and builds a 3D map, does loop closure detection (recognizes when you revisit an area), and publishes the map→odom transform to correct drift.

However, the grid explorer doesn't actually use the SLAM-corrected position - it just uses raw odometry relative to the start point. This works fine in simulation where odometry is perfect, but on a real robot you'd want to look up the map→base_link transform instead. That's a TODO for future work.

---

## Getting Started

### Prerequisites
You need ROS2 Humble and Gazebo Fortress installed. If you don't have them:
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Gazebo Fortress
sudo apt install ignition-fortress

# Required packages
sudo apt install ros-humble-ros-ign-gazebo \
                 ros-humble-ros-ign-bridge \
                 ros-humble-robot-state-publisher \
                 ros-humble-rtabmap-ros \
                 ros-humble-slam-toolbox
```

### Build It
```bash
# Create workspace (or use existing)
mkdir -p ~/41068_ws/src
cd ~/41068_ws/src

# Clone this repo (or however you got the code here)
# ...

# Build
cd ~/41068_ws
colcon build --packages-select navigation
source install/setup.bash
```

---

## Running the System

You need two terminals minimum. Three if you want to monitor things.

### Terminal 1: Launch Simulation
This starts Gazebo with the terrain, spawns the drone, launches all the sensor nodes, and starts RTAB-Map SLAM.

```bash
cd ~/41068_ws
source install/setup.bash
ros2 launch navigation scout_launch.py
```

Wait for everything to fully load. You should see:
- Gazebo window with terrain and drone
- RTAB-Map visualization window (might take a few seconds)
- A bunch of ROS nodes starting up in the terminal

Give it a good 5-10 seconds before launching the explorer. If you launch too early, the odometry might not be ready and the drone will think it's at the wrong position.

### Terminal 2: Start Exploring
```bash
cd ~/41068_ws
source install/setup.bash
ros2 run navigation grid_explorer_node
```

The drone should:
1. Take off to 15m altitude
2. Start flying to grid points in a spiral pattern
3. Print progress messages like "Visited grid [1, 0] | Progress: 5/25 (20%)"
4. Slow down and navigate around any obstacles (trees, rocks)
5. Eventually return home and land

The default grid is 5x5 (25 points) with 20m spacing, covering about 100m x 100m. Takes roughly 4-6 minutes depending on obstacles.

---

## Configuration & Tuning

You can pass parameters when launching to customize behavior:

### Grid Size
```bash
# Smaller, faster exploration
ros2 run navigation grid_explorer_node --ros-args \
  -p grid_spacing:=15.0 \
  -p grid_radius:=1

# Larger area
ros2 run navigation grid_explorer_node --ros-args \
  -p grid_spacing:=25.0 \
  -p grid_radius:=3
```

**grid_radius**: Number of cells in each direction from center
- `radius=1` → 3x3 grid (9 cells)
- `radius=2` → 5x5 grid (25 cells) ← default
- `radius=3` → 7x7 grid (49 cells)

**grid_spacing**: Meters between grid points (default: 20m)

### Speed & Aggression
```bash
# Conservative (dense forest)
ros2 run navigation grid_explorer_node --ros-args \
  -p cruise_speed:=1.0 \
  -p obstacle_distance:=4.0

# Aggressive (open terrain)
ros2 run navigation grid_explorer_node --ros-args \
  -p cruise_speed:=2.0 \
  -p obstacle_distance:=2.0
```

**cruise_speed**: Flight speed in m/s (default: 1.5)
**obstacle_distance**: When to start avoiding in meters (default: 3.0)

### Altitude
```bash
# Fly lower (more detail but riskier)
ros2 run navigation grid_explorer_node --ros-args \
  -p target_agl:=10.0

# Fly higher (safer but less detail)
ros2 run navigation grid_explorer_node --ros-args \
  -p target_agl:=20.0
```

**target_agl**: Altitude above ground in meters (default: 15.0)

---

## Monitoring & Debugging

### Check if it's working
```bash
# Is odometry publishing?
ros2 topic hz /odometry
# Should show ~20 Hz

# Is LIDAR working?
ros2 topic hz /scan
# Should show ~3 Hz

# Current position
ros2 topic echo /odometry | grep -A 3 "position:"

# Current altitude above ground
ros2 topic echo /drone/agl_distance

# Closest obstacle distance (watch for approaching obstacles)
ros2 topic echo /scan | head -n 20
```

### Common Issues

**Drone doesn't take off:**
- Make sure odometry is publishing (`ros2 topic hz /odometry`)
- Wait longer after launching scout_launch.py (need 5-10 seconds)
- Check that no errors in Terminal 1

**Drone flies away immediately:**
- You probably launched the explorer before odometry was ready
- Kill everything (Ctrl+C both terminals)
- Restart scout_launch.py, wait 10 seconds, then launch explorer

**RTAB-Map window doesn't appear:**
- It takes a few seconds to start
- Check if rtabmap node is running: `ros2 node list | grep rtabmap`
- If not found, check the launch file includes rtabmap_launch

**Explorer says "unreachable" for every cell:**
- Probably obstacle_distance is too large or terrain is very cluttered
- Try increasing stuck timeout: `-p blocked_timeout:=60.0`
- Or increase obstacle stop distance: `-p obstacle_distance:=2.0`

**Drone gets stuck circling an obstacle:**
- The stuck detection should catch this after 12 seconds
- If not, there's a bug - kill it and try again with different parameters

---

## Code Structure

Here's what's actually running and where the code lives:

### Main Algorithm: grid_explorer.cpp
This is the brains of the operation. It's a ROS2 node that:
- Subscribes to `/odometry`, `/scan`, `/drone/agl_distance`
- Publishes velocity commands to `/cmd_vel`
- Runs at 20Hz (50ms loop)

Key functions:
- `generateSquareSpiral()`: Creates the grid waypoints
- `controlLoop()`: Main decision loop (goal selection, navigation, stuck detection)
- `applyCollisionAvoidance()`: The 3-tier obstacle avoidance logic
- `returnHome()`: Navigate back to start and land

### Supporting Nodes

**odometry_offset_node** (src/utils/odometry_offset.cpp):
- Takes raw odometry from Gazebo (which is in world coordinates like -183, -7938, 574)
- Zeros it to (0, 0, 0) at spawn point
- Publishes the offset odometry to `/odometry`
- Also publishes the odom→base_link TF transform

**agl_parser** (src/utils/agl_parser.cpp):
- Takes the downward laser sensor data
- Applies median filtering to reduce noise
- Publishes clean altitude-above-ground to `/drone/agl_distance`
- Falls back to odometry Z-coordinate if sensor fails

**quadcopter_node** (src/main.cpp + src/core/quadcopter.cpp):
- Low-level flight controller (currently disabled to avoid conflict)
- Provides services like `/reach_goal` and `/wander_mode`
- If you want to use it, you'd modify grid_explorer to publish goals instead of cmd_vel

### Launch Files

**scout_launch.py**:
- Launches Gazebo with terrain model
- Spawns the drone at specified coordinates
- Starts ROS-Ignition bridge (maps Gazebo topics to ROS)
- Launches robot_state_publisher (publishes TF tree from URDF)
- Starts odometry_offset_node and agl_parser
- Includes rtabmap_launch.py for SLAM
- Also launches slam_toolbox (2D SLAM backup)

**rtabmap_launch.py**:
- Configures and launches RTAB-Map in LIDAR-only mode
- Opens the visualization window
- Subscribes to `/scan` and `/imu`
- Publishes `/map` and map→odom transform

### Config Files

**gazebo_bridge.yaml**: Maps Gazebo topics to ROS topics
- Things like `/scan`, `/imu`, `/cmd_vel`, `/odometry_raw`

**slam_toolbox.yaml**: Configuration for SLAM Toolbox (2D SLAM)

---

## Results & Performance

In testing on various simulated terrains:

**Open terrain** (no obstacles):
- Coverage: 100% (25/25 cells)
- Time: ~3-4 minutes
- Collisions: 0

**Sparse forest** (10-15 trees):
- Coverage: 92-96% (23-24/25 cells)
- Time: ~4-5 minutes
- Collisions: 0
- Typical unreachable: 1-2 cells in dense clusters

**Dense forest** (30-40 trees):
- Coverage: 72-80% (18-20/25 cells)
- Time: ~5-7 minutes
- Collisions: 0
- Multiple unreachable cells where trees block access

The stuck detection works well - I haven't seen it get permanently stuck in an infinite loop. Worst case, it marks a point unreachable after 12 seconds and moves on.

The climb-over maneuver (rise 5m to clear obstacles) works about 30% of the time. Often the obstacle is too large or the path is still blocked at higher altitude. But when it works, it's pretty cool to watch.

---

## Known Limitations

**Not using SLAM correction**: 
The grid explorer uses raw odometry in the `odom` frame, not the SLAM-corrected `map` frame. In simulation this is fine (perfect odometry), but on a real robot it would drift. Should probably add TF lookups to use the corrected pose.

**Fixed grid pattern**: 
Once it generates the spiral, that's it - no replanning. If it discovers the center is blocked, it still tries to visit those points even though it could be smarter about skipping them.

**No global path planning**: 
It just flies directly toward each waypoint. Doesn't use the SLAM map to plan paths around known obstacles. This means it might repeatedly hit the same obstacle on different attempts.

**Memory not cleared**: 
The visited/unreachable sets never get cleared. For very long missions this could use a lot of memory. (Though realistically, even 10,000 cells is only ~80 KB so it's not a real problem.)

**Timeout is per-cell**: 
If you have 5 unreachable cells, you waste 12 seconds on each one (60 seconds total). Could be smarter about detecting patterns and skipping similar cells.

**No battery model**: 
In simulation, infinite battery. Real drone would need return-to-home triggered by low battery.

---

## Future Work

Some ideas if I come back to this:

1. **Use SLAM-corrected pose**: Modify grid_explorer to look up map→base_link transform
2. **Dynamic grid**: Add/remove grid points based on discovered obstacles
3. **Global planner integration**: Use Nav2 with the SLAM map for path planning
4. **Frontier-based exploration**: Instead of fixed grid, explore toward unknown areas
5. **Multi-altitude**: Explore at different heights for better 3D coverage
6. **Real robot testing**: Port to actual drone hardware (lots of tuning needed)

---

## Dependencies & Tech Stack

- **ROS2 Humble**: Main framework
- **Ignition Gazebo Fortress**: Physics simulation
- **RTAB-Map**: 3D SLAM and mapping
- **SLAM Toolbox**: 2D SLAM (backup)
- **C++17**: Main code
- **Python 3**: Launch files

All tested on Ubuntu 22.04.

---


## Acknowledgments

This was built for 41068 Robotics Studio 1. The basic URDF setup came from course materials. Everything else (grid explorer, collision avoidance logic, SLAM integration) is original.

Tested exclusively in simulation. Would need significant work for real hardware.

AI was used in debugging and understanding of coding objectives. Further This readMe file was generated using AI. All AI content was thoroughly reviewed before implementation for accuracy and relevance.

