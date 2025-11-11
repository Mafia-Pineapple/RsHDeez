# CBOOS - Central Base of Operational Surveying

##  Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
  - [Basic Operation](#basic-operation)
  - [Configuration Parameters](#configuration-parameters)
  - [GUI Controls](#gui-controls)
- [Technical Details](#technical-details)
- [Results & Performance](#results--performance)
- [Known Issues](#known-issues)
- [Future Work](#future-work)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

---

##  Overview

**CBOOS (Central Base of Operational Surveying)** addresses the challenge of monitoring wildlife in large and remote environments where manual surveying is time-consuming and costly. The system combines an autonomous aerial survey drone ("Scout") with a ground homebase station for sustainable operations.

### Problem Statement

Field teams in wildlife conservation, infrastructure monitoring, and disaster response face:
- Time-consuming manual surveys in dangerous or inaccessible areas
- Limited real-time situational awareness
- High operational costs
- Risks to human safety

### Solution

CBOOS provides an integrated robotic surveying system that:
- Autonomously navigates and maps unmapped terrain
- Detects and identifies animals using thermal, RGB, and depth cameras
- Avoids obstacles in real-time using LiDAR
- Provides live operator feedback through an intuitive GUI
- Returns to homebase for charging and data transfer

### Target Stakeholder

**Senior Wildlife Conservation Officer – NSW National Parks & Wildlife Service**

Managing biodiversity across vast national parks requires accurate, long-term data on wildlife populations and habitat utilisation. CBOOS enables continuous monitoring of endangered species across large, inaccessible areas, improving conservation decision-making while reducing field survey costs.

---

##  Features

### Core Capabilities

-  **3D SLAM with RTAB-Map** - Real-time mapping using LiDAR
-  **Autonomous Grid Exploration** - Systematic "square spiral" coverage pattern
-  **Multi-Sensor Fusion** - RGB, depth, thermal, and LiDAR integration
-  **Obstacle Avoidance** - 8-sector LiDAR collision detection
-  **Terrain Following** - Maintains altitude via downward AGL sensor
-  **Thermal Animal Detection** - Heat signature-based wildlife identification
-  **Live GUI Dashboard** - Real-time camera feeds and map visualisation
-  **Autonomous Return Home** - Automatic homebase navigation

### Sensor Suite

| Sensor | Purpose | Specifications |
|--------|---------|----------------|
| **3D LiDAR** | 360° obstacle detection & mapping | 40m range, ~3Hz |
| **RGB Camera** | Visual identification & tracking | 1920×1080, 30fps |
| **Depth Camera** | 3D perception & obstacle avoidance | 640×480, 30fps |
| **Thermal Camera** | Heat signature detection | 640×480, thermal range |
| **Downward Laser** | Altitude above ground (AGL) | 50m range |
| **IMU** | Orientation & acceleration | 100Hz |

---

##  System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  GAZEBO SIMULATION                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │   Terrain    │  │ Scout Drone  │  │  Homebase    │   │
│  │  (Forest)    │  │  (Sensors)   │  │  (Charging)  │   │
│  └──────────────┘  └──────────────┘  └──────────────┘   │
└───────────────────────────┬─────────────────────────────┘
                            │ ROS-IGN Bridge
┌───────────────────────────┴─────────────────────────────┐
│                    ROS2 LAYER                           │
│  ┌────────────────────────────────────────────────────┐ │
│  │  PERCEPTION & MAPPING                              │ │
│  │  • RTAB-Map SLAM        • Point Cloud Generation   │ │
│  │  • LiDAR Processing     • Occupancy Grid Mapping   │ │
│  └────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────┐ │
│  │  DECISION MAKING & PLANNING                        │ │
│  │  • Grid Explorer        • Collision Avoidance      │ │
│  │  • Path Planning        • Altitude Control         │ │
│  └────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────┐ │
│  │  USER INTERFACE                                    │ │
│  │  • Camera Control GUI   • Mission Status           │ │
│  │  • Real-time Feeds      • Map Visualisation        │ │
│  └────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### Component Overview

| Component | Purpose | Implementation | Status |
|-----------|---------|----------------|--------|
| **Grid Explorer** | Autonomous waypoint navigation with square-spiral pattern | C++ node (`grid_explorer.cpp`) | ✅ Complete |
| **Collision Avoidance** | 8-sector LiDAR obstacle detection & velocity modification | Embedded in Grid Explorer | ✅ Complete |
| **RTAB-Map SLAM** | 3D mapping & loop closure detection | Off-the-shelf (rtabmap_ros) | ✅ Complete |
| **AGL Parser** | Altitude-above-ground filtering | C++ node (`agl_parser.cpp`) | ✅ Complete |
| **Odometry Offset** | Zero-referenced local coordinates | C++ node (`odometry_offset.cpp`) | ✅ Complete |
| **Camera Suite** | RGB/Depth/Thermal perception | Gazebo sensors + ROS bridges | ✅ Complete |
| **Camera Control GUI** | Live feed display & mission control | Python/PySide6 (`camera_control_gui.py`) | ✅ Complete |
| **Homebase Station** | Data logging & weather integration | C++ node (`main.cpp`) | ⚠️ Implemented but not integrated |

---

##  Prerequisites

### System Requirements

- **OS:** Ubuntu 22.04 LTS
- **ROS2:** Humble Hawksbill
- **Simulator:** Ignition Gazebo Fortress
- **RAM:** Minimum 8GB (16GB recommended)
- **GPU:** Dedicated GPU recommended for smooth simulation

### Required Software Stack

```bash
# Core dependencies
ROS2 Humble
Ignition Gazebo Fortress
Python 3.10+
C++17 compiler (GCC 11+)
```

---

##  Installation

### 1. Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install Ignition Gazebo Fortress

```bash
sudo apt-get update
sudo apt-get install ignition-fortress -y
```

### 3. Install ROS2 Packages

```bash
sudo apt install -y \
  ros-humble-ros-ign-gazebo \
  ros-humble-ros-ign-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-rtabmap-ros \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-rclpy \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-nav-msgs
```

### 4. Install Python Dependencies

```bash
# Core Python packages
pip install PySide6 opencv-python-headless numpy psutil

# Additional system libraries for Qt
sudo apt install -y \
  python3-opencv \
  python3-numpy \
  libxcb-cursor0 \
  libxkbcommon-x11-0 \
  libxcb-icccm4 \
  libxcb-image0 \
  libxcb-keysyms1 \
  libxcb-render-util0
```

### 5. Clone and Build the Project

```bash
# Create workspace
mkdir -p ~/41068_ws/src
cd ~/41068_ws/src

# Clone repository
git clone https://github.com/Mafia-Pineapple/RsHDeez.git
cd ~/41068_ws

# Build
colcon build 

# Source the workspace
echo "source ~/41068_ws/install/setup.bash" >> ~/.bashrc
source ~/41068_ws/install/setup.bash
```

### 6. Verify Installation

```bash
# Check package is recognised
ros2 pkg list | grep navigation
# Should output: navigation

# Check nodes are available
ros2 pkg executables navigation
# Should list: grid_explorer_node, agl_parser, etc.
```

---

##  Quick Start
### Using Python GUI
```bash
cd ~/41068_ws
source install/setup.bash
Python3 camera_control_gui.py
```
1. Select desired cameras or maintain defaults.

2. Launch the Simulation with the launch button
3. Wait for Gazeb to initialise, then press the play button in the bottom left
4. Run the grid explorer node

*Cuation! Known issues with the GUI include having to reset device each time gazebo is restarted.*

### TWO-Terminal Setup (NO GUI)

**Terminal 1: Launch Simulation**
```bash
cd ~/41068_ws
source install/setup.bash
ros2 launch navigation scout.launch.py
```
*Starts Gazebo with forest terrain, spawns Scout drone, initialises sensors, and runs RTAB-MAP*


**Terminal 2: Launch Explorer**
```bash
cd ~/41068_ws
source install/setup.bash
ros2 run navigation grid_explorer_node
```
*drone begins autonomous exploration*

### Expected Behavior

1. **0-5 seconds:** Drone takes off and stabilises at target altitude
2. **5-300 seconds:** Systematic grid exploration in square-spiral pattern
3. **Continuous:** Live RGB/Depth/Thermal feeds in GUI
4. **Continuous:** 3D map builds in RTAB-Map viewer
5. **On completion:** Automatic return to homebase

---

##  Usage

### Basic Operation

#### 1. Conservative Exploration (Dense Forest)

```bash
ros2 run navigation grid_explorer_node --ros-args \
  -p cruise_speed:=1.0 \
  -p obstacle_distance:=4.0 \
  -p safe_distance:=7.0 \
  -p grid_spacing:=15.0
```

**Use case:** High obstacle density, requires careful navigation  
**Coverage:** ~9-25 waypoints in 5-7 minutes

#### 2. Balanced Exploration (Mixed Terrain)

```bash
ros2 run navigation grid_explorer_node --ros-args \
  -p cruise_speed:=1.5 \
  -p obstacle_distance:=3.0 \
  -p safe_distance:=5.0 \
  -p grid_spacing:=20.0 \
  -p grid_radius:=2
```

**Use case:** Default configuration for most environments  
**Coverage:** 25 waypoints (5×5 grid) in 8-10 minutes

#### 3. Aggressive Exploration (Open Areas)

```bash
ros2 run navigation grid_explorer_node --ros-args \
  -p cruise_speed:=2.0 \
  -p obstacle_distance:=2.0 \
  -p safe_distance:=4.0 \
  -p grid_spacing:=25.0 \
  -p grid_radius:=3
```

**Use case:** Minimal obstacles, fast coverage needed  
**Coverage:** 49 waypoints (7×7 grid) in 12-15 minutes

---

### Configuration Parameters

#### Grid Explorer Node

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `grid_spacing` | float | 20.0 | 10-50 | Meters between waypoints |
| `grid_radius` | int | 2 | 1-5 | Grid size (2 = 5×5 = 25 waypoints) |
| `cruise_speed` | float | 1.5 | 0.5-3.0 | Flight speed in m/s |
| `target_agl` | float | 15.0 | 5.0-30.0 | Altitude above ground (meters) |
| `obstacle_distance` | float | 3.0 | 1.5-5.0 | Emergency stop distance (meters) |
| `safe_distance` | float | 5.0 | 3.0-10.0 | Slow-down distance (meters) |
| `avoidance_gain` | float | 1.5 | 0.5-3.0 | Collision avoidance strength |
| `waypoint_tolerance` | float | 2.0 | 0.5-5.0 | Distance to consider waypoint reached |

#### Example: Custom Grid Size

```bash
# Small area (3×3 = 9 waypoints)
ros2 run navigation grid_explorer_node --ros-args \
  -p grid_radius:=1 \
  -p grid_spacing:=15.0

# Large area (7×7 = 49 waypoints)
ros2 run navigation grid_explorer_node --ros-args \
  -p grid_radius:=3 \
  -p grid_spacing:=25.0
```

#### Altitude Configuration

```bash
# Fly lower (more detail, riskier)
ros2 run navigation grid_explorer_node --ros-args -p target_agl:=10.0

# Fly higher (safer, less detail)
ros2 run navigation grid_explorer_node --ros-args -p target_agl:=20.0
```

---

### GUI Controls

#### Camera Control GUI Configuration

Edit image topics in `camera_control_gui.py`:

```python
# Lines 25-27: Configure image topics
self.rgb_topic = "/camera/rgb/image"        # RGB camera feed
self.depth_topic = "/camera/depth/image"    # Depth camera feed
self.thermal_topic = "/camera/thermal/image" # Thermal camera feed
```

#### Environment Sourcing

```python
# Lines 45-47: Configure workspace path
workspace_path = os.path.expanduser("~/41068_ws")
source_command = f"source {workspace_path}/install/setup.bash"
```

#### Launch Commands

```python
# Lines 180-185: Modify launch behavior
gazebo_cmd = "ros2 launch navigation scout.launch.py"
rtabmap_cmd = "ros2 launch navigation rtabmap_launch.py"
explorer_cmd = "ros2 run navigation grid_explorer_node"
```

---

##  Technical Details

### Perception & Mapping

#### SLAM Pipeline

1. **Sensor Fusion:** LiDAR scans + IMU data → RTAB-Map
2. **Loop Closure:** Automatic detection when revisiting areas
3. **Map Output:** 3D point cloud + 2D occupancy grid
4. **Localisation:** Real-time pose estimation in `map` frame

**Key Topics:**
- `/scan` → LiDAR input (sensor_msgs/LaserScan)
- `/imu` → Orientation data (sensor_msgs/Imu)
- `/rtabmap/grid_map` → 2D map (nav_msgs/OccupancyGrid)
- `/rtabmap/cloud_map` → 3D point cloud (sensor_msgs/PointCloud2)

#### Collision Avoidance Logic

**8-Sector LiDAR Analysis:**

```
       [0] Front
   [7]           [1]
[6]                 [2]
   [5]           [3]
       [4] Rear
```

**Three-Tier Response System:**

| Zone | Distance | Action | Speed Multiplier |
|------|----------|--------|------------------|
| Critical | < 3.0m | Emergency stop + escape maneuver | 0.0 |
| Warning | 3.0-5.0m | Slow down + lateral adjustment | 0.3-0.7 |
| Safe | > 5.0m | Normal operation | 1.0 |

**Avoidance Strategy:**
1. Analyse all 8 sectors for closest obstacles
2. Identify most open direction (highest minimum distance)
3. Apply lateral velocity toward open space
4. Reduce forward velocity proportionally to threat
5. Resume course when clear

---

### Decision Making & Planning

#### Grid Exploration Algorithm

**Square Spiral Pattern:**

```python
# Waypoint generation (pseudo-code)
center = home_position
for radius in range(1, grid_radius + 1):
    for x in range(-radius, radius + 1):
        for y in range(-radius, radius + 1):
            if max(abs(x), abs(y)) == radius:  # Only perimeter
                waypoint = center + (x * spacing, y * spacing)
                waypoints.append(waypoint)
```

**Navigation Flowchart:**

```
START
  ↓
[Take Off] → Reach target_agl
  ↓
[Get Next Waypoint] → From spiral sequence
  ↓
[Navigate to Waypoint]
  ├─ Continuous collision avoidance
  ├─ Maintain AGL altitude
  └─ Monitor SLAM localisation
  ↓
[Reached Waypoint?]
  ├─ Yes → [More Waypoints?]
  │         ├─ Yes → [Get Next Waypoint]
  │         └─ No  → [Return Home]
  └─ No  → [Continue Navigation]
  ↓
[Return Home] → Navigate to (0, 0, target_agl)
  ↓
[Land] → Descend to ground
  ↓
END
```

**Altitude Control:**

```cpp
// Maintain altitude above ground
double current_agl = agl_distance_;  // From downward laser
double error = target_agl_ - current_agl;
double vertical_velocity = kp_altitude_ * error;  // P-controller
vertical_velocity = clamp(vertical_velocity, -1.0, 1.0);
```

---

### User Interface

#### GUI Features

- **Live Camera Feeds:** RGB, Depth, Thermal (switchable)
- **Mission Control:** Start/Stop Gazebo, RTAB-Map, Explorer
- **Status Monitoring:** Terminal output in real-time
- **Environment Check:** Validates ROS2 workspace sourcing

#### Implementation: PySide6

```python
class CameraControlGUI(QMainWindow):
    def __init__(self):
        # Image display widgets
        self.rgb_display = ImageDisplayWidget("/camera/rgb/image")
        self.depth_display = ImageDisplayWidget("/camera/depth/image")
        self.thermal_display = ImageDisplayWidget("/camera/thermal/image")
        
        # Process management
        self.gazebo_process = None
        self.rtabmap_process = None
        self.explorer_process = None
```

---

## 📊 Results & Performance

### Demonstration Scenarios

#### Scenario 1: Open Terrain (5×5 Grid)

**Configuration:**
- Grid: 5×5 (25 waypoints), 20m spacing
- Speed: 1.5 m/s
- Altitude: 15m AGL

**Results:**
-  100% waypoint completion
-  Zero collisions
-  8 minutes 42 seconds total mission time
-  Successful return to home

**Video Evidence:** [Demo Video](https://www.youtube.com/watch?v=MwTPJN2bUqQ) @ 0:00-8:42

#### Scenario 2: Dense Forest (3×3 Grid)

**Configuration:**
- Grid: 3×3 (9 waypoints), 15m spacing
- Speed: 1.0 m/s (conservative)
- Obstacle distance: 4.0m

**Results:**
-  100% waypoint completion
-  3 successful obstacle avoidances
-  Smooth collision-free navigation
-  5 minutes 18 seconds mission time

**Video Evidence:** [Demo Video](https://www.youtube.com/watch?v=MwTPJN2bUqQ) @ 2:00-3:30

---

### Requirements Verification

| Req ID | Requirement | Status | Evidence |
|--------|-------------|--------|----------|
| **R1** | Robot navigates between points | ✅ Met | Video @ 2:49 - Grid navigation |
| **R2** | Detect wildlife with thermal | ✅ Met | Video @ 9:09 - Bear thermal signature |
| **R3** | Generate terrain map | ✅ Met | Video @ 9:42 - RTAB-Map output |
| **R4** | Obstacle detection & avoidance | ✅ Met | Video @ 2:00 - Tree avoidance |
| **R5** | Drone with homebase station | ✅ Met | Video @ 1:48 - Takeoff/landing |
| **R6** | Random animal spawning | ✅ Met | Video @ 0:50 - Animals in scene |
| **R7** | Thermal camera integration | ✅ Met | Video (entirety) - Live thermal feed |
| **R8** | User interface (GUI) | ⚠️ Partial | Video @ 1:17 - Camera feeds working |
| **R9** | Animal heat signatures | ✅ Met | Video @ 9:09 - Thermal detection |
| **SR1** | Animal tracking | ❌ Not Met | Detection bug (see Known Issues) |
| **SR2** | Multi-drone operation | ❌ Stretch | Not implemented |

**Overall MVP Achievement:** 8/9 requirements met (89%)

---

##  Known Issues

### 1. Animal Detection Not Functional

**Description:**  
The thermal camera successfully captures heat signatures from animals, but the animal detection pipeline fails to process and classify detections.

**Symptoms:**
- Thermal feed shows animals clearly (Video @ 9:09)
- No bounding boxes or labels appear on RGB feed
- No detection messages published to `/detections` topic

**Root Cause:**  
Suspected data transfer bug between ROS2 and OpenCV bridge. Image encoding mismatch or topic synchronisation issue.

**Impact:**  
Main use-case feature (wildlife identification) is non-functional despite successful thermal imaging.

**Next Steps:**
1. Debug cv_bridge image conversion
2. Verify thermal topic encoding (mono8 vs. bgr8)
3. Test with simplified detection script
4. Consider switching to direct Gazebo sensor plugin

**Workaround:**  
Manual inspection of thermal feed in GUI confirms animal presence.

---

### 2. GUI Process Management Issue

**Description:**  
When the Camera Control GUI is closed, child processes (Gazebo, RTAB-Map) are not properly terminated.

**Symptoms:**
- Ignition server remains running after GUI close
- Relaunching GUI starts second Gazebo instance → conflicts
- Memory/CPU leak from zombie processes

**Observed In:**  
Video demonstration @ 1:17 (GUI carefully not closed during demo)

**Impact:**  
Requires manual process termination:
```bash
killall ign gazebo
```

**Next Steps:**
1. Implement proper signal handling (SIGTERM/SIGKILL)
2. Track all subprocess PIDs in GUI
3. Add cleanup routine to `closeEvent()`

**Workaround:**  
Do not close GUI during operation; restart system between runs.

---

### 3. Homebase Integration Incomplete

**Description:**  
The Homebase node (`main.cpp`) is implemented but not integrated into the main system.

**Missing Features:**
- No weather data displayed in GUI
- No automatic data logging from Scout
- No charging status simulation

**Impact:**  
Stakeholder vision of "Central Base" is conceptual rather than functional.

**Next Steps:**
1. Add weather widget to GUI
2. Implement `/homebase/sightings` subscriber
3. Create battery simulation for charging behavior

---

##  Future Work

### Immediate Priorities (0-3 Months)

#### 1. Fix Animal Detection Pipeline
- **Why:** Core requirement for stakeholder use-case
- **Tasks:**
  - Debug OpenCV-ROS2 bridge
  - Implement YOLO or TensorFlow detection
  - Add classification labels (bear, deer, etc.)
- **Success Metric:** 80%+ detection accuracy in simulation

#### 2. Complete Homebase Integration
- **Why:** Necessary for "central base" concept
- **Tasks:**
  - Display weather data in GUI
  - Log mission sightings to database
  - Simulate battery charging behavior
- **Success Metric:** GUI shows live homebase status

#### 3. Improve GUI Stability
- **Why:** Operator reliability
- **Tasks:**
  - Fix process termination on close
  - Add error handling for crashed nodes
  - Implement auto-reconnect for lost topics
- **Success Metric:** 100 GUI launches without manual cleanup

---

### Medium-Term Goals (3-6 Months)

#### 4. Multi-Drone Support
- **Why:** 24/7 surveillance via relay operations
- **Tasks:**
  - Implement drone-drone communication
  - Add battery handoff logic
  - Create shared map database
- **Success Metric:** Two drones alternating without gaps

#### 5. Real-World Deployment Prep
- **Why:** Transition from simulation to field testing
- **Tasks:**
  - Port to PX4/ArduPilot flight controller
  - Add GPS waypoint navigation
  - Integrate cellular/radio telemetry
- **Success Metric:** Successful field test in small park

#### 6. Advanced Animal Tracking
- **Why:** Enhanced monitoring capability
- **Tasks:**
  - Track individual animal movement over time
  - Estimate population counts
  - Predict migration patterns
- **Success Metric:** Track 5+ animals over 30-minute session

---

### Long-Term Vision (6-12 Months)

#### 7. Machine Learning Optimisation
- Train custom models on NSW wildlife species
- Implement edge computing for on-drone inference
- Add behavior classification (feeding, resting, fleeing)

#### 8. Fleet Management Dashboard
- Multi-site monitoring (multiple parks)
- Automated mission scheduling
- Historical data analysis & reporting

#### 9. Regulatory Compliance
- CASA (Civil Aviation Safety Authority) approval
- Privacy-compliant data handling
- Environmental impact assessment

---

##  Contributing

### Team Members

| Name | Role | Contributions |
|------|------|---------------|
| **Nicholas Sabatta** | Navigation Lead | Collision avoidance, SLAM integration, odometry offset, Grid explorer algorithm, mission planning logic, altitude control |
| **Caleb Chadwick** | Navigation Support | Image display widgets,  sensor fusion, navigation support|
| **Anton Cecire** | UI Lead | Camera control GUI, launch management |
| **James Hart** | Environment Lead | Gazebo world modeling, homebase node, animal spawning |

### Development Workflow

1. **Communication:** WhatsApp for daily updates, Teams for meetings
2. **Version Control:** GitHub with feature branches
3. **Code Review:** Peer review before merging to main
4. **Testing:** Simulation testing before integration

### Gen-AI Usage Statement

Generative AI (ChatGPT, Claude, GitHub Copilot) was used for:
- Code scaffolding and boilerplate generation
- Debugging assistance (error message interpretation)
- Documentation wording improvements
- Diagram conceptualisation

**All submitted code is understood and explained by team members. No AI-generated code was used without comprehension.**

---


## Contact & Support

### Project Repository
**GitHub:** https://github.com/Mafia-Pineapple/RsHDeez 

### Team Contacts

**Project Lead:** Caleb Chadwick  
📧 [caleb.j.chadwick@student.uts.edu.au](mailto:caleb.j.chadwick@student.uts.edu.au)

**Technical Queries:** Nicholas Sabatta  
📧 [Nicholas.b.sabatta@student.uts.edu.au](mailto:Nicholas.b.sabatta@student.uts.edu.au)

**Stakeholder Communication:** Anton Cecire  
📧 [Anton.n.cecire@student.uts.edu.au](mailto:Anton.n.cecire@student.uts.edu.au)

### Academic Context

**Course:** 41068 Robotics Studio 1  
**Institution:** University of Technology Sydney (UTS)  
**Semester:** Autumn 2025

---

## 🎥 Demo Videos

**Full System Demonstration:** [YouTube](https://www.youtube.com/watch?v=MwTPJN2bUqQ)

**Key Timestamps:**
- 0:00 - System startup sequence
- 0:50 - Animal spawning with thermal signatures
- 1:17 - Camera Control GUI walkthrough
- 1:48 - Drone takeoff from homebase
- 2:00 - First obstacle avoidance event
- 2:49 - Grid navigation pattern
- 9:09 - Thermal detection of bear
- 9:42 - RTAB-Map 3D reconstruction

---

##  Acknowledgments

- **ROS2 Community:** For extensive documentation and support
- **RTAB-Map Developers:** For robust SLAM implementation


---

<div align="center">

**CBOOS - Autonomous Wildlife Monitoring for a Sustainable Future**

*Built with ❤️ by Team 33*

Thank you to all that contributed!

[⬆ Back to Top](#cboos---central-base-of-operational-surveying)

</div>