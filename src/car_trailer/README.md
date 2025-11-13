# Car with 2 Trailers - ROS2 Gazebo Simulation

A complete ROS2 package for simulating a car with 2 trailers in Gazebo, featuring teleoperation control, autonomous navigation with closed-loop control, and pose plotting capabilities.

---

# Table of Contents

- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
  - [Teleoperation (Competition Arena)](#teleoperation-competition-arena)
  - [Autonomous Navigation (Empty World)](#autonomous-navigation-empty-world)
- [Package Structure](#package-structure)
- [Robot Description](#robot-description)
- [Scripts Overview](#scripts-overview)
- [Output Files](#output-files)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)

---

# Features

- **Differential Drive Control**: Rear-wheel drive with odometry feedback
- **Keyboard Teleoperation**: Manual control using w/a/s/d keys
- **Autonomous Navigation**: Closed-loop proportional controller
- **Pose Plotting**: Real-time data collection and visualization
- **Multiple Environments**: Competition arena for teleop, empty world for autonomous
- **Complete TF Tree**: 14 links with proper transforms
- **High-Quality Meshes**: Detailed STL models from SolidWorks

---

#  System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Gazebo**: 11.10.2
- **Python**: 3.10+
- **Python Libraries**: matplotlib, numpy

---

# Installation

### 1. Install ROS2 Dependencies

```bash
sudo apt-get update
sudo apt-get install ros-humble-gazebo-ros-pkgs
sudo apt-get install ros-humble-robot-state-publisher
sudo apt-get install ros-humble-joint-state-publisher
```

### 2. Install Python Dependencies

```bash
pip3 install matplotlib numpy
```

### 3. Build the Package

```bash
cd ~/Downloads/ros2_ws
colcon build --packages-select car_trailer
source install/setup.bash
```

---

# Quick Start

### Teleoperation (Competition Arena)

```bash
# Terminal 1 - Launch Gazebo with competition arena
cd ~/Downloads/ros2_ws
source install/setup.bash
ros2 launch car_trailer teleop_arena.launch.py

# Terminal 2 - Run keyboard teleop
cd ~/Downloads/ros2_ws
source install/setup.bash
ros2 run car_trailer keyboard_teleop.py
```

**Controls:**
- `w` - Move forward
- `s` - Move backward  
- `a` - Turn left
- `d` - Turn right
- `Space` - Emergency stop
- `Ctrl+C` - Exit

### Autonomous Navigation (Empty World)

```bash
# Terminal 1 - Launch Gazebo with empty world
cd ~/Downloads/ros2_ws
source install/setup.bash
ros2 launch car_trailer teleop.launch.py

# Terminal 2 - Run autonomous navigation with plotting
cd ~/Downloads/ros2_ws
source install/setup.bash
ros2 run car_trailer autonomous_with_plots.py
```

**Wait 60-90 seconds for completion. Plots auto-generate and save.**

---

# Usage

### Teleoperation (Competition Arena)

For manual control in the competition arena environment:

1. **Launch the competition arena:**
   ```bash
   ros2 launch car_trailer teleop_arena.launch.py
   ```

2. **Start keyboard teleop:**
   ```bash
   ros2 run car_trailer keyboard_teleop.py
   ```

3. **Drive the robot** using keyboard controls
4. **Press Ctrl+C** to exit when done

See [`TELEOP_INSTRUCTIONS.md`](TELEOP_INSTRUCTIONS.md) for detailed instructions.

### Autonomous Navigation (Empty World)

For autonomous waypoint following with pose plotting:

1. **Launch the empty world:**
   ```bash
   ros2 launch car_trailer teleop.launch.py
   ```

2. **Run autonomous navigation:**
   ```bash
   ros2 run car_trailer autonomous_with_plots.py
   ```

3. **Wait for completion** - Do not interrupt!
   - Follows 4 waypoints in a 3m x 3m square path
   - Takes 60-90 seconds to complete
   - Automatically generates plots when finished

4. **View outputs:**
   - PNG plot: `/home/shanthosh/Downloads/ros2_ws/autonomous_pose_plot_*.png`
   - CSV data: `/home/shanthosh/Downloads/ros2_ws/autonomous_pose_data_*.csv`

See [`AUTONOMOUS_GUIDE.md`](AUTONOMOUS_GUIDE.md) for detailed instructions.

---

# Package Structure

```
car_trailer/
├── CMakeLists.txt                          # Build configuration
├── package.xml                             # Package metadata
├── README.md                               # This file
├── TELEOP_INSTRUCTIONS.md                  # Teleoperation guide
├── AUTONOMOUS_GUIDE.md                     # Autonomous navigation guide
│
├── launch/
│   ├── gazebo.launch.py                    # Base Gazebo launch
│   ├── rviz.launch.py                      # RViz visualization
│   ├── teleop.launch.py                    # Empty world (for autonomous)
│   └── teleop_arena.launch.py              # Competition arena (for teleop)
│
├── urdf/
│   ├── car_2trailers.urdf.xacro            # Robot description with differential drive
│   └── car_2trailers.csv                   # Additional data
│
├── meshes/                                 # STL mesh files (14 files)
│   ├── base_link.STL                       # Main car body
│   ├── lidar.STL                           # LiDAR sensor
│   ├── steer_left.STL                      # Left steering mechanism
│   ├── steer_right.STL                     # Right steering mechanism
│   ├── trailer1.STL                        # First trailer
│   ├── trailer2.STL                        # Second trailer
│   ├── wheel_front_left.STL                # Front left wheel
│   ├── wheel_front_right.STL               # Front right wheel
│   ├── wheel_rear_left.STL                 # Rear left wheel (driven)
│   ├── wheel_rear_right.STL                # Rear right wheel (driven)
│   ├── wheel_trailer1_left.STL             # Trailer 1 left wheel
│   ├── wheel_trailer1_right.STL            # Trailer 1 right wheel
│   ├── wheel_trailer2_left.STL             # Trailer 2 left wheel
│   └── wheel_trailer2_right.STL            # Trailer 2 right wheel
│
├── worlds/
│   ├── empty.world                         # Empty world for autonomous navigation
│   └── competition_arena.world             # Competition arena for teleoperation
│
└── scripts/
    ├── keyboard_teleop.py                  # Keyboard teleoperation
    ├── autonomous_nav.py                   # Autonomous navigation (standalone)
    ├── pose_plotter.py                     # Pose plotting (standalone)
    └── autonomous_with_plots.py            # Combined autonomous + plotting (MAIN)
```

---

#  Robot Description

### Physical Specifications

- **Total Links**: 14 (car body + 2 trailers + wheels + sensors)
- **Driven Wheels**: Rear 2 wheels (differential drive)
- **Wheel Diameter**: 0.2 meters
- **Wheel Separation**: 0.5 meters
- **Control**: Differential drive plugin

### Differential Drive Configuration

```xml
<wheel_separation>0.5</wheel_separation>
<wheel_diameter>0.2</wheel_diameter>
<max_wheel_torque>20</max_wheel_torque>
<command_topic>/cmd_vel</command_topic>
<odometry_topic>/odom</odometry_topic>
<odometry_frame>odom</odometry_frame>
<robot_base_frame>base_link</robot_base_frame>
<publish_odom>true</publish_odom>
<publish_odom_tf>true</publish_odom_tf>
<publish_wheel_tf>true</publish_wheel_tf>
<odometry_rate>50.0</odometry_rate>
```

# Topics

- **Subscribed**: `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
- **Published**: `/odom` (nav_msgs/Odometry) - Odometry at 50 Hz

---

# Scripts Overview

### 1. keyboard_teleop.py
- **Purpose**: Manual keyboard control
- **Usage**: `ros2 run car_trailer keyboard_teleop.py`
- **Controls**: w/a/s/d for movement, space to stop
- **Speed**: Linear 1.5 m/s, Angular 2.0 rad/s

# 2. autonomous_with_plots.py (RECOMMENDED)
- **Purpose**: Autonomous navigation with integrated pose plotting
- **Usage**: `ros2 run car_trailer autonomous_with_plots.py`
- **Features**:
  - Closed-loop proportional controller
  - Follows 4 waypoints: (3,0) → (3,3) → (0,3) → (0,0)
  - Real-time pose data collection at 10 Hz
  - Auto-generates 4 plots: X vs time, Y vs time, combined, 2D trajectory
  - Saves PNG (300 DPI) and CSV files
- **Duration**: 60-90 seconds
- **Controller Gains**: k_linear=1.2, k_angular=3.0

# 3. autonomous_nav.py
- **Purpose**: Standalone autonomous navigation (without plotting)
- **Usage**: `ros2 run car_trailer autonomous_nav.py`
- **Note**: Use `autonomous_with_plots.py` instead for project submission

# 4. pose_plotter.py
- **Purpose**: Standalone pose plotting (requires separate autonomous run)
- **Usage**: `ros2 run car_trailer pose_plotter.py`
- **Note**: Use `autonomous_with_plots.py` instead for integrated solution

---

# Output Files

When running `autonomous_with_plots.py`, the following files are automatically generated:

### 1. PNG Plot (300 DPI)
- **Filename**: `autonomous_pose_plot_YYYYMMDD_HHMMSS.png`
- **Location**: `/home/shanthosh/Downloads/ros2_ws/`
- **Contents**: 4 subplots
  1. X Position vs Time
  2. Y Position vs Time
  3. X and Y vs Time (combined)
  4. 2D Trajectory with waypoints

### 2. CSV Data File
- **Filename**: `autonomous_pose_data_YYYYMMDD_HHMMSS.csv`
- **Location**: `/home/shanthosh/Downloads/ros2_ws/`
- **Columns**: Time (s), X (m), Y (m), Theta (rad)
- **Sample Rate**: 10 Hz

### 3. Terminal Statistics
```
Mission Statistics:
   Duration: 75.32 seconds
   Total distance: 12.15 meters
   X range: [0.00, 3.05] m
   Y range: [0.00, 3.02] m
   Data points: 753
```

---

## 🔍 Troubleshooting

### Gazebo doesn't start
```bash
killall -9 gzserver gzclient
ros2 launch car_trailer teleop.launch.py
```

### Robot doesn't move
- Check if differential drive plugin is loaded: `ros2 topic list | grep cmd_vel`
- Verify odometry: `ros2 topic echo /odom --once`
- Ensure Gazebo is fully loaded before running scripts

### Plots not generating
- Let autonomous script complete all 4 waypoints (don't interrupt!)
- Check matplotlib installation: `python3 -c "import matplotlib; print('OK')"`
- Look for output files in workspace root: `ls ~/Downloads/ros2_ws/*.png`

### "No module named 'matplotlib'"
```bash
pip3 install matplotlib numpy
```

# Build errors
```bash
cd ~/Downloads/ros2_ws
colcon build --packages-select car_trailer --cmake-clean-cache
source install/setup.bash
```

---


# Controller Algorithm

The autonomous navigation uses a **proportional controller**:

### Position Error
```python
error_x = goal_x - current_x
error_y = goal_y - current_y
distance = sqrt(error_x² + error_y²)
```

# Angle Control
```python
desired_theta = atan2(error_y, error_x)
angle_error = normalize(desired_theta - current_theta)
angular_velocity = k_angular * angle_error
```

# Linear Control
```python
linear_velocity = k_linear * distance
```

# Velocity Limits
- Max linear: 2.0 m/s
- Max angular: 2.5 rad/s
- Goal tolerance: 0.4 m

# Video Recording

For project submission, record two videos:

# 1. Teleoperation Video (1-2 minutes)
- Show competition arena launch command
- Demonstrate keyboard control (w/a/s/d)
- Show robot navigating through arena

# 2. Autonomous Navigation Video (2-3 minutes)
- Show empty world launch command
- Show autonomous script execution
- Capture waypoint progress in terminal
- Show generated plots at the end

See VIDEO_RECORDING for detailed instructions.

