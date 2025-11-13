# Autonomous Navigation Guide - Car with 2 Trailers

Complete guide for running autonomous waypoint navigation with integrated pose plotting in the **Empty World** environment.

---

# Overview

The autonomous navigation mode uses a **closed-loop proportional controller** to navigate the robot through a pre-defined square path while collecting pose data for plotting. This is ideal for:
- Demonstrating autonomous navigation capabilities
- Generating pose plots for project submission
- Understanding feedback control systems
- Analyzing robot trajectory accuracy

---

# Quick Start

### Step 1: Launch Gazebo with Empty World

Open a terminal and run:

```bash
cd ~/Downloads/ros2_ws
source install/setup.bash
ros2 launch car_trailer teleop.launch.py
```

**Wait for Gazebo to fully load** - The robot should appear in an empty world at origin.

### Step 2: Run Autonomous Navigation with Plotting

Open a **new terminal** and run:

```bash
cd ~/Downloads/ros2_ws
source install/setup.bash
ros2 run car_trailer autonomous_with_plots.py
```

### Step 3: Wait for Completion

**DO NOT INTERRUPT!** Let the script complete all 4 waypoints.

You should see:

```
[INFO] ============================================================
[INFO] Autonomous Controller with Pose Plotter Started
[INFO] Following 4 waypoints in square path
[INFO] Recording pose data for plotting...
[INFO] ============================================================
[INFO] ✓ Waypoint 1/4 reached! -> (3.0, 0.0)
[INFO] ✓ Waypoint 2/4 reached! -> (3.0, 3.0)
[INFO] ✓ Waypoint 3/4 reached! -> (0.0, 3.0)
[INFO] ✓ Waypoint 4/4 reached! -> (0.0, 0.0)
[INFO] ✅ All waypoints reached! Mission complete.

Generating plots...
[INFO] Generating plots with 753 data points...
[INFO] Plot saved to: autonomous_pose_plot_20251113_140532.png
[INFO] Data saved to CSV: autonomous_pose_data_20251113_140532.csv

Mission Statistics:
   Duration: 75.32 seconds
   Total distance: 12.15 meters
   X range: [0.00, 3.05] m
   Y range: [0.00, 3.02] m
   Data points: 753
```

### Step 4: View Results

**Plot window will automatically open** showing 4 graphs. Files are saved in `/home/shanthosh/Downloads/ros2_ws/`

---

## Mission Details

### Waypoint Path

The robot follows a **3m x 3m square path**:

```
(0,3) ←------ (3,3)
  ↓             ↑
  ↓             ↑
(0,0) ------→ (3,0)
```

**Waypoints**:
1. **(3.0, 0.0)** - Move forward 3 meters
2. **(3.0, 3.0)** - Turn left and move forward 3 meters
3. **(0.0, 3.0)** - Turn left and move forward 3 meters
4. **(0.0, 0.0)** - Turn left and return to start

### Expected Performance

-  **Total Duration**: 60-90 seconds
- **Total Distance**: ~12 meters
-  **Goal Tolerance**: 0.4 meters
- **Data Points**: ~600-900 (10 Hz sampling)
-  **Success Rate**: Should reach all 4 waypoints

---

## Controller Algorithm

### Proportional Control

The autonomous controller uses a **closed-loop proportional controller** with odometry feedback:

#### 1. Position Error Calculation
```python
error_x = goal_x - current_x
error_y = goal_y - current_y
distance = sqrt(error_x² + error_y²)
```

#### 2. Desired Heading
```python
desired_theta = atan2(error_y, error_x)
angle_error = normalize(desired_theta - current_theta)
```

#### 3. Control Law
```python
# Angular velocity (turn toward goal)
angular_velocity = k_angular * angle_error

# Linear velocity (move toward goal)
linear_velocity = k_linear * distance

# Apply velocity limits
linear_velocity = clamp(linear_velocity, 0, max_linear)
angular_velocity = clamp(angular_velocity, -max_angular, max_angular)
```

### Controller Parameters

```python
# Gains (optimized for speed)
k_linear = 1.2      # Linear velocity gain
k_angular = 3.0     # Angular velocity gain

# Speed limits
max_linear = 2.0 m/s     # Maximum forward speed
max_angular = 2.5 rad/s  # Maximum turn rate

# Goal tolerance
goal_tolerance = 0.4 m   # Distance to consider waypoint reached
```

### Control Loop

- **Frequency**: 10 Hz (100ms update rate)
- **Feedback**: Odometry from `/odom` topic (50 Hz)
- **Commands**: Published to `/cmd_vel` topic

---

##  Generated Plots

The script automatically generates **4 subplots** in a single PNG image:

### 1. X Position vs Time
- **X-axis**: Time (seconds)
- **Y-axis**: X position (meters)
- **Shows**: Forward/backward movement over time

### 2. Y Position vs Time
- **X-axis**: Time (seconds)
- **Y-axis**: Y position (meters)
- **Shows**: Left/right movement over time

### 3. X and Y Position vs Time (Combined)
- **X-axis**: Time (seconds)
- **Y-axis**: Position (meters)
- **Shows**: Both X (blue) and Y (red) on same graph
- **Legend**: Distinguishes X and Y traces

### 4. 2D Trajectory
- **X-axis**: X position (meters)
- **Y-axis**: Y position (meters)
- **Shows**: Bird's-eye view of robot path
- **Features**:
  - Green star: Start position
  - Red cross: End position
  - Red squares: Waypoint goals
  - Blue line: Actual robot trajectory
  - Grid: Position reference

---

##  Output Files

### PNG Plot File

- **Filename**: `autonomous_pose_plot_YYYYMMDD_HHMMSS.png`
- **Location**: `/home/shanthosh/Downloads/ros2_ws/`
- **Resolution**: 300 DPI (high quality for reports)
- **Size**: ~16x12 inches
- **Format**: 4 subplots in 2x2 grid

### CSV Data File

- **Filename**: `autonomous_pose_data_YYYYMMDD_HHMMSS.csv`
- **Location**: `/home/shanthosh/Downloads/ros2_ws/`
- **Columns**:
  - `Time (s)` - Elapsed time since start
  - `X (m)` - X position in meters
  - `Y (m)` - Y position in meters
  - `Theta (rad)` - Orientation angle in radians

**Example CSV content**:
```csv
Time (s),X (m),Y (m),Theta (rad)
0.00,-0.0001,0.0002,0.0003
0.10,0.0245,0.0012,0.0567
0.20,0.0523,0.0034,0.1123
...
```

### Terminal Statistics

At completion, the terminal displays:

```
Mission Statistics:
   Duration: 75.32 seconds       # Total mission time
   Total distance: 12.15 meters  # Path length traveled
   X range: [0.00, 3.05] m      # Min/max X position
   Y range: [0.00, 3.02] m      # Min/max Y position
   Data points: 753              # Number of samples collected
```

---

## ⚙️ Technical Details

### ROS2 Topics

**Subscribed**:
- `/odom` (nav_msgs/Odometry) - Robot pose feedback at 50 Hz

**Published**:
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands at 10 Hz

### Data Collection

- **Sampling Rate**: 10 Hz (every 100ms)
- **Data Stored**:
  - Timestamps (relative to start)
  - X, Y positions from odometry
  - Theta orientation (yaw angle)
- **Duration**: From script start to mission completion

### Odometry Source

The differential drive plugin publishes odometry:
- **Frame**: `odom` → `base_link`
- **Update Rate**: 50 Hz
- **Accuracy**: Based on wheel encoder simulation

