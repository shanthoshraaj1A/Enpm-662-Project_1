# Teleoperation Instructions - Car with 2 Trailers

Complete guide for manual keyboard control of the car with 2 trailers in the **Competition Arena** environment.

---

##  Overview

The teleoperation mode allows you to manually control the robot using keyboard inputs in a competition arena environment. This is ideal for:
- Manual navigation testing
- Exploring the environment
- Video recording for project submission
- Understanding robot dynamics

---

## Quick Start

### Step 1: Launch Gazebo with Competition Arena

Open a terminal and run:

```bash
cd ~/Downloads/ros2_ws
source install/setup.bash
ros2 launch car_trailer teleop_arena.launch.py
```

**Wait for Gazebo to fully load** - The robot should appear in the competition arena.

### Step 2: Start Keyboard Teleop

Open a **new terminal** and run:

```bash
cd ~/Downloads/ros2_ws
source install/setup.bash
ros2 run car_trailer keyboard_teleop.py
```

You should see:

```
Car Trailer Keyboard Teleop Controller
======================================
Controls:
  w - Move forward
  s - Move backward
  a - Turn left
  d - Turn right
  space - Emergency stop
  Ctrl+C - Exit

Current speed settings:
  Linear: 1.50 m/s
  Angular: 2.00 rad/s

Keep this terminal in focus and start driving!
```

### Step 3: Drive the Robot

With the teleop terminal in focus:
- Press `w` to move forward
- Press `a` to turn left while moving
- Press `d` to turn right while moving
- Press `s` to move backward
- Press `space` to stop immediately
- Press `Ctrl+C` to exit

---

##  Controls Reference

| Key | Action | Speed |
|-----|--------|-------|
| `w` | Move forward | 1.5 m/s |
| `s` | Move backward | -1.5 m/s |
| `a` | Turn left | 2.0 rad/s |
| `d` | Turn right | -2.0 rad/s |
| `space` | Emergency stop | 0 m/s |
| `Ctrl+C` | Exit program | - |

---

## ⚙️ Technical Details

### Robot Control

- **Control Type**: Differential drive (rear wheels only)
- **Command Topic**: `/cmd_vel` (geometry_msgs/Twist)
- **Update Rate**: 10 Hz (100ms per command)
- **Wheel Separation**: 0.5 meters
- **Wheel Diameter**: 0.2 meters

### Keyboard Input

The keyboard teleop script uses `pynput` library to capture keyboard events:
- Non-blocking input (continues sending commands while key is held)
- Key release detection (stops movement when key released)
- Multiple key combination support (can move and turn simultaneously)

### Velocity Commands

Commands are sent as `geometry_msgs/Twist` messages:

```python
# Forward movement
cmd_vel.linear.x = 1.5   # m/s
cmd_vel.angular.z = 0.0  # rad/s

# Turning
cmd_vel.linear.x = 0.0   # m/s  
cmd_vel.angular.z = 2.0  # rad/s (left) or -2.0 (right)

# Combined (forward + turn)
cmd_vel.linear.x = 1.5   # m/s
cmd_vel.angular.z = 2.0  # rad/s
```

---

##  Competition Arena Environment

The competition arena world includes:
- **Ground plane**: Flat surface for navigation
- **Obstacles**: Various objects to navigate around
- **Boundaries**: Arena walls/limits
- **Lighting**: Simulated ambient lighting

**Spawn Position**: Robot starts at origin (0, 0, 0.5)

