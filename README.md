# 10x_submission

A ROS 2 implementation of trajectory tracking for TurtleBot3 Waffle using pure pursuit control with B-spline path smoothing.

## Features
- B-spline path smoothing with trapezoidal velocity profile
- Pure pursuit controller with adaptive lookahead distance
- Real-time visualization in Gazebo and RViz
- Performance: < 0.1m max tracking error, < 0.05m average tracking error

## Prerequisites
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- C++17 compiler

## Installation

```bash
# 1. Clone repository
git clone https://github.com/jbhati305/10x_submission.git
cd 10x_submission

# 2. Install dependencies
sudo apt update
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-nav-msgs ros-jazzy-geometry-msgs ros-jazzy-visualization-msgs

# 3. Build and source
colcon build --symlink-install
source install/setup.bash
```

## Usage

Launch the simulation:
```bash
ros2 launch tb3_simulation trajectory_tracking.launch.py
```

### Configuration
- Edit waypoints: `src/trajectory_tracking/config/waypoints.yaml`
- Modify parameters via launch arguments:
```bash
ros2 launch tb3_simulation trajectory_tracking.launch.py \
  lookahead_distance:=0.5 \
  linear_velocity:=0.2 \
  max_angular_velocity:=1.0
```

### Key Parameters
- **Trajectory**: B-spline degree (3), max velocity (0.5 m/s), min velocity (0.15 m/s)
- **Controller**: Lookahead distance (0.5m), max angular velocity (1.0 rad/s)
- **Robot**: Initial pose configurable (x, y, yaw)

## Visualization

### Gazebo View
![Gazebo Simulation showing TurtleBot3 Waffle](/Images_&_videos/tb3_gazebo.png)
*TurtleBot3 Waffle robot in Gazebo Harmonic simulation environment*

### RViz View
![RViz Path Visualization](/Images_&_videos/tb3_rviz.png)
*Path visualization in RViz showing waypoints (red dots), smoothed trajectory (blue/green line), and robot's current position*

## Troubleshooting
1. **Robot not moving**: Check `/path` and `/cmd_vel` topics, verify TF tree
2. **Simulation issues**: Verify Gazebo installation and dependencies
3. **Build errors**: Try `rm -rf build/ install/ log/ && colcon build`

## Project Structure
```
10x_submission/
├── src/
│   ├── trajectory_tracking/      # Core implementation
│   │   ├── src/                 # Source files
│   │   ├── include/            # Headers
│   │   └── config/             # Parameters
│   └── tb3_simulation/         # Simulation setup
       ├── launch/
       ├── worlds/
       └── urdf/
```


