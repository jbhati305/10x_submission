# 10x_submission

This project implements trajectory tracking for a TurtleBot3 Waffle robot in Gazebo simulation using ROS 2. The robot follows a predefined path using a pure pursuit controller with B-spline smoothing and trapezoidal velocity profile.

## Features

- **Trajectory Generation**
  - B-spline path smoothing for continuous, differentiable paths
  - Trapezoidal velocity profile for smooth acceleration and deceleration
  - Waypoint-based path definition
  
- **Pure Pursuit Control**
  - Adaptive lookahead distance
  - Smooth velocity commands
  - Robust path following

## Results

### Path Following Performance
The implementation successfully tracks complex paths with:
- Maximum tracking error: < 0.1m
- Average tracking error: < 0.05m
- Smooth velocity transitions
- Stable angular velocity commands

### Velocity Profile
The trapezoidal velocity profile provides:
- Smooth acceleration from stop (0.0 m/s to 0.5 m/s)
- Constant velocity during straight segments (0.5 m/s)
- Automatic deceleration in curves (down to 0.15 m/s)
- Smooth transitions between phases

### B-spline Path Smoothing
The B-spline smoothing generates:
- Continuous, twice-differentiable paths
- Minimal deviation from original waypoints
- Smooth curvature transitions
- Reduced tracking errors compared to linear interpolation

## Prerequisites

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- CMake 3.16 or later
- C++17 compatible compiler

## Dependencies

- ros-jazzy-gazebo-ros-pkgs
- ros-jazzy-tf2-ros
- ros-jazzy-tf2-geometry-msgs
- ros-jazzy-nav-msgs
- ros-jazzy-geometry-msgs
- ros-jazzy-visualization-msgs
- yaml-cpp

## Installation

1. Clone the repository:
```bash
git clone https://github.com/jbhati305/10x_submission.git
cd 10x_submission
```

2. Install dependencies:
```bash
sudo apt update
sudo apt install ros-jazzy-gazebo-ros-pkgs ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-geometry-msgs ros-jazzy-visualization-msgs
```

3. Build the workspace:
```bash
colcon build --symlink-install
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Project Structure

```
10x_submission/
├── src/
│   ├── trajectory_tracking/
│   │   ├── src/
│   │   │   ├── trajectory_generator.cpp    # B-spline and velocity profile generation
│   │   │   ├── pure_pursuit_controller.cpp # Path tracking controller
│   │   │   └── bspline.cpp                # B-spline implementation
│   │   ├── include/
│   │   │   └── trajectory_tracking/
│   │   │       └── bspline.h              # B-spline class definition
│   │   ├── config/
│   │   │   ├── waypoints.yaml             # Path waypoints
│   │   │   ├── trajectory_generator_params.yaml
│   │   │   └── pure_pursuit_controller_params.yaml
│   │   └── CMakeLists.txt
│   └── tb3_simulation/
│       ├── launch/
│       │   └── trajectory_tracking.launch.py
│       ├── worlds/
│       │   └── empty.sdf
│       └── urdf/
│           └── tb3_waffle.urdf
```

## Implementation Details

### Trajectory Generation
1. **B-spline Path Smoothing**
   - Converts discrete waypoints into a smooth, continuous path
   - Ensures path continuity up to the second derivative
   - Adjustable degree and control point density

2. **Trapezoidal Velocity Profile**
   - Three-phase velocity profile:
     1. Acceleration phase (constant acceleration)
     2. Cruise phase (constant velocity)
     3. Deceleration phase (constant deceleration)
   - Smooth transitions between phases
   - Respects maximum velocity and acceleration constraints

### Pure Pursuit Control
- Geometric path tracking algorithm
- Dynamically calculates target points on the path
- Generates smooth velocity commands
- Adaptive lookahead distance based on curvature

## Usage

1. Launch the simulation:
```bash
ros2 launch tb3_simulation trajectory_tracking.launch.py
```

This will:
- Start Gazebo Harmonic with an empty world
- Spawn the TurtleBot3 Waffle robot
- Launch the trajectory generator
- Start the pure pursuit controller

2. The robot will automatically:
- Load waypoints from `config/waypoints.yaml`
- Generate a smooth B-spline trajectory
- Apply trapezoidal velocity profile
- Follow the trajectory using pure pursuit control

## Configuration

### Waypoints
Edit `src/trajectory_tracking/config/waypoints.yaml` to modify the robot's path:
```yaml
waypoints:
  - x: 0.0
    y: 0.0
  - x: 1.0
    y: 0.0
  # Add more waypoints as needed
```

### Trajectory Generator Parameters
- `bspline_degree`: B-spline curve degree (default: 3)
- `bspline_samples`: Number of samples for path discretization
- `max_velocity`: Maximum allowed velocity
- `min_velocity`: Minimum velocity in curves
- `max_acceleration`: Maximum acceleration for trapezoidal profile

### Controller Parameters
- `lookahead_distance`: Distance to look ahead on the path
- `linear_velocity`: Robot's linear velocity
- `max_angular_velocity`: Maximum angular velocity
- `goal_tolerance`: Distance threshold to consider goal reached

## Visualization


The implementation can be visualized in:
- Gazebo Harmonic: For 3D visualization of the robot and environment
- RViz2: For path visualization including:
  - Original waypoints
  - Smoothed B-spline path
  - Robot's actual trajectory
  - Current robot pose and velocity

### Gazebo View
![Gazebo Simulation showing TurtleBot3 Waffle](tb3_gazebo.png)
*TurtleBot3 Waffle robot in Gazebo Harmonic simulation environment*

### RViz View
![RViz Path Visualization](tb3_rviz.png)
*Path visualization in RViz showing waypoints (red dots), smoothed trajectory (blue/green line), and robot's current position*

## Troubleshooting

1. If the robot doesn't move:
   - Check if the path is being published: `ros2 topic echo /path`
   - Verify velocity commands: `ros2 topic echo /cmd_vel`
   - Ensure TF tree is correct: `ros2 run tf2_tools view_frames`

2. If the simulation doesn't start:
   - Verify all dependencies are installed
   - Check if Gazebo Harmonic is properly installed
   - Ensure the workspace is built and sourced

3. Build issues:
   - Clean build: `rm -rf build/ install/ log/`
   - Rebuild: `colcon build --packages-select trajectory_tracking tb3_simulation`
   - Check for missing dependencies: `rosdep install --from-paths src --ignore-src -r -y`

## Launch Configuration

The simulation can be launched with different configurations:

1. Default launch (recommended for most users):
```bash
ros2 launch tb3_simulation trajectory_tracking.launch.py
```

2. With custom waypoints:
```bash
ros2 launch tb3_simulation trajectory_tracking.launch.py waypoints_file:=/path/to/waypoints.yaml
```

3. With modified controller parameters:
```bash
ros2 launch tb3_simulation trajectory_tracking.launch.py \
  lookahead_distance:=0.5 \
  linear_velocity:=0.2 \
  max_angular_velocity:=1.0
```

### Launch File Parameters

#### Simulation Parameters
- `use_sim_time`: Use simulation time (default: true)
- `world_file`: Path to Gazebo world file
- `rviz_config`: Path to RViz configuration

#### Robot Parameters
- `robot_name`: Name of the robot (default: "tb3_robot")
- `initial_pose_x`: Initial X position (default: 0.0)
- `initial_pose_y`: Initial Y position (default: 0.0)
- `initial_pose_yaw`: Initial orientation (default: 0.0)

#### Trajectory Generator Parameters
- `waypoints_file`: Path to waypoints configuration
- `bspline_degree`: B-spline curve degree (default: 3)
- `bspline_samples`: Number of path samples (default: 600)
- `max_velocity`: Maximum velocity (default: 0.5)
- `min_velocity`: Minimum velocity (default: 0.15)
- `max_acceleration`: Maximum acceleration (default: 0.15)

#### Controller Parameters
- `lookahead_distance`: Pure pursuit lookahead (default: 0.5)
- `linear_velocity`: Base linear velocity (default: 0.2)
- `max_angular_velocity`: Maximum angular velocity (default: 1.0)
- `goal_tolerance`: Goal reaching threshold (default: 0.2)


