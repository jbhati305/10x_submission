# 10x_submission

A ROS 2 implementation of trajectory tracking for TurtleBot3 Waffle using pure pursuit control with B-spline path smoothing.

[![Simulation Video](/Images_&_videos/tb3_gazebo.png)](/Images_&_videos/simulation_video.mp4)

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



## Deliverables
### 1. Smooth Path Generation
- **Function**: `BSpline::generatePath()` in `src/trajectory_tracking/src/bspline.cpp`
- Takes discrete waypoints and generates a smooth, continuous B-spline path
- Ensures C2 continuity (continuous position, velocity, and acceleration)

### 2. Time-parameterized Trajectory
- **Function**: `TrajectoryGenerator::generateTrajectory()` in `src/trajectory_tracking/src/trajectory_generator.cpp`
- Outputs: `[(x0, y0, t0), (x1, y1, t1), ..., (xn, yn, tn)]`
- Trajectory data saved to `/src/trajectory_tracking/time_parameterized_traj/bspline_trajectory.csv` for analysis
- Implements trapezoidal velocity profile for smooth acceleration/deceleration

### 3. Controller Implementation
- **Function**: `PurePursuitController::computeVelocityCommands()` in `src/trajectory_tracking/src/pure_pursuit_controller.cpp`
- Inputs: Current robot state and reference trajectory
- Outputs: Linear and angular velocity commands
- Adaptive lookahead distance based on path curvature

#### Pure Pursuit Control Logic
The pure pursuit controller works by:
1. Finding a target point on the path ahead of the robot
2. Computing the curvature needed to reach that point
3. Generating velocity commands to follow the computed arc

```python
# Pseudocode for Pure Pursuit Control
def compute_velocity_commands(current_pose, trajectory):
    # 1. Find the closest point on trajectory to robot
    closest_point = find_closest_point(current_pose, trajectory)
    
    # 2. Calculate adaptive lookahead distance based on path curvature
    local_curvature = compute_path_curvature(closest_point)
    lookahead = base_lookahead + k_curvature * (1 - local_curvature)
    
    # 3. Find target point at lookahead distance
    target_point = find_lookahead_point(closest_point, lookahead)
    
    # 4. Transform target to robot's local frame
    target_local = transform_to_local_frame(target_point, current_pose)
    
    # 5. Compute curvature (1/radius) of arc to target
    curvature = 2 * target_local.y / (lookahead * lookahead)
    
    # 6. Generate velocity commands
    linear_vel = min(max_velocity, 
                    base_velocity * (1 - abs(curvature)/max_curvature))
    angular_vel = linear_vel * curvature
    
    return linear_vel, angular_vel
```

Key Features:
- **Adaptive Lookahead**: Adjusts based on path curvature
  - Longer on straight sections (faster tracking)
  - Shorter on curves (better accuracy)
- **Velocity Scaling**: Reduces speed in sharp turns
- **Smooth Control**: Continuous velocity commands for stable motion
- **Path Recovery**: Can recover if robot deviates from path

### 4. Simulation & Visualization
- Gazebo simulation with TurtleBot3 Waffle (shown below)
- RViz visualization of:
  - Original waypoints
  - Smoothed trajectory
  - Robot's actual path
  - Real-time tracking performance