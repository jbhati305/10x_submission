# 10x_submission

This project implements trajectory tracking for a TurtleBot3 Waffle robot in Gazebo simulation using ROS 2. The robot follows a predefined path using a pure pursuit controller.

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

Basic build:
```bash
colcon build
```

Build with specific packages:
```bash
colcon build --packages-select trajectory_tracking tb3_simulation
```

Build with debug symbols:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

Build with optimization:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build with parallel jobs (faster build):
```bash
colcon build --parallel-workers 4
```

Build with symlink install (faster development):
```bash
colcon build --symlink-install
```

4. Source the workspace:
```bash
source install/setup.bash
```

To make the sourcing permanent, add it to your `.bashrc`:
```bash
echo "source ~/10x_submission/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Project Structure

```
10x_submission/
├── src/
│   ├── trajectory_tracking/
│   │   ├── src/
│   │   │   ├── trajectory_generator.cpp
│   │   │   └── pure_pursuit_controller.cpp
│   │   ├── config/
│   │   │   └── waypoints.yaml
│   │   └── CMakeLists.txt
│   └── tb3_simulation/
│       ├── launch/
│       │   └── trajectory_tracking.launch.py
│       ├── worlds/
│       │   └── empty.sdf
│       └── urdf/
│           └── tb3_waffle.urdf
```

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
- Generate a smooth trajectory
- Follow the trajectory using the pure pursuit controller

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

### Controller Parameters
The pure pursuit controller parameters can be adjusted in the launch file:
- `lookahead_distance`: Distance to look ahead on the path (default: 0.5m)
- `linear_velocity`: Robot's linear velocity (default: 0.2m/s)
- `max_angular_velocity`: Maximum angular velocity (default: 1.0rad/s)
- `goal_tolerance`: Distance threshold to consider goal reached (default: 0.2m)

## Visualization

The simulation can be visualized in:
- Gazebo Harmonic: For 3D visualization of the robot and environment
- RViz2: For path visualization and debugging

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

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a new Pull Request

## TODO

### Visualization and Debugging
- [ ] Add a dedicated trajectory visualizer node
  - Real-time visualization of current trajectory
  - Display of lookahead point and robot orientation
  - Path following error visualization
  - Velocity and control command plots

### Trajectory Generation
- [ ] Implement alternative trajectory smoothing algorithms
  - [ ] B-spline interpolation
  - [ ] Bezier curves
  - [ ] Dubins paths
  - [ ] Clothoid curves
- [ ] Add trajectory optimization for smoother motion
- [ ] Implement dynamic trajectory replanning

### Code Organization
- [ ] Refactor code into smaller, more focused classes
- [ ] Add comprehensive unit tests
- [ ] Implement proper error handling and logging
- [ ] Add detailed code documentation
- [ ] Create a proper CI/CD pipeline

### Obstacle Avoidance
- [ ] Implement basic obstacle detection using laser scan data
- [ ] Add dynamic obstacle avoidance
- [ ] Integrate with existing path planning algorithms
- [ ] Add safety margins and collision checking
- [ ] Implement emergency stop functionality

### User Interface
- [ ] Create a GUI for waypoint configuration
  - [ ] Interactive waypoint placement
  - [ ] Real-time trajectory preview
  - [ ] Waypoint editing and deletion
  - [ ] Save/load waypoint configurations
- [ ] Add a control panel for:
  - [ ] Controller parameter tuning
  - [ ] Robot state monitoring
  - [ ] Emergency controls
  - [ ] Simulation controls

### Performance Optimization
- [ ] Profile and optimize trajectory generation
- [ ] Improve controller performance
- [ ] Add multi-threading support
- [ ] Optimize memory usage

### Documentation
- [ ] Add detailed API documentation
- [ ] Create user guides
- [ ] Add example configurations
- [ ] Create tutorial videos