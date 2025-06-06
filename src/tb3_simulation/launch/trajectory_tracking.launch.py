import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tb3_simulation = get_package_share_directory('tb3_simulation')
    pkg_trajectory_tracking = get_package_share_directory('trajectory_tracking')
    
    # Correctly set the Gazebo resource path
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_tb3_simulation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', os.path.join(pkg_tb3_simulation, 'worlds', 'empty.sdf')],
        output='screen'
    )

    # Robot State Publisher
    urdf_file_path = PathJoinSubstitution(
        [pkg_tb3_simulation, "urdf", "tb3_waffle.urdf"]
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(["xacro", " ", urdf_file_path])
        }],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tb3_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            'scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            'imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    # Trajectory Generator
    trajectory_generator = Node(
        package='trajectory_tracking',
        executable='trajectory_generator',
        name='trajectory_generator',
        parameters=[{
            'use_sim_time': True,
            'waypoints_file': os.path.join(pkg_trajectory_tracking, 'config', 'waypoints.yaml')
        }]
    )

    # Pure Pursuit Controller
    pure_pursuit_controller = Node(
        package='trajectory_tracking',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'lookahead_distance': 0.5,
            'linear_velocity': 0.2,
            'max_angular_velocity': 1.0,
            'goal_tolerance': 0.2
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        trajectory_generator,
        pure_pursuit_controller,
    ]) 