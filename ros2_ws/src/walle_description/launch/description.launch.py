"""
description.launch.py
=====================
A ROS2 launch file for the robot_state_publisher with the URDF model.

WHAT IS A LAUNCH FILE?
  A launch file is a Python script that starts multiple ROS2 nodes
  at once, with their configurations. Instead of opening 5 separate
  terminals and running 5 separate commands, you run ONE launch file.

  Launch files live in the `launch/` directory of a package.
  You run them with: ros2 launch <package> <launch_file>

WHAT THIS LAUNCH FILE STARTS:
  1. robot_state_publisher → reads the URDF, subscribes to /joint_states,
     and broadcasts TF transforms for every joint in the robot.

  TF (Transform Frames) is ROS2's coordinate system manager.
  It answers questions like: "Where is the IMU relative to the left wheel?"
  RViz2 uses TF to know where to draw each part of the robot.

  Without robot_state_publisher + URDF:
    RViz can't draw your robot
    You can't see where sensors are in 3D space

  With it:
    RViz draws a 3D robot that tilts and spins its wheels in real-time
"""

# ── ROS2 Launch imports ────────────────────────────────────────────────────────
# These are the building blocks of every launch file:
from launch import LaunchDescription             # Container for all launch actions
from launch.actions import DeclareLaunchArgument # Lets user pass arguments to launch
from launch.substitutions import (
    LaunchConfiguration,                         # Read a launch argument value
    Command,                                     # Run a shell command (for xacro)
    PathJoinSubstitution                         # Build file paths safely
)
from launch_ros.actions import Node              # Start a ROS2 node
from launch_ros.descriptions import ParameterValue   # Explicit parameter typing
from launch_ros.substitutions import FindPackageShare  # Find a package's share dir


def generate_launch_description():
    """
    Every launch file must have this function.
    It returns a LaunchDescription containing all the nodes/actions to start.

    IMPORTANT: Launch files are declarative, not procedural.
    You're not writing "do this then that" — you're describing
    "here are the things that should exist when this launches."
    ROS2's launch system figures out the order.
    """

    # ── Find the package share directory ──────────────────────────────────────
    # FindPackageShare('walle_description') returns the path to:
    # ~/sid_ws/install/walle_description/share/walle_description/
    # This is where colcon installs the URDF, rviz files, etc.
    pkg_share = FindPackageShare('walle_description')

    # ── Build the path to the URDF xacro file ─────────────────────────────────
    # PathJoinSubstitution safely joins path components (handles OS differences)
    urdf_xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'walle.urdf.xacro'])

    # ── Process the xacro file → plain URDF ───────────────────────────────────
    # xacro is a preprocessor: it expands macros and properties in the .xacro file
    # and outputs a plain XML URDF string that robot_state_publisher can read.
    #
    # Command() runs a shell command and returns its output as a string.
    # `xacro /path/to/walle.urdf.xacro` → outputs the processed URDF XML
    robot_description = Command(['xacro ', urdf_xacro_path])

    # ── Define the robot_state_publisher node ─────────────────────────────────
    # robot_state_publisher is a standard ROS2 node (comes pre-installed).
    # It reads the robot_description (URDF) parameter, subscribes to
    # /joint_states, and broadcasts TF transforms for every joint.
    #
    # We pass the URDF as a parameter called 'robot_description'.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',   # Which ROS2 package contains this node
        executable='robot_state_publisher', # Name of the executable in that package
        name='robot_state_publisher',       # What to call this node in the ROS2 graph
        output='screen',                    # Print node output to terminal (for debugging)
        parameters=[{
            # robot_description: the full URDF XML string wrapped in ParameterValue
            # to prevent ROS 2 Jazzy from trying to parse raw XML as YAML
            'robot_description': ParameterValue(robot_description, value_type=str),

            # use_sim_time: False means use wall clock (real time)
            # True would mean use /clock topic (for simulation/rosbag replay)
            'use_sim_time': False,
        }]
    )

    # ── Joint State Publisher ─────────────────────────────────────────────────
    # Joint state publisher publishes default positions (0.0 rad) for continuous
    # and revolute joints (like left_wheel_joint and right_wheel_joint) when no
    # hardware joint state is broadcast, allowing robot_state_publisher to render
    # the wheels and establish all TF transforms without error.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # ── Return the LaunchDescription ──────────────────────────────────────────
    # Everything listed here will be started when you run this launch file.
    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])
