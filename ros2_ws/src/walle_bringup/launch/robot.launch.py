"""
robot.launch.py
===============
THE MASTER LAUNCH FILE — starts your entire robot system with one command.

USAGE:
  ros2 launch walle_bringup robot.launch.py

WHAT THIS STARTS:
  1. micro-ROS Agent        → UDP bridge between ESP32 and ROS2
  2. robot_state_publisher  → URDF + TF transforms for RViz
  3. monitor_node           → Live telemetry dashboard

OPTIONAL (launch separately):
  - ros2 run plotjuggler_ros plotjuggler    (real-time graphs)
  - ros2 launch walle_bringup rviz.launch.py  (3D visualization)
  - ros2 run teleop_twist_keyboard teleop_twist_keyboard (manual control)
  - ros2 bag record -a -o my_session       (record everything)

ARCHITECTURE OF THIS FILE:
  Launch files can include OTHER launch files (like nesting functions).
  This file includes description.launch.py from walle_description package.
  This keeps each package focused on its own responsibility.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,   # Declare parameters the user can pass at launch time
    IncludeLaunchDescription, # Include another launch file
    ExecuteProcess,           # Run an arbitrary shell command/process
    LogInfo,                  # Print a message during launch
)
from launch.conditions import IfCondition  # Conditionally start nodes
from launch.substitutions import (
    LaunchConfiguration,
    FindExecutable,           # Find an executable on PATH
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ── Launch Arguments ───────────────────────────────────────────────────────
    # LaunchArguments let users customize the launch from command line:
    # ros2 launch walle_bringup robot.launch.py agent_port:=9999
    #
    # DeclareLaunchArgument: name, default_value, description

    # The UDP port the micro-ROS agent listens on
    # Must match CONFIG_MICRO_ROS_AGENT_PORT in ESP32 firmware sdkconfig
    agent_port_arg = DeclareLaunchArgument(
        'agent_port',
        default_value='8888',
        description='UDP port for micro-ROS agent'
    )

    # Whether to launch the monitor node (default: yes)
    monitor_arg = DeclareLaunchArgument(
        'launch_monitor',
        default_value='true',
        description='Whether to launch the monitor node'
    )

    # ── micro-ROS Agent ────────────────────────────────────────────────────────
    # The micro-ROS Agent is the critical bridge between ESP32 and ROS2.
    # Without it, the ESP32's micro-ROS node has nowhere to connect.
    #
    # Command: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
    #
    # ExecuteProcess runs a shell command as part of the launch.
    # We use it here because micro_ros_agent is its own executable.
    micro_ros_agent = ExecuteProcess(
        cmd=[
            FindExecutable(name='ros2'),
            'run', 'micro_ros_agent', 'micro_ros_agent',
            'udp4',                          # Transport: UDP IPv4
            '--port', LaunchConfiguration('agent_port'),  # Port from argument
        ],
        output='screen',
        name='micro_ros_agent',
    )

    # ── Include the description launch file ───────────────────────────────────
    # Instead of duplicating the robot_state_publisher setup here,
    # we INCLUDE the description launch file from walle_description package.
    # This is the ROS2 equivalent of calling a function from another module.
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('walle_description'),
                'launch',
                'description.launch.py'
            ])
        ])
    )

    # ── Monitor Node ───────────────────────────────────────────────────────────
    # The live telemetry dashboard from walle_control package.
    # Only started if launch_monitor argument is 'true'.
    monitor_node = Node(
        package='walle_control',
        executable='monitor_node',
        name='walle_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_monitor'))
        # IfCondition: only start this node if the argument is 'true'
    )

    # ── IMU TF Broadcaster ─────────────────────────────────────────────────────
    # Subscribes to /imu/data from ESP32 and broadcasts odom -> base_link TF
    # transform, causing the 3D model in RViz to tilt dynamically in real time.
    imu_tf_broadcaster = Node(
        package='walle_control',
        executable='imu_tf_broadcaster',
        name='imu_tf_broadcaster',
        output='screen'
    )

    # ── LSA RViz Visualizer ────────────────────────────────────────────────────
    # Subscribes to /line_sensors and publishes interactive 3D markers in RViz
    # showing real-time sensor intensities and line tracking target.
    lsa_visualizer_node = Node(
        package='walle_control',
        executable='lsa_visualizer_node',
        name='lsa_visualizer_node',
        output='screen'
    )

    # ── Startup Info Messages ──────────────────────────────────────────────────
    # LogInfo prints messages to the terminal during launch.
    # Useful to tell the user what's happening.
    startup_msg = LogInfo(msg=[
        '\n',
        '=' * 50, '\n',
        '  Wall-E Robot System Starting...\n',
        '  micro-ROS Agent: UDP port ',
        LaunchConfiguration('agent_port'), '\n',
        '  Now power on your ESP32!\n',
        '  Run: ros2 node list  (should show /walle_esp32)\n',
        '=' * 50,
    ])

    return LaunchDescription([
        # Declare arguments first (order matters here!)
        agent_port_arg,
        monitor_arg,
        # Then start everything
        startup_msg,
        micro_ros_agent,
        description_launch,
        imu_tf_broadcaster,
        lsa_visualizer_node,
        monitor_node,
    ])
