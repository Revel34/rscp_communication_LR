#!/usr/bin/env python3
"""Top-level bringup: starts the whole communication layer in one command.

Includes the bridge package's launch file and adds the mission orchestrator.
Worker nodes should be added to this file as your team builds them.

Usage:
    # All logs in one terminal (default):
    ros2 launch rscp_mission_orchestrator rover_bringup.launch.py port:=/dev/ttyACM0

    # Each node logs to its own file under ~/.ros/log/ (use 'ros2 launch --screen' to still see output):
    ros2 launch rscp_mission_orchestrator rover_bringup.launch.py port:=/dev/ttyACM0 split_logs:=true

    # For fully separate terminals use launch_rover_comms.sh (tmux) instead.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0",
        description="Serial device path for the RSCP link.")
    baudrate_arg = DeclareLaunchArgument(
        "baudrate", default_value="115200",
        description="Serial baud rate.")
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="Logger level for all nodes.")
    split_logs_arg = DeclareLaunchArgument(
        "split_logs", default_value="false",
        description="If true, write each node's log to a separate file "
                    "under ~/.ros/log/ instead of printing to screen.")

    # --- Include the bridge (translator) launch file -------------------------
    bridge_share = get_package_share_directory("rscp_bridge_node")
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "rscp_bridge.launch.py")
        ),
        launch_arguments={
            "port": LaunchConfiguration("port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    # --- Mission orchestrator — screen output (split_logs=false) -------------
    orchestrator_screen = Node(
        package="rscp_mission_orchestrator",
        executable="mission_orchestrator",
        name="mission_orchestrator",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        condition=UnlessCondition(LaunchConfiguration("split_logs")),
    )

    # --- Mission orchestrator — file output (split_logs=true) ----------------
    orchestrator_log = Node(
        package="rscp_mission_orchestrator",
        executable="mission_orchestrator",
        name="mission_orchestrator",
        output="log",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        condition=IfCondition(LaunchConfiguration("split_logs")),
    )

    # --- Worker nodes (added by your team) -----------------------------------
    # Uncomment and edit as workers become available:
    #
    # arm_node = Node(package="arm_control", executable="arm_control_node",
    #                 name="arm_control_node", output="screen")
    # nav_node = Node(package="navigation", executable="navigation_node",
    #                 name="navigation_node", output="screen")
    # search_node = Node(package="search", executable="search_area_node",
    #                    name="search_area_node", output="screen")
    # explore_node = Node(package="exploration", executable="exploration_node",
    #                     name="exploration_node", output="screen")

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        log_level_arg,
        split_logs_arg,
        bridge_launch,
        orchestrator_screen,
        orchestrator_log,
        # arm_node, nav_node, search_node, explore_node,
    ])
