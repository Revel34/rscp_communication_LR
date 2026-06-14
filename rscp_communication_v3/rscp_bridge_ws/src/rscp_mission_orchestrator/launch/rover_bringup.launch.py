#!/usr/bin/env python3
"""Top-level bringup: starts the bridge and mission orchestrator together.

Worker nodes should be added to this file as your team builds them.

Usage:
    # Default — all logs in one terminal:
    ros2 launch rscp_mission_orchestrator rover_bringup.launch.py port:=/dev/ttyACM0

    # Custom serial settings:
    ros2 launch rscp_mission_orchestrator rover_bringup.launch.py \
        port:=/dev/ttyACM0 baudrate:=115200

    # Custom watchdog timeouts (seconds; 0 disables):
    ros2 launch rscp_mission_orchestrator rover_bringup.launch.py \
        search_area_timeout_sec:=120.0 \
        navigate_to_gps_timeout_sec:=180.0 \
        explore_timeout_sec:=300.0

    # Debug logging:
    ros2 launch rscp_mission_orchestrator rover_bringup.launch.py log_level:=debug
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---- Serial / bridge arguments ------------------------------------------
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0",
        description="Serial device path for the RSCP link.")
    baudrate_arg = DeclareLaunchArgument(
        "baudrate", default_value="115200",
        description="Serial baud rate.")

    # ---- Watchdog timeout arguments (orchestrator) --------------------------
    search_timeout_arg = DeclareLaunchArgument(
        "search_area_timeout_sec", default_value="120.0",
        description="Seconds before the search_area watchdog fires. 0 = disabled.")
    nav_timeout_arg = DeclareLaunchArgument(
        "navigate_to_gps_timeout_sec", default_value="180.0",
        description="Seconds before the navigate_to_gps watchdog fires. 0 = disabled.")
    explore_timeout_arg = DeclareLaunchArgument(
        "explore_timeout_sec", default_value="300.0",
        description="Seconds before the explore watchdog fires. 0 = disabled.")
    arm_timeout_arg = DeclareLaunchArgument(
        "arm_service_timeout_sec", default_value="5.0",
        description="Seconds to wait for the arm_disarm service to become available.")

    # ---- Logging ------------------------------------------------------------
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="Logger level for all nodes: debug, info, warn, error, fatal.")

    # --- Include the bridge (translator) launch file -------------------------
    bridge_share = get_package_share_directory("rscp_bridge_node")
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "rscp_bridge.launch.py")
        ),
        launch_arguments={
            "port":      LaunchConfiguration("port"),
            "baudrate":  LaunchConfiguration("baudrate"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    # --- Mission orchestrator ------------------------------------------------
    orchestrator_node = Node(
        package="rscp_mission_orchestrator",
        executable="mission_orchestrator",
        name="mission_orchestrator",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "search_area_timeout_sec":    LaunchConfiguration("search_area_timeout_sec"),
            "navigate_to_gps_timeout_sec": LaunchConfiguration("navigate_to_gps_timeout_sec"),
            "explore_timeout_sec":         LaunchConfiguration("explore_timeout_sec"),
            "arm_service_timeout_sec":     LaunchConfiguration("arm_service_timeout_sec"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # --- Worker nodes (added by your team) -----------------------------------
    # Uncomment and fill in package/executable names as workers become available:
    #
    # arm_node = Node(package="arm_control", executable="arm_control_node",
    #                 name="arm_control_node", output="screen", emulate_tty=True)
    # nav_node = Node(package="navigation",  executable="navigation_node",
    #                 name="navigation_node",  output="screen", emulate_tty=True)
    # search_node = Node(package="search",   executable="search_area_node",
    #                    name="search_area_node", output="screen", emulate_tty=True)
    # explore_node = Node(package="exploration", executable="exploration_node",
    #                     name="exploration_node", output="screen", emulate_tty=True)

    return LaunchDescription([
        # Arguments
        port_arg,
        baudrate_arg,
        search_timeout_arg,
        nav_timeout_arg,
        explore_timeout_arg,
        arm_timeout_arg,
        log_level_arg,
        # Nodes
        bridge_launch,
        orchestrator_node,
        # arm_node, nav_node, search_node, explore_node,
    ])
