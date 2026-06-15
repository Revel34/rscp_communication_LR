#!/usr/bin/env python3
"""Launch file for the RSCP bridge node (translator) only.

The mission orchestrator lives in its own package and is launched together
with this file via the top-level rover_bringup.launch.py.

    ros2 launch rscp_bridge_node rscp_bridge.launch.py port:=/dev/ttyACM0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/ttyUSB0",
        description="Serial device path for the RSCP link.")
    baudrate_arg = DeclareLaunchArgument(
        "baudrate", default_value="115200", description="Serial baud rate.")
    serial_timeout_arg = DeclareLaunchArgument(
        "serial_timeout", default_value="0.1",
        description="Blocking read timeout in seconds for the reader thread.")
    reconnect_period_arg = DeclareLaunchArgument(
        "reconnect_period_sec", default_value="2.0",
        description="How often to retry opening the serial port if unavailable.")
    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value="info",
        description="Logger level: debug, info, warn, error, or fatal.")

    bridge_node = Node(
        package="rscp_bridge_node",
        executable="rscp_bridge_node",
        name="rscp_bridge_node",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baudrate": LaunchConfiguration("baudrate"),
            "serial_timeout": LaunchConfiguration("serial_timeout"),
            "reconnect_period_sec": LaunchConfiguration("reconnect_period_sec"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription([
        port_arg, baudrate_arg, serial_timeout_arg,
        reconnect_period_arg, log_level_arg, bridge_node,
    ])
