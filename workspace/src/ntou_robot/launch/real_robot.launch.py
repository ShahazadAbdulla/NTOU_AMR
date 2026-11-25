"""Real-robot bringup entry point.

This file is intentionally minimal for now: it only starts the Foxglove
WebSocket bridge so the robot can be visualised from a laptop over Wi-Fi.
The rest of the real-robot stack (lidar, base interface, EKF, SLAM, Nav2,
explore) will be added here incrementally per JETSON_PORT_PLAN.md. Do not
reuse the *_sim.launch.py files for hardware: they start Gazebo and hard-code
use_sim_time=True.

Usage (on the robot / Jetson):

    source /opt/ros/<distro>/setup.bash
    source ~/ntou_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ros2 launch ntou_robot real_robot.launch.py

Then connect Foxglove Studio (web or desktop) to:

    ws://<robot-ip>:8765

Arguments:
    port            WebSocket port (default 8765)
    address         Bind address (default 0.0.0.0, i.e. reachable over Wi-Fi)
    use_sim_time    Must stay False on hardware (default False)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    foxglove_share = get_package_share_directory('foxglove_bridge')
    foxglove_launch = os.path.join(
        foxglove_share, 'launch', 'foxglove_bridge_launch.xml')

    port_arg = DeclareLaunchArgument(
        'port', default_value='8765',
        description='Foxglove WebSocket port')
    address_arg = DeclareLaunchArgument(
        'address', default_value='0.0.0.0',
        description='Foxglove bridge bind address (0.0.0.0 for Wi-Fi access)')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (must be false on real hardware)')

    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_launch),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'debug': 'false',
        }.items(),
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        use_sim_time_arg,
        foxglove,
    ])
