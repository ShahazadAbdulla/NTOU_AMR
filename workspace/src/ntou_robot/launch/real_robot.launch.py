"""Real-robot bringup entry point.

This file starts the Foxglove WebSocket bridge (for laptop visualisation over
Wi-Fi) and the Project ZeRo Zenoh/ROS bridge (phone telemetry -> ROS 2
topics). The rest of the real-robot stack (lidar, base interface, EKF, SLAM,
Nav2, explore) will be added here incrementally per JETSON_PORT_PLAN.md.
Do not reuse the *_sim.launch.py files for hardware: they start Gazebo and
hard-code use_sim_time=True.

Usage (on the robot / Jetson):

    source /opt/ros/<distro>/setup.bash
    source ~/ntou_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    ros2 launch ntou_robot real_robot.launch.py

Then connect Foxglove Studio (web or desktop) to:

    ws://<robot-ip>:8765

Arguments:
    port            Foxglove WebSocket port (default 8765)
    address         Bind address (default 0.0.0.0, i.e. reachable over Wi-Fi)
    use_sim_time    Must stay False on hardware (default False)
    project_zero_dir  Path to app/ProjectZeRo (holds examples/ros2_bridge.py)
    bridge_in_xterm   Run the Zenoh/ROS bridge in its own xterm window (default false)
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


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

    # Resolve app/ProjectZeRo whether this launch file is run from the source
    # tree (workspace/src/ntou_robot/launch) or the installed tree
    # (workspace/install/ntou_robot/share/ntou_robot/launch). Pick the first
    # candidate that actually contains examples/ros2_bridge.py.
    _here = os.path.dirname(os.path.abspath(__file__))
    _candidates = [
        os.path.abspath(os.path.join(_here, '..', '..', '..', '..', 'app', 'ProjectZeRo')),
        os.path.abspath(os.path.join(_here, '..', '..', '..', '..', '..', 'app', 'ProjectZeRo')),
    ]
    default_pz = next(
        (c for c in _candidates
         if os.path.isfile(os.path.join(c, 'examples', 'ros2_bridge.py'))),
        _candidates[0])
    pz_dir_arg = DeclareLaunchArgument(
        'project_zero_dir', default_value=default_pz,
        description='Path to app/ProjectZeRo (holds examples/ros2_bridge.py)')
    bridge_xterm_arg = DeclareLaunchArgument(
        'bridge_in_xterm', default_value='false',
        description='Run the Zenoh/ROS bridge in its own xterm window instead of inline')

    pz_dir = LaunchConfiguration('project_zero_dir')
    bridge_script = PathJoinSubstitution([pz_dir, 'examples', 'ros2_bridge.py'])

    # Make project_zero_sdk importable by the bridge process. Use the resolved
    # (possibly overridden) project_zero_dir so overrides keep working.
    set_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=[
            LaunchConfiguration('project_zero_dir'),
            os.pathsep,
            os.environ.get('PYTHONPATH', ''),
        ],
    )

    bridge_inline = ExecuteProcess(
        cmd=[sys.executable, bridge_script],
        output='screen',
        name='project_zero_bridge',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('bridge_in_xterm'),
                              "' == 'false'"])),
    )
    bridge_xterm = ExecuteProcess(
        cmd=['xterm', '-title', 'ProjectZeRo Bridge', '-e',
             sys.executable, bridge_script],
        output='screen',
        name='project_zero_bridge_xterm',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('bridge_in_xterm'),
                              "' == 'true'"])),
    )

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
        pz_dir_arg,
        bridge_xterm_arg,
        set_pythonpath,
        foxglove,
        bridge_inline,
        bridge_xterm,
    ])
