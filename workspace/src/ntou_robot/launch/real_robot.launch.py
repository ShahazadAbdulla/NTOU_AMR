import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml',
    )

    venv_python = '/home/jetson/venvs/torch-cu132/bin/python'
    ros_site_packages = '/opt/ros/jazzy/lib/python3.12/site-packages'

    port_arg = DeclareLaunchArgument('port', default_value='8765')
    address_arg = DeclareLaunchArgument('address', default_value='0.0.0.0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    pz_dir_arg = DeclareLaunchArgument(
        'project_zero_dir',
        default_value='/home/jetson/NTOU_AMR/app/ProjectZeRo',
    )

    pz_dir = LaunchConfiguration('project_zero_dir')
    bridge_script = PathJoinSubstitution([pz_dir, 'examples', 'ros2_bridge.py'])

    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(foxglove_launch),
        launch_arguments={
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'debug': 'false',
        }.items(),
    )

    project_zero_bridge = ExecuteProcess(
        cmd=[venv_python, bridge_script],
        additional_env={
            'PYTHONPATH': [
                pz_dir,
                os.pathsep,
                ros_site_packages,
                os.pathsep,
                os.environ.get('PYTHONPATH', ''),
            ],
        },
        output='screen',
        name='project_zero_bridge',
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        use_sim_time_arg,
        pz_dir_arg,
        foxglove,
        project_zero_bridge,
    ])
