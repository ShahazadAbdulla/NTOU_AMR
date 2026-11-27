import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_path = get_package_share_directory('ntou_robot')
    slam_config_path = os.path.join(pkg_path, 'config', 'slam_toolbox_real.yaml')

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={
            'slam_params_file': slam_config_path,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([
        slam_toolbox_launch,
    ])
