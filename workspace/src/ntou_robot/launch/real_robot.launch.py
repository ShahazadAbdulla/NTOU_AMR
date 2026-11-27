import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ntou = get_package_share_directory('ntou_robot')
    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml',
    )

    hw_launch = os.path.join(
        get_package_share_directory('ntou_hardware_interface'),
        'launch',
        'hardware_interface.launch.py',
    )

    urdf_path = os.path.join(pkg_ntou, 'urdf', 'robot.urdf.xacro')
    lidar_config = os.path.join(pkg_ntou, 'config', 'lslidar_m10p.yaml')

    venv_python = '/home/jetson/venvs/torch-cu132/bin/python'
    ros_site_packages = '/opt/ros/jazzy/lib/python3.12/site-packages'

    port_arg = DeclareLaunchArgument('port', default_value='8765')
    address_arg = DeclareLaunchArgument('address', default_value='0.0.0.0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    pz_dir_arg = DeclareLaunchArgument(
        'project_zero_dir',
        default_value='/home/jetson/NTOU_AMR/app/ProjectZeRo',
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ntou_stm32',
    )

    teleop_port_arg = DeclareLaunchArgument('teleop_port', default_value='8080')
    teleop_pin_arg = DeclareLaunchArgument('teleop_pin', default_value='101010')

    pz_dir = LaunchConfiguration('project_zero_dir')
    bridge_script = PathJoinSubstitution([pz_dir, 'examples', 'ros2_bridge.py'])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )

    ekf_config = os.path.join(
        get_package_share_directory('ntou_robot'),
        'config',
        'ekf_real.yaml'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
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

    lidar_driver = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        namespace='x10',
        parameters=[lidar_config],
        remappings=[
            ('scan', '/scan'),
            ('lslidar_point_cloud', '/lslidar_point_cloud'),
        ],
        output='screen',
    )

    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hw_launch),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
        }.items(),
    )

    web_teleop = Node(
        package='ntou_robot',
        executable='web_teleop.py',
        name='ntou_web_teleop',
        output='screen',
        additional_env={'PYTHONUNBUFFERED': '1'},
        arguments=[
            '--http-port', LaunchConfiguration('teleop_port'),
            '--pin', LaunchConfiguration('teleop_pin'),
        ],
    )
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )
    
    return LaunchDescription([
        port_arg,
        address_arg,
        use_sim_time_arg,
        pz_dir_arg,
        serial_port_arg,
        teleop_port_arg,
        teleop_pin_arg,
        robot_state_publisher,
        foxglove,
        project_zero_bridge,
        lidar_driver,
        hardware_interface,
        web_teleop,
        ekf_node,
    ])
