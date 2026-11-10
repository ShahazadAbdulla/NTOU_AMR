import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('ntou_robot')
    
    controller_config_file = os.path.join(pkg_path, 'config', 'ackermann_controller.yaml')

    # 1. Parse Xacro and pass the yaml parameter path explicitly
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(
        xacro_file,
        mappings={'controllers_yaml': controller_config_file}
    )
    robot_description = {'robot_description': doc.toxml(), 'use_sim_time': True}

    # World file path
    world_path = os.path.join(pkg_path, 'worlds', 'ntou.sdf')
    
    # Path to EKF config
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    
    slam_config_path = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')

    # Node: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Process: Gazebo Harmonic with empty world
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # Node: Spawn Robot from /robot_description
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'ntou_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '1.5708'
        ]
    )

    # Node: ROS-GZ Bridge for Clock, sensors and ground-truth odometry
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Ground-truth pose from the physics engine (diagnostics only).
            '/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Node: Controller Manager - Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Node: Controller Manager - Ackermann Steering Controller Spawner
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen'
    )

    # Node: Standalone odometry scale debugger (wheel vs. ground truth)
    # odom_debugger_node = Node(
    #     package='ntou_robot',
    #     executable='odom_debugger.py',
    #     name='odom_debugger',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )
    
    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path,
                    {'use_sim_time': True}
                    ]
    )
    
    # Standalone RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'sim.rviz')]
    )
    
    # Teleop time stamped twist node
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',
        parameters=[{
            'stamped': True,
            'frame_id': 'base_link',
            'use_sim_time': True
        }],
        remappings=[
            ('cmd_vel', '/ackermann_steering_controller/reference')
        ],
    )
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={
            'slam_params_file': slam_config_path,
            'use_sim_time': 'true'
        }.items()
    )
    
    # Start EKF/SLAM/RViz only after the controller (and therefore the wheel
    # odometry topic) is up, so the filter never runs on stale data.
    load_mobile_system = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ackermann_controller_spawner,
            on_exit=[ekf_node, slam_toolbox_launch, rviz_node, teleop_node]
        )
    )

    return LaunchDescription([
        gazebo_process,
        robot_state_publisher_node,
        spawn_entity_node,
        ros_gz_bridge_node,
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner,
        load_mobile_system
    ])
