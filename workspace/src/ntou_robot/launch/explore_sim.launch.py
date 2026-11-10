import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('ntou_robot')
    nav2_bringup_path = get_package_share_directory('nav2_bringup')

    # Configuration Paths
    controller_config_file = os.path.join(pkg_path, 'config', 'ackermann_controller.yaml')
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    slam_config_path = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')
    nav2_explore_params = os.path.join(pkg_path, 'config', 'nav2_explore_params.yaml')
    explore_params = os.path.join(pkg_path, 'config', 'explore_params.yaml')
    world_path = os.path.join(pkg_path, 'worlds', 'ntou.sdf')
    bt_xml_path = os.path.join(pkg_path, 'behavior_trees', 'ackermann_nav.xml')

    nav2_rviz_file = os.path.join(pkg_path, 'rviz', 'nav2.rviz')
    if not os.path.exists(nav2_rviz_file):
        nav2_rviz_file = os.path.join(nav2_bringup_path, 'rviz', 'nav2_default_view.rviz')

    # Parse Xacro
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'controllers_yaml': controller_config_file})
    robot_description = {'robot_description': doc.toxml(), 'use_sim_time': True}

    # Gazebo & Core Sim Nodes
    gazebo_process = ExecuteProcess(cmd=['gz', 'sim', '-r', world_path], output='screen')

    robot_state_publisher_node = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_description]
    )

    spawn_entity_node = Node(
        package='ros_gz_sim', executable='create',
        output='screen', arguments=['-name', 'ntou_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.5', '-Y', '1.5708']
    )

    ros_gz_bridge_node = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen', parameters=[{'use_sim_time': True}]
    )

    # Hardware Controllers
    joint_state_broadcaster_spawner = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    ackermann_controller_spawner = Node(package='controller_manager', executable='spawner', arguments=['ackermann_steering_controller'], output='screen')

    # EKF Node
    ekf_node = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen', parameters=[ekf_config_path, {'use_sim_time': True}])

    # ---------------------------------------------------------
    # EXPLORATION STACK: SLAM + Nav2 (No AMCL) + Explore Lite
    # ---------------------------------------------------------

    # 1. Dynamic Mapping (SLAM Toolbox)
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={'slam_params_file': slam_config_path, 'use_sim_time': 'true'}.items()
    )

    # 2. Nav2 Core Action Servers
    controller_server_node = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', parameters=[nav2_explore_params, {'use_sim_time': True}],
        remappings=[('cmd_vel', '/ackermann_steering_controller/reference')]
    )

    planner_server_node = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[nav2_explore_params, {'use_sim_time': True}]
    )

    behavior_server_node = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=[nav2_explore_params, {'use_sim_time': True}],
        remappings=[('cmd_vel', '/ackermann_steering_controller/reference')]
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[nav2_explore_params, {
            'use_sim_time': True,
            'default_nav_to_pose_bt_xml': bt_xml_path,
            'default_nav_through_poses_bt_xml': bt_xml_path
        }]
    )

    # 3. Nav2 Lifecycle Manager (Only manages the 4 active navigation nodes)
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen', parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
        }]
    )

    # 4. Autonomous Agent (m-explore-ros2)
    explore_node = Node(
        package='explore_lite', executable='explore', name='explore_node',
        output='screen', parameters=[explore_params]
    )
    
    delayed_explorer = TimerAction(
        period=5.0, # Wait 5 seconds after Nav2 boots
        actions=[explore_node]
    )

    rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', parameters=[{'use_sim_time': True}], arguments=['-d', nav2_rviz_file])

    # Event Handler: Chain execution to avoid DDS discovery timeouts
    load_exploration_system = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ackermann_controller_spawner,
            on_exit=[
                ekf_node,
                slam_toolbox_launch,
                controller_server_node,
                planner_server_node,
                behavior_server_node,
                bt_navigator_node,
                lifecycle_manager_nav,
                delayed_explorer
            ]
        )
    )

    return LaunchDescription([
        gazebo_process,
        robot_state_publisher_node,
        spawn_entity_node,
        ros_gz_bridge_node,
        joint_state_broadcaster_spawner,
        ackermann_controller_spawner,
        load_exploration_system,
        rviz_node
    ])
