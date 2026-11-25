from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='STM32 USB virtual COM port'),
        Node(
            package='ntou_hardware_interface',
            executable='ntou_hardware_interface_node',
            name='ntou_hardware_interface',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': 115200,
                'max_linear_x_mps': 0.5,
                'max_pwm_percent': 50,
                'max_angular_z_rad_s': 2.0,
                'max_steering_degrees': 40,
                'cmd_vel_timeout_sec': 0.5,
            }],
        ),
    ])
