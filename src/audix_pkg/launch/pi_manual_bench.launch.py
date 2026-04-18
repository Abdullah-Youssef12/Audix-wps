import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('audix')

    serial_device = LaunchConfiguration('serial_device')
    serial_baud = LaunchConfiguration('serial_baud')
    use_micro_ros_agent = LaunchConfiguration('use_micro_ros_agent')
    use_pi_ir_gpio = LaunchConfiguration('use_pi_ir_gpio')
    use_rviz = LaunchConfiguration('use_rviz')

    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'pi_hardware.launch.py'),
        ]),
        launch_arguments={
            'serial_device': serial_device,
            'serial_baud': serial_baud,
            'use_micro_ros_agent': use_micro_ros_agent,
            'use_pi_ir_gpio': use_pi_ir_gpio,
            'use_start_stop': 'false',
            'use_arena_roamer': 'false',
            'use_rviz': use_rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_device', default_value='/dev/serial0'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),
        DeclareLaunchArgument('use_micro_ros_agent', default_value='true'),
        DeclareLaunchArgument('use_pi_ir_gpio', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        hardware_launch,
    ])
