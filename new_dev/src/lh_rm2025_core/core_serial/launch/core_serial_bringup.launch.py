import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('core_serial'), 'config', 'core_serial_config.yaml')

    core_serial_node = Node(
        package='core_serial',
        executable='core_serial_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([core_serial_node])
