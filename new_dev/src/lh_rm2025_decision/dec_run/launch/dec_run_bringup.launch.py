import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dec_run'), 'config', 'dec_run_config.yaml')

    dec_run_node = Node(
        package='dec_run',
        executable='dec_run_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    

    return LaunchDescription([dec_run_node])
