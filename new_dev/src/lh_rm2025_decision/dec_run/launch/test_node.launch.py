from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    test_node = Node(
        package='dec_run',
        executable='test_node',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([test_node])
