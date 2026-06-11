from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
    ld =  LaunchDescription()

    start_core_serial= Node(
        package='core_serial',
        executable='core_serial_node',
    )

    start_decision= Node(
        package='lh_rm2024_decision',
        executable='dec_auto_node',
        output='screen'
    )

    # start_vision= Node(
    #     package='lh_rm2024_vision',
    #     executable='vision_node',
    # )

    # start_core_watchdog= Node(
    #     package='core_watchdog',
    #     executable='core_watchdog_node',
    # )

    ld.add_action(start_core_serial)
    ld.add_action(start_decision)
    #ld.add_action(start_vision)
    # ld.add_action(start_core_watchdog)
    return ld
