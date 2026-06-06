from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # launches transceiver
    reader_node = Node(
        package='measurement',
        executable='measurement',
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([reader_node])
