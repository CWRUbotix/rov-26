from launch.launch_description import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # dual_cam = Node(
    #     package='luxonis_cam',
    #     executable='luxonis_cam_driver',
    #     name='luxonis_cam',
    #     exec_name='luxonis_cam',
    #     emulate_tty=True,
    #     output='screen',
    # )

    dual_cam = Node(
        package='luxonis_cam',
        executable='measurement_cam_driver',
        name='measurement_cam',
        exec_name='measurement_cam',
        emulate_tty=True,
        output='screen',
    )

    return LaunchDescription([dual_cam])
