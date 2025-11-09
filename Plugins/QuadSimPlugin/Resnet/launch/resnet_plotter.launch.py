from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    plotter = Node(
        package='adaptive_resnet_controller',
        executable='resnet_plotter',
        name='resnet_plotter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([plotter])

