import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('beginner_tutorials'), 'rviz', 'turtle_status.rviz')

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtle_sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='turtle_ctrl',
            output='screen',
            prefix='xterm -e'
        ),
        Node(
            package='beginner_tutorials',
            executable='turtle_status.py',
            name='turtle_status'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_dir]
        )
    ])