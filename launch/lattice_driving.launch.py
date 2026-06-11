import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_beginner_tutorials = get_package_share_directory('beginner_tutorials')
    rviz_config_file = os.path.join(pkg_beginner_tutorials, 'rviz', 'lattice_driving_rviz.rviz')

    return LaunchDescription([
        Node(
            package='beginner_tutorials',
            executable='global_path_pub.py',
            name='read_path_pub'
        ),
        Node(
            package='beginner_tutorials',
            executable='gpsimu_parser.py',
            name='GPS_IMU_parser'
        ),
        Node(
            package='beginner_tutorials',
            executable='local_path_pub.py',
            name='path_pub'
        ),
        Node(
            package='beginner_tutorials',
            executable='lattice_planner.py',
            name='lattice_planner',
            output='screen'
        ),
        Node(
            package='beginner_tutorials',
            executable='pure_pursuit_pid_velocity_planning.py',
            name='pure_pursuit_pid_velocity_planning',
            output='screen'
        ),
        Node(
            package='beginner_tutorials',
            executable='mgeo_pub.py',
            name='mgeo_pub'
        ),
        Node(
            package='beginner_tutorials',
            executable='tf_pub.py',
            name='tf'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_file]
        )
    ])