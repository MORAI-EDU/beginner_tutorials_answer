from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Talker 노드 실행
        Node(
            package='beginner_tutorials',
            executable='talker.py',
            name='talker'
        ),
        
        # Listener 노드 실행 (output="screen" 적용)
        Node(
            package='beginner_tutorials',
            executable='listener.py',
            name='listener',
            output='screen'
        )
    ])