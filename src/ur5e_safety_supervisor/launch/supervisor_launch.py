from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur5e_safety_supervisor',
            executable='supervisor',
            name='safety_supervisor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])