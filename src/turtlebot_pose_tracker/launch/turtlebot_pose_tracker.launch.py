from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_pose_tracker',
            executable='turtlebot_pose_tracker',
            name='turtlebot_pose_tracker',
            output='screen',
            emulate_tty=True,
        )
    ])