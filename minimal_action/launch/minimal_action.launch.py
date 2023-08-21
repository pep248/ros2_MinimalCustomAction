# Import the necessary launch libraries
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the action server node
        Node(
            package='minimal_action',
            executable='minimal_action_server',
            output='screen'
        ),

        # Launch the action client node
        Node(
            package='minimal_action',
            executable='minimal_action_client',
            output='screen'
        ),
    ])
