from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the aura_joy node (your joystick-to-servo node)
        Node(
            package='joy2uf850',  # Replace with your package name
            executable='aura_joy',  # Name of your Python script (no .py extension)
            output='screen',
            name='aura_joy',
        ),
        
        # Optionally, launch the joy_node for joystick input
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            name='joy_node',
        )
    ])