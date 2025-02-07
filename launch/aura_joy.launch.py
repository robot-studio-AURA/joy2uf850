#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    aura_joy_node = Node(
        package='joy2uf850',
        executable='aura_joy',
        output='screen',
        name='aura_joy',
    )
    # # Start the joystick driver node
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     output='screen',
    #     parameters=[{'autorepeat_rate': 20.0}]
    # ),

    uf850_fake = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_moveit_servo'),  # Replace with the package name containing the child launch file
            'launch',
            'uf850_moveit_servo_fake.launch.py'  # Replace with the name of the child launch file
        ])),
        # Optionally pass arguments to the included launch file
        launch_arguments={
            'joystick_type': '1'
        }.items()
    )
    
    return LaunchDescription([
        uf850_fake,
        # joy_node,
        aura_joy_node,
    ])