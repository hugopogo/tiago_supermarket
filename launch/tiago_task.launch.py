import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tiago_supermarket',
            executable='motion_control',
            name='motion_control_node',
            output='screen'
        ),
        Node(
            package='tiago_supermarket',
            executable='arm_control',
            name='arm_control_node',
            output='screen'
        ),
        Node(
            package='tiago_supermarket',
            executable='object_detection',
            name='object_detection_node',
            output='screen'
        ),
        Node(
            package='tiago_supermarket',
            executable='aruco',
            name='aruco_node',
            output='screen'
        ),
    ])
