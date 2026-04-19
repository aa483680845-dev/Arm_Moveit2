from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_utils import get_moveit_config

def generate_launch_description():

    moveit_config = get_moveit_config()

    moveit_interface_node = Node(
    package="moveit_interfaces",
    executable="moveit_interface_main",
    output="screen",
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
    ],
    )

    return LaunchDescription(
        [
            moveit_interface_node,
        ]
    )


