import sys
import os
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_utils import get_moveit_config

def generate_launch_description():
 
    use_cartesian = LaunchConfiguration("use_Cartesian")
    moveit_config = get_moveit_config()
     
    target_yaml = PathJoinSubstitution([
        FindPackageShare("moveit_interfaces"),
        "config",
        "target_pose.yaml"
    ])

    cartesian_yaml = PathJoinSubstitution([
        FindPackageShare("moveit_interfaces"),
        "config",
        "cartesian_waypoints.yaml"
    ])

    return LaunchDescription(
        [

            DeclareLaunchArgument(
                "use_Cartesian",
                default_value="false",
                description="Use Cartesian path planning"
            ),
            Node(
                package="moveit_interfaces",
                executable="moveit_interface_main",
                output="screen",
                condition= UnlessCondition(use_cartesian),
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    target_yaml,
                    
                ],
            ),
            Node(
                package="moveit_interfaces",
                executable="moveit_cartesian",
                output="screen",
                condition=IfCondition(use_cartesian),
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    cartesian_yaml,
                ],
            ),

        ]
    )


