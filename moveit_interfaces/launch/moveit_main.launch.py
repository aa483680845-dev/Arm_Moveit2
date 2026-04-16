import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    robot_description_path = os.path.join(
        get_package_share_directory("robot_config"),
        "config",
        "robot_1.urdf.xacro",
    )
    tarjectory_execution_path = os.path.join(
        get_package_share_directory("robot_config"),
        "config",
        "moveit_controllers.yaml",
    )   

    moveit_config = (
        MoveItConfigsBuilder("robot_1",package_name="robot_config")
        .robot_description(file_path = robot_description_path)
        .trajectory_execution(file_path = tarjectory_execution_path)
        .to_moveit_configs()
    )

    moveit_interface_node = Node(
    package="moveit_interfaces",
    executable="moveit_cartesian",
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


