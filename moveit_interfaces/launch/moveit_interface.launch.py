from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from moveit_utils import get_moveit_config

def generate_launch_description():
    
   
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("robot_config"), "config", "moveit.rviz"]
    )
    moveit_config = get_moveit_config()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    output="log",
    arguments=["-d", rviz_config_path],
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        # moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        # moveit_config.joint_limits,
    ],
    )

    static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="static_transform_publisher",
    output="log",
    arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
)

    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("robot_config"), 
         "config", 
         "ros2_controllers.yaml"]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    load_controllers = []
    for controller in [
        "arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]


    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )

