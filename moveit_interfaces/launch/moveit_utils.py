from moveit_configs_utils import MoveItConfigsBuilder

def get_moveit_config():
    return(
        MoveItConfigsBuilder("robot_1",package_name="robot_config")
        .robot_description(file_path = "config/robot_1.urdf.xacro")
        .trajectory_execution(file_path = "config/moveit_controllers.yaml")
        .to_moveit_configs()
    )