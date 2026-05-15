from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot_cell", package_name="moveit_config")
        .robot_description(file_path="config/robot_cell.urdf")
        .robot_description_semantic(file_path="config/robot_cell.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )
    return generate_moveit_rviz_launch(moveit_config)
