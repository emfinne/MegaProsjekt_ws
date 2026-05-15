import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot_cell", package_name="moveit_config")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(
            "ompl",
            ["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"],
        )
        .moveit_cpp(
            os.path.join(
                get_package_share_directory("motion_pipeline"),
                "config",
                "parallel_planning_moveit.yaml",  # the request-presets YAML
            )
        )
        .to_moveit_configs()
    )

    ompl_planning_pipeline_config = {
        "ompl_2": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        }
    }
    ompl_planning_yaml = load_yaml("moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl_2"].update(ompl_planning_yaml)

    parallel_planning_node = Node(
        name="parallel_planning",
        package="motion_pipeline",
        executable="motion_pipeline_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ompl_planning_pipeline_config,
        ],
    )

    return LaunchDescription([parallel_planning_node])