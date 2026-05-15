from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    apple_rendering_arg = DeclareLaunchArgument(
        "apple_rendering",
        default_value="false",
        description="Enable apple silicon rendering for RViz"
    )
    apple_rendering = SetEnvironmentVariable(
        name="LIBGL_ALWAYS_SOFTWARE",
        value="1",
        condition=IfCondition(LaunchConfiguration("apple_rendering"))
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("control"), "launch", "start_robot.launch.py"])
        ),
        launch_arguments={
            "use_mock_hardware": "true",
            "headless_mode":"true",
            "launch_rviz": "false",
        }.items(),
    )

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("moveit_config"), "launch", "move_group.launch.py"])
        )
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("moveit_config"),
                "launch",
                "moveit_rviz.launch.py"
            ])
        )
    )


    return LaunchDescription([
        apple_rendering_arg,
        apple_rendering,
        control_launch,
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_launch,
                         moveit_rviz_launch],
            )
        ),
    ])