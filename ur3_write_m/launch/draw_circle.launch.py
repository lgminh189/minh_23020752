"""Start UR3 + MoveIt and execute a separate 360-degree circle trajectory."""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from moveit_configs_utils.moveit_configs_builder import load_yaml


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    execute = LaunchConfiguration("execute")
    kinematics_config = {
        "robot_description_kinematics": load_yaml(
            Path(
                get_package_share_directory("ur_moveit_config")
            ) / "config" / "kinematics.yaml"
        )
    }
    params = ParameterFile(
        PathJoinSubstitution([FindPackageShare("ur3_write_m"), "config", "circle.yaml"]),
        allow_substs=True,
    )
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_simulation_gz"), "launch", "ur_sim_control.launch.py"
        ])),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": "false",
            "initial_joint_controller": "joint_trajectory_controller",
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur3_write_m"), "launch", "ur3_moveit_gz.launch.py"
        ])),
        launch_arguments={"ur_type": ur_type, "use_sim_time": "true", "launch_rviz": "true"}.items(),
    )
    circle = Node(
        package="ur3_write_m", executable="draw_circle_node", name="draw_circle_node", output="screen",
        parameters=[params, kinematics_config, {"execute": execute, "use_sim_time": True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3", choices=["ur3", "ur3e"]),
        DeclareLaunchArgument("execute", default_value="true"),
        simulation, moveit,
        TimerAction(period=60.0, actions=[circle]),
    ])
