"""Bring up UR3 in Gazebo/GZ with MoveIt/RViz, then execute the M trajectory."""

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import PathJoinSubstitution
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
        PathJoinSubstitution([FindPackageShare("ur3_write_m"), "config", "write_m.yaml"]),
        allow_substs=True,
    )
    ur_simulation = IncludeLaunchDescription(
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
    writer = Node(
        package="ur3_write_m",
        executable="write_m_node",
        name="write_m_node",
        output="screen",
        # The embedded MoveIt instance runs from the simulator clock as well.
        parameters=[params, kinematics_config, {"execute": execute, "use_sim_time": True}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3",
                              choices=["ur3", "ur3e"],
                              description="Robot model to simulate"),
        DeclareLaunchArgument("execute", default_value="true"),
        ur_simulation,
        moveit,
        # On GZ Jazzy the ros2_control plugin can need 40+ seconds before it
        # configures both controllers and begins publishing /joint_states.
        TimerAction(period=60.0, actions=[writer]),
    ])
