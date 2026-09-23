"""MoveIt/RViz for GZ using joint_trajectory_controller, not the scaled JTC."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Trích xuất giá trị chuỗi thực tế của LaunchConfiguration
    ur_type_str = LaunchConfiguration("ur_type").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    controller_yaml = os.path.join(
        get_package_share_directory("ur3_write_m"), "config", "moveit_controllers_gz.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": ur_type_str})
        .trajectory_execution(controller_yaml, moveit_manage_controllers=False)
        .to_moveit_configs()
    )

    wait_description = Node(
        package="ur_robot_driver", 
        executable="wait_for_robot_description", 
        output="screen"
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "publish_robot_description_semantic": True,
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"])],
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
    )

    return [
        wait_description,
        RegisterEventHandler(
            OnProcessExit(target_action=wait_description, on_exit=[move_group, rviz])
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3", choices=["ur3", "ur3e"]),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])