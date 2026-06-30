#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bcr_pkg_share = get_package_share_directory("bcr_bot")
    mogi_pkg_share = get_package_share_directory("bme_ros2_navigation")
    mogi_pkg_parent = os.path.dirname(mogi_pkg_share)
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world_file = LaunchConfiguration("world_file")
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    position_z = LaunchConfiguration("position_z")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")

    urdf_file = os.path.join(mogi_pkg_share, "urdf", "mogi_bot.urdf")
    gz_bridge_config = os.path.join(mogi_pkg_share, "config", "gz_bridge.yaml")

    # Gazebo needs the package parent in its resource path so package:// mesh URIs resolve.
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            TextSubstitution(text=os.pathsep),
            TextSubstitution(text=mogi_pkg_parent),
        ],
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"]),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro", " ", urdf_file]),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_mogi_bot",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "mogi_bot",
            "-allow_renaming",
            "true",
            "-x",
            position_x,
            "-y",
            position_y,
            "-z",
            position_z,
            "-Y",
            orientation_yaw,
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="mogi_gz_bridge",
        output="screen",
        arguments=["--ros-args", "-p", f"config_file:={gz_bridge_config}"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.path.join(bcr_pkg_share, "worlds"),
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.path.join(bcr_pkg_share, "models"),
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=os.path.join(bcr_pkg_share, "worlds", "maze_ignition.world"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("position_x", default_value="0.0"),
            DeclareLaunchArgument("position_y", default_value="0.0"),
            DeclareLaunchArgument("position_z", default_value="0.5"),
            DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
            gz_resource_path,
            world_launch,
            robot_state_publisher,
            gz_spawn_entity,
            gz_bridge,
        ]
    )
