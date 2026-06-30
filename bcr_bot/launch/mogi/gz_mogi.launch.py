#!/usr/bin/python3

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bcr_bot_path = get_package_share_directory("bcr_bot")
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    world_file = LaunchConfiguration(
        "world_file",
        default=join(bcr_bot_path, "worlds", "maze_ignition.world"),
    )
    position_x = LaunchConfiguration("position_x", default="0.0")
    position_y = LaunchConfiguration("position_y", default="0.0")
    position_z = LaunchConfiguration("position_z", default="0.5")
    orientation_yaw = LaunchConfiguration("orientation_yaw", default="0.0")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    spawn_mogi_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(bcr_bot_path, "launch", "mogi", "mogi_bot_gz_spawn.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "position_x": position_x,
            "position_y": position_y,
            "position_z": position_z,
            "orientation_yaw": orientation_yaw,
        }.items(),
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=join(bcr_bot_path, "worlds"),
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=join(bcr_bot_path, "models"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
            DeclareLaunchArgument("world_file", default_value=world_file),
            DeclareLaunchArgument("position_x", default_value=position_x),
            DeclareLaunchArgument("position_y", default_value=position_y),
            DeclareLaunchArgument("position_z", default_value=position_z),
            DeclareLaunchArgument("orientation_yaw", default_value=orientation_yaw),
            gz_sim,
            spawn_mogi_bot_node,
        ]
    )
