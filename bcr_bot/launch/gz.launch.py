#!/usr/bin/python3

from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    bcr_bot_path = get_package_share_directory("bcr_bot")
    world_file = LaunchConfiguration("world_file", default = join(bcr_bot_path, "worlds", "shopping_mall_ignition.sdf"))
    gz_sim_share = get_package_share_directory("ros_gz_sim")

    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    position_x = LaunchConfiguration("position_x", default="0.0")
    position_y = LaunchConfiguration("position_y", default="0.0")
    orientation_yaw = LaunchConfiguration("orientation_yaw", default="0.0")
    odometry_source = LaunchConfiguration("odometry_source", default="world")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : PythonExpression(["'", world_file, " -r'"])
        }.items()
    )

    spawn_bcr_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bcr_bot_path, "launch", "bcr_bot_gz_spawn.launch.py")),
        launch_arguments={
            "camera_enabled": camera_enabled,
            "stereo_camera_enabled": stereo_camera_enabled,
            "two_d_lidar_enabled": two_d_lidar_enabled,
            "position_x": position_x,
            "position_y": position_y,
            "orientation_yaw": orientation_yaw,
            "odometry_source": odometry_source,
        }.items()
    )

    return LaunchDescription([

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(bcr_bot_path, "worlds")),

        AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=join(bcr_bot_path, "models")),

        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        DeclareLaunchArgument("camera_enabled", default_value=camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value=stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value=two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value=position_x),
        DeclareLaunchArgument("position_y", default_value=position_y),
        DeclareLaunchArgument("orientation_yaw", default_value=orientation_yaw),
        DeclareLaunchArgument("odometry_source", default_value=odometry_source),

        gz_sim, spawn_bcr_bot_node
    ])
