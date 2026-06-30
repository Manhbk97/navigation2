import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_bcr = get_package_share_directory('bcr_bot')
    mogi_pkg_share = get_package_share_directory("bme_ros2_navigation")


    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_bcr, 'config', 'map', 'maze.yaml')
    )
    nav2_params = os.path.join(mogi_pkg_share, 'config', 'navigation.yaml')

    configured_nav2_params = ReplaceString(
        source_file=nav2_params,
        replacements={
            '/bcr_bot/odom': '/odom',
            '/bcr_bot/scan': '/scan',
            '/bcr_bot/cmd_vel': '/cmd_vel',
        },
    )

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_file,
            'params_file': configured_nav2_params,
            'package_path': pkg_bcr,
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-d',
            os.path.join(
                mogi_pkg_share,
                'rviz',
                'navigation.rviz'
            )
        ]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(pkg_bcr, 'config','map', 'maze.yaml')}],
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    remapper_node = Node(
        package='bcr_bot',
        executable='remapper.py',
        name='remapper',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_bcr, 'config', 'map', 'maze.yaml'),
            description='Absolute path to the occupancy grid yaml map file',
        )
    )
    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(static_transform_publisher_node)


    return ld
