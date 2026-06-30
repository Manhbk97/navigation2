from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ──────────────────────────────────────────────────────
    namespace_arg = DeclareLaunchArgument(
        'nav2_namespace',
        default_value='',
        description='Nav2 namespace to monitor (e.g. "apple"). '
                    'Leave empty if Nav2 is running without a namespace.'
    )

    # ZMQ ports — must not overlap with turtle_bt which uses 1666/1667.
    zmq_pub_port_arg = DeclareLaunchArgument(
        'zmq_publisher_port',
        default_value='1670',
        description='Groot ZMQ publisher port (default 1670; turtle_bt uses 1666).'
    )
    zmq_srv_port_arg = DeclareLaunchArgument(
        'zmq_server_port',
        default_value='1671',
        description='Groot ZMQ server port (default 1671; turtle_bt uses 1667).'
    )

    # ── monitoring_nav2 node ──────────────────────────────────────────────────
    monitoring_node = Node(
        package='turtlesim_bt',
        executable='monitoring_nav2',
        name='nav2_bt_monitor',
        output='screen',
        parameters=[{
            'nav2_namespace':    LaunchConfiguration('nav2_namespace'),
            'zmq_publisher_port': LaunchConfiguration('zmq_publisher_port'),
            'zmq_server_port':    LaunchConfiguration('zmq_server_port'),
        }],
    )

    return LaunchDescription([
        namespace_arg,
        zmq_pub_port_arg,
        zmq_srv_port_arg,
        monitoring_node,
    ])
