from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_projector',
            executable='path_projector_node',
            name='path_projector',
            namespace='apple',
            output='screen',
            parameters=[{
                # ── Frames ──
                'map_frame':          'map',

                # ── Drawing ──
                'path_step':          5,       # sample every 5th pose
                'dot_radius':         4,       # pixels
                'line_thickness':     2,
                'dot_color_bgr':      [0, 255, 0],
                'line_color_bgr':     [0, 200, 0],

                # ── TF ──
                'tf_timeout_sec':     0.1,
            }],
        ),
    ])