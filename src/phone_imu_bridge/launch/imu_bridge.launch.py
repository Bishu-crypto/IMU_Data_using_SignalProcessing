"""Launch all phone_imu_bridge nodes.

Starts the HTTP receiver, Madgwick filter, spectral analysis,
inertial navigation, and WebSocket bridge nodes with parameters
loaded from ``config/params.yaml``.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('phone_imu_bridge')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'http_port', default_value='5555',
            description='TCP port for the HTTP IMU receiver'),

        DeclareLaunchArgument(
            'ws_port', default_value='8765',
            description='TCP port for the WebSocket bridge'),

        # ── Data Acquisition ──────────────────────────────────────
        Node(
            package='phone_imu_bridge',
            executable='http_receiver',
            name='http_receiver_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── Sensor Fusion ─────────────────────────────────────────
        Node(
            package='phone_imu_bridge',
            executable='madgwick_filter',
            name='madgwick_filter_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── DSP Analysis ──────────────────────────────────────────
        Node(
            package='phone_imu_bridge',
            executable='spectral_analysis',
            name='spectral_analysis_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── Inertial Navigation ───────────────────────────────────
        Node(
            package='phone_imu_bridge',
            executable='inertial_nav',
            name='inertial_nav_node',
            parameters=[params_file],
            output='screen',
        ),

        # ── Web Dashboard Bridge ──────────────────────────────────
        Node(
            package='phone_imu_bridge',
            executable='ws_bridge',
            name='ws_bridge_node',
            parameters=[params_file],
            output='screen',
        ),
    ])
