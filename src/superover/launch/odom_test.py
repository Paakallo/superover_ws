from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os

def generate_launch_description():
    # Path to your Nav2 bringup launch file and params
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )]
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(
                get_package_share_directory('superover'),
                'config',
                'nav2_params.yaml'
            )
        }.items()
    )

    return LaunchDescription([
        # Odometry bridge node
        Node(
            package='ned_enu',
            executable='odom_conversion',
            name='ned_enu',
            output='screen',
        ),

        # Static transform: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Include Nav2 bringup
        nav2_launch,

        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'config.rviz'
            )]
        )
    ])
