"""Launch SLAM Toolbox, Nav2, and the delivery mission manager."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Create the online mapping and navigation nodes."""
    package_share = Path(get_package_share_directory('delivery_robot'))
    nav2_params = package_share / 'config' / 'nav2.yaml'
    slam_params = package_share / 'config' / 'slam.yaml'
    mission_params = package_share / 'config' / 'navigation.yaml'
    slam_launch = (
        Path(get_package_share_directory('slam_toolbox'))
        / 'launch' / 'online_async_launch.py'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(slam_launch)),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': str(slam_params),
        }.items(),
    )

    navigation_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[str(nav2_params)],
            remappings=[
                ('cmd_vel', '/delivery_robot/cmd_vel'),
                ('odom', '/delivery_robot/odom'),
            ],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[str(nav2_params)],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[str(nav2_params)],
            remappings=[('cmd_vel', '/delivery_robot/cmd_vel')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[str(nav2_params)],
        ),
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }
        ],
    )

    mission_manager = Node(
        package='delivery_robot',
        executable='delivery_robot_node',
        name='delivery_robot_node',
        output='screen',
        parameters=[str(mission_params)],
    )

    return LaunchDescription(
        [slam_toolbox]
        + navigation_nodes
        + [lifecycle_manager, mission_manager]
    )
