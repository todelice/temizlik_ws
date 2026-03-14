#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_model = LaunchConfiguration('tb3_model')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    coverage_params_file = LaunchConfiguration('coverage_params_file')
    bridge_params_file = LaunchConfiguration('bridge_params_file')
    autostart_coverage = LaunchConfiguration('autostart_coverage')

    default_nav2_params_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'param',
        PythonExpression([
            "'waffle_pi.yaml' if '", tb3_model,
            "' == 'waffle_pi' else ('waffle.yaml' if '", tb3_model,
            "' == 'waffle' else 'burger.yaml')"
        ])
    ])

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('temizlik_navigation'),
                'launch',
                'turtlebot3_nav2_coverage_server.launch.py'
            ])
        ),
        launch_arguments={
            'tb3_model': tb3_model,
            'use_rviz': use_rviz,
            'map': map_file,
            'nav2_params_file': nav2_params_file,
            'coverage_params_file': coverage_params_file,
            'autostart_coverage': autostart_coverage,
        }.items()
    )

    coverage_drive_node = Node(
        package='temizlik_navigation',
        executable='opennav_coverage_waypoint_node.py',
        name='opennav_coverage_waypoint_node',
        output='screen',
        parameters=[bridge_params_file],
    )

    return LaunchDescription([
        DeclareLaunchArgument('tb3_model', default_value='burger'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('turtlebot3_navigation2'),
                'map',
                'map.yaml'
            ])
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params_file
        ),
        DeclareLaunchArgument(
            'coverage_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('temizlik_navigation'),
                'config',
                'nav2_coverage_server_params.yaml'
            ])
        ),
        DeclareLaunchArgument(
            'bridge_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('temizlik_navigation'),
                'config',
                'opennav_coverage_nav2_params.yaml'
            ])
        ),
        DeclareLaunchArgument('autostart_coverage', default_value='true'),
        base_stack,
        coverage_drive_node,
    ])
