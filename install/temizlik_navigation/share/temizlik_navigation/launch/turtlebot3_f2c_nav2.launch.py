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
    params_file = LaunchConfiguration('params_file')
    f2c_params_file = LaunchConfiguration('f2c_params_file')

    default_params_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'param',
        PythonExpression([
            "'waffle_pi.yaml' if '", tb3_model,
            "' == 'waffle_pi' else ('waffle.yaml' if '", tb3_model,
            "' == 'waffle' else 'burger.yaml')"
        ])
    ])

    sim_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('temizlik_navigation'),
                'launch',
                'turtlebot3_gazebo_nav2.launch.py'
            ])
        ),
        launch_arguments={
            'tb3_model': tb3_model,
            'use_rviz': use_rviz,
            'map': map_file,
            'params_file': params_file,
        }.items()
    )

    f2c_nav_node = Node(
        package='temizlik_navigation',
        executable='f2c_nav2_waypoint_node',
        name='f2c_nav2_waypoint_node',
        output='screen',
        parameters=[f2c_params_file],
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
            'params_file',
            default_value=default_params_file
        ),
        DeclareLaunchArgument(
            'f2c_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('temizlik_navigation'),
                'config',
                'f2c_nav2_params.yaml'
            ])
        ),
        sim_nav2_launch,
        f2c_nav_node,
    ])
