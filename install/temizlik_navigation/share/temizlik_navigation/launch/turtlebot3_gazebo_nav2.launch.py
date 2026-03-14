#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_model = LaunchConfiguration('tb3_model')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    default_map_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'map',
        'map.yaml'
    ])

    default_params_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'param',
        PythonExpression([
            "'waffle_pi.yaml' if '", tb3_model,
            "' == 'waffle_pi' else ('waffle.yaml' if '", tb3_model,
            "' == 'waffle' else 'burger.yaml')"
        ])
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('temizlik_navigation'),
                'launch',
                'turtlebot3_nav2.launch.py'
            ])
        ),
        launch_arguments={
            'tb3_model': tb3_model,
            'use_sim_time': 'true',
            'use_rviz': use_rviz,
            'map': map_file,
            'params_file': params_file,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('tb3_model', default_value='burger'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map_file),
        DeclareLaunchArgument('params_file', default_value=default_params_file),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),

        gazebo_launch,
        nav2_launch,
    ])
