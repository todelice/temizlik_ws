#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_model = LaunchConfiguration('tb3_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
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

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={
            'slam': 'False',
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'rviz_launch.py'
            ])
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('tb3_model', default_value='burger'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map_file),
        DeclareLaunchArgument('params_file', default_value=default_params_file),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),

        nav2_bringup_launch,
        rviz_launch,
    ])
