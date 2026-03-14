#!/usr/bin/env python3

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
import yaml


def _prepare_bt_params(context, *args, **kwargs):
    del args, kwargs
    src_params = LaunchConfiguration('nav2_params_file').perform(context)

    with open(src_params, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    bt_nav = params.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})

    navigators = list(bt_nav.get('navigators', []))
    if not navigators:
        navigators = ['navigate_to_pose', 'navigate_through_poses']
    if 'navigate_complete_coverage' not in navigators:
        navigators.append('navigate_complete_coverage')
    bt_nav['navigators'] = navigators

    bt_nav.setdefault('navigate_complete_coverage', {})
    bt_nav['navigate_complete_coverage']['plugin'] = 'opennav_coverage_navigator/CoverageNavigator'

    plugin_libs = list(bt_nav.get('plugin_lib_names', []))
    for lib in [
        'opennav_compute_complete_coverage_action_bt_node',
        'opennav_cancel_complete_coverage_action_bt_node',
    ]:
        if lib not in plugin_libs:
            plugin_libs.append(lib)
    bt_nav['plugin_lib_names'] = plugin_libs

    error_code_names = list(bt_nav.get('error_code_names', []))
    if 'compute_coverage_error_code' not in error_code_names:
        error_code_names.append('compute_coverage_error_code')
    bt_nav['error_code_names'] = error_code_names

    bt_nav.setdefault(
        'default_coverage_bt_xml',
        os.path.join(
            get_package_share_directory('opennav_coverage_bt'),
            'behavior_trees',
            'navigate_w_basic_complete_coverage.xml',
        ),
    )

    tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='_nav2_coverage_bt.yaml')
    with tmp as out:
        yaml.safe_dump(params, out, sort_keys=False)

    return [SetLaunchConfiguration('merged_nav2_params_file', tmp.name)]


def generate_launch_description():
    tb3_model = LaunchConfiguration('tb3_model')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    coverage_params_file = LaunchConfiguration('coverage_params_file')
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
        DeclareLaunchArgument('autostart_coverage', default_value='true'),
        OpaqueFunction(function=_prepare_bt_params),
        IncludeLaunchDescription(
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
                'nav2_params_file': LaunchConfiguration('merged_nav2_params_file'),
                'coverage_params_file': coverage_params_file,
                'autostart_coverage': autostart_coverage,
            }.items()
        ),
    ])
