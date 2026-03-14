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


def _deep_merge_dicts(base, override):
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_dicts(base[key], value)
        else:
            base[key] = value
    return base


def _prepare_bt_params(context, *args, **kwargs):
    del args, kwargs
    src_params = LaunchConfiguration('nav2_params_file').perform(context)
    override_params = LaunchConfiguration('nav2_overrides_file').perform(context)
    use_ground_truth_localization = (
        LaunchConfiguration('use_ground_truth_localization').perform(context).lower() == 'true')

    with open(src_params, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    if override_params and os.path.exists(override_params):
        with open(override_params, 'r', encoding='utf-8') as f:
            override_data = yaml.safe_load(f) or {}
        _deep_merge_dicts(params, override_data)

    if use_ground_truth_localization:
        gt_override_path = os.path.join(
            get_package_share_directory('temizlik_navigation'),
            'config',
            'ground_truth_localization_overrides.yaml')
        with open(gt_override_path, 'r', encoding='utf-8') as f:
            gt_override_data = yaml.safe_load(f) or {}
        _deep_merge_dicts(params, gt_override_data)

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
    world_file = LaunchConfiguration('world')
    use_gz_gui = LaunchConfiguration('use_gz_gui')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')
    initial_pose_delay = LaunchConfiguration('initial_pose_delay')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')
    use_ground_truth_localization = LaunchConfiguration('use_ground_truth_localization')

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
                FindPackageShare('aws_robomaker_small_warehouse_world'),
                'maps',
                '005',
                'map.yaml'
            ])
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=default_nav2_params_file
        ),
        DeclareLaunchArgument(
            'nav2_overrides_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('temizlik_navigation'),
                'config',
                'warehouse_nav2_overrides.yaml'
            ])
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
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([
                FindPackageShare('aws_robomaker_small_warehouse_world'),
                'worlds',
                'small_warehouse',
                'small_warehouse.world'
            ])
        ),
        DeclareLaunchArgument('use_gz_gui', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='-3.071'),
        DeclareLaunchArgument('y_pose', default_value='3.583'),
        DeclareLaunchArgument('z_pose', default_value='0.12'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('publish_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_pose_delay', default_value='3.0'),
        DeclareLaunchArgument('nav2_start_delay', default_value='5.0'),
        DeclareLaunchArgument('use_ground_truth_localization', default_value='false'),
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
                'world': world_file,
                'use_gz_gui': use_gz_gui,
                'x_pose': x_pose,
                'y_pose': y_pose,
                'z_pose': z_pose,
                'yaw': yaw,
                'publish_initial_pose': publish_initial_pose,
                'initial_pose_delay': initial_pose_delay,
                'nav2_start_delay': nav2_start_delay,
                'use_ground_truth_localization': use_ground_truth_localization,
            }.items()
        ),
    ])
