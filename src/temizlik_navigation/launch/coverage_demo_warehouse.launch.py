# Copyright (c) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Based on coverage_demo_launch.py from opennav_coverage_demo,
# adapted to run in the AWS small warehouse world.
# Supports both navigate_to_pose (regular nav) and navigate_complete_coverage
# (coverage nav). Send goals manually via RViz or CLI — demo_coverage is not
# started automatically.

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _deep_merge_dicts(base, override):
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_dicts(base[key], value)
        else:
            base[key] = value
    return base


def _prepare_params(context, *args, **kwargs):
    """
    Merge coverage demo params with nav2 overrides, then inject:
      - navigate_to_pose + navigate_through_poses navigators into bt_navigator
      - navigate_complete_coverage (CoverageNavigator) into bt_navigator
      - coverage BT plugin libs and error codes
    Writes the merged result to a temp file and exposes it as
    'merged_params_file' launch configuration.
    """
    del args, kwargs

    coverage_params_file = LaunchConfiguration('coverage_params_file').perform(context)
    nav2_overrides_file = LaunchConfiguration('nav2_overrides_file').perform(context)

    with open(coverage_params_file, 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f) or {}

    if nav2_overrides_file and os.path.exists(nav2_overrides_file):
        with open(nav2_overrides_file, 'r', encoding='utf-8') as f:
            override_data = yaml.safe_load(f) or {}
        _deep_merge_dicts(params, override_data)

    bt_nav = params.setdefault('bt_navigator', {}).setdefault('ros__parameters', {})

    # Ensure all three navigators are present
    navigators = list(bt_nav.get('navigators', []))
    for nav in ['navigate_to_pose', 'navigate_through_poses', 'navigate_complete_coverage']:
        if nav not in navigators:
            navigators.append(nav)
    bt_nav['navigators'] = navigators

    bt_nav.setdefault('navigate_to_pose', {})
    bt_nav['navigate_to_pose']['plugin'] = 'nav2_bt_navigator::NavigateToPoseNavigator'

    bt_nav.setdefault('navigate_through_poses', {})
    bt_nav['navigate_through_poses']['plugin'] = \
        'nav2_bt_navigator::NavigateThroughPosesNavigator'

    bt_nav.setdefault('navigate_complete_coverage', {})
    bt_nav['navigate_complete_coverage']['plugin'] = \
        'opennav_coverage_navigator/CoverageNavigator'

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

    bt_nav['default_coverage_bt_xml'] = os.path.join(
        get_package_share_directory('opennav_coverage_bt'),
        'behavior_trees',
        'navigate_w_basic_complete_coverage_nav_to_start.xml',
    )

    tmp = tempfile.NamedTemporaryFile(
        mode='w', delete=False, suffix='_coverage_demo_warehouse.yaml'
    )
    with tmp as out:
        yaml.safe_dump(params, out, sort_keys=False)

    return [SetLaunchConfiguration('merged_params_file', tmp.name)]


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_demo_dir = get_package_share_directory('opennav_coverage_demo')
    temizlik_nav_dir = get_package_share_directory('temizlik_navigation')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')

    world = os.path.join(
        warehouse_dir, 'worlds', 'small_warehouse', 'small_warehouse.world'
    )
    robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')

    rviz_config_file = os.path.join(
        temizlik_nav_dir, 'config', 'opennav_coverage_demo_warehouse.rviz'
    )
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(coverage_demo_dir, 'rviz_config.rviz')

    # Gazebo server (headless simulation)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r -s -v2 {world}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
    )

    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': '-v4 -g '}.items(),
    )

    start_gazebo_spawner_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'robot_name': 'turtlebot3_waffle',
            'robot_sdf': robot_sdf,
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': '0.1',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    # Static map->odom transform (fake localization)
    fake_localization_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            LaunchConfiguration('x_pose'),
            LaunchConfiguration('y_pose'),
            '0', '0', '0', '0', 'map', 'odom',
        ],
    )

    # Map server — publishes the warehouse map so the global costmap's static_layer
    # can use it for navigate_to_pose path planning.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': os.path.join(
                warehouse_dir, 'maps', '005', 'map.yaml'
            ),
        }],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # Navigation bringup: controller_server + bt_navigator + velocity_smoother +
    # coverage_server — all in one composable container.
    # Delayed so planner_server and behavior_server are active before bt_navigator
    # tries to call ComputePathToPose in the nav_to_start coverage BT.
    bringup_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(coverage_demo_dir, 'bringup_launch.py')
                ),
                launch_arguments={
                    'params_file': LaunchConfiguration('merged_params_file'),
                }.items(),
            )
        ],
    )

    # Standalone planner_server so navigate_to_pose goals can be served.
    # It uses the same merged params file for its configuration.
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('merged_params_file')],
    )

    # Behavior server provides spin/backup/wait recovery actions needed by
    # the default navigate_to_pose BT (navigate_to_pose_w_replanning_and_recovery.xml).
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[LaunchConfiguration('merged_params_file')],
    )

    lifecycle_manager_planner = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['planner_server', 'behavior_server'],
        }],
    )

    # RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'namespace': '', 'rviz_config': rviz_config_file}.items(),
    )

    ld = LaunchDescription()

    # ── Arguments ─────────────────────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='-3.071'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='3.583'))
    ld.add_action(DeclareLaunchArgument(
        'coverage_params_file',
        default_value=os.path.join(
            temizlik_nav_dir, 'config',
            'opennav_coverage_demo_warehouse_coverage_params.yaml'),
    ))
    ld.add_action(DeclareLaunchArgument(
        'nav2_overrides_file',
        default_value=os.path.join(
            temizlik_nav_dir, 'config',
            'opennav_coverage_demo_warehouse_nav2_overrides.yaml',
        ),
    ))

    # Make warehouse models available to Gazebo
    ld.add_action(
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(warehouse_dir, 'models'),
        )
    )
    ld.add_action(
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            warehouse_dir,
        )
    )

    # ── Merge params (creates 'merged_params_file') ───────────────────────────
    ld.add_action(OpaqueFunction(function=_prepare_params))

    # ── Simulation ────────────────────────────────────────────────────────────
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    # ── Localization (static fake: map→odom fixed at spawn position) ──────────
    ld.add_action(fake_localization_cmd)

    # ── Navigation (coverage + regular nav) ───────────────────────────────────
    ld.add_action(map_server)
    ld.add_action(lifecycle_manager_map)
    ld.add_action(bringup_cmd)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(lifecycle_manager_planner)

    # ── Visualization ─────────────────────────────────────────────────────────
    ld.add_action(rviz_cmd)

    return ld
