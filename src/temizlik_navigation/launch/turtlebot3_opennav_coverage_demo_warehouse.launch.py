#!/usr/bin/env python3

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def _deep_merge_dicts(base, override):
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge_dicts(base[key], value)
        else:
            base[key] = value
    return base


def _create_turtlebot_nodes(context, *args, **kwargs):
    del args, kwargs

    tb3_model = LaunchConfiguration('tb3_model').perform(context)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    z_pose = LaunchConfiguration('z_pose').perform(context)
    use_ground_truth_localization = (
        LaunchConfiguration('use_ground_truth_localization').perform(context).lower() == 'true')

    turtlebot3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    temizlik_navigation_share = get_package_share_directory('temizlik_navigation')

    if tb3_model == 'burger':
        model_file = os.path.join(
            temizlik_navigation_share,
            'models',
            'turtlebot3_burger_cpu_lidar',
            'model.sdf')
    else:
        model_file = os.path.join(
            turtlebot3_gazebo_share,
            'models',
            f'turtlebot3_{tb3_model}',
            'model.sdf')

    bridge_params = os.path.join(
        turtlebot3_gazebo_share,
        'params',
        f'turtlebot3_{tb3_model}_bridge.yaml')

    actions = [
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', tb3_model,
                '-file', model_file,
                '-x', x_pose,
                '-y', y_pose,
                '-z', z_pose,
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}',
            ],
            output='screen',
        ),
    ]

    if tb3_model != 'burger':
        actions.append(
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                arguments=['/camera/image_raw'],
                output='screen',
            )
        )

    if use_ground_truth_localization:
        actions.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/model/{tb3_model}/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '--ros-args',
                    '-r',
                    f'/model/{tb3_model}/pose:=/ground_truth/model_pose',
                ],
                output='screen',
            )
        )

    return actions


def _prepare_demo_params(context, *args, **kwargs):
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
    if 'navigate_to_pose' not in navigators:
        navigators.append('navigate_to_pose')
    if 'navigate_through_poses' not in navigators:
        navigators.append('navigate_through_poses')
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

    bt_nav['default_coverage_bt_xml'] = os.path.join(
        get_package_share_directory('opennav_coverage_bt'),
        'behavior_trees',
        'navigate_w_basic_complete_coverage_nav_to_start.xml',
    )

    tmp = tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='_coverage_demo_warehouse.yaml')
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

    coverage_demo_rviz = PathJoinSubstitution([
        FindPackageShare('temizlik_navigation'),
        'config',
        'opennav_coverage_demo_warehouse.rviz'
    ])

    default_nav2_params_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_navigation2'),
        'param',
        PythonExpression([
            "'waffle_pi.yaml' if '", tb3_model,
            "' == 'waffle_pi' else ('waffle.yaml' if '", tb3_model,
            "' == 'waffle' else 'burger.yaml')"
        ])
    ])

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(use_gz_gui)
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    spawn_turtlebot_launch = OpaqueFunction(function=_create_turtlebot_nodes)

    nav2_launch = TimerAction(
        period=nav2_start_delay,
        actions=[
            IncludeLaunchDescription(
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
                    'use_rviz': 'false',
                    'map': map_file,
                    'params_file': LaunchConfiguration('merged_nav2_params_file'),
                }.items()
            )
        ]
    )

    initial_pose_publisher = Node(
        package='temizlik_navigation',
        executable='initial_pose_publisher.py',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'map',
            'x': x_pose,
            'y': y_pose,
            'yaw': yaw,
            'delay_sec': initial_pose_delay,
            'repeat_count': 5,
            'repeat_interval_sec': 1.0,
            'wait_for_odom': True,
            'wait_for_scan': True,
            'startup_timeout_sec': 20.0,
        }],
        condition=IfCondition(PythonExpression([
            "'", publish_initial_pose, "' == 'true' and '",
            use_ground_truth_localization, "' != 'true'"
        ]))
    )

    ground_truth_tf = Node(
        package='temizlik_navigation',
        executable='gazebo_ground_truth_tf.py',
        name='gazebo_ground_truth_tf',
        parameters=[{
            'use_sim_time': True,
            'ground_truth_topic': '/ground_truth/model_pose',
            'ground_truth_child_frame': tb3_model,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'robot_base_frame': 'base_footprint',
            'lookup_timeout_sec': 0.2,
        }],
        output='screen',
        condition=IfCondition(use_ground_truth_localization)
    )

    coverage_server = Node(
        package='opennav_coverage',
        executable='opennav_coverage',
        name='coverage_server',
        output='screen',
        parameters=[coverage_params_file],
    )

    lifecycle_manager_coverage = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_coverage',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': autostart_coverage,
            'node_names': ['coverage_server'],
        }],
    )

    rviz_launch = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', coverage_demo_rviz],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('tb3_model', default_value='waffle'),
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
                'opennav_coverage_demo_warehouse_nav2_overrides.yaml'
            ])
        ),
        DeclareLaunchArgument(
            'coverage_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('temizlik_navigation'),
                'config',
                'opennav_coverage_demo_warehouse_coverage_params.yaml'
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
        SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([
                FindPackageShare('aws_robomaker_small_warehouse_world'),
                'models',
            ])
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([
                FindPackageShare('aws_robomaker_small_warehouse_world'),
            ])
        ),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'models',
            ])
        ),
        OpaqueFunction(function=_prepare_demo_params),
        gzserver_launch,
        gzclient_launch,
        robot_state_publisher_launch,
        spawn_turtlebot_launch,
        nav2_launch,
        initial_pose_publisher,
        ground_truth_tf,
        coverage_server,
        lifecycle_manager_coverage,
        rviz_launch,
    ])
