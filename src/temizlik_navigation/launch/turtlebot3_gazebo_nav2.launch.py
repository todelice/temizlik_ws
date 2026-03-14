#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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


def generate_launch_description():
    tb3_model = LaunchConfiguration('tb3_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_gz_gui = LaunchConfiguration('use_gz_gui')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw_pose = LaunchConfiguration('yaw')
    world_file = LaunchConfiguration('world')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')
    initial_pose_delay = LaunchConfiguration('initial_pose_delay')
    nav2_start_delay = LaunchConfiguration('nav2_start_delay')
    use_ground_truth_localization = LaunchConfiguration('use_ground_truth_localization')

    default_map_file = PathJoinSubstitution([
        FindPackageShare('aws_robomaker_small_warehouse_world'),
        'maps',
        '005',
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

    default_world_file = PathJoinSubstitution([
        FindPackageShare('aws_robomaker_small_warehouse_world'),
        'worlds',
        'small_warehouse',
        'small_warehouse.world'
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
                    'use_rviz': use_rviz,
                    'map': map_file,
                    'params_file': params_file,
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
            'yaw': yaw_pose,
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

    return LaunchDescription([
        DeclareLaunchArgument('tb3_model', default_value='burger'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gz_gui', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map_file),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('world', default_value=default_world_file),
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

        gzserver_launch,
        gzclient_launch,
        robot_state_publisher_launch,
        spawn_turtlebot_launch,
        nav2_launch,
        initial_pose_publisher,
        ground_truth_tf,
    ])
