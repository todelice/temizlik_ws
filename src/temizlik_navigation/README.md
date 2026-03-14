# temizlik_navigation

Workspace-local launch wrappers for TurtleBot3 + Nav2.

## Jazzy setup for Coverage Server

This workspace now vendors:
- `src/opennav_coverage` (branch: `jazzy`)
- `src/Fields2Cover` (branch: `v1.2.1-devel`)

Build inside the devcontainer:

```bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF -DBUILD_TUTORIALS=OFF
source install/setup.bash
```

## Launch Nav2 only (robot already running)

```bash
ros2 launch temizlik_navigation turtlebot3_nav2.launch.py
```

## Launch TurtleBot3 Gazebo + Nav2 together

```bash
ros2 launch temizlik_navigation turtlebot3_gazebo_nav2.launch.py
```

## Launch TurtleBot3 + Nav2 + Fields2Cover coverage

```bash
ros2 launch temizlik_navigation turtlebot3_f2c_nav2.launch.py
```

Edit coverage area and Fields2Cover parameters in:
`config/f2c_nav2_params.yaml`
Use `exclude_wkts` in that file to define obstacle/no-go polygons.

When `auto_start` is `false` (default), start the mission after Nav2 is ready:

```bash
ros2 service call /start_coverage std_srvs/srv/Trigger {}
```

Recommended flow:
1. Start launch above.
2. In RViz set initial pose (`2D Pose Estimate`).
3. Add a `MarkerArray` display and set topic to `/coverage_markers`.
4. Add a `Path` display and set topic to `/coverage_path`.
5. Call `/start_coverage`.

## Launch TurtleBot3 + Nav2 + Nav2 Coverage Server

```bash
ros2 launch temizlik_navigation turtlebot3_nav2_coverage_server.launch.py
```

Coverage Server parameters are in:
`config/nav2_coverage_server_params.yaml`

RViz topics published by Coverage Server:
`/coverage_server/field_boundary`, `/coverage_server/planning_field`,
`/coverage_server/swaths`, `/coverage_server/coverage_plan`

Important:
`opennav_coverage` is not available as a Jazzy binary package in apt.
Install/build it from source (with `opennav_coverage_msgs`) before using this launch.

## Launch TurtleBot3 + Nav2 + Coverage Server + Auto-Drive Bridge

This launch computes coverage with `opennav_coverage` and then sends waypoints to Nav2 so the robot moves.

```bash
ros2 launch temizlik_navigation turtlebot3_opennav_coverage_drive.launch.py
```

Bridge parameters (field polygon, exclusions, auto start) are in:
`config/opennav_coverage_nav2_params.yaml`

If `auto_start` is `false`, trigger execution with:

```bash
ros2 service call /start_coverage std_srvs/srv/Trigger {}
```

## Launch TurtleBot3 + Nav2 + Coverage BT Navigator (Recommended)

This launch enables the `navigate_complete_coverage` BT navigator plugin and drives the robot directly via Behavior Tree execution.

```bash
ros2 launch temizlik_navigation turtlebot3_opennav_coverage_bt.launch.py
```

Send a `NavigateCompleteCoverage` goal:

```bash
ros2 action send_goal /navigate_complete_coverage opennav_coverage_msgs/action/NavigateCompleteCoverage \
"{frame_id: map, polygons: [{points: [{x: -1.47, y: 1.53, z: 0.0}, {x: 1.66, y: 1.35, z: 0.0}, {x: 1.68, y: -1.68, z: 0.0}, {x: -1.61, y: -1.56, z: 0.0}, {x: -1.47, y: 1.53, z: 0.0}]}]}" --feedback
```

## Optional model selection

```bash
ros2 launch temizlik_navigation turtlebot3_gazebo_nav2.launch.py tb3_model:=waffle_pi
```

## Fields2Cover smoke test

```bash
ros2 run temizlik_navigation f2c_smoke_test
```
