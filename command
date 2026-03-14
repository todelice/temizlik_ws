ros2 action send_goal /navigate_complete_coverage opennav_coverage_msgs/action/NavigateCompleteCoverage "{frame_id: map, polygons: [{points: [
{x: -4.0, y:  -1.1, z: 0.0},
{x: -0.1, y:  -1.0, z: 0.0},
{x: -0.1, y:  -4.0, z: 0.0},
{x: -4.0, y:  -4.0, z: 0.0},
{x: -4.0, y:  -1.1, z: 0.0}
]}]}" --feedback
