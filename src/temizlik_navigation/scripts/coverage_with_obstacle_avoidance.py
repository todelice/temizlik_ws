#!/usr/bin/env python3
# Coverage with obstacle avoidance.
#
# Strategy:
#   1. Extract obstacles from the global costmap (or static map) within the
#      coverage polygon and pass them as void polygons to ComputeCoveragePath.
#   2. Get individual swath start/end points from the result.
#   3. Navigate to each swath start using navigate_to_pose (global planner →
#      obstacle-aware, avoids obstacles during inter-row transitions).
#   4. Follow each swath using follow_path on the controller directly
#      (precise straight-line coverage).
#
# This eliminates the Dubins-curve inter-swath turns that could cross
# through obstacles.

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

import numpy as np
import cv2

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path

from nav2_msgs.action import FollowPath, NavigateToPose
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates


# ── helpers ───────────────────────────────────────────────────────────────────

def _yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))


def _make_swath_path(start, end, frame_id='map', n_points=20) -> Path:
    """Straight-line nav_msgs/Path from Point32 start to Point32 end."""
    path = Path()
    path.header.frame_id = frame_id
    dx = end.x - start.x
    dy = end.y - start.y
    yaw = math.atan2(dy, dx)
    quat = _yaw_to_quat(yaw)
    for i in range(n_points + 1):
        t = i / n_points
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(start.x + t * dx)
        ps.pose.position.y = float(start.y + t * dy)
        ps.pose.orientation = quat
        path.poses.append(ps)
    return path


def _to_coordinates(pts_world) -> Coordinates:
    """Convert list of (x, y) tuples to opennav Coordinates (cartesian)."""
    coords = Coordinates()
    for x, y in pts_world:
        c = Coordinate()
        c.axis1 = float(x)
        c.axis2 = float(y)
        coords.coordinates.append(c)
    return coords


class CoverageWithObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('coverage_with_obstacle_avoidance')

        transient_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.costmap_data = None
        self.map_data = None

        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 self._costmap_cb, transient_qos)
        self.create_subscription(OccupancyGrid, '/map',
                                 self._map_cb, transient_qos)

        self._compute_client = ActionClient(
            self, ComputeCoveragePath, 'compute_coverage_path')
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self._follow_client = ActionClient(
            self, FollowPath, 'follow_path')

    def _costmap_cb(self, msg):
        self.costmap_data = msg

    def _map_cb(self, msg):
        self.map_data = msg

    # ── coordinate helpers ────────────────────────────────────────────────────

    def _w2p(self, grid, wx, wy):
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        return int((wx - ox) / res), int((wy - oy) / res)

    def _p2w(self, grid, col, row):
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        return ox + col * res, oy + row * res

    # ── obstacle extraction ───────────────────────────────────────────────────

    def _extract_voids(self, grid, outer_pts, threshold=65):
        w, h = grid.info.width, grid.info.height
        res = grid.info.resolution

        data = np.array(grid.data, dtype=np.int8).reshape((h, w))
        binary = np.zeros((h, w), dtype=np.uint8)
        binary[data > threshold] = 255

        outer_px = np.array(
            [self._w2p(grid, x, y) for x, y in outer_pts], dtype=np.int32)
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [outer_px], 255)
        inside = cv2.bitwise_and(binary, mask)

        # Close small gaps then dilate minimally (coverage server handles margin)
        inside = cv2.morphologyEx(
            inside, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        dil = max(1, int(0.05 / res))
        inside = cv2.dilate(
            inside, np.ones((dil * 2 + 1, dil * 2 + 1), np.uint8))

        contours, _ = cv2.findContours(
            inside, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_px = 0.05 / (res * res)
        voids = []
        for c in contours:
            if cv2.contourArea(c) < min_px:
                continue
            approx = cv2.approxPolyDP(c, 0.02 * cv2.arcLength(c, True), True)
            if len(approx) < 3:
                continue
            pts = []
            for pt in approx:
                col, row = pt[0]
                pts.append(self._p2w(grid, col, row))
            voids.append(pts)
        return voids

    # ── action helpers ────────────────────────────────────────────────────────

    def _navigate_to(self, x, y, yaw=0.0) -> bool:
        """Navigate to (x, y, yaw) using the global planner."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation = _yaw_to_quat(yaw)

        self._nav_client.wait_for_server()
        fut = self._nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error(
                f'navigate_to_pose rejected for ({x:.2f}, {y:.2f})')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        ok = res_fut.result().status == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            self.get_logger().warn(
                f'navigate_to_pose failed for ({x:.2f}, {y:.2f})')
        return ok

    def _follow_path(self, path: Path) -> bool:
        """Follow a nav_msgs/Path with the controller directly."""
        goal = FollowPath.Goal()
        goal.path = path
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'

        self._follow_client.wait_for_server()
        fut = self._follow_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('follow_path rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        ok = res_fut.result().status == GoalStatus.STATUS_SUCCEEDED
        if not ok:
            self.get_logger().warn('follow_path did not succeed')
        return ok

    # ── main entry point ──────────────────────────────────────────────────────

    def run(self, outer_polygon_closed):
        """
        outer_polygon_closed: list of (x, y), first == last.
        """
        # 1. Get costmap / map
        self.get_logger().info(
            'Waiting for global costmap (includes dynamic obstacles)...')
        deadline = self.get_clock().now().nanoseconds + 10_000_000_000
        while (self.costmap_data is None
               and self.get_clock().now().nanoseconds < deadline):
            rclpy.spin_once(self, timeout_sec=0.5)

        if self.costmap_data is not None:
            grid = self.costmap_data
            threshold = 65
            self.get_logger().info('Using global costmap.')
        else:
            self.get_logger().warn('Costmap not available — using static map.')
            while self.map_data is None:
                rclpy.spin_once(self, timeout_sec=0.5)
            grid = self.map_data
            threshold = 65

        # 2. Extract obstacle voids
        unique_pts = outer_polygon_closed[:-1]
        void_pts_list = self._extract_voids(grid, unique_pts, threshold)
        self.get_logger().info(
            f'Found {len(void_pts_list)} obstacle void(s).')

        # 3. Build ComputeCoveragePath goal
        compute_goal = ComputeCoveragePath.Goal()
        compute_goal.frame_id = 'map'
        compute_goal.generate_headland = True
        compute_goal.generate_route = True
        compute_goal.generate_path = False  # we handle turns ourselves

        # Outer polygon
        outer_coords = _to_coordinates(outer_polygon_closed)
        void_coords = [_to_coordinates(v + [v[0]]) for v in void_pts_list]
        compute_goal.polygons = [outer_coords] + void_coords

        # 4. Compute swaths
        self.get_logger().info('Computing coverage path...')
        self._compute_client.wait_for_server()
        fut = self._compute_client.send_goal_async(compute_goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('ComputeCoveragePath rejected!')
            return
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        result = res_fut.result().result

        if result.error_code != 0:
            self.get_logger().error(
                f'ComputeCoveragePath failed, error code: {result.error_code}')
            return

        swaths = result.coverage_path.swaths
        if not swaths:
            self.get_logger().error('No swaths computed!')
            return
        self.get_logger().info(f'Computed {len(swaths)} swath(s).')

        # 5. Execute: navigate_to_pose (planner) → follow_path (controller)
        #    for each swath, inter-swath moves use the global planner so they
        #    automatically route around obstacles.
        for i, swath in enumerate(swaths):
            s = swath.start
            e = swath.end
            yaw = math.atan2(e.y - s.y, e.x - s.x)

            self.get_logger().info(
                f'[{i+1}/{len(swaths)}] Navigating to swath start '
                f'({s.x:.2f}, {s.y:.2f})...')
            self._navigate_to(s.x, s.y, yaw)

            self.get_logger().info(
                f'[{i+1}/{len(swaths)}] Following swath...')
            path = _make_swath_path(s, e)
            self._follow_path(path)

        self.get_logger().info('Coverage complete.')


def main():
    rclpy.init()
    node = CoverageWithObstacleAvoidance()

    # ── Define your coverage area here (must be closed: first == last) ────────
    outer_polygon = [
        (-3.0,  3.0),
        ( 0.0,  3.0),
        ( 1.0, -4.0),
        (-3.0, -4.0),
        (-3.0,  3.0),  # closing point
    ]

    node.run(outer_polygon)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
