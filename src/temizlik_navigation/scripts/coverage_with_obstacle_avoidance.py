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
from geometry_msgs.msg import Point, Point32, Polygon, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from nav2_msgs.action import FollowPath, NavigateToPose
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates


# ── helpers ───────────────────────────────────────────────────────────────────

def _yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))


def _make_swath_path(start, end, frame_id='map',
                     step_m=0.10, approach_m=0.40) -> Path:
    """Straight-line path with a run-up segment before the swath.

    The path begins `approach_m` metres behind `start` so the robot is already
    aligned and moving when it crosses the swath start point.
    """
    path = Path()
    path.header.frame_id = frame_id
    dx = end.x - start.x
    dy = end.y - start.y
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    quat = _yaw_to_quat(yaw)

    cx, cy = dx / length, dy / length          # unit vector along swath

    # Approach segment: approach_m before start → start
    n_app = max(1, int(approach_m / step_m))
    for i in range(n_app + 1):
        t = i / n_app
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(start.x - approach_m * cx
                                   + t * approach_m * cx)
        ps.pose.position.y = float(start.y - approach_m * cy
                                   + t * approach_m * cy)
        ps.pose.orientation = quat
        path.poses.append(ps)

    # Main swath: start → end
    n_swath = max(1, int(length / step_m))
    for i in range(1, n_swath + 1):
        t = i / n_swath
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(start.x + t * dx)
        ps.pose.position.y = float(start.y + t * dy)
        ps.pose.orientation = quat
        path.poses.append(ps)

    return path


def _make_perimeter_path(pts, step_m=0.12, frame_id='map') -> Path:
    """Dense closed-loop path along a polygon perimeter (wall-following)."""
    path = Path()
    path.header.frame_id = frame_id
    loop = pts + [pts[0]]          # close the polygon

    for i in range(len(loop) - 1):
        x0, y0 = loop[i]
        x1, y1 = loop[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        quat = _yaw_to_quat(yaw)
        n = max(1, int(seg / step_m))
        for j in range(n):
            t = j / n
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(x0 + t * (x1 - x0))
            ps.pose.position.y = float(y0 + t * (y1 - y0))
            ps.pose.orientation = quat
            path.poses.append(ps)

    # Final closing point
    x, y = loop[-1]
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.orientation = path.poses[-1].pose.orientation
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
        self._marker_pub = self.create_publisher(
            MarkerArray, '/coverage_markers', 10)

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
        # Erode the outer polygon mask by a few pixels so that obstacle
        # contours which touch the boundary are clipped strictly inside.
        shrink_px = max(2, int(0.10 / res))
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [outer_px], 255)
        mask = cv2.erode(mask, np.ones((shrink_px * 2 + 1,
                                        shrink_px * 2 + 1), np.uint8))

        inside = cv2.bitwise_and(binary, mask)

        # Close small gaps only — the costmap already inflates by robot radius.
        inside = cv2.morphologyEx(
            inside, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        contours, _ = cv2.findContours(
            inside, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_px = 0.10 / (res * res)   # ignore contours < 0.10 m²
        voids = []
        for c in contours:
            if cv2.contourArea(c) < min_px:
                continue
            # Larger epsilon → fewer vertices → cleaner polygon
            approx = cv2.approxPolyDP(c, 0.05 * cv2.arcLength(c, True), True)
            if len(approx) < 3:
                continue
            pts = []
            for pt in approx:
                col, row = pt[0]
                pts.append(self._p2w(grid, col, row))
            self.get_logger().info(
                f'  Void polygon: {len(pts)} vertices, '
                f'area≈{cv2.contourArea(c) * res * res:.2f} m²')
            voids.append(pts)
        return voids

    # ── visualization ─────────────────────────────────────────────────────────

    def _publish_markers(self, outer_pts, void_pts_list, swaths=None):
        """Publish coverage polygon, void polygons, and swath lines to RViz."""
        ma = MarkerArray()
        mid = 0

        def _line_strip(pts_closed, r, g, b, ns, marker_id):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = marker_id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
            for x, y in pts_closed:
                m.points.append(Point(x=float(x), y=float(y), z=0.0))
            return m

        # Outer polygon — green
        closed = outer_pts + [outer_pts[0]]
        ma.markers.append(_line_strip(closed, 0.0, 1.0, 0.0, 'outer', mid))
        mid += 1

        # Void polygons — red
        for void in void_pts_list:
            closed = void + [void[0]]
            ma.markers.append(_line_strip(closed, 1.0, 0.0, 0.0, 'voids', mid))
            mid += 1

        # Swaths — blue lines
        if swaths:
            for i, swath in enumerate(swaths):
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'swaths'
                m.id = mid + i
                m.type = Marker.LINE_STRIP
                m.action = Marker.ADD
                m.scale.x = 0.03
                m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
                m.points.append(
                    Point(x=float(swath.start.x), y=float(swath.start.y), z=0.0))
                m.points.append(
                    Point(x=float(swath.end.x), y=float(swath.end.y), z=0.0))
                ma.markers.append(m)

        self._marker_pub.publish(ma)

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

    # ── edge cleaning ─────────────────────────────────────────────────────────

    def _offset_polygon_outward(self, pts, offset):
        """Shift each vertex outward from the polygon centroid by offset metres."""
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        result = []
        for x, y in pts:
            dx, dy = x - cx, y - cy
            dist = math.hypot(dx, dy)
            if dist < 1e-6:
                result.append((x, y))
            else:
                result.append((x + offset * dx / dist,
                                y + offset * dy / dist))
        return result

    def _clean_obstacle_edge(self, void_pts, edge_offset=0.15) -> bool:
        """Wall-follow around one obstacle void polygon using follow_path.

        Generates a dense continuous path along the offset perimeter and
        sends it directly to the controller — no replanning between waypoints.
        void_pts – list of (x, y), NOT closed.
        """
        if len(void_pts) < 3:
            return True

        outer = self._offset_polygon_outward(void_pts, edge_offset)

        # Navigate to the first perimeter point with the global planner
        yaw0 = math.atan2(outer[1][1] - outer[0][1],
                          outer[1][0] - outer[0][0])
        self.get_logger().info(
            f'  Edge cleaning: approaching perimeter start '
            f'({outer[0][0]:.2f}, {outer[0][1]:.2f})...')
        self._navigate_to(outer[0][0], outer[0][1], yaw0)

        # Build a dense closed-loop path and follow it in one smooth action
        path = _make_perimeter_path(outer)
        self.get_logger().info(
            f'  Edge cleaning: wall-following {len(path.poses)} waypoints...')
        ok = self._follow_path(path)
        if not ok:
            self.get_logger().warn('Edge cleaning did not fully succeed (continuing)')
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

        # 3. Publish initial markers (polygon + voids, no swaths yet)
        self._publish_markers(unique_pts, void_pts_list)
        self.get_logger().info('Published coverage area markers to /coverage_markers.')

        # Edge-clean each obstacle before main coverage
        if void_pts_list:
            self.get_logger().info(
                f'Edge cleaning {len(void_pts_list)} obstacle(s)...')
            for idx, void_pts in enumerate(void_pts_list):
                self.get_logger().info(
                    f'  Obstacle {idx + 1}/{len(void_pts_list)}...')
                self._clean_obstacle_edge(void_pts)
        else:
            self.get_logger().info('No obstacles found — skipping edge cleaning.')

        # 4. Build ComputeCoveragePath goal
        compute_goal = ComputeCoveragePath.Goal()
        compute_goal.frame_id = 'map'
        compute_goal.generate_headland = False  # headland shrinks coverage area
        compute_goal.generate_route = True
        compute_goal.generate_path = False  # we handle turns ourselves

        # Outer polygon
        outer_coords = _to_coordinates(outer_polygon_closed)
        void_coords = [_to_coordinates(v + [v[0]]) for v in void_pts_list]
        compute_goal.polygons = [outer_coords] + void_coords

        # 5. Compute swaths
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
                f'ComputeCoveragePath failed, error_code={result.error_code} '
                f'(801=F2C_error, 802=invalid_mode, 803=invalid_coords)')
            return

        swaths = result.coverage_path.swaths
        if not swaths:
            self.get_logger().error('No swaths computed!')
            return
        self.get_logger().info(f'Computed {len(swaths)} swath(s).')

        # Republish markers with swath lines visible in RViz
        self._publish_markers(unique_pts, void_pts_list, swaths)

        # 6. Execute: navigate_to_pose (planner) → follow_path (controller)
        #    for each swath, inter-swath moves use the global planner so they
        #    automatically route around obstacles.
        APPROACH_M = 0.40   # run-up distance before swath start

        for i, swath in enumerate(swaths):
            s = swath.start
            e = swath.end
            yaw = math.atan2(e.y - s.y, e.x - s.x)
            cx, cy = math.cos(yaw), math.sin(yaw)

            # Navigate to the approach point (behind swath start) so the
            # robot arrives already aligned — prevents heading-correction flailing.
            ax = float(s.x) - APPROACH_M * cx
            ay = float(s.y) - APPROACH_M * cy
            self.get_logger().info(
                f'[{i+1}/{len(swaths)}] Navigating to approach point '
                f'({ax:.2f}, {ay:.2f}) yaw={math.degrees(yaw):.0f}°...')
            self._navigate_to(ax, ay, yaw)

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
