#!/usr/bin/env python3

import math
import re
from typing import List, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Path
from opennav_coverage_msgs.action import ComputeCoveragePath
from opennav_coverage_msgs.msg import Coordinate, Coordinates
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


class OpenNavCoverageWaypointNode(Node):
    def __init__(self):
        super().__init__("opennav_coverage_waypoint_node")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("action_name", "/follow_waypoints")
        self.declare_parameter("auto_start", False)
        self.declare_parameter("waypoint_spacing", 0.25)

        self.declare_parameter(
            "field_wkt",
            "POLYGON((-1.47 1.53, 1.66 1.35, 1.68 -1.68, -1.61 -1.56, -1.47 1.53))",
        )
        self.declare_parameter("exclude_wkts", [])

        self.declare_parameter("use_gml_file", False)
        self.declare_parameter("gml_field", "")
        self.declare_parameter("generate_headland", True)
        self.declare_parameter("generate_route", True)
        self.declare_parameter("generate_path", True)

        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.action_name = self.get_parameter("action_name").get_parameter_value().string_value
        self.auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value
        self.waypoint_spacing = self.get_parameter("waypoint_spacing").get_parameter_value().double_value

        self.field_wkt = self.get_parameter("field_wkt").get_parameter_value().string_value
        self.exclude_wkts = list(self.get_parameter("exclude_wkts").get_parameter_value().string_array_value)

        self.use_gml_file = self.get_parameter("use_gml_file").get_parameter_value().bool_value
        self.gml_field = self.get_parameter("gml_field").get_parameter_value().string_value
        self.generate_headland = self.get_parameter("generate_headland").get_parameter_value().bool_value
        self.generate_route = self.get_parameter("generate_route").get_parameter_value().bool_value
        self.generate_path = self.get_parameter("generate_path").get_parameter_value().bool_value

        if self.waypoint_spacing <= 0.0:
            raise RuntimeError("waypoint_spacing must be > 0.0")

        self.coverage_client = ActionClient(self, ComputeCoveragePath, "/compute_coverage_path")
        self.follow_client = ActionClient(self, FollowWaypoints, self.action_name)

        qos_path = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, "coverage_path", qos_path)

        self.goal_sent = False
        self.start_service = self.create_service(Trigger, "start_coverage", self.handle_start_coverage)

        self.start_timer = None
        if self.auto_start:
            self.start_timer = self.create_timer(1.0, self.try_start_once)
        else:
            self.get_logger().info("auto_start=false. Call /start_coverage to start.")

    def try_start_once(self):
        if self.goal_sent:
            if self.start_timer:
                self.start_timer.cancel()
            return
        self.start_coverage()

    def handle_start_coverage(self, _req, resp):
        if self.goal_sent:
            resp.success = False
            resp.message = "Coverage execution already started."
            return resp
        ok, msg = self.start_coverage()
        resp.success = ok
        resp.message = msg
        return resp

    def start_coverage(self):
        if not self.coverage_client.wait_for_server(timeout_sec=1.0):
            return False, "Coverage action server (/compute_coverage_path) not ready."
        if not self.follow_client.wait_for_server(timeout_sec=1.0):
            return False, f"FollowWaypoints action server ({self.action_name}) not ready."

        goal = self.build_coverage_goal()
        self.goal_sent = True
        self.get_logger().info("Sending coverage planning goal.")
        send_goal_future = self.coverage_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.coverage_goal_response_cb)
        return True, "Coverage goal sent."

    def coverage_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_sent = False
            self.get_logger().error("Coverage planning goal rejected.")
            return
        self.get_logger().info("Coverage planning goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.coverage_result_cb)

    def coverage_result_cb(self, future):
        wrapped = future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.goal_sent = False
            self.get_logger().error(f"Coverage planning failed with status {wrapped.status}.")
            return

        result = wrapped.result
        if result.error_code != ComputeCoveragePath.Result.NONE:
            self.goal_sent = False
            self.get_logger().error(f"Coverage server returned error_code={result.error_code}.")
            return

        nav_path = result.nav_path
        if len(nav_path.poses) == 0:
            self.goal_sent = False
            self.get_logger().error("Coverage planning returned empty nav_path.")
            return

        if not nav_path.header.frame_id:
            nav_path.header.frame_id = self.frame_id
        self.path_pub.publish(nav_path)

        goal = FollowWaypoints.Goal()
        goal.poses = self.downsample_poses(nav_path.poses, self.waypoint_spacing)

        self.get_logger().info(f"Sending {len(goal.poses)} waypoints to Nav2.")
        send_follow_future = self.follow_client.send_goal_async(goal)
        send_follow_future.add_done_callback(self.follow_goal_response_cb)

    def follow_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_sent = False
            self.get_logger().error("FollowWaypoints goal rejected.")
            return
        self.get_logger().info("FollowWaypoints goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.follow_result_cb)

    def follow_result_cb(self, future):
        wrapped = future.result()
        self.goal_sent = False
        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Coverage execution completed.")
        else:
            self.get_logger().error(f"Coverage execution failed with status {wrapped.status}.")

    def build_coverage_goal(self):
        goal = ComputeCoveragePath.Goal()
        goal.generate_headland = self.generate_headland
        goal.generate_route = self.generate_route
        goal.generate_path = self.generate_path
        goal.frame_id = self.frame_id

        if self.use_gml_file:
            goal.use_gml_file = True
            goal.gml_field = self.gml_field
            return goal

        goal.use_gml_file = False
        rings = self.parse_polygon_wkt(self.field_wkt)
        for ex_wkt in self.exclude_wkts:
            rings.extend(self.parse_polygon_wkt(ex_wkt))

        if len(rings) == 0:
            raise RuntimeError("No valid polygon rings parsed from field_wkt/exclude_wkts.")

        goal.polygons = [self.ring_to_msg(ring) for ring in rings]
        return goal

    def parse_polygon_wkt(self, wkt: str) -> List[List[Tuple[float, float]]]:
        text = wkt.strip()
        if not text.upper().startswith("POLYGON"):
            raise RuntimeError(f"Only POLYGON WKT is supported, got: {wkt}")

        # Extract "(...),(...)" body from POLYGON((...))
        open_idx = text.find("((")
        close_idx = text.rfind("))")
        if open_idx < 0 or close_idx < 0 or close_idx <= open_idx + 1:
            raise RuntimeError(f"Invalid POLYGON WKT: {wkt}")
        body = text[open_idx + 2:close_idx]

        raw_rings = re.split(r"\)\s*,\s*\(", body)
        rings = []
        for raw in raw_rings:
            pts = []
            for token in raw.split(","):
                pair = token.strip().split()
                if len(pair) < 2:
                    continue
                pts.append((float(pair[0]), float(pair[1])))
            if len(pts) < 3:
                raise RuntimeError(f"Ring has fewer than 3 points in WKT: {wkt}")
            if pts[0] != pts[-1]:
                pts.append(pts[0])
            rings.append(pts)
        return rings

    def ring_to_msg(self, ring: List[Tuple[float, float]]) -> Coordinates:
        coords = Coordinates()
        for x, y in ring:
            c = Coordinate()
            c.axis1 = float(x)
            c.axis2 = float(y)
            coords.coordinates.append(c)
        return coords

    def downsample_poses(self, poses: List[PoseStamped], spacing: float) -> List[PoseStamped]:
        if len(poses) <= 2:
            return poses

        sampled = [poses[0]]
        last = poses[0]
        for p in poses[1:]:
            dx = p.pose.position.x - last.pose.position.x
            dy = p.pose.position.y - last.pose.position.y
            if math.hypot(dx, dy) >= spacing:
                sampled.append(p)
                last = p

        if sampled[-1] != poses[-1]:
            sampled.append(poses[-1])
        return sampled


def main():
    rclpy.init()
    node = OpenNavCoverageWaypointNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
