#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay_sec', 5.0)
        self.declare_parameter('repeat_count', 3)
        self.declare_parameter('repeat_interval_sec', 1.0)
        self.declare_parameter('wait_for_odom', True)
        self.declare_parameter('wait_for_scan', True)
        self.declare_parameter('startup_timeout_sec', 20.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.delay_sec = float(self.get_parameter('delay_sec').value)
        self.repeat_count = int(self.get_parameter('repeat_count').value)
        self.repeat_interval_sec = float(self.get_parameter('repeat_interval_sec').value)
        self.wait_for_odom = bool(self.get_parameter('wait_for_odom').value)
        self.wait_for_scan = bool(self.get_parameter('wait_for_scan').value)
        self.startup_timeout_sec = float(self.get_parameter('startup_timeout_sec').value)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.odom_ready = not self.wait_for_odom
        self.scan_ready = not self.wait_for_scan
        self.started_waiting = False
        self.sent_count = 0
        self.timer = self.create_timer(self.delay_sec, self._start_publishing)
        self.publish_timer = None
        self.shutdown_timer = None
        self.timeout_timer = None
        self.odom_sub = None
        self.scan_sub = None

        if self.wait_for_odom:
            self.odom_sub = self.create_subscription(
                Odometry, '/odom', self._on_odom, 10)
        if self.wait_for_scan:
            self.scan_sub = self.create_subscription(
                LaserScan, '/scan', self._on_scan, 10)

    def _on_odom(self, _msg):
        if not self.odom_ready:
            self.odom_ready = True
            self.get_logger().info('Observed /odom, initial pose publisher is unblocked on odom.')
            self._maybe_begin_repeating_publish()

    def _on_scan(self, _msg):
        if not self.scan_ready:
            self.scan_ready = True
            self.get_logger().info('Observed /scan, initial pose publisher is unblocked on scan.')
            self._maybe_begin_repeating_publish()

    def _start_publishing(self):
        self.timer.cancel()
        self.started_waiting = True

        if self.odom_ready and self.scan_ready:
            self._begin_repeating_publish()
            return

        waiting_on = []
        if not self.odom_ready:
            waiting_on.append('/odom')
        if not self.scan_ready:
            waiting_on.append('/scan')
        self.get_logger().info(
            f'Waiting for {" and ".join(waiting_on)} before publishing initial pose.')
        self.timeout_timer = self.create_timer(self.startup_timeout_sec, self._publish_with_timeout)

    def _maybe_begin_repeating_publish(self):
        if self.started_waiting and self.odom_ready and self.scan_ready:
            self._begin_repeating_publish()

    def _begin_repeating_publish(self):
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        if self.publish_timer is not None:
            return
        self._publish_once()
        if self.repeat_count > 1:
            self.publish_timer = self.create_timer(self.repeat_interval_sec, self._publish_once)

    def _publish_with_timeout(self):
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None
        self.get_logger().warning(
            'Startup timeout reached before all required topics were observed. '
            'Publishing initial pose anyway.')
        self._begin_repeating_publish()

    def _publish_once(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        half_yaw = self.yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        covariance = [0.0] * 36
        covariance[0] = 0.25
        covariance[7] = 0.25
        covariance[35] = 0.068
        msg.pose.covariance = covariance

        self.publisher.publish(msg)
        self.sent_count += 1
        self.get_logger().info(
            f'Published initial pose {self.sent_count}/{self.repeat_count}: '
            f'x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}')

        if self.sent_count >= self.repeat_count and self.publish_timer is not None:
            self.publish_timer.cancel()
            self.shutdown_timer = self.create_timer(0.5, self._shutdown)

    def _shutdown(self):
        if self.shutdown_timer is not None:
            self.shutdown_timer.cancel()
        self.get_logger().info('Initial pose publishing complete.')
        self.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
