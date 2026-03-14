#!/usr/bin/env python3

import math
from typing import Optional

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def _quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_normalize(q):
    norm = math.sqrt(sum(v * v for v in q))
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / norm for v in q)


def _rotate_vector(q, v):
    qn = _quat_normalize(q)
    vx, vy, vz = v
    vq = (vx, vy, vz, 0.0)
    rq = _quat_multiply(_quat_multiply(qn, vq), _quat_conjugate(qn))
    return (rq[0], rq[1], rq[2])


def _invert_transform(translation, rotation):
    q_inv = _quat_normalize(_quat_conjugate(rotation))
    t_inv = _rotate_vector(q_inv, (-translation[0], -translation[1], -translation[2]))
    return t_inv, q_inv


def _compose_transform(t1, q1, t2, q2):
    t_rot = _rotate_vector(q1, t2)
    t_out = (t1[0] + t_rot[0], t1[1] + t_rot[1], t1[2] + t_rot[2])
    q_out = _quat_normalize(_quat_multiply(q1, q2))
    return t_out, q_out


class GazeboGroundTruthTF(Node):
    def __init__(self):
        super().__init__('gazebo_ground_truth_tf')

        self.declare_parameter('ground_truth_topic', '/ground_truth/model_pose')
        self.declare_parameter('ground_truth_child_frame', 'burger')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('lookup_timeout_sec', 0.1)

        self.ground_truth_topic = self.get_parameter('ground_truth_topic').value
        self.ground_truth_child_frame = self.get_parameter('ground_truth_child_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.lookup_timeout = Duration(seconds=float(
            self.get_parameter('lookup_timeout_sec').value))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_warn_missing_gt = False
        self.subscription = self.create_subscription(
            TFMessage, self.ground_truth_topic, self._on_ground_truth, 10)

    def _find_ground_truth_transform(self, msg: TFMessage) -> Optional[TransformStamped]:
        candidates = (
            self.ground_truth_child_frame,
            f'{self.ground_truth_child_frame}',
            f'{self.ground_truth_child_frame}/{self.robot_base_frame}',
            self.robot_base_frame,
        )
        for transform in msg.transforms:
            child_frame = transform.child_frame_id.lstrip('/')
            if any(
                child_frame == candidate or child_frame.endswith(f'/{candidate}')
                for candidate in candidates
            ):
                return transform
        return None

    def _on_ground_truth(self, msg: TFMessage):
        gt_transform = self._find_ground_truth_transform(msg)
        if gt_transform is None:
            if not self.last_warn_missing_gt:
                child_frames = ', '.join(t.child_frame_id for t in msg.transforms[:10])
                self.get_logger().warning(
                    f'Ground-truth transform for "{self.ground_truth_child_frame}" not found. '
                    f'Available child frames include: {child_frames}')
                self.last_warn_missing_gt = True
            return

        self.last_warn_missing_gt = False

        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.robot_base_frame,
                rclpy.time.Time(),
                timeout=self.lookup_timeout)
        except TransformException as exc:
            self.get_logger().debug(f'Waiting for {self.odom_frame}->{self.robot_base_frame}: {exc}')
            return

        gt_t = (
            gt_transform.transform.translation.x,
            gt_transform.transform.translation.y,
            gt_transform.transform.translation.z,
        )
        gt_q = (
            gt_transform.transform.rotation.x,
            gt_transform.transform.rotation.y,
            gt_transform.transform.rotation.z,
            gt_transform.transform.rotation.w,
        )

        odom_t = (
            odom_to_base.transform.translation.x,
            odom_to_base.transform.translation.y,
            odom_to_base.transform.translation.z,
        )
        odom_q = (
            odom_to_base.transform.rotation.x,
            odom_to_base.transform.rotation.y,
            odom_to_base.transform.rotation.z,
            odom_to_base.transform.rotation.w,
        )

        inv_odom_t, inv_odom_q = _invert_transform(odom_t, odom_q)
        map_to_odom_t, map_to_odom_q = _compose_transform(gt_t, gt_q, inv_odom_t, inv_odom_q)

        correction = TransformStamped()
        correction.header.stamp = gt_transform.header.stamp
        if correction.header.stamp.sec == 0 and correction.header.stamp.nanosec == 0:
            correction.header.stamp = self.get_clock().now().to_msg()
        correction.header.frame_id = self.map_frame
        correction.child_frame_id = self.odom_frame
        correction.transform.translation.x = map_to_odom_t[0]
        correction.transform.translation.y = map_to_odom_t[1]
        correction.transform.translation.z = map_to_odom_t[2]
        correction.transform.rotation.x = map_to_odom_q[0]
        correction.transform.rotation.y = map_to_odom_q[1]
        correction.transform.rotation.z = map_to_odom_q[2]
        correction.transform.rotation.w = map_to_odom_q[3]
        self.tf_broadcaster.sendTransform(correction)


def main():
    rclpy.init()
    node = GazeboGroundTruthTF()
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
