#!/usr/bin/env python3
"""Republish Odin topics for open3d_loc + ego + dddmr (Mid360 stack untouched).

open3d_loc expects /cloud_registered_1 in the odom/camera_init frame
(same as Mid360 FAST-LIO registered cloud), then crops with an OBB at
baselink2odom and applies odom→map. Do NOT convert slam cloud to body
for that topic.

Also publish a body-frame local cloud for dddmr perception.
"""
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs_py import point_cloud2 as pc2


def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


class OdinLocAdapter(Node):
    def __init__(self):
        super().__init__("odin1_loc_topic_adapter")
        self.declare_parameter("odom_in", "/odin1/odometry")
        self.declare_parameter("cloud_slam_in", "/odin1/cloud_slam")
        self.declare_parameter("odom_out", "/Odometry_loc")
        self.declare_parameter("cloud_out", "/cloud_registered_1")
        self.declare_parameter("cloud_body_out", "/cloud_registered_body_1")
        self.declare_parameter("body_frame", "body")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("max_points_body", 40000)
        self.declare_parameter("body_radius", 40.0)

        self.body = self.get_parameter("body_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.max_points_body = int(self.get_parameter("max_points_body").value)
        self.body_radius = float(self.get_parameter("body_radius").value)
        self._lock = threading.Lock()
        self._latest_odom = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.tf_br = TransformBroadcaster(self)
        self.pub_odom = self.create_publisher(
            Odometry, self.get_parameter("odom_out").value, 50
        )
        self.pub_cloud = self.create_publisher(
            PointCloud2, self.get_parameter("cloud_out").value, 10
        )
        self.pub_cloud_body = self.create_publisher(
            PointCloud2, self.get_parameter("cloud_body_out").value, 10
        )
        self.create_subscription(
            Odometry, self.get_parameter("odom_in").value, self.on_odom, 50
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("cloud_slam_in").value,
            self.on_cloud_slam,
            qos,
        )
        self.get_logger().info(
            "adapter: /Odometry_loc + /cloud_registered_1(odom) + "
            f"/cloud_registered_body_1({self.body})"
        )

    def on_odom(self, msg: Odometry):
        with self._lock:
            self._latest_odom = msg
        stamp = self.get_clock().now().to_msg()
        out = Odometry()
        out.header = msg.header
        out.header.stamp = stamp
        if not out.header.frame_id:
            out.header.frame_id = self.odom_frame
        out.child_frame_id = self.body
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub_odom.publish(out)

        t = TransformStamped()
        t.header = out.header
        t.child_frame_id = self.body
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_br.sendTransform(t)

    def on_cloud_slam(self, msg: PointCloud2):
        # Restamp to Jetson clock (slightly in the past) so ego can look up
        # map<-odom TF that open3d_loc publishes from odometry callbacks.
        stamp = (self.get_clock().now() - rclpy.duration.Duration(seconds=0.1)).to_msg()
        # open3d_loc / ego: keep odom-frame registered cloud
        out = PointCloud2()
        out.header = msg.header
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = msg.data
        out.is_dense = msg.is_dense
        self.pub_cloud.publish(out)

        with self._lock:
            odom = self._latest_odom
        if odom is None:
            return
        try:
            pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        except Exception as e:
            self.get_logger().error(f"read slam cloud failed: {e}")
            return
        if not pts:
            return
        arr = np.asarray([(p[0], p[1], p[2]) for p in pts], dtype=np.float64)
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        t = np.array([p.x, p.y, p.z], dtype=np.float64)
        body = (arr - t) @ R
        r2 = self.body_radius * self.body_radius
        mask = np.sum(body * body, axis=1) <= r2
        body = body[mask]
        if len(body) == 0:
            return
        if len(body) > self.max_points_body:
            idx = np.linspace(0, len(body) - 1, self.max_points_body).astype(np.int32)
            body = body[idx]
        header = msg.header
        header.stamp = stamp
        header.frame_id = self.body
        self.pub_cloud_body.publish(pc2.create_cloud_xyz32(header, body.astype(np.float32)))

def main():
    rclpy.init()
    node = OdinLocAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
