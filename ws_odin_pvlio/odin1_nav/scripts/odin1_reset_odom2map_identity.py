#!/usr/bin/env python3
"""Force open3d_loc map≡odom for continuous Odin mapping→nav sessions.

Publishes /initialpose (identity) until /odom2map is near identity, then exits.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class ResetOdom2Map(Node):
    def __init__(self):
        super().__init__("odin1_reset_odom2map_identity")
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.sub = self.create_subscription(Odometry, "/odom2map", self.on_o2m, 10)
        self.ok = False
        self.timer = self.create_timer(0.5, self.tick)
        self.t0 = time.time()

    def on_o2m(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if abs(p.x) < 0.15 and abs(p.y) < 0.15 and abs(p.z) < 0.25 and abs(yaw) < math.radians(8):
            self.ok = True
            self.get_logger().info(
                f"odom2map near identity: ({p.x:.3f},{p.y:.3f},{p.z:.3f}) yaw={math.degrees(yaw):.1f}deg"
            )

    def tick(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.orientation.w = 1.0
        self.pub.publish(msg)
        if self.ok or time.time() - self.t0 > 20.0:
            raise SystemExit(0 if self.ok else 1)


def main():
    rclpy.init()
    node = ResetOdom2Map()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = int(e.code) if e.code is not None else 0
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(code)


if __name__ == "__main__":
    main()
