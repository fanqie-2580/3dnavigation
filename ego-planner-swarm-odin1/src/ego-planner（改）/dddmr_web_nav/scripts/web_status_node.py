#!/usr/bin/env python3

from __future__ import annotations

import json
from typing import Dict

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class WebStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("web_status_node")

        self.declare_parameter("map_topic", "/web/mapground")
        self.declare_parameter("ego_obstacle_topic", "/web/ego_obstacles")
        self.declare_parameter("odom_topic", "/baselink2map")
        self.declare_parameter("path_topic", "/global_path")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("status_topic", "/web_nav/status")
        self.declare_parameter("alive_timeout_sec", 2.0)
        self.declare_parameter("publish_period_sec", 1.0)

        self._last_seen: Dict[str, rclpy.time.Time] = {}
        self._last_map_points = 0
        self._last_ego_obstacle_points = 0
        self._last_path_size = 0
        self._last_goal = None

        self.create_subscription(Marker, self.get_parameter("map_topic").value, self._on_map, 1)
        self.create_subscription(
            Marker,
            self.get_parameter("ego_obstacle_topic").value,
            self._on_ego_obstacle,
            1,
        )
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10)
        self.create_subscription(Path, self.get_parameter("path_topic").value, self._on_path, 10)
        self.create_subscription(
            PointStamped,
            self.get_parameter("clicked_point_topic").value,
            self._on_clicked_point,
            10,
        )
        self._status_pub = self.create_publisher(String, self.get_parameter("status_topic").value, 10)

        period = max(0.2, float(self.get_parameter("publish_period_sec").value))
        self.create_timer(period, self._publish_status)

        self.get_logger().info(
            "web_status_node started. status=%s" % self.get_parameter("status_topic").value
        )

    def _mark_seen(self, key: str) -> None:
        self._last_seen[key] = self.get_clock().now()

    def _on_map(self, msg: Marker) -> None:
        self._mark_seen("map")
        self._last_map_points = len(msg.points)

    def _on_ego_obstacle(self, msg: Marker) -> None:
        self._mark_seen("ego_obstacle")
        self._last_ego_obstacle_points = len(msg.points)

    def _on_odom(self, msg: Odometry) -> None:
        self._mark_seen("odom")

    def _on_path(self, msg: Path) -> None:
        self._mark_seen("path")
        self._last_path_size = len(msg.poses)

    def _on_clicked_point(self, msg: PointStamped) -> None:
        self._mark_seen("clicked_point")
        self._last_goal = {
            "x": msg.point.x,
            "y": msg.point.y,
            "z": msg.point.z,
            "frame_id": msg.header.frame_id,
        }

    def _is_alive(self, key: str, now: rclpy.time.Time, timeout_sec: float) -> bool:
        stamp = self._last_seen.get(key)
        if stamp is None:
            return False
        return (now - stamp).nanoseconds * 1.0e-9 <= timeout_sec

    def _publish_status(self) -> None:
        now = self.get_clock().now()
        timeout_sec = max(0.1, float(self.get_parameter("alive_timeout_sec").value))
        payload = {
            "map_alive": self._is_alive("map", now, timeout_sec),
            "ego_obstacle_alive": self._is_alive("ego_obstacle", now, timeout_sec),
            "odom_alive": self._is_alive("odom", now, timeout_sec),
            "path_alive": self._is_alive("path", now, timeout_sec),
            "clicked_point_alive": self._is_alive("clicked_point", now, timeout_sec),
            "map_points": self._last_map_points,
            "ego_obstacle_points": self._last_ego_obstacle_points,
            "path_size": self._last_path_size,
            "last_goal": self._last_goal,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebStatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
