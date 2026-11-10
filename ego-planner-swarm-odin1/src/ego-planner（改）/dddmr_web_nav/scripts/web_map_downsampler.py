#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker


VoxelKey = Tuple[int, int, int]


class WebMapDownsampler(Node):
    def __init__(self) -> None:
        super().__init__("web_map_downsampler")

        self.declare_parameter("input_topic", "/mapground")
        self.declare_parameter("output_topic", "/web/mapground")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("sample_stride", 2)
        self.declare_parameter("max_points", 25000)
        self.declare_parameter("voxel_size", 0.12)
        self.declare_parameter("point_scale", 0.08)
        self.declare_parameter("marker_ns", "dddmr_web_mapground")
        self.declare_parameter("input_transient_local", False)
        self.declare_parameter("color_r", 0.18)
        self.declare_parameter("color_g", 0.95)
        self.declare_parameter("color_b", 0.48)
        self.declare_parameter("color_a", 0.85)

        self._latest_cloud: PointCloud2 | None = None
        self._latest_stamp_ns = -1
        self._published_stamp_ns = -1
        self._cached_marker: Marker | None = None

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        input_transient_local = bool(self.get_parameter("input_transient_local").value)

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1 if input_transient_local else 10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
            if input_transient_local
            else DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PointCloud2, input_topic, self._on_cloud, input_qos)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._marker_pub = self.create_publisher(Marker, output_topic, qos)

        period = max(0.1, float(self.get_parameter("publish_period_sec").value))
        self.create_timer(period, self._publish_downsampled_marker)

        self.get_logger().info(
            f"web_map_downsampler started. input={input_topic} output={output_topic} "
            f"input_transient_local={input_transient_local}"
        )

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._latest_cloud = msg
        self._latest_stamp_ns = self._stamp_to_ns(msg)

    @staticmethod
    def _stamp_to_ns(msg: PointCloud2) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _publish_downsampled_marker(self) -> None:
        if self._latest_cloud is None:
            return
        if (
            self._latest_stamp_ns == self._published_stamp_ns
            and self._cached_marker is not None
        ):
            self._cached_marker.header.stamp = self.get_clock().now().to_msg()
            self._marker_pub.publish(self._cached_marker)
            return

        cloud = self._latest_cloud
        sample_stride = max(1, int(self.get_parameter("sample_stride").value))
        max_points = max(1, int(self.get_parameter("max_points").value))
        voxel_size = max(0.0, float(self.get_parameter("voxel_size").value))

        points: list[Point] = []
        occupied_voxels: Dict[VoxelKey, None] = {}

        for index, xyz in enumerate(
            point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)
        ):
            if index % sample_stride != 0:
                continue
            x = float(xyz[0])
            y = float(xyz[1])
            z = float(xyz[2])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if voxel_size > 1.0e-6:
                key = (
                    int(math.floor(x / voxel_size)),
                    int(math.floor(y / voxel_size)),
                    int(math.floor(z / voxel_size)),
                )
                if key in occupied_voxels:
                    continue
                occupied_voxels[key] = None

            point = Point()
            point.x = x
            point.y = y
            point.z = z
            points.append(point)
            if len(points) >= max_points:
                break

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = str(self.get_parameter("target_frame").value)
        marker.ns = str(self.get_parameter("marker_ns").value)
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(self.get_parameter("point_scale").value)
        marker.scale.y = float(self.get_parameter("point_scale").value)
        marker.color.r = float(self.get_parameter("color_r").value)
        marker.color.g = float(self.get_parameter("color_g").value)
        marker.color.b = float(self.get_parameter("color_b").value)
        marker.color.a = float(self.get_parameter("color_a").value)
        marker.points = points

        self._marker_pub.publish(marker)
        self._cached_marker = marker
        self._published_stamp_ns = self._latest_stamp_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebMapDownsampler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
