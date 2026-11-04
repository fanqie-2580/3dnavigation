#!/usr/bin/env python3
"""Voxel-downsample Odin1 /odin1/cloud_raw for FAST_LIO_MULTI second lidar input.

Preserves intensity (or fills 0) so PCL XYZI conversion does not warn.

Odin TCP stamps are device uptime (~1e4), not ROS time. Restamp to the latest
Mid /Odometry stamp (same clock as FAST-LIO) so fusion can time-align.

Publish at most one downsampled cloud per Mid odom stamp. Odin is ~30 Hz while
Mid odom is ~10 Hz; reusing the same stamp for several scans makes LIO/viz apply
one pose to multiple body orientations → orange fan on yaw.
"""
from __future__ import annotations

import struct
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class OdinRawDownsampleBridge(Node):
    def __init__(self) -> None:
        super().__init__('odin_raw_downsample_bridge')
        self.declare_parameter('input_topic', '/odin1/cloud_raw')
        self.declare_parameter('output_topic', '/odin1/cloud_raw_ds')
        self.declare_parameter('voxel_size', 0.08)
        self.declare_parameter('max_points', 20000)
        self.declare_parameter('restamp_from_odom', True)
        self.declare_parameter('one_cloud_per_odom', True)
        self.declare_parameter('odom_topic', '/Odometry')

        self.voxel = float(self.get_parameter('voxel_size').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.restamp_from_odom = bool(self.get_parameter('restamp_from_odom').value)
        self.one_cloud_per_odom = bool(self.get_parameter('one_cloud_per_odom').value)
        inn = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self._odom_stamp: Optional[object] = None
        self._odom_stamp_key: Optional[Tuple[int, int]] = None
        self._last_pub_key: Optional[Tuple[int, int]] = None
        self._pending: Optional[PointCloud2] = None

        # Match FAST_LIO SensorDataQoS (BEST_EFFORT) so laserMapping always receives.
        from rclpy.qos import qos_profile_sensor_data
        self.pub = self.create_publisher(PointCloud2, out, qos_profile_sensor_data)
        self.sub = self.create_subscription(PointCloud2, inn, self.cb, qos_profile_sensor_data)
        if self.restamp_from_odom:
            self.create_subscription(
                Odometry, self.get_parameter('odom_topic').value, self.on_odom, 50
            )
        self.get_logger().info(
            f'downsample {inn} -> {out} voxel={self.voxel}m max_points={self.max_points} '
            f'restamp_from_odom={self.restamp_from_odom} '
            f'one_cloud_per_odom={self.one_cloud_per_odom}'
        )

    def on_odom(self, msg: Odometry) -> None:
        key = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        self._odom_stamp = msg.header.stamp
        self._odom_stamp_key = key
        # Flush newest pending Odin scan exactly once per Mid odom tick.
        if self.one_cloud_per_odom and self._pending is not None and self._last_pub_key != key:
            self._publish_arr_from_msg(self._pending, msg.header.stamp)
            self._last_pub_key = key
            self._pending = None

    def cb(self, msg: PointCloud2) -> None:
        # Before Mid odom exists, still publish (restamp=now). Bundle sync needs
        # lidar2; gating on odom here deadlocks LIO (no ds → no odom → no ds).
        if self.restamp_from_odom and self._odom_stamp is None:
            self._publish_arr_from_msg(msg, self.get_clock().now().to_msg())
            return

        if self.restamp_from_odom and self.one_cloud_per_odom:
            # Keep latest scan; publish once per fresh Mid odom stamp.
            self._pending = msg
            if (
                self._odom_stamp_key is not None
                and self._last_pub_key != self._odom_stamp_key
            ):
                self._publish_arr_from_msg(msg, self._odom_stamp)
                self._last_pub_key = self._odom_stamp_key
                self._pending = None
            return
        stamp = self._odom_stamp if (self.restamp_from_odom and self._odom_stamp is not None) else self.get_clock().now().to_msg()
        self._publish_arr_from_msg(msg, stamp)

    def _publish_arr_from_msg(self, msg: PointCloud2, stamp) -> None:
        names = [f.name for f in msg.fields]
        want = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
        pts = list(point_cloud2.read_points(msg, field_names=want, skip_nans=True))
        if not pts:
            return
        if len(want) == 4:
            arr = np.asarray([[p[0], p[1], p[2], p[3]] for p in pts], dtype=np.float32)
        else:
            arr = np.asarray([[p[0], p[1], p[2], 0.0] for p in pts], dtype=np.float32)

        if self.voxel > 0:
            keys = np.floor(arr[:, :3] / self.voxel).astype(np.int32)
            _, idx = np.unique(keys, axis=0, return_index=True)
            arr = arr[np.sort(idx)]
        if len(arr) > self.max_points:
            step = int(np.ceil(len(arr) / self.max_points))
            arr = arr[::step]

        header = Header()
        header.stamp = stamp
        header.frame_id = msg.header.frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        data = b''.join(struct.pack('<ffff', float(x), float(y), float(z), float(i)) for x, y, z, i in arr)
        out = PointCloud2()
        out.header = header
        out.height = 1
        out.width = len(arr)
        out.fields = fields
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * len(arr)
        out.is_dense = True
        out.data = data
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = OdinRawDownsampleBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
