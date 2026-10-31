#!/usr/bin/env python3
"""Accumulate Odin cloud_raw into a map using Odin's own /odin1/odometry (not PV/CT-LIO)."""
import math
import threading
from pathlib import Path as FsPath

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from sensor_msgs_py import point_cloud2 as pc2


def quat_to_rot(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def se3_from_pose(pose):
    p = pose.position
    q = pose.orientation
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(q.x, q.y, q.z, q.w)
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class OdinOdomMapper(Node):
    def __init__(self):
        super().__init__("odin_odom_mapper")
        self.declare_parameter("cloud_topic", "/odin1/cloud_raw")
        self.declare_parameter("odom_topic", "/odin1/odometry")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("save_map_path", "/home/ll/ws_odin_pvlio/maps/odin1_odom_map.pcd")
        self.declare_parameter("voxel_size", 0.05)
        self.declare_parameter("min_confidence", 30)
        self.declare_parameter("blind", 0.2)
        # imu -> lidar from live Odin TF (nearly identity)
        self.declare_parameter("imu_to_lidar_xyz", [-0.004, 0.013, 0.026])
        self.declare_parameter("imu_to_lidar_rpy_deg", [0.66, -0.11, -0.29])

        self.map_frame = self.get_parameter("map_frame").value
        self.save_path = self.get_parameter("save_map_path").value
        self.voxel = float(self.get_parameter("voxel_size").value)
        self.min_conf = int(self.get_parameter("min_confidence").value)
        self.blind2 = float(self.get_parameter("blind").value) ** 2

        xyz = list(self.get_parameter("imu_to_lidar_xyz").value)
        rpy = [math.radians(v) for v in self.get_parameter("imu_to_lidar_rpy_deg").value]
        cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
        cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
        cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        self.T_imu_lidar = np.eye(4)
        self.T_imu_lidar[:3, :3] = Rz @ Ry @ Rx
        self.T_imu_lidar[:3, 3] = xyz

        self._lock = threading.Lock()
        self._latest_odom = None
        self._map_pts = []  # list of Nx3 arrays, voxel-merged periodically
        self._voxel_dict = {}
        self._path = Path()
        self._path.header.frame_id = self.map_frame

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.pub_scan = self.create_publisher(PointCloud2, "/cloud_registered", 10)
        self.pub_map = self.create_publisher(PointCloud2, "/Laser_map", 1)
        self.pub_odom = self.create_publisher(Odometry, "/Odometry", 50)
        self.pub_path = self.create_publisher(Path, "/path", 10)
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # map == odom in Odin mode 0
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = self.map_frame
        static_tf.child_frame_id = "odom"
        static_tf.transform.rotation.w = 1.0
        self.static_br.sendTransform(static_tf)

        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom, 50
        )
        self.create_subscription(
            PointCloud2, self.get_parameter("cloud_topic").value, self.on_cloud, qos
        )
        self.create_service(Trigger, "/save_map", self.on_save)
        self.create_timer(1.0, self.publish_map)
        self.get_logger().info(
            f"Odin-odom mapper ready. save -> {self.save_path} voxel={self.voxel}"
        )

    def on_odom(self, msg: Odometry):
        with self._lock:
            self._latest_odom = msg
        # republish for RViz
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.map_frame
        out.child_frame_id = "base_link"
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub_odom.publish(out)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_br.sendTransform(t)

        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.map_frame
        ps.pose = msg.pose.pose
        self._path.header.stamp = msg.header.stamp
        self._path.poses.append(ps)
        if len(self._path.poses) > 20000:
            self._path.poses = self._path.poses[-10000:]
        self.pub_path.publish(self._path)

    def on_cloud(self, msg: PointCloud2):
        with self._lock:
            odom = self._latest_odom
        if odom is None:
            return

        names = [f.name for f in msg.fields]
        fields = ["x", "y", "z"]
        has_conf = "confidence" in names
        if has_conf:
            fields.append("confidence")
        try:
            pts = list(pc2.read_points(msg, field_names=fields, skip_nans=True))
        except Exception as e:
            self.get_logger().error(f"read_points failed: {e}")
            return
        if not pts:
            return

        arr = np.zeros((len(pts), 3), dtype=np.float64)
        keep = []
        for i, p in enumerate(pts):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            conf = int(p[3]) if has_conf else 255
            if conf < self.min_conf:
                continue
            if x * x + y * y + z * z < self.blind2:
                continue
            keep.append((x, y, z))
        if not keep:
            return
        arr = np.asarray(keep, dtype=np.float64)

        T_odom_imu = se3_from_pose(odom.pose.pose)
        T_odom_lidar = T_odom_imu @ self.T_imu_lidar
        R = T_odom_lidar[:3, :3]
        t = T_odom_lidar[:3, 3]
        world = (arr @ R.T) + t

        # publish current scan
        header = msg.header
        header.frame_id = self.map_frame
        scan_msg = pc2.create_cloud_xyz32(header, world.astype(np.float32))
        self.pub_scan.publish(scan_msg)

        # insert into voxel map
        inv = 1.0 / max(self.voxel, 1e-3)
        keys = np.floor(world * inv).astype(np.int32)
        with self._lock:
            for (kx, ky, kz), pt in zip(keys, world):
                self._voxel_dict[(int(kx), int(ky), int(kz))] = pt

    def publish_map(self):
        with self._lock:
            if not self._voxel_dict:
                return
            pts = np.asarray(list(self._voxel_dict.values()), dtype=np.float32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        self.pub_map.publish(pc2.create_cloud_xyz32(header, pts))

    def on_save(self, request, response):
        with self._lock:
            if not self._voxel_dict:
                response.success = False
                response.message = "map empty"
                return response
            pts = np.asarray(list(self._voxel_dict.values()), dtype=np.float32)
        path = FsPath(self.save_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        # binary PCD
        with open(path, "w", encoding="ascii") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")
            f.write("FIELDS x y z\n")
            f.write("SIZE 4 4 4\n")
            f.write("TYPE F F F\n")
            f.write("COUNT 1 1 1\n")
            f.write(f"WIDTH {len(pts)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(pts)}\n")
            f.write("DATA binary\n")
        with open(path, "ab") as f:
            f.write(pts.astype("<f4").tobytes())
        response.success = True
        response.message = f"Saved {len(pts)} points to {path}"
        self.get_logger().info(response.message)
        return response


def main():
    rclpy.init()
    node = OdinOdomMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
