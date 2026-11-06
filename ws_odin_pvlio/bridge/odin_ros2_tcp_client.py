#!/usr/bin/env python3
"""TCP -> ROS2 (Jetson) bridge client for Odin imu/cloud_slam/odometry."""
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

MAGIC = b"ODIN"
TYPE_IMU = 1
TYPE_CLOUD = 2
TYPE_ODOM = 3
HDR_SIZE = 1 + 4


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf.extend(chunk)
    return bytes(buf)


def decode_imu(payload):
    sec, nsec, flen = struct.unpack_from("<I I H", payload, 0)
    off = 10
    frame = payload[off : off + flen].decode("utf-8")
    off += flen
    gx, gy, gz, ax, ay, az = struct.unpack_from("<6d", payload, off)
    msg = Imu()
    msg.header = Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=frame)
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x = gx
    msg.angular_velocity.y = gy
    msg.angular_velocity.z = gz
    msg.linear_acceleration.x = ax
    msg.linear_acceleration.y = ay
    msg.linear_acceleration.z = az
    return msg


def decode_cloud(payload):
    sec, nsec, flen = struct.unpack_from("<I I H", payload, 0)
    off = 10
    frame = payload[off : off + flen].decode("utf-8")
    off += flen
    height, width, be, dense, point_step, row_step, nfields = struct.unpack_from(
        "<I I B B I I H", payload, off
    )
    off += 20
    fields = []
    for _ in range(nfields):
        (nlen,) = struct.unpack_from("<H", payload, off)
        off += 2
        name = payload[off : off + nlen].decode("utf-8")
        off += nlen
        offset, datatype, count = struct.unpack_from("<I B I", payload, off)
        off += 9
        pf = PointField(name=name, offset=offset, datatype=datatype, count=count)
        fields.append(pf)
    (dlen,) = struct.unpack_from("<I", payload, off)
    off += 4
    msg = PointCloud2()
    msg.header = Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=frame)
    msg.height = height
    msg.width = width
    msg.fields = fields
    msg.is_bigendian = bool(be)
    msg.point_step = point_step
    msg.row_step = row_step
    msg.is_dense = bool(dense)
    msg.data = payload[off : off + dlen]
    return msg


def decode_odom(payload):
    sec, nsec, flen = struct.unpack_from("<I I H", payload, 0)
    off = 10
    frame = payload[off : off + flen].decode("utf-8")
    off += flen
    (clen,) = struct.unpack_from("<H", payload, off)
    off += 2
    child = payload[off : off + clen].decode("utf-8")
    off += clen
    px, py, pz, qx, qy, qz, qw = struct.unpack_from("<7d", payload, off)
    msg = Odometry()
    msg.header = Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=frame)
    msg.child_frame_id = child
    msg.pose = PoseWithCovariance()
    msg.pose.pose.position.x = px
    msg.pose.pose.position.y = py
    msg.pose.pose.position.z = pz
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    msg.twist = TwistWithCovariance()
    return msg


class OdinBridgeClient(Node):
    def __init__(self):
        super().__init__("odin_ros2_tcp_client")
        self.declare_parameter("server_host", "192.168.123.18")
        self.declare_parameter("server_port", 5601)
        self.declare_parameter("imu_topic", "/odin1/imu")
        self.declare_parameter("cloud_topic", "/odin1/cloud_raw")
        self.declare_parameter("odom_topic", "/odin1/odometry")
        # Dock often publishes device uptime; map once onto host ROS time.
        self.declare_parameter("align_to_host_time", True)
        self.declare_parameter("uptime_stamp_threshold_sec", 1.6e9)
        # device: keep aligned device stamp; host_receive: stamp clouds at arrival
        # (fusion with mode0 lidar-frame raw — kills multi-second systematic lag).
        self.declare_parameter("cloud_stamp_policy", "host_receive")
        # Drop only extreme backlog (after policy). Soft gate; dock drop-old is primary.
        self.declare_parameter("max_cloud_age_sec", 1.0)
        self.host = self.get_parameter("server_host").value
        self.port = int(self.get_parameter("server_port").value)
        imu_topic = self.get_parameter("imu_topic").value
        cloud_topic = self.get_parameter("cloud_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.align_to_host_time = bool(self.get_parameter("align_to_host_time").value)
        self.uptime_thr = float(self.get_parameter("uptime_stamp_threshold_sec").value)
        self.cloud_stamp_policy = str(self.get_parameter("cloud_stamp_policy").value).strip().lower()
        self.max_cloud_age_sec = float(self.get_parameter("max_cloud_age_sec").value)
        self._time_offset_sec = None  # host = device + offset
        self._n_cloud_drop_age = 0

        self.pub_imu = self.create_publisher(Imu, imu_topic, 200)
        self.pub_cloud = self.create_publisher(PointCloud2, cloud_topic, 10)
        # also republish as cloud_raw alias only if topic is slam (mapper may listen slam)
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 50)
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.create_timer(2.0, self._age_stats)
        self.get_logger().info(
            f"bridging tcp://{self.host}:{self.port} -> {imu_topic}, {cloud_topic}, {odom_topic} "
            f"align_to_host_time={self.align_to_host_time} "
            f"cloud_stamp_policy={self.cloud_stamp_policy} max_cloud_age={self.max_cloud_age_sec:.2f}s"
        )

    def _age_stats(self) -> None:
        if self._n_cloud_drop_age:
            self.get_logger().info(
                f"tcp client dropped {self._n_cloud_drop_age} stale clouds "
                f"(age>{self.max_cloud_age_sec:.2f}s)"
            )
            self._n_cloud_drop_age = 0

    def _stamp_to_sec(self, stamp: Time) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _sec_to_stamp(self, t: float) -> Time:
        if t < 0.0:
            t = 0.0
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        if nsec >= 1_000_000_000:
            sec += 1
            nsec -= 1_000_000_000
        return Time(sec=sec, nanosec=nsec)

    def _align_header_stamp(self, stamp: Time) -> Time:
        if not self.align_to_host_time:
            return stamp
        device_t = self._stamp_to_sec(stamp)
        if device_t >= self.uptime_thr:
            return stamp  # already host-like
        host_now = self.get_clock().now().nanoseconds * 1e-9
        if self._time_offset_sec is None:
            self._time_offset_sec = host_now - device_t
            self.get_logger().info(
                f"Odin device-time offset locked: {self._time_offset_sec:.3f} s "
                f"(device={device_t:.3f} host={host_now:.3f})"
            )
        return self._sec_to_stamp(device_t + self._time_offset_sec)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()

    def _run(self):
        while not self._stop and rclpy.ok():
            try:
                self._session()
            except Exception as e:
                self.get_logger().warn(f"bridge reconnect soon: {e}")
                time.sleep(1.0)

    def _session(self):
        sock = socket.create_connection((self.host, self.port), timeout=5.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(5.0)
        self.get_logger().info(f"connected to {self.host}:{self.port}")
        try:
            while not self._stop and rclpy.ok():
                magic = recv_exact(sock, 4)
                if magic != MAGIC:
                    raise RuntimeError(f"bad magic {magic!r}")
                hdr = recv_exact(sock, HDR_SIZE)
                msg_type, length = struct.unpack("<BI", hdr)
                payload = recv_exact(sock, length)
                if msg_type == TYPE_IMU:
                    msg = decode_imu(payload)
                    msg.header.stamp = self._align_header_stamp(msg.header.stamp)
                    self.pub_imu.publish(msg)
                elif msg_type == TYPE_CLOUD:
                    msg = decode_cloud(payload)
                    msg.header.stamp = self._align_header_stamp(msg.header.stamp)
                    host_now = self.get_clock().now().nanoseconds * 1e-9
                    if self.cloud_stamp_policy in ("host_receive", "arrival", "receive"):
                        # Stamp slightly behind host_now so EXACT Mid pair (|dt|~Mid lag) hits.
                        msg.header.stamp = self._sec_to_stamp(host_now - 0.05)
                    elif self.max_cloud_age_sec > 0.0:
                        age = host_now - self._stamp_to_sec(msg.header.stamp)
                        if age > self.max_cloud_age_sec:
                            self._n_cloud_drop_age += 1
                            continue
                    self.pub_cloud.publish(msg)
                elif msg_type == TYPE_ODOM:
                    msg = decode_odom(payload)
                    msg.header.stamp = self._align_header_stamp(msg.header.stamp)
                    self.pub_odom.publish(msg)
        finally:
            try:
                sock.close()
            except Exception:
                pass


def main():
    rclpy.init()
    node = OdinBridgeClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
