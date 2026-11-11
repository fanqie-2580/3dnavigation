#!/usr/bin/env python3
"""TCP -> ROS2 (Jetson) bridge client for Odin imu/cloud_slam/odometry."""
import socket
import statistics
import struct
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter("cloud_topic", "/odin1/cloud_slam")
        self.declare_parameter("odom_topic", "/odin1/odometry")
        # Dock clock is often wrong. Restamp on receive, then subtract the
        # offset of /Odometry (Livox scan time). /livox/imu is host time and
        # sits ~1s ahead of the lidar stamp the pose factor compares against.
        self.declare_parameter("restamp_on_receive", True)
        self.declare_parameter("sync_to_livox", True)
        self.declare_parameter("livox_clock_topic", "/Odometry")
        self.host = self.get_parameter("server_host").value
        self.port = int(self.get_parameter("server_port").value)
        imu_topic = self.get_parameter("imu_topic").value
        cloud_topic = self.get_parameter("cloud_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.restamp_on_receive = bool(self.get_parameter("restamp_on_receive").value)
        self.sync_to_livox = bool(self.get_parameter("sync_to_livox").value)
        self._offsets = deque(maxlen=40)
        self._offset = None
        self._offset_lock = threading.Lock()

        self.pub_imu = self.create_publisher(Imu, imu_topic, 200)
        self.pub_cloud = self.create_publisher(PointCloud2, cloud_topic, 10)
        # also republish as cloud_raw alias only if topic is slam (mapper may listen slam)
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 50)
        if self.sync_to_livox:
            clock_qos = QoSProfile(
                depth=50,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter("livox_clock_topic").value),
                self._on_livox_clock,
                clock_qos,
            )
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"bridging tcp://{self.host}:{self.port} -> {imu_topic}, {cloud_topic}, {odom_topic} "
            f"restamp={self.restamp_on_receive} sync_to_livox={self.sync_to_livox}"
        )

    def _on_livox_clock(self, msg: Odometry) -> None:
        st = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if st < 1.0e9:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._offset_lock:
            if self._offset is not None and len(self._offsets) >= 30:
                return
            self._offsets.append(now - st)
            self._offset = float(statistics.median(self._offsets))
            frozen = len(self._offsets) >= 30
        if frozen:
            self.get_logger().info(
                f"Livox lidar clock offset frozen at {self._offset:.3f}s "
                f"(from {self.get_parameter('livox_clock_topic').value})"
            )

    def _livox_stamp(self):
        now = self.get_clock().now()
        with self._offset_lock:
            off = self._offset
        if off is None:
            return None
        t = now.nanoseconds * 1e-9 - off
        sec = int(t)
        nsec = int(round((t - sec) * 1e9))
        if nsec >= 1000000000:
            sec += 1
            nsec -= 1000000000
        if nsec < 0:
            sec -= 1
            nsec += 1000000000
        return Time(sec=sec, nanosec=nsec)

    def _maybe_restamp(self, msg):
        if not self.restamp_on_receive:
            return msg
        if self.sync_to_livox:
            stamp = self._livox_stamp()
            if stamp is None:
                return None
            msg.header.stamp = stamp
            return msg
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

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
        # Dense cloud_slam frames can exceed 5s on a congested dock link; a short
        # socket timeout caused reconnect flaps that looked like Odin "delay".
        sock = socket.create_connection((self.host, self.port), timeout=10.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
        except Exception:
            pass
        sock.settimeout(60.0)
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
                    msg = self._maybe_restamp(decode_imu(payload))
                    if msg is not None:
                        self.pub_imu.publish(msg)
                elif msg_type == TYPE_CLOUD:
                    msg = self._maybe_restamp(decode_cloud(payload))
                    if msg is not None:
                        self.pub_cloud.publish(msg)
                elif msg_type == TYPE_ODOM:
                    msg = self._maybe_restamp(decode_odom(payload))
                    if msg is not None:
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
