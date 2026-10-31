#!/usr/bin/env python3
"""TCP -> ROS2 (Jetson) bridge client for Odin imu/cloud_raw/odometry."""
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
        self.host = self.get_parameter("server_host").value
        self.port = int(self.get_parameter("server_port").value)
        imu_topic = self.get_parameter("imu_topic").value
        cloud_topic = self.get_parameter("cloud_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.pub_imu = self.create_publisher(Imu, imu_topic, 200)
        self.pub_cloud = self.create_publisher(PointCloud2, cloud_topic, 10)
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 50)
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"bridging tcp://{self.host}:{self.port} -> {imu_topic}, {cloud_topic}, {odom_topic}"
        )

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
                    self.pub_imu.publish(decode_imu(payload))
                elif msg_type == TYPE_CLOUD:
                    self.pub_cloud.publish(decode_cloud(payload))
                elif msg_type == TYPE_ODOM:
                    self.pub_odom.publish(decode_odom(payload))
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
