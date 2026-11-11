#!/usr/bin/env python3
"""ROS1 (dock) -> TCP bridge for Odin /imu, /cloud_slam (or raw), /odometry.

In custom_map_mode=1, prefer /odin1/cloud_slam (already in odom frame).
"""
from __future__ import print_function
import queue
import socket
import struct
import threading
import time

import rospy
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry

MAGIC = b"ODIN"
TYPE_IMU = 1
TYPE_CLOUD = 2
TYPE_ODOM = 3
DEFAULT_PORT = 5601


def pack_frame(msg_type, payload):
    return MAGIC + struct.pack("<BI", msg_type, len(payload)) + payload


def encode_imu(msg):
    frame = msg.header.frame_id.encode("utf-8")
    return struct.pack(
        "<I I H",
        msg.header.stamp.secs,
        msg.header.stamp.nsecs,
        len(frame),
    ) + frame + struct.pack(
        "<6d",
        msg.angular_velocity.x,
        msg.angular_velocity.y,
        msg.angular_velocity.z,
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z,
    )


def encode_cloud(msg):
    frame = msg.header.frame_id.encode("utf-8")
    parts = [
        struct.pack(
            "<I I H",
            msg.header.stamp.secs,
            msg.header.stamp.nsecs,
            len(frame),
        ),
        frame,
        struct.pack(
            "<I I B B I I H",
            msg.height,
            msg.width,
            1 if msg.is_bigendian else 0,
            1 if msg.is_dense else 0,
            msg.point_step,
            msg.row_step,
            len(msg.fields),
        ),
    ]
    for f in msg.fields:
        name = f.name.encode("utf-8")
        parts.append(struct.pack("<H", len(name)))
        parts.append(name)
        parts.append(struct.pack("<I B I", f.offset, f.datatype, f.count))
    parts.append(struct.pack("<I", len(msg.data)))
    parts.append(bytes(bytearray(msg.data)))
    return b"".join(parts)


def encode_odom(msg):
    frame = msg.header.frame_id.encode("utf-8")
    child = msg.child_frame_id.encode("utf-8")
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return (
        struct.pack("<I I H", msg.header.stamp.secs, msg.header.stamp.nsecs, len(frame))
        + frame
        + struct.pack("<H", len(child))
        + child
        + struct.pack("<7d", p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    )


class ClientHub(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._clients = []
        # Small queue for imu/odom. Clouds use a single latest-slot so a slow
        # TCP link never ships a backlog of stale clouds behind IMU floods.
        self._q = queue.Queue(maxsize=64)
        self._latest_cloud = None
        self._cloud_lock = threading.Lock()
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()

    def add(self, conn):
        with self._lock:
            self._clients.append(conn)
        print("[bridge-server] client +1 total=%d" % len(self._clients))

    def enqueue(self, data, is_cloud=False):
        if is_cloud:
            with self._cloud_lock:
                self._latest_cloud = data
            return
        try:
            self._q.put_nowait(data)
        except queue.Full:
            try:
                self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait(data)
            except queue.Full:
                pass

    def _write_loop(self):
        while not rospy.is_shutdown():
            # Send at most one latest cloud, then flush imu/odom. A pure
            # "cloud-first continue" starved odometry and broke unproject on
            # Jetson (skip_unproj climbed while odin_odom barely increased).
            cloud = None
            with self._cloud_lock:
                if self._latest_cloud is not None:
                    cloud = self._latest_cloud
                    self._latest_cloud = None
            if cloud is not None:
                self._broadcast(cloud)
            drained = 0
            try:
                while drained < 32:
                    data = self._q.get_nowait()
                    self._broadcast(data)
                    drained += 1
            except queue.Empty:
                pass
            if cloud is None and drained == 0:
                time.sleep(0.005)

    def _broadcast(self, data):
        dead = []
        with self._lock:
            clients = list(self._clients)
        for c in clients:
            try:
                c.sendall(data)
            except Exception:
                dead.append(c)
        if dead:
            with self._lock:
                for c in dead:
                    if c in self._clients:
                        self._clients.remove(c)
                    try:
                        c.close()
                    except Exception:
                        pass
            print("[bridge-server] client -%d total=%d" % (len(dead), len(self._clients)))


def accept_loop(sock, hub):
    while not rospy.is_shutdown():
        try:
            sock.settimeout(1.0)
            conn, addr = sock.accept()
        except socket.timeout:
            continue
        except Exception as e:
            if rospy.is_shutdown():
                break
            print("[bridge-server] accept error:", e)
            time.sleep(0.5)
            continue
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        # larger buffer helps bursty clouds
        try:
            conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        except Exception:
            pass
        print("[bridge-server] connected", addr)
        hub.add(conn)


def main():
    rospy.init_node("odin_ros1_tcp_server", anonymous=False)
    port = int(rospy.get_param("~port", DEFAULT_PORT))
    imu_topic = rospy.get_param("~imu_topic", "/odin1/imu")
    # SLAM cloud is already in odom frame; raw lidar frame needs host-side transform
    # Densify/accum: cloud_slam (odom-frame) is stable. Pose-factor uses /odometry.
    # Set ~cloud_topic:=/odin1/cloud_raw only when testing raw L2 point residuals.
    cloud_topic = rospy.get_param("~cloud_topic", "/odin1/cloud_slam")
    odom_topic = rospy.get_param("~odom_topic", "/odin1/odometry")

    hub = ClientHub()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.listen(4)
    print("[bridge-server] listen 0.0.0.0:%d" % port)
    print("[bridge-server] bridge %s + %s + %s" % (imu_topic, cloud_topic, odom_topic))

    t = threading.Thread(target=accept_loop, args=(sock, hub), daemon=True)
    t.start()

    def on_imu(msg):
        hub.enqueue(pack_frame(TYPE_IMU, encode_imu(msg)), is_cloud=False)

    def on_cloud(msg):
        hub.enqueue(pack_frame(TYPE_CLOUD, encode_cloud(msg)), is_cloud=True)

    def on_odom(msg):
        hub.enqueue(pack_frame(TYPE_ODOM, encode_odom(msg)), is_cloud=False)

    # Keep only the latest cloud to cut TCP backlog latency on Jetson.
    # IMU queue small: high-rate IMU previously filled the shared send queue
    # and delayed cloud frames by hundreds of ms.
    rospy.Subscriber(imu_topic, Imu, on_imu, queue_size=10)
    rospy.Subscriber(cloud_topic, PointCloud2, on_cloud, queue_size=1, buff_size=2**24)
    rospy.Subscriber(odom_topic, Odometry, on_odom, queue_size=20)
    rospy.spin()
    sock.close()


if __name__ == "__main__":
    main()
