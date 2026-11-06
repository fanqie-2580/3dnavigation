#!/usr/bin/env python3
"""ROS1 (dock) -> TCP bridge for Odin /imu, cloud, /odometry.

Fusion default: /odin1/cloud_raw (sensor frame, smaller than cloud_slam).
Nav/open3d may still pass ~cloud_topic:=/odin1/cloud_slam.

Cloud path is drop-old (keep latest only) so a slow TCP client cannot
build multi-second backlog — that backlog was painting Mid poses ~4s late.
"""
from __future__ import print_function
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


def subsample_cloud(msg, stride):
    """Keep every stride-th point to cut TCP payload / Jetson decode CPU."""
    stride = int(stride)
    if stride <= 1:
        return msg
    n = int(msg.width) * int(msg.height)
    step = int(msg.point_step)
    if n <= 0 or step <= 0:
        return msg
    raw = bytes(bytearray(msg.data))
    out = bytearray()
    for i in range(0, n, stride):
        b = i * step
        out.extend(raw[b : b + step])
    n_keep = len(out) // step
    msg2 = PointCloud2()
    msg2.header = msg.header
    msg2.height = 1
    msg2.width = n_keep
    msg2.fields = msg.fields
    msg2.is_bigendian = msg.is_bigendian
    msg2.point_step = step
    msg2.row_step = n_keep * step
    msg2.is_dense = msg.is_dense
    msg2.data = bytes(out)
    return msg2


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
    """IMU/odom: small FIFO. Cloud: single slot (drop-old)."""

    def __init__(self, light_queue_max=64):
        self._lock = threading.Lock()
        self._clients = []
        self._light = []  # list of packed imu/odom frames
        self._light_max = int(light_queue_max)
        self._cloud = None  # latest packed cloud or None
        self._cv = threading.Condition(self._lock)
        self._writer = threading.Thread(target=self._write_loop, daemon=True)
        self._writer.start()
        self._n_cloud_drop = 0

    def add(self, conn):
        with self._lock:
            self._clients.append(conn)
        print("[bridge-server] client +1 total=%d" % len(self._clients))

    def enqueue_light(self, data):
        with self._cv:
            self._light.append(data)
            while len(self._light) > self._light_max:
                self._light.pop(0)
            self._cv.notify()

    def enqueue_cloud(self, data):
        with self._cv:
            if self._cloud is not None:
                self._n_cloud_drop += 1
            self._cloud = data
            self._cv.notify()

    def _pop_batch(self):
        with self._cv:
            while not self._light and self._cloud is None and not rospy.is_shutdown():
                self._cv.wait(timeout=0.5)
            if rospy.is_shutdown():
                return []
            out = list(self._light)
            self._light = []
            if self._cloud is not None:
                out.append(self._cloud)
                self._cloud = None
            return out

    def _write_loop(self):
        last_report = time.time()
        while not rospy.is_shutdown():
            batch = self._pop_batch()
            if not batch:
                continue
            dead = []
            with self._lock:
                clients = list(self._clients)
                drops = self._n_cloud_drop
                self._n_cloud_drop = 0
            for data in batch:
                for c in clients:
                    try:
                        c.sendall(data)
                    except Exception:
                        if c not in dead:
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
            now = time.time()
            if drops and (now - last_report) > 2.0:
                print("[bridge-server] cloud drop-old count=%d (keep latest only)" % drops)
                last_report = now


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
    # Fusion: raw sensor-frame cloud (smaller, correct for L2/unproject).
    # Nav may override to /odin1/cloud_slam.
    cloud_topic = rospy.get_param("~cloud_topic", "/odin1/cloud_raw")
    odom_topic = rospy.get_param("~odom_topic", "/odin1/odometry")
    # Default stride=2: ~24k pts/frame — denser yellow than stride=4, still ~10 Hz.
    cloud_stride = int(rospy.get_param("~cloud_stride", 2))

    hub = ClientHub(light_queue_max=64)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.listen(4)
    print("[bridge-server] listen 0.0.0.0:%d" % port)
    print(
        "[bridge-server] bridge %s + %s + %s (cloud=drop-old stride=%d)"
        % (imu_topic, cloud_topic, odom_topic, cloud_stride)
    )

    t = threading.Thread(target=accept_loop, args=(sock, hub), daemon=True)
    t.start()

    def on_imu(msg):
        hub.enqueue_light(pack_frame(TYPE_IMU, encode_imu(msg)))

    def on_cloud(msg):
        hub.enqueue_cloud(
            pack_frame(TYPE_CLOUD, encode_cloud(subsample_cloud(msg, cloud_stride)))
        )

    def on_odom(msg):
        hub.enqueue_light(pack_frame(TYPE_ODOM, encode_odom(msg)))

    # queue_size=1 on cloud: ROS also drop-old at subscribe side
    rospy.Subscriber(imu_topic, Imu, on_imu, queue_size=50)
    rospy.Subscriber(cloud_topic, PointCloud2, on_cloud, queue_size=1)
    rospy.Subscriber(odom_topic, Odometry, on_odom, queue_size=20)
    rospy.spin()
    sock.close()


if __name__ == "__main__":
    main()
