#!/usr/bin/env python3
"""ROS1 Odin SLAM map accumulator — run on the Go2 dock next to the driver.

Subscribes /odin1/cloud_slam (already in odom) and voxels into a global PCD.
Does NOT re-apply odometry. Save via: rosservice call /save_map
"""
from __future__ import print_function

import os
import struct
import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdinOdomMapperRos1(object):
    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/odin1/cloud_slam")
        self.odom_topic = rospy.get_param("~odom_topic", "/odin1/odometry")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.save_path = rospy.get_param(
            "~save_map_path", "/home/unitree/odin_bridge/maps/odin1_4floor_map.pcd"
        )
        self.voxel = float(rospy.get_param("~voxel_size", 0.05))
        self.min_conf = int(rospy.get_param("~min_confidence", 30))
        self.cloud_in_odom = bool(rospy.get_param("~cloud_in_odom_frame", True))

        self._lock = threading.Lock()
        self._voxel = {}
        self._path = Path()
        self._path.header.frame_id = self.map_frame

        self.pub_scan = rospy.Publisher("/cloud_registered", PointCloud2, queue_size=2)
        self.pub_map = rospy.Publisher("/Laser_map", PointCloud2, queue_size=1, latch=True)
        self.pub_odom = rospy.Publisher("/Odometry", Odometry, queue_size=20)
        self.pub_path = rospy.Publisher("/path", Path, queue_size=2)
        self.tf_br = TransformBroadcaster()
        self.static_br = StaticTransformBroadcaster()

        static_tf = TransformStamped()
        static_tf.header.stamp = rospy.Time.now()
        static_tf.header.frame_id = self.map_frame
        static_tf.child_frame_id = "odom"
        static_tf.transform.rotation.w = 1.0
        self.static_br.sendTransform(static_tf)

        rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.on_cloud, queue_size=2)
        rospy.Service("/save_map", Trigger, self.on_save)
        rospy.Timer(rospy.Duration(1.0), self.publish_map)

        rospy.loginfo(
            "Odin ROS1 mapper ready. cloud=%s in_odom=%s save=%s voxel=%.3f",
            self.cloud_topic,
            self.cloud_in_odom,
            self.save_path,
            self.voxel,
        )

    def on_odom(self, msg):
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

    def on_cloud(self, msg):
        field_names = [f.name for f in msg.fields]
        want = ["x", "y", "z"]
        has_conf = "confidence" in field_names
        if has_conf:
            want.append("confidence")
        try:
            pts = list(pc2.read_points(msg, field_names=want, skip_nans=True))
        except Exception as e:
            rospy.logerr("read_points failed: %s", e)
            return
        if not pts:
            return

        keep = []
        for p in pts:
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            conf = int(p[3]) if has_conf else 255
            if conf < self.min_conf:
                continue
            keep.append((x, y, z))
        if not keep:
            return
        world = np.asarray(keep, dtype=np.float64)
        # cloud_slam already in odom; identity path kept for clarity
        if not self.cloud_in_odom:
            rospy.logwarn_throttle(5.0, "cloud_in_odom_frame=False not implemented on ROS1 mapper")

        header = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
        self.pub_scan.publish(pc2.create_cloud_xyz32(header, world.astype(np.float32)))

        inv = 1.0 / max(self.voxel, 1e-3)
        keys = np.floor(world * inv).astype(np.int32)
        with self._lock:
            for (kx, ky, kz), pt in zip(keys, world):
                self._voxel[(int(kx), int(ky), int(kz))] = pt

    def publish_map(self, _evt):
        with self._lock:
            if not self._voxel:
                return
            pts = np.asarray(list(self._voxel.values()), dtype=np.float32)
        header = Header(stamp=rospy.Time.now(), frame_id=self.map_frame)
        self.pub_map.publish(pc2.create_cloud_xyz32(header, pts))

    def on_save(self, _req):
        with self._lock:
            if not self._voxel:
                return TriggerResponse(success=False, message="map empty")
            pts = np.asarray(list(self._voxel.values()), dtype=np.float32)
        path = self.save_path
        parent = os.path.dirname(path)
        if parent:
            try:
                os.makedirs(parent)
            except OSError:
                pass
        with open(path, "wb") as f:
            hdr = (
                "# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\n"
                "FIELDS x y z\n"
                "SIZE 4 4 4\n"
                "TYPE F F F\n"
                "COUNT 1 1 1\n"
                "WIDTH %d\n"
                "HEIGHT 1\n"
                "VIEWPOINT 0 0 0 1 0 0 0\n"
                "POINTS %d\n"
                "DATA binary\n"
            ) % (len(pts), len(pts))
            f.write(hdr.encode("ascii"))
            f.write(pts.astype("<f4").tobytes())
        msg = "Saved %d points to %s" % (len(pts), path)
        rospy.loginfo(msg)
        return TriggerResponse(success=True, message=msg)


def main():
    rospy.init_node("odin_odom_mapper")
    OdinOdomMapperRos1()
    rospy.spin()


if __name__ == "__main__":
    main()
