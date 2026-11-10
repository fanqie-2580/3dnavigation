import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def read_xyzi(msg):
    points = point_cloud2.read_points(
        msg,
        field_names=("x", "y", "z", "intensity"),
        skip_nans=True,
    )

    if isinstance(points, np.ndarray) and points.dtype.names:
        return np.column_stack(
            [points["x"], points["y"], points["z"], points["intensity"]]
        ).astype(np.float32, copy=False)

    return np.asarray(list(points), dtype=np.float32)


def main():
    rclpy.init()
    node = rclpy.create_node("pcd_stats_once")

    def cb(msg):
        pts = read_xyzi(msg)
        if pts.size == 0:
            print("frame:", msg.header.frame_id)
            print("points: 0")
            rclpy.shutdown()
            return

        print("frame:", msg.header.frame_id)
        print("points:", len(pts))
        print("xyz min:", pts[:, :3].min(axis=0))
        print("xyz max:", pts[:, :3].max(axis=0))
        print(
            "intensity min/max/mean:",
            pts[:, 3].min(),
            pts[:, 3].max(),
            pts[:, 3].mean(),
        )
        print("ratio intensity < 0.5:", np.mean(pts[:, 3] < 0.5))
        print("ratio intensity < 1.0:", np.mean(pts[:, 3] < 1.0))

        rclpy.shutdown()

    node.create_subscription(
        PointCloud2,
        "/perception_3d_global/lidar/dGraph",
        cb,
        10,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
