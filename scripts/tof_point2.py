#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class SingleFrameVisualizer:
    def __init__(self):
        rospy.init_node("single_frame_visualizer")

        # 初始化可视化窗口
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("TOF Single Frame", 1024, 768)

        # 初始化点云对象
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

        # 订阅ROS话题
        rospy.Subscriber(
            "/xv_sdk/xv_dev/tof_camera/point_cloud", PointCloud2, self.cloud_callback
        )

        # 初始化坐标系
        self.vis.add_geometry(
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        )

    def cloud_callback(self, msg):
        # 转换点云数据
        # 使用numpy向量化操作
        points = np.array(
            list(
                point_cloud2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=True
                )
            )
        )

        # 清空当前显示
        self.pcd.points = o3d.utility.Vector3dVector()

        # 更新点云数据
        if points.shape[0] > 0:
            self.pcd.points = o3d.utility.Vector3dVector(points)

        # 更新可视化
        self.vis.update_geometry(self.pcd)
        self.vis.reset_view_point(True)

    def run(self):
        while not rospy.is_shutdown():
            self.vis.poll_events()
            self.vis.update_renderer()
            rospy.Rate(30).sleep()

        self.vis.destroy_window()


if __name__ == "__main__":
    visualizer = SingleFrameVisualizer()
    visualizer.run()
