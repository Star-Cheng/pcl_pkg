#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
from threading import Lock
from xv_sdk.msg import PoseStampedConfidence

class SLAM3DVisualizer:
    def __init__(self):
        rospy.init_node('slam_3d_visualizer_py')
        
        # 路径数据存储
        self.path_points = []
        self.lock = Lock()
        
        # 初始化Open3D可视化
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("SLAM 3D Path", 1024, 768)
        
        # 创建路径几何体
        self.line_set = o3d.geometry.LineSet()
        self.vis.add_geometry(self.line_set)
        
        # 添加坐标系
        mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        self.vis.add_geometry(mesh_frame)
        
        # ROS订阅
        rospy.Subscriber('/xv_sdk/xv_dev/slam/pose', PoseStampedConfidence, self.callback)
        rospy.Timer(rospy.Duration(0.1), self.update_visualization)

    def callback(self, msg):
        position = msg.poseMsg.pose.position
        with self.lock:
            self.path_points.append([position.x, position.y, position.z])

    def update_visualization(self, event):
        with self.lock:
            points = np.array(self.path_points)
        
        if len(points) < 2:
            return

        # 创建线段连接连续点
        lines = [[i, i+1] for i in range(len(points)-1)]
        colors = [[0,1,0] for _ in range(len(lines))]  # 绿色路径
        
        # 更新几何体
        self.line_set.points = o3d.utility.Vector3dVector(points)
        self.line_set.lines = o3d.utility.Vector2iVector(lines)
        self.line_set.colors = o3d.utility.Vector3dVector(colors)
        
        # 更新可视化
        self.vis.update_geometry(self.line_set)
        self.vis.poll_events()
        self.vis.update_renderer()

if __name__ == '__main__':
    visualizer = SLAM3DVisualizer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        visualizer.vis.destroy_window()