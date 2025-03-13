#!/usr/bin/env python
import rospy
import threading
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from xv_sdk.msg import PoseStampedConfidence

# 共享数据及锁
current_cloud = None
path_points = []
data_lock = threading.Lock()

def point_cloud_callback(msg):
    global current_cloud
    # 转换点云为numpy数组
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    np_cloud = np.array(list(pc_data), dtype=np.float32)
    # 反转 y 坐标  
    np_cloud[:, 1] = -np_cloud[:, 1]
    np_cloud[:, 2] = -np_cloud[:, 2]
    # 过滤x值在[-1,1]范围外的点
    # filtered_cloud = np_cloud[(np_cloud[:,0] >= -0.1) & (np_cloud[:,0] <= 0.1)]
    # 过滤y值在[-1,1]范围外的点
    filtered_cloud = np_cloud[(np_cloud[:,1] >= -1) & (np_cloud[:,1] <= 1)]
    # 过滤y值在[-1,1]范围外的点
    # filtered_cloud = np_cloud[(np_cloud[:,2] >= -0.1) & (np_cloud[:,2] <= 0.1)]s
    with data_lock:
        global current_cloud
        current_cloud = filtered_cloud

def pose_callback(msg):
    global path_points
    point = [msg.poseMsg.pose.position.x,
             -msg.poseMsg.pose.position.y,
             -msg.poseMsg.pose.position.z]
    
    with data_lock:
        path_points.append(point)

def visualize_combined():
    # 创建可视化窗口
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='3D Viewer', width=800, height=600)
    # 获取渲染选项并设置背景颜色
    render_opt = vis.get_render_option()
    render_opt.background_color = np.array([0.0, 0.0, 0.0])  # RGB范围[0-1]
    render_opt.point_size = 0.6  # 设置点云大小，可调整此数值
    
    # 初始化坐标系
    coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    vis.add_geometry(coordinate)
    
    # 初始化点云和路径几何体
    cloud = o3d.geometry.PointCloud()
    path = o3d.geometry.PointCloud()
    
    last_cloud_size = 0
    last_path_size = 0
    
    while True:
        with data_lock:
            # 更新点云数据
            if current_cloud is not None and len(current_cloud) > 0:
                cloud.points = o3d.utility.Vector3dVector(current_cloud)
                cloud.paint_uniform_color([1, 1, 1])  # 白色
                
                if last_cloud_size == 0:
                    vis.add_geometry(cloud)
                else:
                    vis.update_geometry(cloud)
                last_cloud_size = len(current_cloud)
            
            # 更新路径数据
            if len(path_points) > 0:
                np_path = np.array(path_points, dtype=np.float32)
                path.points = o3d.utility.Vector3dVector(np_path)
                path.paint_uniform_color([1, 0, 0])  # 红色
                
                if last_path_size == 0:
                    vis.add_geometry(path)
                else:
                    vis.update_geometry(path)
                
                last_path_size = len(path_points)
        
        # 刷新显示
        vis.poll_events()
        vis.update_renderer()
        
        # 检查窗口是否关闭
        if not vis.poll_events():
            break
        
        rospy.sleep(0.1)

def main():
    rospy.init_node('combined_visualizer_py')
    
    # 创建订阅者
    rospy.Subscriber("/xv_sdk/xv_dev/tof_camera/point_cloud", 
                    PointCloud2, point_cloud_callback)
    rospy.Subscriber("/xv_sdk/xv_dev/slam/pose", 
                    PoseStampedConfidence, pose_callback)
    
    # 启动可视化线程
    vis_thread = threading.Thread(target=visualize_combined)
    vis_thread.daemon = True
    vis_thread.start()
    
    # 保持主线程运行
    rospy.spin()

if __name__ == '__main__':
    main()