import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 初始化点云
current_cloud = None

def point_cloud_callback(data):
    global current_cloud
    # 将ROS消息转换为PCL点云
    gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    current_cloud = np.array(list(gen))  # 转换为NumPy数组

def visualize_point_cloud():
    global current_cloud
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    while not rospy.is_shutdown():
        if current_cloud is not None:
            ax.clear()  # 清除上一帧数据
            ax.scatter(current_cloud[:, 0], current_cloud[:, 1], current_cloud[:, 2], s=1, c='r')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            plt.draw()
            plt.pause(0.01)
        else:
            plt.pause(0.1)

if __name__ == "__main__":
    rospy.init_node("point_cloud_visualizer", anonymous=True)
    rospy.Subscriber("/xv_sdk/xv_dev/tof_camera/point_cloud", PointCloud2, point_cloud_callback)
    visualize_point_cloud()