import rospy
from xv_sdk.msg import PoseStampedConfidence
import numpy as np
import cv2

# 初始化路径点列表
path_points = []

def pose_callback(data):
    global path_points
    # 提取位置信息
    x = data.poseMsg.pose.position.x
    y = data.poseMsg.pose.position.y
    z = data.poseMsg.pose.position.z
    path_points.append((x, y, z))

def visualize_path():
    global path_points
    # 创建一个空白图像
    img = np.zeros((800, 800, 3), dtype=np.uint8)
    img.fill(255)  # 白色背景

    # 绘制路径点
    for i in range(1, len(path_points)):
        pt1 = (int(path_points[i-1][0]*100 + 400), int(path_points[i-1][1]*100 + 400))
        pt2 = (int(path_points[i][0]*100 + 400), int(path_points[i][1]*100 + 400))
        cv2.line(img, pt1, pt2, (0, 0, 255), 2)

    cv2.imshow("SLAM Path", img)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node("slam_path_visualizer", anonymous=True)
    rospy.Subscriber("/xv_sdk/xv_dev/slam/pose", PoseStampedConfidence, pose_callback)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        visualize_path()
        rate.sleep()