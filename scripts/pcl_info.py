#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import std_msgs.msg
import numpy as np
from sensor_msgs import point_cloud2

def callback(msg):
    # 将 PointCloud2 消息转换为 (x, y, z) 点的列表
    pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    
    # 将数据转化为 NumPy 数组
    points = np.array(list(pc_data))
    
    # 打印前几行数据查看
    rospy.loginfo("First few points:\n{}".format(points[:5]))

    # 保存点云数据到文件
    np.savetxt("output.xyz", points, delimiter=" ", header="x y z", comments="")

    rospy.loginfo("PointCloud saved to output.xyz")

def listener():
    rospy.init_node('pointcloud_listener', anonymous=True)

    # 订阅 PointCloud2 消息
    rospy.Subscriber("/xv_sdk/xv_dev/tof_camera/point_cloud", sensor_msgs.msg.PointCloud2, callback)

    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    listener()
