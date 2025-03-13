#include <ros/ros.h>
#include <xv_sdk/PoseStampedConfidence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// 路径点列表
pcl::PointCloud<pcl::PointXYZ>::Ptr path_points(new pcl::PointCloud<pcl::PointXYZ>);

void poseCallback(const xv_sdk::PoseStampedConfidence::ConstPtr& msg) {
    pcl::PointXYZ point;
    point.x = msg->poseMsg.pose.position.x;
    point.y = msg->poseMsg.pose.position.y;
    point.z = msg->poseMsg.pose.position.z;
    path_points->push_back(point);
}

void visualizePath3D() {
    pcl::visualization::CloudViewer viewer("SLAM Path Viewer");
    while (!viewer.wasStopped() && ros::ok()) {
        viewer.showCloud(path_points);  // 使用 showCloud 显示点云
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_path_visualizer");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/slam/pose", 10, poseCallback);

    // 启动可视化线程
    visualizePath3D();

    return 0;
}