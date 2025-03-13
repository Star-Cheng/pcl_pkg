#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

// 可视化窗口
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 将 PointCloud2 转换为 PCL 点云格式
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*msg, *cloud);

    // 清除上一帧的点云
    viewer->removeAllPointClouds();

    // 添加当前帧的点云
    viewer->addPointCloud<pcl::PointXYZRGBA>(cloud, "Cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");

    // 触发更新
    viewer->spinOnce(50);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_visualizer");
    ros::NodeHandle nh;

    // 订阅点云话题
    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 1, pointCloudCallback);

    // 启动可视化窗口
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);

    ros::spin();

    return 0;
}