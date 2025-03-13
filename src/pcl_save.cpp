#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // 声明一个点云对象
    pcl::PointCloud<pcl::PointXYZ> cloud;
    
    // 将 PointCloud2 消息转换为 pcl::PointCloud 格式
    pcl::fromROSMsg(*msg, cloud);

    // 保存点云到 PCD 文件
    pcl::io::savePCDFileASCII("output.pcd", cloud);
    ROS_INFO("PointCloud saved to output.pcd");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_listener");
    ros::NodeHandle nh;

    // 订阅 PointCloud2 消息
    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 1, pointCloudCallback);

    // 保持节点运行
    ros::spin();

    return 0;
}
