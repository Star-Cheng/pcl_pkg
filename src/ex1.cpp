#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <xv_sdk/PoseStampedConfidence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/mutex.hpp>
#include <xv_sdk/GetDevices.h>
#include <xv_sdk/LoadMapAndSwithcCslam.h>

// 共享数据定义
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr path_points(new pcl::PointCloud<pcl::PointXYZ>);
boost::mutex data_mutex;

// 全局发布者定义
ros::Publisher processed_cloud_pub;
ros::Publisher slam_path_pub;

// 点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp_cloud);
    
    // 反转每个点的坐标（根据需求调整）
    for (auto &point : temp_cloud->points)
    {
        point.x = -point.x;  // 反转x轴
        point.y = -point.y;  // 反转y轴
        point.z = -point.z;  // 反转z轴
    }

    // 发布处理后的点云
    sensor_msgs::PointCloud2 processed_msg;
    pcl::toROSMsg(*temp_cloud, processed_msg);
    processed_msg.header = msg->header; // 继承原始消息的header
    processed_cloud_pub.publish(processed_msg);

    // 更新共享点云数据
    boost::mutex::scoped_lock lock(data_mutex);
    current_cloud->swap(*temp_cloud);
}

// SLAM位姿回调函数
void poseCallback(const xv_sdk::PoseStampedConfidence::ConstPtr &msg)
{
    pcl::PointXYZ point;
    point.x = msg->poseMsg.pose.position.x;
    point.y = -msg->poseMsg.pose.position.y;
    point.z = -msg->poseMsg.pose.position.z;

    // 更新路径数据并发布
    boost::mutex::scoped_lock lock(data_mutex);
    path_points->push_back(point);
}

// 定时器回调函数用于发布路径
void pathTimerCallback(const ros::TimerEvent& event)
{
    sensor_msgs::PointCloud2 path_msg;
    {
        boost::mutex::scoped_lock lock(data_mutex);
        pcl::toROSMsg(*path_points, path_msg);
    }
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map"; // 设置正确的坐标系
    slam_path_pub.publish(path_msg);
}

// 可视化主函数
void visualizeCombined()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(current_cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> path_color(path_points, 255, 0, 0);

    while (!viewer->wasStopped() && ros::ok())
    {
        {
            boost::mutex::scoped_lock lock(data_mutex);

            if (!current_cloud->empty())
            {
                viewer->removePointCloud("cloud");
                viewer->addPointCloud<pcl::PointXYZ>(current_cloud, cloud_color, "cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
            }

            if (!path_points->empty())
            {
                viewer->removePointCloud("path");
                viewer->addPointCloud<pcl::PointXYZ>(path_points, path_color, "path");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "path");
            }
        }

        viewer->spinOnce(100);
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combined_visualizer");
    ros::NodeHandle nh;

    // 初始化发布者
    processed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/processed_pointcloud", 10);
    slam_path_pub = nh.advertise<sensor_msgs::PointCloud2>("/slam_path", 10);

    // 创建定时器发布路径（0.1秒间隔）
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), pathTimerCallback);

    // 订阅话题
    ros::Subscriber cloud_sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 10, pointCloudCallback);
    ros::Subscriber pose_sub = nh.subscribe("/xv_sdk/xv_dev/slam/pose", 10, poseCallback);

    // 启动可视化
    visualizeCombined();

    return 0;
}