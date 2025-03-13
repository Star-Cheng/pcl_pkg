#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <xv_sdk/PoseStampedConfidence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/mutex.hpp>

// 共享数据定义
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr path_points(new pcl::PointCloud<pcl::PointXYZ>);
boost::mutex data_mutex;

// 点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp_cloud);
    // 反转每个点的坐标  
    for (auto &point : temp_cloud->points)  
    {  
        point.x = point.x; // 反转 x 值  
        point.y = -point.y; // 反转 y 值  
        point.z = -point.z; // 反转 z 值  
    }  

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

    boost::mutex::scoped_lock lock(data_mutex);
    path_points->push_back(point);
}

// 可视化主函数
void visualizeCombined()
{
    // 创建可视化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.1);
    viewer->initCameraParameters();

    // 颜色定义
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(current_cloud, 255, 255, 255); // 白色点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> path_color(path_points, 255, 0, 0);        // 红色路径

    while (!viewer->wasStopped() && ros::ok())
    {
        // 加锁获取最新数据
        boost::mutex::scoped_lock lock(data_mutex);

        // 更新点云显示
        if (!current_cloud->empty())
        {
            // 移除时通过ID识别
            viewer->removePointCloud("cloud");
            // 添加点云时指定ID
            viewer->addPointCloud<pcl::PointXYZ>(current_cloud, cloud_color, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        }

        // 更新路径显示
        if (!path_points->empty())
        {
            viewer->removePointCloud("path");
            viewer->addPointCloud<pcl::PointXYZ>(path_points, path_color, "path");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "path"); // 5为点的大小
        }

        lock.unlock();

        // 刷新显示
        viewer->spinOnce(100);
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combined_visualizer");
    ros::NodeHandle nh;

    // 订阅两个话题
    ros::Subscriber cloud_sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 10, pointCloudCallback);
    ros::Subscriber pose_sub = nh.subscribe("/xv_sdk/xv_dev/slam/pose", 10, poseCallback);

    // 启动可视化
    visualizeCombined();

    return 0;
}