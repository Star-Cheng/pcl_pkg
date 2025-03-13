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

// 过滤参数配置
struct FilterConfig
{
    float y_threshold = 1.0;   // Y轴过滤阈值，0表示关闭过滤
    bool enable_filter = true; // 总开关
};
FilterConfig filter_cfg;

// 点云回调函数（新增过滤逻辑）
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 执行过滤的条件判断
    if (filter_cfg.enable_filter && filter_cfg.y_threshold != 0)
    {
        for (const auto &point : *temp_cloud)
        {
            if (point.y < filter_cfg.y_threshold && point.y > -filter_cfg.y_threshold)
            {
                // 反转xyz值
                pcl::PointXYZ reversed_point;
                reversed_point.x = point.x;
                reversed_point.y = -point.y;
                reversed_point.z = -point.z;
                filtered_cloud->push_back(reversed_point);
            }
        }
    }
    else
    {
        // 不过滤时，直接反转xyz值
        for (const auto &point : *temp_cloud)
        {
            pcl::PointXYZ reversed_point;
            reversed_point.x = point.x;
            reversed_point.y = -point.y;
            reversed_point.z = -point.z;

            filtered_cloud->push_back(reversed_point);
        }
    }

    boost::mutex::scoped_lock lock(data_mutex);
    current_cloud->swap(*filtered_cloud);
}

// SLAM位姿回调函数（保持不变）
void poseCallback(const xv_sdk::PoseStampedConfidence::ConstPtr &msg)
{
    pcl::PointXYZ point;
    point.x = msg->poseMsg.pose.position.x;
    point.y = -msg->poseMsg.pose.position.y;
    point.z = -msg->poseMsg.pose.position.z;

    boost::mutex::scoped_lock lock(data_mutex);
    path_points->push_back(point);
}

// 可视化主函数（保持不变）
void visualizeCombined()
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(current_cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> path_color(path_points, 255, 0, 0);

    while (!viewer->wasStopped() && ros::ok())
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

        lock.unlock();
        viewer->spinOnce(100);
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "filtered_visualizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    // 从参数服务器读取配置
    private_nh.param<float>("y_threshold", filter_cfg.y_threshold, 1.0);
    private_nh.param<bool>("enable_filter", filter_cfg.enable_filter, true);

    // 订阅话题
    ros::Subscriber cloud_sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 10, pointCloudCallback);
    ros::Subscriber pose_sub = nh.subscribe("/xv_sdk/xv_dev/slam/pose", 10, poseCallback);

    // 启动可视化
    visualizeCombined();

    return 0;
}