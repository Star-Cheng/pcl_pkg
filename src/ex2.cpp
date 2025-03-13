#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Core>

pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
boost::mutex cloud_mutex;

// 过滤参数配置
struct FilterConfig
{
    float y_threshold = 1.0;   // Y轴过滤阈值，0表示关闭过滤
    bool enable_filter = true; // 总开关
};
FilterConfig filter_cfg;

// 伪彩色映射函数
void pseudoColor(float depth, pcl::PointXYZRGB &point) {
    float min_depth = 0.1;      // 最小深度值
    float max_depth = 5.0;      // 最大深度值
    float depth_range = max_depth - min_depth;
    float normalized_depth = (depth - min_depth) / depth_range;

    // 将深度值映射到 RGB 颜色空间 (颜色从蓝色到红色)
    point.r = static_cast<uint8_t>(normalized_depth * 255);    // 红色通道
    point.g = static_cast<uint8_t>(0.0);                        // 绿色通道
    point.b = static_cast<uint8_t>((1.0 - normalized_depth) * 255); // 蓝色通道
}

// 点云回调函数
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
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
                // 反转 xyz 值
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
        // 不过滤时，直接反转 xyz 值
        for (const auto &point : *temp_cloud)
        {
            pcl::PointXYZ reversed_point;
            reversed_point.x = point.x;
            reversed_point.y = -point.y;
            reversed_point.z = -point.z;

            filtered_cloud->push_back(reversed_point);
        }
    }

    // 基于深度值的伪彩色渲染
    colored_cloud->clear();
    for (const auto &point : *filtered_cloud)
    {
        pcl::PointXYZRGB colored_point;
        colored_point.x = point.x;
        colored_point.y = point.y;
        colored_point.z = point.z;

        // 根据深度值设置伪彩色
        pseudoColor(point.z, colored_point);

        colored_cloud->push_back(colored_point);
    }

    boost::mutex::scoped_lock lock(cloud_mutex);
    current_cloud->clear();
    *current_cloud = *filtered_cloud;
}

// 可视化点云
void visualizePointCloud() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.1);

    while (!viewer->wasStopped() && ros::ok()) {
        boost::mutex::scoped_lock lock(cloud_mutex);
        if (!colored_cloud->empty()) {
            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        }
        lock.unlock();
        viewer->spinOnce(100);
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_visualizer");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 10, pointCloudCallback);

    visualizePointCloud();

    return 0;
}