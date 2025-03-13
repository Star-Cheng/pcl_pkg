#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
boost::mutex cloud_mutex;
ros::Publisher pub; // 全局发布器对象

// 过滤参数配置
struct FilterConfig
{
    float y_threshold = 1.0;   // Y轴过滤阈值，0表示关闭过滤
    bool enable_filter = true; // 总开关
};
FilterConfig filter_cfg;

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
                // 反转yz值
                pcl::PointXYZ reversed_point;
                reversed_point.x = point.x;
                reversed_point.y = point.y;
                reversed_point.z = point.z;

                filtered_cloud->push_back(reversed_point);
            }
        }
    }
    else
    {
        // 不过滤时，直接反转yz值
        for (const auto &point : *temp_cloud)
        {
            pcl::PointXYZ reversed_point;
            reversed_point.x = point.x;
            reversed_point.y = point.y;
            reversed_point.z = point.z;

            filtered_cloud->push_back(reversed_point);
        }
    }

    // 将处理后的点云转换为ROS消息并发布
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered_cloud, output_msg);
    output_msg.header = msg->header; // 保持原始消息的header
    pub.publish(output_msg); // 发布处理后的点云

    // 更新当前点云用于可视化
    boost::mutex::scoped_lock lock(cloud_mutex);
    current_cloud->clear();
    *current_cloud = *filtered_cloud;
}

void visualizePointCloud() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    Eigen::Affine3f pose_x = Eigen::Affine3f::Identity();             // 创建单位矩阵
    pose_x.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())); // 沿 X 轴旋转 180 度 (M_PI = 180度)
    viewer->addCoordinateSystem(0.5, pose_x);

    while (!viewer->wasStopped() && ros::ok()) {
        boost::mutex::scoped_lock lock(cloud_mutex);
        if (!current_cloud->empty()) {
            viewer->removeAllPointClouds();
            viewer->addPointCloud<pcl::PointXYZ>(current_cloud, "cloud");
        }
        lock.unlock();
        viewer->spinOnce(100);
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_visualizer");
    ros::NodeHandle nh;

    // 订阅原始点云话题
    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 10, pointCloudCallback);
    // 初始化发布器，话题名为/processed_cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/processed_cloud", 10);

    visualizePointCloud();

    return 0;
}