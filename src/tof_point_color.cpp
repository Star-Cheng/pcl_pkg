#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/thread.hpp>
#include <cmath>

// 修改点类型为带颜色的PointXYZRGB
typedef pcl::PointXYZRGB PointT;
pcl::PointCloud<PointT>::Ptr current_cloud(new pcl::PointCloud<PointT>);
boost::mutex cloud_mutex;

// 新增渲染模式枚举
enum RenderMode { DEPTH_COLOR, NORMAL_COLOR };
RenderMode render_mode = DEPTH_COLOR;

struct FilterConfig {
    float y_threshold = 1.0;
    bool enable_filter = true;
};
FilterConfig filter_cfg;

// 颜色映射函数（HSV转RGB）
void HSVtoRGB(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
    // 确保h在[0,1]范围
    h = std::fmod(h, 1.0f);
    if (h < 0) h += 1.0f;

    // 初始化默认值
    r = g = b = 0;
    
    int i = static_cast<int>(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
        case 0: r=255*v; g=255*t; b=255*p; break;
        case 1: r=255*q; g=255*v; b=255*p; break;
        case 2: r=255*p; g=255*v; b=255*t; break;
        case 3: r=255*p; g=255*q; b=255*v; break;
        case 4: r=255*t; g=255*p; b=255*v; break;
        case 5: r=255*v; g=255*p; b=255*q; break;
        default: break;
    }
}

// 深度值颜色映射（蓝色到红色渐变）
void applyDepthColor(pcl::PointCloud<PointT>::Ptr &cloud) {
    if (cloud->empty()) return;

    // 获取深度范围
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    for (const auto &p : *cloud) {
        if (p.z < min_z) min_z = p.z;
        if (p.z > max_z) max_z = p.z;
    }

    // 归一化并映射颜色
    float range = max_z - min_z;
    for (auto &p : *cloud) {
        float ratio = (range != 0) ? (p.z - min_z) / range : 0.5;
        uint8_t r, g, b;
        HSVtoRGB(0.66f * (1 - ratio), 1.0f, 1.0f, r, g, b); // 蓝(0.66)到红(0.0)
        p.r = r;
        p.g = g;
        p.b = b;
    }
}

// 法线方向颜色映射（RGB对应XYZ分量）
void applyNormalColor(pcl::PointCloud<PointT>::Ptr &cloud) {
    if (cloud->empty()) return;

    // 法线估计
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03); // 根据点云密度调整

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    // 映射法线方向到颜色
    for (size_t i = 0; i < cloud->size(); ++i) {
        Eigen::Vector3f n = normals->points[i].getNormalVector3fMap().normalized();
        cloud->points[i].r = static_cast<uint8_t>((n.x() + 1) * 127);
        cloud->points[i].g = static_cast<uint8_t>((n.y() + 1) * 127);
        cloud->points[i].b = static_cast<uint8_t>((n.z() + 1) * 127);
    }
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp_cloud);
    
    // 创建带颜色的点云
    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);

    // 原有过滤逻辑
    if (filter_cfg.enable_filter && filter_cfg.y_threshold != 0) {
        for (const auto &point : *temp_cloud) {
            if (point.y < filter_cfg.y_threshold && point.y > -filter_cfg.y_threshold) {
                PointT p;
                p.x = point.x;
                p.y = -point.y;  // 反转Y
                p.z = -point.z;  // 反转Z
                filtered_cloud->push_back(p);
            }
        }
    } else {
        for (const auto &point : *temp_cloud) {
            PointT p;
            p.x = point.x;
            p.y = -point.y;
            p.z = -point.z;
            filtered_cloud->push_back(p);
        }
    }

    // 应用颜色渲染
    switch (render_mode) {
        case DEPTH_COLOR: applyDepthColor(filtered_cloud); break;
        case NORMAL_COLOR: applyNormalColor(filtered_cloud); break;
    }

    // 更新当前点云
    boost::mutex::scoped_lock lock(cloud_mutex);
    current_cloud->swap(*filtered_cloud);
}

void visualizePointCloud() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.1);

    while (!viewer->wasStopped() && ros::ok()) {
        boost::mutex::scoped_lock lock(cloud_mutex);
        if (!current_cloud->empty()) {
            viewer->removeAllPointClouds();
            // 使用颜色信息渲染点云
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(current_cloud);
            viewer->addPointCloud<PointT>(current_cloud, rgb, "cloud");
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        }
        lock.unlock();
        viewer->spinOnce(100);
        ros::spinOnce();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "enhanced_point_cloud_visualizer");
    ros::NodeHandle nh;
    
    // 安全的参数获取方式
    int render_mode_int = static_cast<int>(render_mode);
    nh.param<int>("render_mode", render_mode_int, static_cast<int>(DEPTH_COLOR));
    render_mode = static_cast<RenderMode>(render_mode_int);

    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 10, pointCloudCallback);
    visualizePointCloud();
    return 0;
}