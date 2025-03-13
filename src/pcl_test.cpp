#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>  // 新增路径消息头文件
#include <xv_sdk/PoseStampedConfidence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <boost/thread/mutex.hpp>
#include <Eigen/Geometry>
#include <iomanip>
#include <cmath>

// 渲染模式枚举
enum RenderMode
{
    DEPTH_COLOR,
    NORMAL_COLOR,
    HIGH_COLOR
};

// 带颜色的点类型
typedef pcl::PointXYZRGB PointT;

// 共享数据结构
struct SharedData
{
    pcl::PointCloud<PointT>::Ptr cloud; // 彩色点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr path;
    Eigen::Affine3f pose;
    bool pose_updated;
    boost::mutex mutex;

    // 性能监控
    int cloud_width = 0;
    int cloud_height = 0;
    double cloud_fps = 0.0;
    double pose_fps = 0.0;
    ros::Time last_cloud_time;
    ros::Time last_pose_time;

    SharedData() : cloud(new pcl::PointCloud<PointT>),
                   path(new pcl::PointCloud<pcl::PointXYZ>),
                   pose(Eigen::Affine3f::Identity()),
                   pose_updated(false) {}
};

// 系统配置参数
struct Config
{
    // 过滤参数
    struct
    {
        float x_threshold = 10.0;
        float y_threshold = 10.0;
        float z_threshold = 10.0;
        bool enable = true;
    } filter;

    // 可视化参数
    struct
    {
        float coordinate_size = 0.3;
        float line_width = 2.0;
        int point_size = 2;
    } visualization;

    // 渲染参数
    RenderMode render_mode = HIGH_COLOR;
    float normal_search_radius = 0.03;
};

// 全局实例
SharedData g_data;
Config g_config;
ros::Publisher g_pub_processed_cloud;  // 全局发布者
ros::Publisher g_pub_slam_path;

// HSV转RGB颜色映射
void HSVtoRGB(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b)
{
    h = std::fmod(h, 1.0f);
    if (h < 0)
        h += 1.0f;

    int i = static_cast<int>(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6)
    {
    case 0:
        r = 255 * v;
        g = 255 * t;
        b = 255 * p;
        break;
    case 1:
        r = 255 * q;
        g = 255 * v;
        b = 255 * p;
        break;
    case 2:
        r = 255 * p;
        g = 255 * v;
        b = 255 * t;
        break;
    case 3:
        r = 255 * p;
        g = 255 * q;
        b = 255 * v;
        break;
    case 4:
        r = 255 * t;
        g = 255 * p;
        b = 255 * v;
        break;
    case 5:
        r = 255 * v;
        g = 255 * p;
        b = 255 * q;
        break;
    default:
        break;
    }
}

// 深度着色处理
void applyDepthColor(pcl::PointCloud<PointT>::Ptr &cloud)
{
    if (cloud->empty())
        return;

    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    for (const auto &p : *cloud)
    {
        if (p.z < min_z)
            min_z = p.z;
        if (p.z > max_z)
            max_z = p.z;
    }

    float range = max_z - min_z;
    for (auto &p : *cloud)
    {
        float ratio = (range != 0) ? (p.z - min_z) / range : 0.5;
        uint8_t r = 0, g = 0, b = 0; // 显式初始化
        HSVtoRGB(0.66f * (1 - ratio), 1.0f, 1.0f, r, g, b);
        p.r = r;
        p.g = g;
        p.b = b;
    }
}

// 高度着色处理
void applyHighColor(pcl::PointCloud<PointT>::Ptr &cloud)
{
    if (cloud->empty())
        return;

    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    for (const auto &p : *cloud)
    {
        if (p.y < min_y)
            min_y = p.y;
        if (p.y > max_y)
            max_y = p.y;
    }

    float range = max_y - min_y;
    for (auto &p : *cloud)
    {
        float ratio = (range != 0) ? (p.y - min_y) / range : 0.5;
        uint8_t r = 0, g = 0, b = 0; // 显式初始化
        HSVtoRGB(0.66f * (1 - ratio), 1.0f, 1.0f, r, g, b);
        p.r = r;
        p.g = g;
        p.b = b;
    }
}

// 法线着色处理
void applyNormalColor(pcl::PointCloud<PointT>::Ptr &cloud)
{
    if (cloud->empty())
        return;

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(g_config.normal_search_radius);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.compute(*normals);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        Eigen::Vector3f n = normals->points[i].getNormalVector3fMap().normalized();
        cloud->points[i].r = static_cast<uint8_t>((n.x() + 1) * 127);
        cloud->points[i].g = static_cast<uint8_t>((n.y() + 1) * 127);
        cloud->points[i].b = static_cast<uint8_t>((n.z() + 1) * 127);
    }
}

// 点云回调函数
void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp);

    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);

    // 点云过滤
    if (g_config.filter.enable)
    {
        for (const auto &point : *temp)
        {
            if (std::abs(point.x) < g_config.filter.x_threshold &&
                std::abs(point.y) < g_config.filter.y_threshold &&
                std::abs(point.z) < g_config.filter.z_threshold)
            {
                PointT p;
                p.x = point.x;
                p.y = point.y; // 坐标系修正
                p.z = point.z;
                filtered->push_back(p);
            }
        }
    }
    else
    {
        for (const auto &point : *temp)
        {
            PointT p;
            p.x = point.x;
            p.y = point.y;
            p.z = point.z;
            filtered->push_back(p);
        }
    }

    // 应用颜色渲染
    switch (g_config.render_mode)
    {
    case DEPTH_COLOR:
        applyDepthColor(filtered);
        break;
    case NORMAL_COLOR:
        applyNormalColor(filtered);
        break;
    case HIGH_COLOR:
        applyHighColor(filtered);
        break;
    }
    // -----------------------------------------------------
    // 发布处理后的点云
    sensor_msgs::PointCloud2 processed_msg;
    pcl::toROSMsg(*filtered, processed_msg);
    processed_msg.header = msg->header;
    processed_msg.header.frame_id = "map";
    g_pub_processed_cloud.publish(processed_msg);
    // -----------------------------------------------------

    // 更新共享数据
    boost::mutex::scoped_lock lock(g_data.mutex);
    g_data.cloud->swap(*filtered);
    g_data.cloud_width = msg->width;
    g_data.cloud_height = msg->height;

    // 计算点云帧率
    ros::Time now = ros::Time::now();
    if (g_data.last_cloud_time.isValid())
    {
        double delta = (now - g_data.last_cloud_time).toSec();
        if (delta > 0)
            g_data.cloud_fps = 1.0 / delta;
    }
    g_data.last_cloud_time = now;
}

// 位姿回调函数
void poseCallback(const xv_sdk::PoseStampedConfidence::ConstPtr &msg)
{
    Eigen::Vector3f position(
        msg->poseMsg.pose.position.x,
        msg->poseMsg.pose.position.y,
        msg->poseMsg.pose.position.z);

    Eigen::Quaternionf q_orig(
        msg->poseMsg.pose.orientation.w,
        msg->poseMsg.pose.orientation.x,
        msg->poseMsg.pose.orientation.y,
        msg->poseMsg.pose.orientation.z);

    // 坐标系修正
    static const Eigen::Quaternionf q_correct(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.translate(position);
    pose.rotate(q_orig * q_correct);

    pcl::PointXYZ path_point(position.x(), position.y(), position.z());

    // 更新共享数据
    boost::mutex::scoped_lock lock(g_data.mutex);
    g_data.path->push_back(path_point);
    g_data.pose = pose;
    g_data.pose_updated = true;

    // -----------------------------------------------------
    // 发布路径信息
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    // 填充路径消息
    for (const auto& point : *g_data.path) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose.position.x = point.x;
        pose_stamped.pose.position.y = point.y;
        pose_stamped.pose.position.z = point.z;
        pose_stamped.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose_stamped);
    }

    g_pub_slam_path.publish(path_msg);
    // -----------------------------------------------------

    // 计算位姿帧率
    ros::Time now = ros::Time::now();
    if (g_data.last_pose_time.isValid())
    {
        double delta = (now - g_data.last_pose_time).toSec();
        if (delta > 0)
            g_data.pose_fps = 1.0 / delta;
    }
    g_data.last_pose_time = now;
}
void drawCameraIndicator(pcl::visualization::PCLVisualizer &viewer,
    const Eigen::Affine3f &pose)
{
const float size = 0.5;

// 定义局部坐标系三角形顶点（相机坐标系）
Eigen::Vector3f local_points[3] = {
Eigen::Vector3f(0, 0, 0),            // 原点
Eigen::Vector3f(-size, -size, size), // 左下
Eigen::Vector3f(size, -size, size)   // 右下
};

// 转换到世界坐标系
pcl::PointXYZ world_points[3];
for (int i = 0; i < 3; ++i)
{
Eigen::Vector3f p = pose * local_points[i];
world_points[i] = pcl::PointXYZ(p.x(), p.y(), p.z());
}

// 绘制三角形边
viewer.removeShape("cam_line1");
viewer.removeShape("cam_line2");
viewer.removeShape("cam_line3");
viewer.addLine(world_points[0], world_points[1], 0.0, 1.0, 0.0, "cam_line1");
viewer.addLine(world_points[1], world_points[2], 0.0, 1.0, 0.0, "cam_line2");
viewer.addLine(world_points[2], world_points[0], 0.0, 1.0, 0.0, "cam_line3");

// 添加方向箭头
Eigen::Vector3f tip = pose * Eigen::Vector3f(0, 0, size);
viewer.removeShape("cam_arrow");
viewer.addArrow(pcl::PointXYZ(tip.x(), tip.y(), tip.z()),
world_points[0],
0.0, 1.0, 0.0, false, "cam_arrow");
}
// 可视化初始化
void setupVisualizer(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(0.1);
    viewer.addText3D("X", pcl::PointXYZ(0.1, 0, 0), 0.01, 1.0, 0.0, 0.0, "text_X");
    viewer.addText3D("Y", pcl::PointXYZ(0, 0.1, 0), 0.01, 0.0, 1.0, 0.0, "text_Y");
    viewer.addText3D("Z", pcl::PointXYZ(0, 0, 0.1), 0.01, 0.0, 0.0, 1.0, "text_Z");
    viewer.initCameraParameters();
}

// 更新可视化内容
void updateVisualization(pcl::visualization::PCLVisualizer &viewer)
{
    static size_t last_path_size = 0;

    // 更新点云显示
    if (!g_data.cloud->empty())
    {
        viewer.removePointCloud("cloud");
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(g_data.cloud);
        viewer.addPointCloud<PointT>(g_data.cloud, rgb, "cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
            g_config.visualization.point_size, "cloud");
    }

    // 更新路径显示
    if (g_data.path->size() >= 2)
    {
        for (size_t i = 0; i < last_path_size; ++i)
        {
            viewer.removeShape("line_" + std::to_string(i));
        }

        for (size_t i = 0; i < g_data.path->size() - 1; ++i)
        {
            const auto &p1 = g_data.path->at(i);
            const auto &p2 = g_data.path->at(i + 1);
            viewer.addLine<pcl::PointXYZ>(p1, p2, 1.0, 0.0, 0.0, "line_" + std::to_string(i));
            viewer.setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                g_config.visualization.line_width,
                "line_" + std::to_string(i));
        }
        last_path_size = g_data.path->size() - 1;
    }

    // 更新位姿显示
    if (g_data.pose_updated)
    {
        drawCameraIndicator(viewer, g_data.pose);
        viewer.removeCoordinateSystem("camera");
        // viewer.addCoordinateSystem(g_config.visualization.coordinate_size, g_data.pose, "camera");
        g_data.pose_updated = false;
    }

    // 更新文本信息
    Eigen::Vector3f translation = g_data.pose.translation();
    Eigen::Vector3f euler_angles = g_data.pose.rotation().eulerAngles(2, 1, 0);
    float roll = euler_angles[2] * 180.0 / M_PI;
    float pitch = euler_angles[1] * 180.0 / M_PI;
    float yaw = euler_angles[0] * 180.0 / M_PI;

    std::ostringstream info_ss;
    info_ss << std::fixed << std::setprecision(2)
            << "Position (m):\nX: " << translation.x() << "\nY: " << translation.y()
            << "\nZ: " << translation.z() << "\n\nRotation (deg):\nRoll: " << roll
            << "\nPitch: " << pitch << "\nYaw: " << yaw << "\n\nFrame Rates:\nCloud: "
            << std::setprecision(1) << g_data.cloud_fps << " Hz\nPose: " << g_data.pose_fps << " Hz"
            << "\n\nResolution: " << g_data.cloud_width << "x" << g_data.cloud_height;

    viewer.removeText3D("info_text");
    viewer.addText(info_ss.str(), 10, 30, 12, 1.0, 1.0, 1.0, "info_text");
}

// 可视化线程
void visualizationThread()
{
    pcl::visualization::PCLVisualizer viewer("Enhanced SLAM Viewer");
    setupVisualizer(viewer);

    while (!viewer.wasStopped() && ros::ok())
    {
        {
            boost::mutex::scoped_lock lock(g_data.mutex);
            updateVisualization(viewer);
        }
        viewer.spinOnce(50);
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "enhanced_slam_visualizer");
    ros::NodeHandle nh, pnh("~");
    // -----------------------------------------------------
    // 初始化发布者
    g_pub_processed_cloud = nh.advertise<sensor_msgs::PointCloud2>("processed_point_cloud", 1);
    g_pub_slam_path = nh.advertise<nav_msgs::Path>("slam_path", 1);
    // -----------------------------------------------------

    // 加载参数
    pnh.param("filter/enable", g_config.filter.enable, true);
    pnh.param("filter/x_threshold", g_config.filter.x_threshold, 10.0f);
    pnh.param("filter/y_threshold", g_config.filter.y_threshold, 10.0f);
    pnh.param("filter/z_threshold", g_config.filter.z_threshold, 10.0f);

    int render_mode = static_cast<int>(g_config.render_mode);
    pnh.param("render_mode", render_mode, 0);
    g_config.render_mode = static_cast<RenderMode>(render_mode);
    pnh.param("normal_search_radius", g_config.normal_search_radius, 0.03f);

    pnh.param("visualization/point_size", g_config.visualization.point_size, 2);
    pnh.param("visualization/line_width", g_config.visualization.line_width, 2.0f);
    pnh.param("visualization/coordinate_size", g_config.visualization.coordinate_size, 0.3f);

    // 订阅话题
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
        "/xv_sdk/xv_dev/tof_camera/point_cloud", 1, pointcloudCallback);
    ros::Subscriber sub_pose = nh.subscribe<xv_sdk::PoseStampedConfidence>(
        "/xv_sdk/xv_dev/slam/pose", 1, poseCallback);

    // 启动可视化
    visualizationThread();

    return 0;
}