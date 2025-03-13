#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <xv_sdk/PoseStampedConfidence.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/mutex.hpp>
#include <Eigen/Geometry>
#include <iomanip>

// 共享数据结构
struct SharedData
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr path;
    Eigen::Affine3f pose;
    bool pose_updated;
    boost::mutex mutex;

    // 新增数据成员
    int cloud_width = 0;
    int cloud_height = 0;
    double cloud_fps = 0.0;
    double pose_fps = 0.0;
    ros::Time last_cloud_time;
    ros::Time last_pose_time;

    SharedData() : cloud(new pcl::PointCloud<pcl::PointXYZ>),
                   path(new pcl::PointCloud<pcl::PointXYZ>),
                   pose(Eigen::Affine3f::Identity()),
                   pose_updated(false) {}
};

// 系统配置参数
struct Config
{
    struct
    {
        float y_threshold = 1.0;
        bool enable = true;
    } filter;

    struct
    {
        float coordinate_size = 0.3;
        float line_width = 2.0;
    } visualization;
};

// 全局实例
SharedData g_data;
Config g_config;

void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);

    if (g_config.filter.enable && g_config.filter.y_threshold > 0)
    {
        for (const auto &p : *temp)
        {
            if (std::abs(p.y) < g_config.filter.y_threshold)
            {
                filtered->push_back(p);
            }
        }
    }
    else
    {
        *filtered = *temp;
    }

    boost::mutex::scoped_lock lock(g_data.mutex);
    // 更新点云数据及元信息
    g_data.cloud->swap(*filtered);
    g_data.cloud_width = msg->width;
    g_data.cloud_height = msg->height;

    // 计算点云帧率
    ros::Time now = ros::Time::now();
    if (g_data.last_cloud_time.isValid())
    {
        double delta = (now - g_data.last_cloud_time).toSec();
        if (delta > 0)
        {
            g_data.cloud_fps = 1.0 / delta;
        }
    }
    g_data.last_cloud_time = now;
}

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

    // 坐标系修正：绕X轴旋转180度
    static const Eigen::Quaternionf q_correct(
        Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));

    Eigen::Affine3f pose = Eigen::Affine3f::Identity();
    pose.translate(position);
    pose.rotate(q_orig * q_correct); // 应用修正后的旋转

    pcl::PointXYZ path_point(position.x(), position.y(), position.z());

    boost::mutex::scoped_lock lock(g_data.mutex);
    // 更新位姿数据
    g_data.path->push_back(path_point);
    g_data.pose = pose;
    g_data.pose_updated = true;

    // 计算位姿帧率
    ros::Time now = ros::Time::now();
    if (g_data.last_pose_time.isValid())
    {
        double delta = (now - g_data.last_pose_time).toSec();
        if (delta > 0)
        {
            g_data.pose_fps = 1.0 / delta;
        }
    }
    g_data.last_pose_time = now;
}

void setupVisualizer(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(0, 0, 0);
    viewer.initCameraParameters();
}

void updateVisualization(pcl::visualization::PCLVisualizer &viewer)
{
    static size_t last_path_size = 0;

    // 更新点云显示
    if (!g_data.cloud->empty())
    {
        viewer.removePointCloud("cloud");
        viewer.addPointCloud<pcl::PointXYZ>(g_data.cloud, "cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR,
            0.9, 0.9, 0.9, "cloud");
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

            viewer.addLine<pcl::PointXYZ>(
                p1, p2, 1.0, 0.0, 0.0,
                "line_" + std::to_string(i));
            viewer.setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                g_config.visualization.line_width,
                "line_" + std::to_string(i));
        }
        last_path_size = g_data.path->size() - 1;
    }

    // 更新坐标系显示
    if (g_data.pose_updated)
    {
        viewer.removeCoordinateSystem("camera");
        viewer.addCoordinateSystem(
            g_config.visualization.coordinate_size,
            g_data.pose, "camera");
        g_data.pose_updated = false;
    }

    // 更新文本信息显示
    Eigen::Vector3f translation = g_data.pose.translation();
    Eigen::Vector3f euler_angles = g_data.pose.rotation().eulerAngles(2, 1, 0);

    float roll = euler_angles[2] * 180.0 / M_PI;
    float pitch = euler_angles[1] * 180.0 / M_PI;
    float yaw = euler_angles[0] * 180.0 / M_PI;

    std::ostringstream pose_ss;
    pose_ss << "Position (m):\n"
            << std::fixed << std::setprecision(2)
            << "X: " << translation.x() << "\n"
            << "Y: " << translation.y() << "\n"
            << "Z: " << translation.z() << "\n"
            << "Rotation (deg):\n"
            << "Roll: " << roll << "\n"
            << "Pitch: " << pitch << "\n"
            << "Yaw: " << yaw;

    std::ostringstream fps_ss;
    fps_ss << "Frame Rates (Hz):\n"
           << std::fixed << std::setprecision(1)
           << "PointCloud: " << g_data.cloud_fps << "\n"
           << "Pose: " << g_data.pose_fps;

    std::ostringstream res_ss;
    res_ss << "Resolution: "
           << (g_data.cloud_width > 0 ? std::to_string(g_data.cloud_width) : "N/A")
           << "x"
           << (g_data.cloud_height > 0 ? std::to_string(g_data.cloud_height) : "N/A");

    // 清除旧文本
    viewer.removeText3D("pose_text");
    viewer.removeText3D("fps_text");
    viewer.removeText3D("res_text");

    // 添加新文本（左上角显示）
    viewer.addText(pose_ss.str(), 10, 30, 12, 1.0, 1.0, 1.0, "pose_text");
    viewer.addText(fps_ss.str(), 10, 150, 12, 1.0, 1.0, 1.0, "fps_text");
    viewer.addText(res_ss.str(), 10, 200, 12, 1.0, 1.0, 1.0, "res_text");
}

void visualizationThread()
{
    pcl::visualization::PCLVisualizer viewer("SLAM Viewer");
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
    ros::init(argc, argv, "slam_visualizer");
    ros::NodeHandle nh, pnh("~");

    // 加载配置参数
    pnh.param("filter/enable", g_config.filter.enable, true);
    pnh.param("filter/y_threshold", g_config.filter.y_threshold, 1.0f);
    pnh.param("visualization/coordinate_size",
              g_config.visualization.coordinate_size, 0.3f);
    pnh.param("visualization/line_width",
              g_config.visualization.line_width, 2.0f);

    // 订阅话题
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(
        "/xv_sdk/xv_dev/tof_camera/point_cloud", 1, pointcloudCallback);
    ros::Subscriber sub_pose = nh.subscribe<xv_sdk::PoseStampedConfidence>(
        "/xv_sdk/xv_dev/slam/pose", 1, poseCallback);

    // 启动可视化线程
    visualizationThread();

    return 0;
}