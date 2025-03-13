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
ros::Publisher pub; // 全局发布器对象
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
    ros::Time pose_stamp;  // 新增位姿时间戳

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
        float x_threshold = 10.0;
        float y_threshold = 1.0;
        float z_threshold = 10.0;
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
        for (const auto &point : *temp)
        {
            if (std::abs(point.x) < g_config.filter.x_threshold && std::abs(point.y) < g_config.filter.y_threshold && std::abs(point.z) < g_config.filter.z_threshold)
            {

                // 反转xyz值
                pcl::PointXYZ reversed_point;
                reversed_point.x = point.x;
                reversed_point.y = point.y;
                reversed_point.z = point.z;
                filtered->push_back(reversed_point);
            }
        }
    }
    else
    {
        // *filtered = *temp;
        // 不过滤时，直接反转xyz值
        for (const auto &point : *temp)
        {
            pcl::PointXYZ reversed_point;
            reversed_point.x = point.x;
            reversed_point.y = point.y;
            reversed_point.z = point.z;
            filtered->push_back(reversed_point);
        }
    }
    // 将处理后的点云转换为ROS消息并发布
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*filtered, output_msg);
    output_msg.header = msg->header; // 保持原始消息的header
    pub.publish(output_msg); // 发布处理后的点云
    boost::mutex::scoped_lock lock(g_data.mutex);
    g_data.pose_stamp = msg->header.stamp;  // 记录时间戳
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
// 新增键盘回调函数
void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void*) {
    if (event.getKeySym() == "s" && event.keyDown()) {
        boost::mutex::scoped_lock lock(g_data.mutex);
        
        if (!g_data.cloud->empty() && g_data.pose_stamp.isValid()) {
            // 生成时间戳字符串
            const time_t raw_time = g_data.pose_stamp.sec;
            char time_str[20];
            strftime(time_str, 20, "%Y%m%d_%H%M%S", localtime(&raw_time));
            // 确保目录存在
            const std::string dir_path = "data";
            if (!boost::filesystem::exists(dir_path)) {
                if (!boost::filesystem::create_directory(dir_path)) {
                    std::cerr << "Failed to create directory: " << dir_path << std::endl;
                    return;
                }
            }
            // 构造文件名
            // std::string filename = "./data/" + std::string(time_str) + "_" + 
            //                      std::to_string(g_data.pose_stamp.nsec) + ".pcd";
            std::string filename = "./data/" + std::to_string(g_data.pose_stamp.sec) + "_" + 
                                 std::to_string(g_data.pose_stamp.nsec) + ".pcd";
            
            // 保存点云
            if (pcl::io::savePCDFileASCII(filename, *g_data.cloud) == 0) {
                std::cout << "Saved: " << filename << std::endl;
            } else {
                std::cerr << "Save failed: " << filename << std::endl;
            }
        } else {
            std::cerr << "No cloud data or invalid timestamp" << std::endl;
        }
    }
}
void drawCameraIndicator(pcl::visualization::PCLVisualizer &viewer,
                         const Eigen::Affine3f &pose)
{
    const float size = 0.5;

    // 定义局部坐标系三角形顶点（相机坐标系）
    Eigen::Vector3f local_points[3] = {
        Eigen::Vector3f(0, 0, 0),            // 原点
        Eigen::Vector3f(size, size, -size), // 左下
        Eigen::Vector3f(-size, size, -size)   // 右下
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
    Eigen::Vector3f tip = pose * Eigen::Vector3f(0, 0, -size);
    viewer.removeShape("cam_arrow");
    viewer.addArrow(pcl::PointXYZ(tip.x(), tip.y(), tip.z()),
                    world_points[0],
                    0.0, 1.0, 0.0, false, "cam_arrow");
}
void setupVisualizer(pcl::visualization::PCLVisualizer &viewer)
{
    viewer.setBackgroundColor(0, 0, 0);
    Eigen::Affine3f pose_x = Eigen::Affine3f::Identity();             // 创建单位矩阵
    pose_x.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX())); // 沿 X 轴旋转 180 度 (M_PI = 180度)
    viewer.addCoordinateSystem(0.1, pose_x);
    // viewer.addCoordinateSystem(0.1);
    // 添加坐标轴标签
    viewer.addText3D("X", pcl::PointXYZ(0.1, 0, 0), 0.01, 1.0, 0.0, 0.0, "text_X"); // 红色
    viewer.addText3D("Y", pcl::PointXYZ(0, -0.1, 0), 0.01, 0.0, 1.0, 0.0, "text_Y"); // 绿色
    viewer.addText3D("Z", pcl::PointXYZ(0, 0, -0.1), 0.01, 0.0, 0.0, 1.0, "text_Z"); // 蓝色
    viewer.initCameraParameters();
    viewer.registerKeyboardCallback(keyboardCallback);  // 注册键盘回调
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
        drawCameraIndicator(viewer, g_data.pose);
        viewer.removeCoordinateSystem("camera");
        // viewer.addCoordinateSystem(
        //     g_config.visualization.coordinate_size,
        //     g_data.pose, "camera");
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
    // 初始化发布器，话题名为/processed_cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/processed_cloud", 10);
    // 启动可视化线程
    visualizationThread();

    return 0;
}