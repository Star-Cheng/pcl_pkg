#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <xv-sdk.h>
#include <opencv2/opencv.hpp>
#include <memory>

// 创建ROS图像发布器对象
std::shared_ptr<ros::Publisher> left_pub;
std::shared_ptr<ros::Publisher> right_pub;

// 回调函数，处理双目图像数据
void stereoCallback(const xv::FisheyeImages &stereo)
{
    // 确保有两个图像（左/右）
    if(stereo.images.size() < 2) return;

    // 获取当前ROS时间
    ros::Time timestamp = ros::Time::now();

    // 处理左图像
    {
        cv::Mat left_img(stereo.images[0].height, 
                        stereo.images[0].width,
                        CV_8UC1, 
                        const_cast<uint8_t*>(stereo.images[0].data.get()));

        // 创建ROS图像消息
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            std_msgs::Header(), 
            "mono8",   // 灰度图像编码
            left_img
        ).toImageMsg();

        // 设置时间戳和帧ID
        msg->header.stamp = timestamp;
        msg->header.frame_id = "left_camera";

        // 发布消息
        left_pub->publish(msg);
    }

    // 处理右图像（与左图像逻辑相同）
    {
        cv::Mat right_img(stereo.images[1].height,
                         stereo.images[1].width,
                         CV_8UC1,
                         const_cast<uint8_t*>(stereo.images[1].data.get()));

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
            std_msgs::Header(),
            "mono8",
            right_img
        ).toImageMsg();

        msg->header.stamp = timestamp;
        msg->header.frame_id = "right_camera";
        right_pub->publish(msg);
    }
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "stereo_camera_publisher");
    ros::NodeHandle nh;

    // 创建发布器
    left_pub = std::make_shared<ros::Publisher>(
        nh.advertise<sensor_msgs::Image>("camera/left", 1));
    right_pub = std::make_shared<ros::Publisher>(
        nh.advertise<sensor_msgs::Image>("camera/right", 1));

    try {
        // 初始化设备（保持原设备初始化代码）
        xv::setLogLevel(xv::LogLevel::info);
        auto devices = xv::getDevices(10.);
        if(devices.empty()) {
            ROS_ERROR("No device found!");
            return EXIT_FAILURE;
        }

        auto device = devices.begin()->second;
        if(!device->fisheyeCameras()) {
            ROS_ERROR("No fisheye cameras!");
            return EXIT_FAILURE;
        }

        // 注册回调
        device->fisheyeCameras()->registerCallback(stereoCallback);
        device->fisheyeCameras()->start();

        ROS_INFO("Camera started. Publishing images...");

        // ROS异步spinfdfd
        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::Rate loop_rate(30); // 设置循环频率为 30Hz  
        // 主循环
        while(ros::ok()) {
            // 可在此添加其他逻辑
            ros::spinOnce(); // 处理回调  
            loop_rate.sleep(); // 睡眠到下一个循环  
        }

        // 停止设备
        device->fisheyeCameras()->stop();
        ROS_INFO("Camera stopped.");

    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}