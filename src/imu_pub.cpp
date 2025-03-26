#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <xv-sdk.h>
#include <memory>
#include <functional>

class IMUDataInterface {
public:
    using IMUCallback = std::function<void(const xv::Imu&)>;

    IMUDataInterface(std::shared_ptr<xv::Device> device) 
        : m_device(device), m_callbackId(-1), m_isRunning(false) {}

    bool registerIMUCallback(IMUCallback callback) {
        if (!m_device || !m_device->imuSensor()) {
            ROS_ERROR("Device or IMU sensor not available");
            return false;
        }

        m_callback = callback;
        m_callbackId = m_device->imuSensor()->registerCallback(
            [this](xv::Imu const& imu) {
                if (m_callback) {
                    m_callback(imu);
                }
            });
            
        if (m_callbackId == -1) {
            ROS_ERROR("Failed to register IMU callback");
            return false;
        }
        return true;
    }

    bool start() {
        if (m_isRunning) return true;
        if (!m_device || !m_device->imuSensor()) {
            ROS_ERROR("Device or IMU sensor not available");
            return false;
        }

        if (m_callbackId == -1 && !registerIMUCallback(nullptr)) {
            return false;
        }

        m_device->imuSensor()->start();
        m_isRunning = true;
        ROS_INFO("IMU data stream started");
        return true;
    }

    bool stop() {
        if (!m_isRunning) return true;
        if (!m_device || !m_device->imuSensor()) {
            ROS_ERROR("Device or IMU sensor not available");
            return false;
        }

        if (m_callbackId != -1) {
            m_device->imuSensor()->unregisterCallback(m_callbackId);
            m_callbackId = -1;
        }

        m_device->imuSensor()->stop();
        m_isRunning = false;
        ROS_INFO("IMU data stream stopped");
        return true;
    }

    ~IMUDataInterface() { stop(); }

private:
    std::shared_ptr<xv::Device> m_device;
    IMUCallback m_callback;
    int m_callbackId;
    bool m_isRunning;
};

class IMUROSPublisher {
public:
    IMUROSPublisher() : nh("~") {
        // 初始化XV-SDK设备
        auto devices = xv::getDevices(10.);
        if (devices.empty()) {
            ROS_ERROR("No XV-SDK device found");
            throw std::runtime_error("No XV-SDK device found");
        }
        device = devices.begin()->second;

        // 创建IMU接口
        imuInterface = std::make_unique<IMUDataInterface>(device);

        // 创建ROS发布者
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data", 1000);

        // 设置IMU回调
        imuInterface->registerIMUCallback(
            [this](const xv::Imu& imu) {
                publishIMUData(imu);
            });

        // 启动IMU数据流
        if (!imuInterface->start()) {
            throw std::runtime_error("Failed to start IMU data stream");
        }
    }

    void publishIMUData(const xv::Imu& imu) {
        sensor_msgs::Imu imu_msg;
        
        // 设置消息头
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

        // 设置角速度 (rad/s)
        imu_msg.angular_velocity.x = imu.gyro[0];
        imu_msg.angular_velocity.y = imu.gyro[1];
        imu_msg.angular_velocity.z = imu.gyro[2];

        // 设置线加速度 (m/s^2)
        imu_msg.linear_acceleration.x = imu.accel[0];
        imu_msg.linear_acceleration.y = imu.accel[1];
        imu_msg.linear_acceleration.z = imu.accel[2];

        // 发布消息
        imu_pub.publish(imu_msg);
    }

    ~IMUROSPublisher() {
        imuInterface->stop();
    }

private:
    ros::NodeHandle nh;
    ros::Publisher imu_pub;
    std::shared_ptr<xv::Device> device;
    std::unique_ptr<IMUDataInterface> imuInterface;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_publisher");
    
    try {
        IMUROSPublisher publisher;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in IMU publisher: %s", e.what());
        return 1;
    }

    return 0;
}