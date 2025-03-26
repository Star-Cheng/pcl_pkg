#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <xv-sdk.h>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define M_PI 3.14159265358979323846

// Global variables
std::shared_ptr<xv::Device> device = nullptr;
std::atomic<bool> stop(false);
ros::Publisher imu_pub;

// SLAM data callback function
void slamCallback(const xv::Pose &pose)
{
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);

    // Create IMU message
    sensor_msgs::Imu imu_msg;
    
    // Set header
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "slam_frame"; // Change to your frame ID
    
    // Get orientation from rotation matrix
    auto rotation = pose.rotation();
    tf2::Matrix3x3 rot_matrix(
        rotation[0], rotation[1], rotation[2],
        rotation[3], rotation[4], rotation[5],
        rotation[6], rotation[7], rotation[8]
    );
    
    tf2::Quaternion q;
    rot_matrix.getRotation(q);
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    
    // For IMU, we typically don't fill position data, but you can add it to another message type
    // If you need to publish position, consider using nav_msgs/Odometry instead
    
    // Get angular velocity (not provided in your original code, set to 0 if unavailable)
    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;
    imu_msg.angular_velocity.z = 0.0;
    
    // Get linear acceleration (not provided in your original code, set to 0 if unavailable)
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;
    
    // Publish the message
    imu_pub.publish(imu_msg);
    
    // Optional: Print data for debugging
    auto pitchYawRoll = xv::rotationToPitchYawRoll(pose.rotation());
    ROS_DEBUG("Position: (%.3f, %.3f, %.3f), Orientation: (%.1f°, %.1f°, %.1f°), Confidence: %.2f",
              pose.x(), pose.y(), pose.z(),
              pitchYawRoll[0] * 180.0 / M_PI,
              pitchYawRoll[1] * 180.0 / M_PI,
              pitchYawRoll[2] * 180.0 / M_PI,
              pose.confidence());
}

int main(int argc, char *argv[])
{
    try
    {
        // Initialize ROS node
        ros::init(argc, argv, "slam_6dof_publisher");
        ros::NodeHandle nh;
        
        // Create publisher
        imu_pub = nh.advertise<sensor_msgs::Imu>("slam/6dof", 10);
        
        // Get version info - modified to handle whatever type xv::version() returns
        // auto version_info = xv::version();
        ROS_INFO("Starting SLAM 6-DOF publisher");
        // If you need to display version info, you might need to check the actual type
        // For example, if it's a struct with major/minor/patch members:
        // ROS_INFO("xvsdk version: %d.%d.%d", version_info.major, version_info.minor, version_info.patch);

        // Get device
        auto devices = xv::getDevices(10., "");
        if (devices.empty())
        {
            ROS_ERROR("Timeout: no device found");
            return EXIT_FAILURE;
        }

        device = devices.begin()->second;

        // Check if SLAM module exists
        if (!device->slam())
        {
            ROS_ERROR("No SLAM module found");
            return EXIT_FAILURE;
        }

        // Start SLAM module
        device->slam()->start();
        
        // Register SLAM callback function
        int slamCallbackId = device->slam()->registerCallback(slamCallback);

        ROS_INFO("SLAM started. Press Ctrl+C to stop.");
        
        // ROS spin loop
        ros::spin();
        
        // Cleanup when ROS is shutting down
        device->slam()->unregisterCallback(slamCallbackId);
        device->slam()->stop();
        ROS_INFO("SLAM stopped");

        return EXIT_SUCCESS;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return EXIT_FAILURE;
    }
}