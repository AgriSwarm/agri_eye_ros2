#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"

using std::placeholders::_1;

class DataCollector : public rclcpp::Node {
public:
    DataCollector() : Node("data_collector"), count_(0) {
        using namespace std::chrono_literals;

        freq_ = this->declare_parameter("freq", 10);

        // Initialize subscribers
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DataCollector::odometry_callback, this, _1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&DataCollector::imu_callback, this, _1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_image", 10, std::bind(&DataCollector::image_callback, this, _1));

        // Initialize the timer
        timer_ = this->create_wall_timer(
            1000ms / freq_, std::bind(&DataCollector::timer_callback, this));
    }

    ~DataCollector() {
        // Save to file upon destruction
        save_to_file();
    }

private:
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odometry_ = *msg;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_ = *msg;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    void timer_callback() {
        RCLCPP_INFO(this->get_logger(), "Taking a snapshot...");

        // Save the snapshot of current data
        snapshots_.push_back(std::make_tuple(last_odometry_, last_imu_, last_image_));
    }

    void save_to_file() {
        RCLCPP_INFO(this->get_logger(), "Saving snapshots to file...");

        // seq_{frequency}hz
        std::string dir_name = "seq_" + std::to_string(freq_) + "hz";

        std::filesystem::create_directory(dir_name);
        std::filesystem::create_directory(dir_name + "/images");
        
        // Open CSV file for odometry and IMU data
        std::ofstream odom_imu_file(dir_name + "/odom_imu_data.csv");

        // Write header for odometry and IMU data
        odom_imu_file << "timestamp,pose_x,pose_y,pose_z,";
        odom_imu_file << "orientation_x,orientation_y,orientation_z,orientation_w,";
        odom_imu_file << "angular_velocity_x,angular_velocity_y,angular_velocity_z,";
        odom_imu_file << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z\n";

        // Image files counter
        int image_counter = 0;

        // Iterate through the collected snapshots and write to the CSV file
        for (const auto &snapshot : snapshots_) {
            const auto &odom = std::get<0>(snapshot);
            const auto &imu = std::get<1>(snapshot);
            const auto &image = std::get<2>(snapshot);

            // Write odometry and IMU data
            odom_imu_file << odom.header.stamp.sec << "." << odom.header.stamp.nanosec << ",";
            odom_imu_file << odom.pose.pose.position.x << "," << odom.pose.pose.position.y << "," << odom.pose.pose.position.z << ",";
            odom_imu_file << odom.pose.pose.orientation.x << "," << odom.pose.pose.orientation.y << ",";
            odom_imu_file << odom.pose.pose.orientation.z << "," << odom.pose.pose.orientation.w << ",";
            odom_imu_file << imu.angular_velocity.x << "," << imu.angular_velocity.y << "," << imu.angular_velocity.z << ",";
            odom_imu_file << imu.linear_acceleration.x << "," << imu.linear_acceleration.y << "," << imu.linear_acceleration.z << "\n";

            std::ostringstream img_filename;
            img_filename << dir_name + "/images/image_" << image_counter++ << ".jpg";
            cv::imwrite(img_filename.str(), image);

            RCLCPP_INFO(this->get_logger(), "Data snapshot saved.");
        }

        // Close the file
        odom_imu_file.close();
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    nav_msgs::msg::Odometry last_odometry_;
    sensor_msgs::msg::Imu last_imu_;
    cv::Mat last_image_;
    std::vector<std::tuple<nav_msgs::msg::Odometry, sensor_msgs::msg::Imu, cv::Mat>> snapshots_;

    int count_;
    int freq_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataCollector>());
    rclcpp::shutdown();
    return 0;
};