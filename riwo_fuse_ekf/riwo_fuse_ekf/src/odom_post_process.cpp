#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
// add imu msg
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ImuPostProcess : public rclcpp::Node
{
public:
    ImuPostProcess()
        : Node("odom_post_process")
    {
        RCLCPP_INFO(this->get_logger(), "Initial Pose Publisher Node has been started.");
        // get param for subscription topics
        this->declare_parameter<std::string>("wheel_odom_topic", "/wheel_odom");
        this->declare_parameter<std::string>("wheel_odom_out_topic", "/wheel_odom_fixed");

        std::string wheel_odom_topic;
        std::string wheel_odom_out_topic;

        this->get_parameter("wheel_odom_topic", wheel_odom_topic);
        this->get_parameter("wheel_odom_out_topic", wheel_odom_out_topic);

        wheel_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            wheel_odom_topic,
            10,
            std::bind(&ImuPostProcess::wheel_odom_callback, this, std::placeholders::_1));

        wheel_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(wheel_odom_out_topic, 10);
    }

private:
    void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom_out = *msg;

        // Change frames
        odom_out.header.frame_id = "odom";
        odom_out.child_frame_id = "base_link";
        // get covraince matrix

        odom_out.twist.covariance[0] = 0.03;   // vx
        odom_out.twist.covariance[7] = 0.03;   // vy
        odom_out.twist.covariance[14] = 0.03;  // vz
        odom_out.twist.covariance[21] = 0.03;  // wx
        odom_out.twist.covariance[28] = 0.03;  // wy
        odom_out.twist.covariance[35] = 0.025; // wz

        // Republish
        wheel_odom_pub->publish(odom_out);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPostProcess>());
    rclcpp::shutdown();
    return 0;
}