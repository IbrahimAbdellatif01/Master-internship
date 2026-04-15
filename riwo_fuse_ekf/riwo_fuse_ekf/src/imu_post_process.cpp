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
        : Node("pose_log")
    {   
        RCLCPP_INFO(this->get_logger(), "Initial Pose Publisher Node has been started.");
        // get param for subscription topics 
        this->declare_parameter<std::string>("odom_gps","/odom_filtered");
        this->declare_parameter<std::string>("imu_topic", "/imu/data");    
        std::string imu_topic;

        this->get_parameter("imu_topic",imu_topic);

        imu_topic_sub = this->create_subscription<sensor_msgs::msg::Imu>(
                imu_topic,
                rclcpp::SensorDataQoS(),
                std::bind(&ImuPostProcess::imu_callback, this, std::placeholders::_1)
        );

        imu_filtered_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/filtered", 10);
      
    }

private:
    void read_tum_dataset()
    {
        // read tum dataset and save to log file
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        //    imu mesaage
        sensor_msgs::msg::Imu  imu_filtered;
        //  std::cout << "imu callback" << std::endl;
        imu_filtered = *msg;
        // add covariance to imu message
        imu_filtered.orientation_covariance[0] = 0.01;
        imu_filtered.orientation_covariance[4] = 0.01;
        imu_filtered.orientation_covariance[8] = 0.01;
        imu_filtered.angular_velocity_covariance[0] = 0.01;
        imu_filtered.angular_velocity_covariance[4] = 0.01;
        imu_filtered.angular_velocity_covariance[8] = 0.01;
        imu_filtered.linear_acceleration_covariance[0] = 0.1;
        imu_filtered.linear_acceleration_covariance[4] = 0.1;
        imu_filtered.linear_acceleration_covariance[8] = 0.1;
        imu_filtered_pub->publish(imu_filtered);

    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_topic_sub;
    // imu filtered message variable 
    
    // publish filtered imu add covariance
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_filtered_pub;
    std::string pose_file_name;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPostProcess>());
    rclcpp::shutdown();
    return 0;
}