#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
public:
    InitialPosePublisher()
        : Node("pose_log")
    {
        pose_file_name = generateFilename();
        std::cout << "pose log file name:" << pose_file_name << std::endl;

        // getting parameter for subscription topics
        this->declare_parameter<std::string>("odom_gps","/odometry/ekf_global");
        this->declare_parameter<std::string>("odom_local","/odometry/fuse_tier1");
        this->declare_parameter<std::string>("odom_global","/odometry/fuse_tier2");
        this->declare_parameter<std::string>("imu_topic","/imu/data");
        this->declare_parameter<std::string>("wheel_odom_topic","/wheel_odom");
        this->declare_parameter<std::string>("wheel_odom_out_topic","/wheel_odom_fixed");

        std::string odom_gps;
        std::string odom_local;
        std::string odom_global;
        std::string imu_topic;
        std::string wheel_odom_topic;
        std::string wheel_odom_out_topic;

        this->get_parameter("odom_gps",odom_gps);
        this->get_parameter("odom_local",odom_local);
        this->get_parameter("odom_global",odom_global);
        this->get_parameter("imu_topic", imu_topic);
        this->get_parameter("wheel_odom_topic", wheel_odom_topic);
        this->get_parameter("wheel_odom_out_topic", wheel_odom_out_topic);

        imu_topic_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic,
            rclcpp::SensorDataQoS(),
            std::bind(&InitialPosePublisher::imu_callback, this, std::placeholders::_1));

        imu_filtered_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/filtered", 10);

        gps_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_gps, 10, std::bind(&InitialPosePublisher::gps_odom_callback, this, std::placeholders::_1));
        local_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_local, 10, std::bind(&InitialPosePublisher::local_odom_callback, this, std::placeholders::_1));
        global_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_global, 10, std::bind(&InitialPosePublisher::global_odom_callback, this, std::placeholders::_1));
        wheel_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            wheel_odom_topic,
            10,
            std::bind(&InitialPosePublisher::wheel_odom_callback, this, std::placeholders::_1));

        wheel_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(wheel_odom_out_topic, 10);
    }
      
private:
    std::string generateFilename()
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "/home/ibrahim/pose_logs/";
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S");
        return ss.str();
    }

    double calc_distance(nav_msgs::msg::Odometry pose1, nav_msgs::msg::Odometry pose2){
        double x1 = pose1.pose.pose.position.x;
        double y1 = pose1.pose.pose.position.y;
        double z1 = pose1.pose.pose.position.z;
        double x2 = pose2.pose.pose.position.x;
        double y2 = pose2.pose.pose.position.y;
        double z2 = pose2.pose.pose.position.z;
        return sqrt(pow(x1 - x2, 2)+ pow(y1 -y2,2)+pow(z1-z2,2));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr global_odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_topic_sub;

    //publish filtered topics
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_filtered_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub;
    std::string pose_file_name;
    
    //pervious pose msg
    nav_msgs::msg::Odometry pre_gps_pose;
    nav_msgs::msg::Odometry pre_local_pose;
    nav_msgs::msg::Odometry pre_global_pose;

    bool gps_init = false;
    bool local_init = false;
    bool global_init = false;

    double th_distance = 0.2;

   void gps_odom_callback(nav_msgs::msg::Odometry msg){
        std::cout << "gps callback" << std::endl;
        
        if (gps_init && calc_distance(pre_gps_pose, msg)< th_distance){
            return;
        }
        std::ofstream pose_log;
        // Set fixed-point notation and high precision (9 digits after decimal)
        pose_log.open(pose_file_name + "_gps_poses", std::ios::app);
        pose_log << std::fixed << std::setprecision(9);
        double timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
        pose_log << timestamp << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << msg.pose.pose.position.z << " " << msg.pose.pose.orientation.x << " " << msg.pose.pose.orientation.y << " " << msg.pose.pose.orientation.z << " " << msg.pose.pose.orientation.w << std::endl;
        pose_log.close();
        pre_gps_pose = msg;
        gps_init = true;
    };

   void local_odom_callback(nav_msgs::msg::Odometry msg)
    {

        if (local_init && calc_distance(pre_local_pose, msg) < th_distance)
        {
            return;
        }
        std::ofstream pose_log;
        double timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

        // Set fixed-point notation and high precision (9 digits after decimal)
        pose_log.open(pose_file_name + "_local_poses", std::ios::app);
        pose_log << std::fixed << std::setprecision(9);
        pose_log << timestamp << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << msg.pose.pose.position.z << " " << msg.pose.pose.orientation.x << " " << msg.pose.pose.orientation.y << " " << msg.pose.pose.orientation.z << " " << msg.pose.pose.orientation.w << std::endl;
        pose_log.close();
        pre_local_pose = msg;
        local_init = true;
    };

    void global_odom_callback(nav_msgs::msg::Odometry msg)
    {
         std::cout << "gps global" << std::endl;

        if (global_init && calc_distance(pre_global_pose, msg) < th_distance)
        {
            return;
        }
        std::ofstream pose_log;
        double timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;

        // Set fixed-point notation and high precision (9 digits after decimal)
        pose_log.open(pose_file_name + "_global_poses", std::ios::app);
        pose_log << std::fixed << std::setprecision(9);
        pose_log << timestamp << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << msg.pose.pose.position.z << " " << msg.pose.pose.orientation.x << " " << msg.pose.pose.orientation.y << " " << msg.pose.pose.orientation.z << " " << msg.pose.pose.orientation.w << std::endl;
        pose_log.close();
        pre_global_pose = msg;
        global_init = true;
    };

   void read_tum_dataset()
   {

   };

   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
    sensor_msgs::msg::Imu imu_filtered = *msg;

    // Add covariance
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
    };
   void wheel_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
   {
    nav_msgs::msg::Odometry odom_out = *msg;

    // change frames
    odom_out.header.frame_id = "odom";
    odom_out.child_frame_id = "base_link";

    //get covariance matrix
    odom_out.twist.covariance[0]=0.03; //vx
    odom_out.twist.covariance[7]=0.03; //vy
    odom_out.twist.covariance[14]=0.03; //vz
    odom_out.twist.covariance[21]=0.03; //wx
    odom_out.twist.covariance[28]=0.03; //wy
    odom_out.twist.covariance[35]=0.03; //wz

    //republish the topic
    wheel_odom_pub->publish(odom_out);
    };
   
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}