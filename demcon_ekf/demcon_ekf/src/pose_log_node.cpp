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
        this->declare_parameter<std::string>("odom_gps","/odometry/gps");
        this->declare_parameter<std::string>("odom_local","/odometry/local");
        this->declare_parameter<std::string>("odom_global","/odometry/global");
        
        std::string odom_gps;
        std::string odom_local;
        std::string odom_global;

        this->get_parameter("odom_gps",odom_gps);
        this->get_parameter("odom_local",odom_local);
        this->get_parameter("odom_global",odom_global);

        gps_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_gps, 10, std::bind(&InitialPosePublisher::gps_odom_callback, this, std::placeholders::_1));
        local_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_local, 10, std::bind(&InitialPosePublisher::local_odom_callback, this, std::placeholders::_1));
        global_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_global, 10, std::bind(&InitialPosePublisher::global_odom_callback, this, std::placeholders::_1));

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
   
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPosePublisher>());
    rclcpp::shutdown();
    return 0;
}