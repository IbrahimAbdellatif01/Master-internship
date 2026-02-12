
#include "rclcpp/rclcpp.hpp"
#include "fgo/cpp_header.hpp"
// Include your header file to use it
int main(int argc, char **argv)
{

    // Initiate ROS communications
    rclcpp::init(argc, argv);
    // Instantiate the node
    auto node = std::make_shared<rclcpp::Node>("cpp_node");
    // parms set

    std::cout << "C++ Node has been started." << std::endl;
    // Make the node spin
    rclcpp::spin(node);
    // Shutdown ROS communications
    rclcpp::shutdown();
    return 0;
}
