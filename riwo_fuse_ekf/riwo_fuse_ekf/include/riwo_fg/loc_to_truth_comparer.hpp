#ifndef RIWO_FG_LOC_TO_TRUTH_COMPARER
#define RIWO_FG_LOC_TO_TRUTH_COMPARER

#include <cmath>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

using namespace std::chrono_literals;

/**
 * @brief node that compares the difference between the localized pose (transform world to robot (e.g. map->base_link), NOT the EKF outputs) 
 * and a ground truth pose (e.g. RFID pose received from the navigation antenna). Whenever it receives a ground truth message, the
 * position difference between it and the localization is computed. Results are published on the output topic.
 *
 * @author Riwo Engineering B.V.
 */
class LocToTruthComparer : public rclcpp::Node
{
public:
  /**
     * @brief Construct an new comparer node
     */
  LocToTruthComparer();

private:
  // Parameters
  std::string truth_topic;       // Topic name for ground truth input (e.g. RFID tags)
  std::string robot_frame;       // Robot frame name
  std::string world_frame;       // World frame name
  std::string output_topic;      // Topic name of output
  std::string truth_republish_topic;
  int sub_buffer_size;           // Size of the ground truth message buffer
  int pub_buffer_size;           // Size of the output message buffer

  // Publishers and subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr truth_sub;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr output_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr truth_pub;

  // Transform listener with buffer for localization
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
     * @brief Declare all parameters of this node with the defaults set in the header file
     */
  void declareParams();

  /**
     * @brief Get all initial parameters of this node which are passed to this node on startup, or keep the defaults in
     * place
     */
  void getParams();

  /**
     * @brief Handle a ground truth message. This computes the distance between the pose received from the ground truth topic and the
     * localization. The differences are published on the output topic.
     * @param msg Received message.
     */
  void compare(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr truth_msg);
};

#endif /* RIWO_FG_LOC_TO_TRUTH_COMPARER */
