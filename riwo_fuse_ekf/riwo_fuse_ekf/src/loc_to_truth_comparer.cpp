#include "riwo_fg/loc_to_truth_comparer.hpp"

LocToTruthComparer::LocToTruthComparer() : rclcpp::Node("loc_to_truth_comparer")
{
  // Declare all initial parameters and set them to default values
  this->declareParams();
  this->getParams();

  // Create subscriber
  this->truth_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    this->truth_topic, sub_buffer_size, std::bind(&LocToTruthComparer::compare, this, std::placeholders::_1));

  // Create transform listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Create publisher
  this->output_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(this->output_topic, pub_buffer_size);
  this->truth_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(this->truth_republish_topic, pub_buffer_size);

  // Informative message
  RCLCPP_INFO(
    this->get_logger(),
    "Waiting for ground truth data on topic \"%s\". Comparing to localization from \"%s\" to \"%s\"",
    this->truth_topic.c_str(), this->world_frame.c_str(), this->robot_frame.c_str());
}

void LocToTruthComparer::declareParams()
{
  this->declare_parameter("truth_topic", rclcpp::ParameterValue("/navant0/rfid"));
  this->declare_parameter("robot_frame", rclcpp::ParameterValue("base_link"));
  this->declare_parameter("world_frame", rclcpp::ParameterValue("map"));
  this->declare_parameter("output_topic", rclcpp::ParameterValue("localization_error_fuse"));
  this->declare_parameter("truth_republish_topic", rclcpp::ParameterValue("/navant0/rfid_republished"));
  this->declare_parameter("sub_buffer_size", rclcpp::ParameterValue(10));
  this->declare_parameter("pub_buffer_size", rclcpp::ParameterValue(10));
}

void LocToTruthComparer::getParams()
{
  truth_topic = (this->get_parameter("truth_topic")).as_string();
  robot_frame = (this->get_parameter("robot_frame")).as_string();
  world_frame = (this->get_parameter("world_frame")).as_string();
  output_topic = (this->get_parameter("output_topic")).as_string();
  truth_republish_topic = (this->get_parameter("truth_republish_topic")).as_string();
  sub_buffer_size = (this->get_parameter("sub_buffer_size")).as_int();
  pub_buffer_size = (this->get_parameter("pub_buffer_size")).as_int();
}

// Compare the pose from incoming ground truth message to localization of the robot
void LocToTruthComparer::compare(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr truth_msg)
{
  // Check if the received ground truth has the same frame as the set world_frame
  if (truth_msg->header.frame_id != this->world_frame) {
    RCLCPP_WARN(
      this->get_logger(), "Cannot compare localization frame \"%s\" to ground truth frame \"%s\"",
      this->world_frame.c_str(), truth_msg->header.frame_id.c_str());
    return;
  }

  // Obtain most recent transfrom between world_frame and robot_frame
  geometry_msgs::msg::TransformStamped loc;
  try
  {
    loc = tf_buffer_->lookupTransform(
        this->world_frame, this->robot_frame,
        truth_msg->header.stamp, 50ms);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        this->world_frame.c_str(), this->robot_frame.c_str(), ex.what());
    return;
  }

  // Compute the position error
  float error_x = truth_msg->pose.pose.position.x - loc.transform.translation.x;
  float error_y = truth_msg->pose.pose.position.y - loc.transform.translation.y;
  float error_z = truth_msg->pose.pose.position.z - loc.transform.translation.z;
  float error_d = std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2) + std::pow(error_z, 2));

  // Compute the orientation error
  double roll_truth, roll_loc, pitch_truth, pitch_loc, yaw_truth, yaw_loc; 
  tf2::Quaternion q_truth(truth_msg->pose.pose.orientation.x,truth_msg->pose.pose.orientation.y,truth_msg->pose.pose.orientation.z,truth_msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m_truth(q_truth);
  m_truth.getRPY(roll_truth,pitch_truth,yaw_truth);
  tf2::Quaternion q_loc(loc.transform.rotation.x,loc.transform.rotation.y,loc.transform.rotation.z,loc.transform.rotation.w);
  tf2::Matrix3x3 m_loc(q_loc);
  m_loc.getRPY(roll_loc,pitch_loc,yaw_loc);
  
  // Compute the time error
  float error_t = (truth_msg->header.stamp.sec + 1e-9*truth_msg->header.stamp.nanosec) - (loc.header.stamp.sec + 1e-9*loc.header.stamp.nanosec);

  //
  // Set up and fill diagnostic message
  //
  diagnostic_msgs::msg::DiagnosticArray diagnostic_arr;
  diagnostic_arr.header.stamp = truth_msg->header.stamp;  // Diagnostic message stamp should be equal to ground truth message
  
  // Data on ground truth pose
  diagnostic_msgs::msg::DiagnosticStatus truth_status;  
  truth_status.name = "Ground truth pose";

  diagnostic_msgs::msg::KeyValue tx, ty, tz, troll, tpitch, tyaw;
  tx.key = "Truth x"; tx.value = std::to_string(truth_msg->pose.pose.position.x);
  ty.key = "Truth y"; ty.value = std::to_string(truth_msg->pose.pose.position.y);
  tz.key = "Truth z"; tz.value = std::to_string(truth_msg->pose.pose.position.z);
  troll.key = "Truth roll"; troll.value = std::to_string(roll_truth);
  tpitch.key = "Truth pitch"; tpitch.value = std::to_string(pitch_truth);
  tyaw.key = "Truth yaw"; tyaw.value = std::to_string(yaw_truth);

  truth_status.values.push_back(tx);
  truth_status.values.push_back(ty);
  truth_status.values.push_back(tz);
  truth_status.values.push_back(troll);
  truth_status.values.push_back(tpitch);
  truth_status.values.push_back(tyaw);
  
  // Data on localization pose
  diagnostic_msgs::msg::DiagnosticStatus localization_status;
  localization_status.name = "Localization pose";

  diagnostic_msgs::msg::KeyValue lx, ly, lz, lroll, lpitch, lyaw;
  lx.key = "Localization x"; lx.value = std::to_string(loc.transform.translation.x);
  ly.key = "Localization y"; ly.value = std::to_string(loc.transform.translation.y);
  lz.key = "Localization z"; lz.value = std::to_string(loc.transform.translation.z);
  lroll.key = "Localization roll"; lroll.value = std::to_string(roll_loc);
  lpitch.key = "Localization pitch"; lpitch.value = std::to_string(pitch_loc);
  lyaw.key = "Localization yaw"; lyaw.value = std::to_string(yaw_loc);

  localization_status.values.push_back(lx);
  localization_status.values.push_back(ly);
  localization_status.values.push_back(lz);
  localization_status.values.push_back(lroll);
  localization_status.values.push_back(lpitch);
  localization_status.values.push_back(lyaw);

  // Data on error between truth pose and localization pose
  diagnostic_msgs::msg::DiagnosticStatus error_status;
  error_status.name = "Localization to ground truth error";

  diagnostic_msgs::msg::KeyValue ex, ey, ez, ed, eroll, epitch, eyaw, et;
  ex.key = "Error x"; ex.value = std::to_string(error_x);
  ey.key = "Error y"; ey.value = std::to_string(error_y);
  ez.key = "Error z"; ez.value = std::to_string(error_z);
  ed.key = "Error euclidean distance"; ed.value = std::to_string(error_d);
  eroll.key = "Error roll"; eroll.value = std::to_string(roll_truth-roll_loc);
  epitch.key = "Error pitch"; epitch.value = std::to_string(pitch_truth-pitch_loc);
  eyaw.key = "Error yaw"; eyaw.value = std::to_string(yaw_truth-yaw_loc);
  et.key = "Time difference"; et.value = std::to_string(error_t);

  error_status.values.push_back(ex);
  error_status.values.push_back(ey);
  error_status.values.push_back(ez);
  error_status.values.push_back(ed);
  error_status.values.push_back(eroll);
  error_status.values.push_back(epitch);
  error_status.values.push_back(eyaw);
  error_status.values.push_back(et);

  // Publish diagnostics message
  diagnostic_arr.status.push_back(truth_status);
  diagnostic_arr.status.push_back(localization_status);
  diagnostic_arr.status.push_back(error_status);
  this->output_pub->publish(diagnostic_arr);

  // Print information
  RCLCPP_INFO(
    this->get_logger(), "Localization error: %.3f m (x=%.3f, y=%.3f, z=%.3f). Localization data %.3f s old.", error_d, error_x,
    error_y, error_z, error_t);

  // Republish truth data after comparing loc to truth
  this->truth_pub->publish(*truth_msg);
}

// Run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocToTruthComparer>());
  rclcpp::shutdown();
  return 0;
}