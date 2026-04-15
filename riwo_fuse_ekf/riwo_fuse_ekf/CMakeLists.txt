cmake_minimum_required(VERSION 3.8)
project(riwo_fuse_ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

add_executable(pose_log src/pose_log_node.cpp)
ament_target_dependencies(pose_log rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs sensor_msgs)

add_executable(imu_post_process src/imu_post_process.cpp)
ament_target_dependencies(imu_post_process rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs sensor_msgs)

add_executable(loc_to_truth_comparer src/loc_to_truth_comparer.cpp)
ament_target_dependencies(loc_to_truth_comparer rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs sensor_msgs diagnostic_msgs)
add_executable(odom_post_process src/odom_post_process.cpp)
ament_target_dependencies(odom_post_process rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs sensor_msgs)

install(TARGETS
  pose_log imu_post_process odom_post_process loc_to_truth_comparer
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/imu_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/error_to_csv.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
