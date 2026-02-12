from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os


def generate_launch_description():
    # Path to your custom EKF parameters
    rl_params_file = PathJoinSubstitution(
        [FindPackageShare("demcon"), "config", "ekf_wheel_imu_gps.yaml"]
    )
    bag1_path_arg = DeclareLaunchArgument(
        "bag1_path",
        default_value=os.path.expanduser(
            "~/Demcon_bag_file/rosbag2_open_loop"
        ),
        description="Path to first bag file/directory",
    )

    play_bags_arg = DeclareLaunchArgument(
        "play_bags", default_value="true", description="Whether to play bags or not"
    )

    # Get launch configuration values
    bag1_path = LaunchConfiguration("bag1_path")
    play_bags = LaunchConfiguration("play_bags")
    

    return LaunchDescription(
        [
            bag1_path_arg,
            play_bags_arg,
            # run tf static b.n imu and base link check if /tf is available in bag
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_pub",
                output="screen",
                arguments=[
                    "0.11",
                    "0.0",
                    "0.56",
                    "0",
                    "0",
                    "0",
                    "base_link",
                    "xsens_imu_link",
                ], # imu cofiguration has been edited based on xml file provided by demcon
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_gps",
                output="screen",
                arguments=["-0.31", "-0.25", "0.6", "0", "0", "0", "base_link", "gps"],# imu cofiguration has been edited based on xml file provided by demcon
            ),

            # EKF Localization Node for Odometry Fusion
            Node(
                package="robot_localization",
                executable="ekf_node",  # Note: executable name may vary
                name="ekf_filter_node_odom",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                # Optional: Remap topics as needed
                remappings=[
                     ("odometry/filtered", "odometry/local"),
                #     ("/tf", "tf"),
                #     ("/tf_static", "tf_static"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file,{"use_sim_time": True}],
                remappings=[
                    ("/imu", "/imu/data"),
                    ("/gps/fix", "/fix"),
                    ("/gps/filtered", "/gps/filtered"),
                    ("/odometry/gps","/odometry/gps"),
                    # Use the EKF output topic you actually have:
                    ("/odometry/filtered", "/odometry/global"),
                    # If your EKF publishes a different topic, remap to that instead:
                    # ("/odometry/filtered", "/r2d2/odometry/filtered_tier1_ekf"),
                    ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",  # Note: executable name may vary
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": True}],
                # Optional: Remap topics as needed
                remappings=[
                     ("odometry/filtered", "odometry/global"),
                #     ("/tf", "tf"),
                #     ("/tf_static", "tf_static"),
                ],
            ),

            # Play first bag
            ExecuteProcess(
                condition=IfCondition(play_bags),
                cmd=[
                    "ros2",
                    "bag",
                    "play",
                    bag1_path,
                    "--clock",
                    "--rate",
                    "1.0",
                    # "--start-offset",
                    # "30",  # Start 0.3 seconds later
                    # "--remap",
                    # Example: '/wheel/odom:=/bag2/wheel/odom',
                ],
                output="screen",
                name="bag1_player",
            ),
        ]
    )
