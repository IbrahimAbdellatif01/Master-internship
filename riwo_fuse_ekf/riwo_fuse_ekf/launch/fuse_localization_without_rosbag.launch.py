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
    fuse_params_file = PathJoinSubstitution(
        [FindPackageShare("riwo_fuse_ekf"), "config", "riwo_wheel_imu_gps.yaml"]
    )
    #navsat_params_file = PathJoinSubstitution([FindPackageShare("riwo"), "config", "navsat_transform.yaml"])
    
    return LaunchDescription(
        [   
            # run tf static b.n imu and base link check if /tf is available in bag
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="static_tf_pub_imu",
            #     output="screen",
            #     arguments=[
            #         "0.11",
            #         "0.0",
            #         "0.56",
            #         "0",
            #         "0",
            #         "0",
            #         "base_link",
            #         "xsens_imu_link",
            #     ], # imu cofiguration has been edited based on xml file provided by demcon
            # ),
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     name="static_tf_pub_gps",
            #     output="screen",
            #     arguments=["-0.31", "-0.25", "0.6", "0", "0", "0", "base_link", "gps_link"],# imu cofiguration has been edited based on xml file provided by demcon
            # ),

            # EKF Localization Node for Odometry Fusion
            Node(
                package="fuse_optimizers",
                executable="fixed_lag_smoother_node",  # Note: executable name may vary
                name="fuse_optimizer_tier1",
                output="screen",
                parameters=[fuse_params_file],
                # Optional: Remap topics as needed
                # remappings=[
                #     ("odometry/filtered", "odometry/filtered"),
                #     ("/tf", "tf"),
                #     ("/tf_static", "tf_static"),
                # ],
            ), 
             Node(
                package="fuse_optimizers",
                executable="fixed_lag_smoother_node",  # Note: executable name may vary
                name="fuse_optimizer_tier2",
                output="screen",
                parameters=[fuse_params_file],
                # Optional: Remap topics as needed
                # remappings=[
                #     ("odometry/filtered", "odometry/filtered"),
                #     ("/tf", "tf"),
                #     ("/tf_static", "tf_static"),
                # ],
            ), 
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[{
                    'use_sim_time': True,
                    'frequency': 30.0,
                    'delay': 3.0,
                    'magnetic_declination_radians': 0.0558505,
                    'yaw_offset': 0.0,
                    'zero_altitude': True,
                    'broadcast_utm_transform': False,
                    'publish_filtered_gps': True,
                }],
                remappings=[
                    ("/imu/data", "/imu0/imu"),
                    ("/gps/fix", "/navnat0/rfid_republished"),
                    ("/gps/filtered", "/gps/filtered"),
                    ("/odometry/gps","/odometry/gps"),
                    # Use the EKF output topic you actually have:
                    ("/odometry/filtered", "/odometry/fuse_tier2"),
                    # If your EKF publishes a different topic, remap to that instead:
                    # ("/odometry/filtered", "/r2d2/odometry/filtered_tier1_ekf"),
                ],
            ),
        ]
    )
