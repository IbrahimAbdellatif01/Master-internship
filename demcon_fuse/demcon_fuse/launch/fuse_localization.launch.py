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
    pkg_dir = FindPackageShare('demcon_fuse')
    fuse_params_file = PathJoinSubstitution(
        [FindPackageShare("demcon_fuse"), "config", "fgo_wheel_imu_gps.yaml"]
    )
    bag1_path_arg = DeclareLaunchArgument(
        "bag1_path",
        default_value=os.path.expanduser(
            "~/Downloads/demcon_closed_loop"
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
            #run tf static b.n imu and base link check if /tf is available in bag
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
            Node(
                package="demcon_fuse",
                executable="imu_post_process",
                name="imu_post_process",
                output="screen",
                parameters=[{"use_sim_time":True}],
            ),
                        Node(
                package="demcon_fuse",
                executable="pose_log",
                name="pose_log",
                output="screen",
                parameters=[{"use_sim_time":True}],
            ),
            # EKF Localization Node for Odometry Fusion
            Node(
                package="fuse_optimizers",
                executable="fixed_lag_smoother_node",  # Note: executable name may vary
                name="fuse_optimizer_tier1",
                output="screen",
                parameters=[fuse_params_file],
                # Optional: Remap topics as needed
                remappings=[
                    ("odometry/filtered", "odometry/fuse_tier1"),
                #     ("/tf", "tf"),
                #     ("/tf_static", "tf_static"),
                ],
            ), 
            Node(
                package="fuse_optimizers",
                executable="fixed_lag_smoother_node",  # Note: executable name may vary
                name="fuse_optimizer_tier2",
                output="screen",
                parameters=[fuse_params_file],
                # Optional: Remap topics as needed
                remappings=[
                     ("odometry/filtered", "odometry/fuse_tier2"),
                #     ("/tf", "tf"),
                #     ("/tf_static", "tf_static"),
                ],
            ), 
            # Node(
            #     package='rviz2',
            #     executable='rviz2',
            #     name='rviz',
            #     arguments=[
            #         '-d', [PathJoinSubstitution([pkg_dir, 'config', 'fuse_simple_tutorial.rviz'])]
            #     ]
            # ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[{
                    'use_sim_time': True,
                    'frequency': 0.05,
                    'delay': 3.0,
                    'magnetic_declination_radians': 0.0,     #0.0558505,
                    'yaw_offset': 0.0,
                    'zero_altitude': True, # it should be true for 2D mode
                    'publish_filtered_gps': True,
                    'wait_for_datum': False,
                    #'broadcast_cartesian_transform': True,
                    'use_odometry_yaw': True,

                }],
                remappings=[
                    ("/imu/data", "/imu/data"),
                    ("/gps/fix", "/fix"),
                    ("/gps/filtered", "/gps/filtered"),
                    ("/odometry/gps","/odometry/gps"),
                    # Use the EKF output topic you actually have:
                    ("/odometry/filtered", "/odometry/fuse_tier2"),
                    # If your EKF publishes a different topic, remap to that instead:
                    # ("/odometry/filtered", "/r2d2/odometry/filtered_tier1_ekf"),
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
            # ExecuteProcess(
            #     cmd=['~/ros2_ws/src/fgo/fgo/src/imu_post_process.cpp'],
            #     output='screen'
            #     name='imu_post_process'
            # ),
        ]
    )
