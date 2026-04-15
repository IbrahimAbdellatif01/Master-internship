#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry as NavMsg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np


class IMUTimestampValidator(Node):
    def __init__(self):
        super().__init__("imu_timestamp_validator")

        # QoS settings for IMU (best effort for high frequency)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, qos_profile
        )
        # subscripe to odometry as well to keep node alive
        self.odom_subscription = self.create_subscription(
            NavMsg,
            "/diff_drive_base_controller/odom",
            self.wheel_odom_callback,
            qos_profile,
        )
        self.publisher = self.create_publisher(Imu, "/imu/filtered", qos_profile)

        # Variables for timestamp tracking
        self.prev_stamp_ns = None
        self.prev_sequence = None
        self.message_count = 0
        self.fixed_count = 0
        self.dropped_count = 0

        # Statistics timer
        self.stats_timer = self.create_timer(5.0, self.print_stats)

        self.get_logger().info("IMU Timestamp Validator started")

    def wheel_odom_callback(self, msg):

        current_stamp_ns = self.timestamp_to_nanoseconds(msg.header.stamp)
        print("Timestamp wheel :", current_stamp_ns)

    def imu_callback(self, msg):
        self.message_count += 1
        current_stamp_ns = self.timestamp_to_nanoseconds(msg.header.stamp)

        # Check if timestamp is valid
        if self.prev_stamp_ns is not None:
            if current_stamp_ns > self.prev_stamp_ns:
                print("Timestamp is vali", current_stamp_ns)
            if current_stamp_ns <= self.prev_stamp_ns:
                print("Timestamp is back", current_stamp_ns)
                self.dropped_count += 1
                return

                # # Option 1: Fix the timestamp (add 1ms)
                # fixed_ns = self.prev_stamp_ns + 1000000  # +1ms
                # self.get_logger().warn(
                #     f"Timestamp went backwards! Fixed {current_stamp_ns} -> {fixed_ns}"
                # )

                # # Convert back to builtin_interfaces/Time
                # msg.header.stamp.sec = fixed_ns // 1000000000
                # msg.header.stamp.nanosec = fixed_ns % 1000000000
                # self.fixed_count += 1

                # Option 2: Use current ROS2 time instead
                # msg.header.stamp = self.get_clock().now().to_msg()

        # Update previous timestamp
        self.prev_stamp_ns = self.timestamp_to_nanoseconds(msg.header.stamp)
        # print(self.prev_stamp_ns)
        # Publish validated message
        self.publisher.publish(msg)

    def timestamp_to_nanoseconds(self, stamp):
        """Convert ROS2 Time to nanoseconds"""
        return stamp.sec * 1000000000 + stamp.nanosec

    def print_stats(self):
        """Print periodic statistics"""
        self.get_logger().info(
            f"IMU Stats - Total: {self.message_count}, "
            f"Fixed: {self.fixed_count}, "
            f"Dropped: {self.dropped_count}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = IMUTimestampValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
