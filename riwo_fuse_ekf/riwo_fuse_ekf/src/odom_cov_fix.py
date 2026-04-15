"""
Odometry Covariance Fixer Node
================================
Republishes /wheel_odom as /wheel_odom_fixed with a minimum covariance
floor on vy (index [1,1] = index 7 in the 6x6 row-major twist covariance).

WHY: The tricycle/diff_drive controller publishes vy covariance as 0.0
because the robot can't move sideways. This is physically correct, but
fuse requires a positive-definite covariance matrix. Setting a small
floor value (default 0.001) makes the matrix valid for the solver.

USAGE:
  Add this node to your launch file BEFORE the fuse nodes.
  Point fuse's odometry_sensor topic to /wheel_odom_fixed.

PARAMETERS:
  min_covariance (double, default=0.001): Floor value for zero-variance
    diagonal entries in the twist covariance matrix.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomCovFixer(Node):
    def __init__(self):
        super().__init__('odom_cov_fixer')

        # Configurable floor value
        self.declare_parameter('min_covariance', 0.001)
        self.min_cov = self.get_parameter('min_covariance').value

        self.pub = self.create_publisher(Odometry, '/wheel_odom_fixed', 10)
        self.sub = self.create_subscription(
            Odometry, '/wheel_odom', self.callback, 10
        )

        self.get_logger().info(
            f'Republishing /wheel_odom -> /wheel_odom_fixed '
            f'(min twist covariance diagonal: {self.min_cov})'
        )

    def callback(self, msg: Odometry):
        cov = list(msg.twist.covariance)

        # Twist covariance is 6x6 row-major:
        #   [0]  vx-vx   [1]  vx-vy   [2]  vx-vz   [3]  vx-wx   [4]  vx-wy   [5]  vx-wz
        #   [6]  vy-vx   [7]  vy-vy   [8]  vy-vz   [9]  vy-wx   [10] vy-wy   [11] vy-wz
        #   [12] vz-vx   [13] vz-vy   [14] vz-vz   ...
        #   ...
        # Diagonal indices: vx=0, vy=7, vz=14, wx=21, wy=28, wz=35

        diagonal_indices = [0, 7, 14, 21, 28, 35]
        for idx in diagonal_indices:
            if cov[idx] < self.min_cov:
                cov[idx] = self.min_cov

        msg.twist.covariance = cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCovFixer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()