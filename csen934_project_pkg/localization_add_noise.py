"""
Localization with Noise.

Subscribes to the ground-truth odometry from Webots, adds zero-mean Gaussian
noise to the quantities listed in the milestone spec, fills the matching
covariance fields, and republishes it as a noisy measurement.

    Subscribes: /Car_Odom_GT     (nav_msgs/msg/Odometry)
    Publishes:  /Car_Odom_Noisy  (nav_msgs/msg/Odometry)

Default standard deviations (spec):
    x, y           0.3 m
    yaw            1 deg
    linear speed   0.3 m/s   (twist.linear.x, forward speed in base_link)
    angular speed  1 deg/s   (twist.angular.z, yaw rate)
"""
import copy
import math

from csen934_project_pkg.helping_functions import (
    euler_to_quaternion,
    normalize_angle,
    quaternion_to_euler,
)
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node

# Indices into the row-major 6x6 covariance arrays [x, y, z, rot_x, rot_y, rot_z].
IDX_X = 0 * 6 + 0
IDX_Y = 1 * 6 + 1
IDX_YAW = 5 * 6 + 5


class LocalizationAddNoise(Node):

    def __init__(self):
        super().__init__('Localization_Add_Noise_Node')

        # Std deviations are parameters so experiments can change them without
        # editing code: ros2 run ... --ros-args -p std_xy:=0.5
        self.std_xy = self.declare_parameter('std_xy', 0.3).value
        self.std_yaw = math.radians(self.declare_parameter('std_yaw_deg', 1.0).value)
        self.std_v = self.declare_parameter('std_linear_speed', 0.3).value
        self.std_w = math.radians(
            self.declare_parameter('std_angular_speed_deg', 1.0).value)
        seed = self.declare_parameter('seed', -1).value
        self.rng = np.random.default_rng(None if seed < 0 else seed)

        self.sub = self.create_subscription(
            Odometry, '/Car_Odom_GT', self.gt_callback, 10)
        self.pub = self.create_publisher(Odometry, '/Car_Odom_Noisy', 10)

        self.get_logger().info(
            'Adding noise: std_xy=%.3f m, std_yaw=%.2f deg, '
            'std_v=%.3f m/s, std_w=%.2f deg/s' % (
                self.std_xy, math.degrees(self.std_yaw),
                self.std_v, math.degrees(self.std_w)))

    def gt_callback(self, gt: Odometry):
        # Keep header (stamp + frames) from GT so the three topics line up in rqt_plot.
        noisy = copy.deepcopy(gt)
        n = self.rng.normal

        # --- Pose: x, y, yaw ---
        noisy.pose.pose.position.x += n(0.0, self.std_xy)
        noisy.pose.pose.position.y += n(0.0, self.std_xy)

        q = gt.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        yaw = normalize_angle(yaw + n(0.0, self.std_yaw))
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        noisy.pose.pose.orientation.x = qx
        noisy.pose.pose.orientation.y = qy
        noisy.pose.pose.orientation.z = qz
        noisy.pose.pose.orientation.w = qw

        # --- Twist: forward speed and yaw rate ---
        noisy.twist.twist.linear.x += n(0.0, self.std_v)
        noisy.twist.twist.angular.z += n(0.0, self.std_w)

        # --- Covariances (variance = std^2) for the noised quantities ---
        # Other entries stay at the GT value (zero): those fields were not perturbed.
        pose_cov = [0.0] * 36
        pose_cov[IDX_X] = self.std_xy ** 2
        pose_cov[IDX_Y] = self.std_xy ** 2
        pose_cov[IDX_YAW] = self.std_yaw ** 2
        noisy.pose.covariance = pose_cov

        twist_cov = [0.0] * 36
        twist_cov[IDX_X] = self.std_v ** 2
        twist_cov[IDX_YAW] = self.std_w ** 2
        noisy.twist.covariance = twist_cov

        self.pub.publish(noisy)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationAddNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
