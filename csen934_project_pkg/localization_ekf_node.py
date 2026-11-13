"""
Extended Kalman Filter (EKF) Localization.

Fuses the noisy odometry with the Ackermann control input to estimate the
vehicle pose and speed.

    Node:       Localization_EKF_Node
    Subscribes: /Car_Odom_Noisy     (nav_msgs/msg/Odometry)          measurement
                /Car_Ackrm          (ackermann_msgs/msg/AckermannDrive) control input
    Publishes:  /Car_Odom_Filtered  (nav_msgs/msg/Odometry)

State:   s = [x, y, yaw, v]   (v = signed forward speed of the rear axle)
Control: u = [v_cmd, delta]   (commanded speed, front steering angle)

Motion model (kinematic bicycle, reference point = rear-axle center, which is
where the Webots Car origin and the GPS are):
    x'   = x + v cos(yaw) dt
    y'   = y + v sin(yaw) dt
    yaw' = yaw + v tan(delta) / L dt
    v'   = v + (v_cmd - v) / tau dt     (first-order lag toward the command)

Measurement model (from /Car_Odom_Noisy):
    z = [x, y, yaw, v, omega],  omega = v tan(delta) / L
R is read from the measurement's covariance fields.
"""
import math

from ackermann_msgs.msg import AckermannDrive
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
IDX_XY = 0 * 6 + 1
IDX_YX = 1 * 6 + 0
IDX_XYAW = 0 * 6 + 5
IDX_YAWX = 5 * 6 + 0
IDX_YYAW = 1 * 6 + 5
IDX_YAWY = 5 * 6 + 1

# Used when a measurement reports zero variance (e.g. fed straight from GT),
# so R stays invertible.
MIN_VARIANCE = 1e-6

# Gaps longer than this (s) mean the stream stopped; re-initialise from the
# next measurement instead of predicting across the gap.
MAX_DT = 1.0


class LocalizationEKF(Node):

    def __init__(self):
        super().__init__('Localization_EKF_Node')

        # Vehicle: TeslaModel3 PROTO (Webots R2025a) has wheelbase 2.94 m.
        self.wheelbase = self.declare_parameter('wheelbase', 2.94).value
        # Multiplies the incoming steering angle so that + = left in the model.
        # /Car_Ackrm currently uses the Webots convention (+ = RIGHT turn, verified
        # in city.wbt), the opposite of AckermannDrive, hence -1.0. Set to 1.0 if
        # the controller and control.py are changed to the AckermannDrive convention.
        self.steering_sign = self.declare_parameter('steering_sign', -1.0).value
        # Time constant (s) of the speed response to a new speed command.
        self.speed_tau = self.declare_parameter('speed_time_constant', 1.0).value

        # Process noise (continuous-time std devs; scaled by dt).
        self.q_pos = self.declare_parameter('process_std_position', 0.1).value
        self.q_yaw = math.radians(
            self.declare_parameter('process_std_yaw_deg', 1.0).value)
        self.q_v = self.declare_parameter('process_std_speed', 1.0).value

        self.s = None          # state [x, y, yaw, v]
        self.P = None          # 4x4 covariance
        self.last_stamp = None
        self.v_cmd = None      # None until the first /Car_Ackrm arrives
        self.delta = 0.0

        self.create_subscription(
            Odometry, '/Car_Odom_Noisy', self.measurement_callback, 10)
        self.create_subscription(
            AckermannDrive, '/Car_Ackrm', self.control_callback, 10)
        self.pub = self.create_publisher(Odometry, '/Car_Odom_Filtered', 10)

        self.get_logger().info(
            'EKF ready: wheelbase=%.2f m, steering_sign=%+.0f, tau=%.2f s' % (
                self.wheelbase, self.steering_sign, self.speed_tau))

    # ------------------------------------------------------------------ input
    def control_callback(self, msg: AckermannDrive):
        self.v_cmd = float(msg.speed)
        self.delta = self.steering_sign * float(msg.steering_angle)

    def measurement_callback(self, msg: Odometry):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        z, R = self.measurement_from_msg(msg)

        if self.s is None or self.last_stamp is None:
            self.initialize(z, R)
        else:
            dt = stamp - self.last_stamp
            if dt <= 0.0:
                # Out-of-order or duplicate stamp: correct without predicting.
                pass
            elif dt > MAX_DT:
                self.get_logger().warn(
                    'Measurement gap of %.2f s, re-initialising EKF' % dt)
                self.initialize(z, R)
                self.last_stamp = stamp
                self.publish(msg)
                return
            else:
                self.predict(dt)
            self.update(z, R)

        self.last_stamp = stamp
        self.publish(msg)

    # ------------------------------------------------------------- EKF steps
    def initialize(self, z, R):
        self.s = z[:4].copy()
        self.P = np.diag(np.diag(R)[:4])

    def predict(self, dt):
        x, y, yaw, v = self.s
        L = self.wheelbase
        tan_d = math.tan(self.delta)
        c, s = math.cos(yaw), math.sin(yaw)

        # Without a speed command, model speed as constant (random walk).
        if self.v_cmd is None:
            dv, dv_dv = 0.0, 0.0
        else:
            dv = (self.v_cmd - v) / self.speed_tau
            dv_dv = -1.0 / self.speed_tau

        self.s = np.array([
            x + v * c * dt,
            y + v * s * dt,
            normalize_angle(yaw + v * tan_d / L * dt),
            v + dv * dt,
        ])

        F = np.array([
            [1.0, 0.0, -v * s * dt, c * dt],
            [0.0, 1.0, v * c * dt, s * dt],
            [0.0, 0.0, 1.0, tan_d / L * dt],
            [0.0, 0.0, 0.0, 1.0 + dv_dv * dt],
        ])
        Q = np.diag([
            self.q_pos ** 2, self.q_pos ** 2, self.q_yaw ** 2, self.q_v ** 2,
        ]) * dt

        self.P = F @ self.P @ F.T + Q

    def update(self, z, R):
        x, y, yaw, v = self.s
        k_w = math.tan(self.delta) / self.wheelbase   # omega = v * k_w

        h = np.array([x, y, yaw, v, v * k_w])
        H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, k_w],
        ])

        innovation = z - h
        innovation[2] = normalize_angle(innovation[2])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.s = self.s + K @ innovation
        self.s[2] = normalize_angle(self.s[2])

        # Joseph form keeps P symmetric and positive definite.
        I_KH = np.eye(4) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    # --------------------------------------------------------------- helpers
    @staticmethod
    def measurement_from_msg(msg: Odometry):
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        z = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        ])
        pc, tc = msg.pose.covariance, msg.twist.covariance
        R = np.diag([
            max(pc[IDX_X], MIN_VARIANCE),
            max(pc[IDX_Y], MIN_VARIANCE),
            max(pc[IDX_YAW], MIN_VARIANCE),
            max(tc[IDX_X], MIN_VARIANCE),
            max(tc[IDX_YAW], MIN_VARIANCE),
        ])
        return z, R

    def publish(self, meas: Odometry):
        x, y, yaw, v = self.s
        out = Odometry()
        out.header = meas.header
        out.child_frame_id = meas.child_frame_id

        out.pose.pose.position.x = float(x)
        out.pose.pose.position.y = float(y)
        out.pose.pose.position.z = meas.pose.pose.position.z
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, float(yaw))
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        out.twist.twist.linear.x = float(v)
        out.twist.twist.angular.z = float(v * math.tan(self.delta) / self.wheelbase)

        P = self.P
        pose_cov = [0.0] * 36
        pose_cov[IDX_X] = P[0, 0]
        pose_cov[IDX_Y] = P[1, 1]
        pose_cov[IDX_YAW] = P[2, 2]
        pose_cov[IDX_XY] = pose_cov[IDX_YX] = P[0, 1]
        pose_cov[IDX_XYAW] = pose_cov[IDX_YAWX] = P[0, 2]
        pose_cov[IDX_YYAW] = pose_cov[IDX_YAWY] = P[1, 2]
        out.pose.covariance = [float(c) for c in pose_cov]

        twist_cov = [0.0] * 36
        twist_cov[IDX_X] = P[3, 3]
        k_w = math.tan(self.delta) / self.wheelbase
        twist_cov[IDX_YAW] = k_w ** 2 * P[3, 3]
        out.twist.covariance = [float(c) for c in twist_cov]

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationEKF()
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
