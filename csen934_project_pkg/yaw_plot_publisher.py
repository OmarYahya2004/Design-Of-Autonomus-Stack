"""
Yaw publisher for rqt_plot.

rqt_plot cannot plot yaw from nav_msgs/Odometry directly (orientation is a
quaternion), so this node republishes the yaw of each odometry topic as a
std_msgs/Float64 in degrees.

    Node:       Yaw_Plot_Node
    Subscribes: /Car_Odom_GT, /Car_Odom_Noisy, /Car_Odom_Filtered  (nav_msgs/msg/Odometry)
    Publishes:  /Car_Yaw_GT, /Car_Yaw_Noisy, /Car_Yaw_Filtered      (std_msgs/msg/Float64, deg)

The yaw is unwrapped (continuous, not limited to +-180 deg) so the curves do not
jump by 360 deg when the car faces ~180 deg. Noisy and Filtered are unwrapped
onto the branch closest to the ground truth, so all three lines share the same
offset and overlay in one plot.
"""
import math

from csen934_project_pkg.helping_functions import normalize_angle, quaternion_to_euler
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

SOURCES = ('GT', 'Noisy', 'Filtered')


class YawPlotPublisher(Node):

    def __init__(self):
        super().__init__('Yaw_Plot_Node')
        self.gt_unwrapped = None   # rad, continuous
        self.last_unwrapped = {}   # per source, rad
        self.pubs = {}
        for src in SOURCES:
            self.pubs[src] = self.create_publisher(Float64, '/Car_Yaw_' + src, 10)
            self.create_subscription(
                Odometry, '/Car_Odom_' + src,
                lambda msg, s=src: self.odom_callback(s, msg), 10)

    def odom_callback(self, src, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)[2]

        # Reference to unwrap around: GT's own previous value, or GT for the others.
        if src == 'GT':
            ref = self.gt_unwrapped
        else:
            ref = self.gt_unwrapped
            if ref is None:
                ref = self.last_unwrapped.get(src)
        unwrapped = yaw if ref is None else ref + normalize_angle(yaw - ref)

        if src == 'GT':
            self.gt_unwrapped = unwrapped
        self.last_unwrapped[src] = unwrapped
        self.pubs[src].publish(Float64(data=math.degrees(unwrapped)))


def main(args=None):
    rclpy.init(args=args)
    node = YawPlotPublisher()
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
