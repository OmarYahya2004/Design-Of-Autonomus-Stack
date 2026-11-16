import math

from sensor_msgs.msg import PointCloud2, Image

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

import rclpy
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from vehicle import Driver

# Import from local helper files
from Helping_Functions import euler_to_quaternion
from Define_Sensors import init_sensors

# 1. Initialize Webots Driver directly
driver = Driver()
timestep = int(driver.getBasicTimeStep())

# 2. Initialize ROS 2 Node
rclpy.init(args=None)
node = rclpy.create_node('Tesla_Node')

# 3. Enable Webots Sensors using Define_Sensors.py
sensors = init_sensors(driver, timestep)
gps = sensors['gps']
imu = sensors['imu']
gyro = sensors['gyro']

# 4. Command Callback
def cmd_callback(msg):
    target_speed_kmh = msg.speed * 3.6
    driver.setCruisingSpeed(target_speed_kmh)
    driver.setSteeringAngle(msg.steering_angle)

# 5. Subscribers and Publishers
sub = node.create_subscription(
    AckermannDrive, 
    '/Car_Ackrm', 
    cmd_callback, 
    10
)

odom_pub = node.create_publisher(
    Odometry, 
    '/Car_Odom_GT', 
    10
)

#Initialize the camera publisher
camera = sensors['camera']
cam_pub = node.create_publisher(Image, '/front_camera/image_raw', 10)
last_cam_t = -1.0
CAM_PERIOD = 0.1  # 10 Hz

#publisher setup
pc_pub = node.create_publisher(PointCloud2, '/lidar_points', 10)
lidar = sensors['lidar']

#Initialize the Broadcaster
tf_broadcaster = TransformBroadcaster(node)
LIDAR_PERIOD = 0.1
last_lidar_t = -1.0

# SLAM DEBUG VARIABLES
last_lidar_debug_t = -1.0
lidar_publish_count = 0
last_tf_debug_t = -1.0


# 6. The Main Webots Execution Loop
while driver.step() != -1:
    rclpy.spin_once(node, timeout_sec=0)
    stamp = node.get_clock().now().to_msg()
    
    if gps and imu and gyro:
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        position = gps.getValues()
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        
        # Orientation using the imported helper function
        roll, pitch, yaw = imu.getRollPitchYaw()
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        
        # Linear Velocity
        vx_w, vy_w, vz_w = gps.getSpeedVector()
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        odom_msg.twist.twist.linear.x = cos_yaw * vx_w + sin_yaw * vy_w
        odom_msg.twist.twist.linear.y = -sin_yaw * vx_w + cos_yaw * vy_w
        odom_msg.twist.twist.linear.z = vz_w
        
        # Angular Velocity
        angular_vel = gyro.getValues()
        odom_msg.twist.twist.angular.x = angular_vel[0]
        odom_msg.twist.twist.angular.y = angular_vel[1]
        odom_msg.twist.twist.angular.z = angular_vel[2]
        
        # Broadcast the TF from odom to base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        # Publish ground-truth odometry
        odom_pub.publish(odom_msg)
        # Broadcast odom -> base_link
        tf_broadcaster.sendTransform(t)
        
        # SLAM DEBUG: Check robot odometry and TF
        if driver.getTime() - last_tf_debug_t >= 2.0:
            node.get_logger().info(
                f'[SLAM DEBUG] ODOM OK | '
                f'X: {position[0]:.2f} | '
                f'Y: {position[1]:.2f} | '
                f'Z: {position[2]:.2f} | '
                f'TF: odom -> base_link'
            )
            last_tf_debug_t = driver.getTime()
    
    if lidar and driver.getTime() - last_lidar_t >= LIDAR_PERIOD - 1e-6:
        last_lidar_t = driver.getTime()
        # Get LiDAR point cloud
        pc_data = lidar.getPointCloud()
        point_count = len(pc_data) if pc_data else 0

        if point_count > 0:
            points = [(p.x, p.y, p.z) for p in pc_data]

            header = Header()
            header.stamp = stamp
            header.frame_id = 'lidar'

            pc2_msg = pc2.create_cloud_xyz32(header, points)
            pc_pub.publish(pc2_msg)

            lidar_publish_count += 1

        # Print debugging information every 2 seconds
        if driver.getTime() - last_lidar_debug_t >= 2.0:
            if point_count > 0:
                node.get_logger().info(
                    f'[SLAM DEBUG] LiDAR OK | '
                    f'Points: {point_count} | '
                    f'Published messages: {lidar_publish_count}'
                )
            else:
                node.get_logger().warning(
                    '[SLAM DEBUG] LiDAR returned EMPTY point cloud!'
                )

            last_lidar_debug_t = driver.getTime()
            
    if camera and driver.getTime() - last_cam_t >= CAM_PERIOD - 1e-6:
        last_cam_t = driver.getTime()
        img_data = camera.getImage()
        
        if img_data:
            img_msg = Image()
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = 'base_link' 
            img_msg.height = camera.getHeight()
            img_msg.width = camera.getWidth()
            img_msg.encoding = 'bgra8'
            img_msg.step = camera.getWidth() * 4
            img_msg.data = img_data
            cam_pub.publish(img_msg)
