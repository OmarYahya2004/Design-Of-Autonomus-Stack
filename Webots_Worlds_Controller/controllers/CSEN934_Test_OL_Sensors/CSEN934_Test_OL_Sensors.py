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

# 6. The Main Webots Execution Loop
while driver.step() != -1:
    rclpy.spin_once(node, timeout_sec=0)
    
    if gps and imu and gyro:
        odom_msg = Odometry()
        odom_msg.header.stamp = node.get_clock().now().to_msg()
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
        linear_vel = gps.getSpeedVector()
        odom_msg.twist.twist.linear.x = linear_vel[0]
        odom_msg.twist.twist.linear.y = linear_vel[1]
        odom_msg.twist.twist.linear.z = linear_vel[2]
        
        # Angular Velocity
        angular_vel = gyro.getValues()
        odom_msg.twist.twist.angular.x = angular_vel[0]
        odom_msg.twist.twist.angular.y = angular_vel[1]
        odom_msg.twist.twist.angular.z = angular_vel[2]
        
        odom_pub.publish(odom_msg)
