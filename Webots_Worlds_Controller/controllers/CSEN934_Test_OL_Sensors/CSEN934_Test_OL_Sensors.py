import rclpy
from ackermann_msgs.msg import AckermannDrive
from vehicle import Driver

# 1. Initialize Webots Driver directly
driver = Driver()
timestep = int(driver.getBasicTimeStep())

# 2. Initialize ROS 2 Node
rclpy.init(args=None)
node = rclpy.create_node('Tesla_Node') # Node named exactly as required

# 3. Enable Webots Sensors
camera = driver.getDevice('front_camera')
if camera:
    camera.enable(timestep)

# 4. Command Callback
def cmd_callback(msg):
    # Convert ROS m/s to Webots km/h
    target_speed_kmh = msg.speed * 3.6
    driver.setCruisingSpeed(target_speed_kmh)
    driver.setSteeringAngle(msg.steering_angle)

# 5. Subscribe to ROS topic
sub = node.create_subscription(
    AckermannDrive, 
    '/Car_Ackrm', 
    cmd_callback, 
    10
)

# 6. The Main Webots Execution Loop
while driver.step() != -1:
    # Process ROS callbacks to receive driving commands
    rclpy.spin_once(node, timeout_sec=0)
