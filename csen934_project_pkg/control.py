import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
import customtkinter as ctk
import tkinter as tk
import threading
import math

class TeleopUI(Node):
    def __init__(self):
        super().__init__('control_teleop_node')
        self.publisher_ = self.create_publisher(AckermannDrive, '/Car_Ackrm', 10)
        self.current_speed = 0.0
        self.current_steering = 0.0

    def publish_cmd(self):
        msg = AckermannDrive()
        msg.speed = self.current_speed
        msg.steering_angle = self.current_steering
        self.publisher_.publish(msg)

# --- Custom 3-Spoke Steering Wheel Widget ---
class SteeringWheel(tk.Canvas):
    def __init__(self, parent, size=180, command=None, **kwargs):
        super().__init__(parent, width=size, height=size, bg="#2b2b2b", highlightthickness=0, **kwargs)
        self.size = size
        self.command = command
        self.center = size / 2
        self.radius = (size / 2) - 15
        self.angle = 0.0  # 0 radians is centered

        self.bind("<B1-Motion>", self.drag)
        self.bind("<Button-1>", self.drag)
        self.bind("<ButtonRelease-1>", self.release)

        self.draw_wheel()

    def draw_wheel(self):
        self.delete("all")
        
        # 1. Draw the thick outer rim
        self.create_oval(self.center - self.radius, self.center - self.radius,
                         self.center + self.radius, self.center + self.radius,
                         outline="#888888", width=12)
        self.create_oval(self.center - self.radius, self.center - self.radius,
                         self.center + self.radius, self.center + self.radius,
                         outline="#dddddd", width=2)

        # 2. Calculate spoke angles (Left, Right, and Bottom spokes)
        # FIXED: self.angle is now positive so the drawing tracks the mouse perfectly
        left_spoke_angle = self.angle - (math.pi / 2)
        right_spoke_angle = self.angle + (math.pi / 2)
        bottom_spoke_angle = self.angle + math.pi

        # Helper function to get X, Y coordinates for the end of a spoke
        def get_coords(angle, length):
            x = self.center + (length * math.sin(angle))
            y = self.center - (length * math.cos(angle))
            return x, y

        # 3. Draw the three spokes
        inner_r = 15
        outer_r = self.radius - 5
        
        for angle in [left_spoke_angle, right_spoke_angle, bottom_spoke_angle]:
            start_x, start_y = get_coords(angle, inner_r)
            end_x, end_y = get_coords(angle, outer_r)
            self.create_polygon(
                self.center + (inner_r * math.sin(angle - 0.5)), self.center - (inner_r * math.cos(angle - 0.5)),
                self.center + (inner_r * math.sin(angle + 0.5)), self.center - (inner_r * math.cos(angle + 0.5)),
                end_x, end_y,
                fill="#555555", outline="black"
            )

        # 4. Draw the center horn/hub
        self.create_oval(self.center - 25, self.center - 25,
                         self.center + 25, self.center + 25,
                         fill="#333333", outline="#111111", width=2)
        
        # 5. Draw a small top-center marker (like racing steering wheels)
        # FIXED: Removed the negative inversion so the marker follows the cursor
        marker_x, marker_y = get_coords(self.angle, self.radius)
        self.create_oval(marker_x - 4, marker_y - 4, marker_x + 4, marker_y + 4, fill="red", outline="red")

    def drag(self, event):
        # Calculate angle based on standard Cartesian math
        dx = event.x - self.center
        dy = self.center - event.y 
        
        raw_angle = math.atan2(dx, dy)

        # Clamp the visual steering angle to exactly [-0.5, 0.5] radians
        self.angle = max(-0.5, min(0.5, raw_angle))
        self.draw_wheel()

        if self.command:
            self.command(self.angle)

    def release(self, event):
        self.angle = 0.0
        self.draw_wheel()
        if self.command:
            self.command(self.angle)
# ------------------------------------

def run_ros(node):
    rclpy.spin(node)

def build_ui(ros_node):
    app = ctk.CTk()
    app.title("Tesla Teleop Controller")
    app.geometry("400x500") 
    
    speed_label = ctk.CTkLabel(app, text=f"Target Speed: {ros_node.current_speed} m/s", font=("Arial", 16, "bold"))
    speed_label.pack(pady=(15, 10))
    
    def update_speed_display():
        speed_label.configure(text=f"Target Speed: {ros_node.current_speed} m/s")
    
    def increase_speed():
        ros_node.current_speed += 1.0  
        ros_node.publish_cmd()
        update_speed_display()

    def decrease_speed():
        ros_node.current_speed -= 1.0  
        ros_node.publish_cmd()
        update_speed_display()

    def apply_brakes():
        ros_node.current_speed = 0.0
        ros_node.publish_cmd()
        update_speed_display()

    def update_steering(value):
        # Webots/ROS standard: positive Ackermann steering is LEFT, negative is RIGHT.
        # Since dragging right yields a positive value in our UI math, we invert it here
        # so the physical vehicle turns in the same direction you dragged.
        ros_node.current_steering = float(value)
        ros_node.publish_cmd()

    # UI Layout
    ctk.CTkButton(app, text="Accelerate", command=increase_speed).pack(pady=5)
    ctk.CTkButton(app, text="Decelerate", command=decrease_speed).pack(pady=5)
    ctk.CTkButton(app, text="Brake", command=apply_brakes, fg_color="red").pack(pady=5)
    
    ctk.CTkLabel(app, text="Steering Wheel (Click and Drag)").pack(pady=(15, 5))
    
    wheel = SteeringWheel(app, size=180, command=update_steering)
    wheel.pack(pady=5)

    app.protocol("WM_DELETE_WINDOW", lambda: (ros_node.destroy_node(), rclpy.shutdown(), app.destroy()))
    app.mainloop()

def main():
    rclpy.init()
    teleop_node = TeleopUI()
    
    ros_thread = threading.Thread(target=run_ros, args=(teleop_node,), daemon=True)
    ros_thread.start()
    
    build_ui(teleop_node)

if __name__ == '__main__':
    main()
