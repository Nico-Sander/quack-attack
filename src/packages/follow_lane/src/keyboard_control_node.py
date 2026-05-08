#!/usr/bin/env python3

import os
import math
import rospy
import tkinter as tk
from duckietown_msgs.msg import Twist2DStamped

class JoystickGUI:
    def __init__(self, master):
        self.master = master
        master.title("Duckiebot Virtual Joystick")

        # ROS Setup
        rospy.init_node('joystick_control_node', anonymous=True)
        vehicle_name = os.environ.get("VEHICLE_NAME", "default_robot")
        topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        self.pub = rospy.Publisher(topic, Twist2DStamped, queue_size=1)

        # Maximum Speeds
        self.max_v = 0.2      # Max linear velocity (m/s)
        self.max_omega = 4.0  # Max angular velocity (rad/s)

        # GUI Dimensions
        self.width = 300
        self.height = 300
        self.center_x = self.width // 2
        self.center_y = self.height // 2
        self.max_radius = 100

        # Create Canvas
        self.canvas = tk.Canvas(master, width=self.width, height=self.height, bg='#f0f0f0')
        self.canvas.pack(padx=20, pady=20)

        # Draw outer boundary (the joystick housing)
        self.canvas.create_oval(
            self.center_x - self.max_radius, self.center_y - self.max_radius,
            self.center_x + self.max_radius, self.center_y + self.max_radius,
            outline='gray', dash=(4, 4), width=2
        )

        # Draw the puck
        self.puck_radius = 20
        self.puck = self.canvas.create_oval(
            self.center_x - self.puck_radius, self.center_y - self.puck_radius,
            self.center_x + self.puck_radius, self.center_y + self.puck_radius,
            fill='#007acc', outline='#005999', width=2
        )

        # Bind mouse events to the canvas
        self.canvas.bind('<B1-Motion>', self.drag)
        self.canvas.bind('<ButtonRelease-1>', self.release)

        # Current velocities
        self.v = 0.0
        self.omega = 0.0
        
        # Start the continuous publishing loop
        self.publish_loop()

    def drag(self, event):
        """Triggers when the mouse is dragged."""
        # Calculate distance from center
        dx = event.x - self.center_x
        dy = event.y - self.center_y
        distance = math.hypot(dx, dy)

        # Clamp the puck to the maximum radius
        if distance > self.max_radius:
            dx = dx * self.max_radius / distance
            dy = dy * self.max_radius / distance

        # Move puck visually on the canvas
        x = self.center_x + dx
        y = self.center_y + dy
        self.canvas.coords(
            self.puck,
            x - self.puck_radius, y - self.puck_radius,
            x + self.puck_radius, y + self.puck_radius
        )

        # Map GUI coordinates to ROS velocities
        # Linear velocity (Y-axis): Up in GUI is negative 'dy', but forward is positive 'v'
        self.v = -(dy / self.max_radius) * self.max_v
        
        # Angular velocity (X-axis): Left in GUI is negative 'dx', but turning left is positive 'omega'
        self.omega = -(dx / self.max_radius) * self.max_omega
        
        self.publish_cmd()

    def release(self, event):
        """Triggers when the mouse button is released."""
        # Snap the puck back to the center (Deadman switch)
        self.canvas.coords(
            self.puck,
            self.center_x - self.puck_radius, self.center_y - self.puck_radius,
            self.center_x + self.puck_radius, self.center_y + self.puck_radius
        )
        # Immediately halt the bot
        self.v = 0.0
        self.omega = 0.0
        self.publish_cmd()

    def publish_cmd(self):
        """Constructs and sends the ROS message."""
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = self.v
        msg.omega = self.omega
        self.pub.publish(msg)

    def publish_loop(self):
        """A watchdog loop that publishes continuously at 10Hz."""
        if not rospy.is_shutdown():
            self.publish_cmd()
            # 100 ms = 10 Hz
            self.master.after(100, self.publish_loop)

if __name__ == '__main__':
    # Initialize the Tkinter root window
    root = tk.Tk()
    
    # Handle shutting down the ROS node if the user clicks the "X" on the window
    root.protocol("WM_DELETE_WINDOW", lambda: (rospy.signal_shutdown("Window closed"), root.destroy()))
    
    app = JoystickGUI(root)
    
    try:
        # Start the GUI event loop
        root.mainloop()
    except rospy.ROSInterruptException:
        pass