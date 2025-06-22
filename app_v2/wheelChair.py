import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import time
import tkinter as tk
from PIL import Image, ImageTk
from baseColors import ORANGE
from tkinter import ttk
import subprocess

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_twist_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 0.15
        self.angular_speed = 0.15
        self.current_twist = Twist()
        self.running = True
        self.get_logger().info("Teleop Twist Keyboard Node has been started.")

        # Start publishing thread
        self.publish_thread = threading.Thread(target=self.publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def move_forward_left(self):
        self.current_twist.linear.x = self.linear_speed
        self.current_twist.angular.z = self.angular_speed

    # def move_forward_left(self):
    #     # self.current_twist.linear.x = self.linear_speed
    #     # self.current_twist.angular.z = self.angular_speed
    #     self.current_twist.linear.x = 0.07
    #     self.current_twist.angular.z = 0.15

    
    def move_forward(self):
        self.current_twist.linear.x = self.linear_speed
        self.current_twist.angular.z = 0.0
    
    def move_forward_right(self):
        self.current_twist.linear.x = self.linear_speed
        self.current_twist.angular.z = -self.angular_speed
    
    def turn_left(self):
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = self.angular_speed
    
    def stop(self):
        self.current_twist = Twist()
    
    def turn_right(self):
        self.current_twist.linear.x = 0.0
        self.current_twist.angular.z = -self.angular_speed
    
    def move_backward_left(self):
        self.current_twist.linear.x = -self.linear_speed
        self.current_twist.angular.z = self.angular_speed
    
    def move_backward(self):
        self.current_twist.linear.x = -self.linear_speed
        self.current_twist.angular.z = 0.0
    
    def move_backward_right(self):
        self.current_twist.linear.x = -self.linear_speed
        self.current_twist.angular.z = -self.angular_speed
    
    def publish_loop(self):
        while self.running and rclpy.ok():
            self.publisher_.publish(self.current_twist)
            self.get_logger().info(f'Publishing: linear={self.current_twist.linear.x}, angular={self.current_twist.angular.z}')
            time.sleep(0.1)
    
    def teleop_loop(self, key):
        key_actions = {
            'u': self.move_forward_left,
            'i': self.move_forward,
            'o': self.move_forward_right,
            'j': self.turn_left,
            'k': self.stop,
            'l': self.turn_right,
            'm': self.move_backward_left,
            ',': self.move_backward,
            '.': self.move_backward_right,
        }
        if key in key_actions:
            key_actions[key]()

class WheelChair:
    def __init__(self, master, teleop_node):
        self.master = master
        self.teleop_node = teleop_node
        self.images = {} 
        self.display_wheelchair()

        self.custom_frame = ttk.Frame(self.master)
        self.custom_frame.place(x=0, y=200)
        

    # def on_enter(self, event, direction):
    #     # Set delay time based on direction
    #     delay_time = 500 if direction == "center" else 1500  # 0.5s for center, 1.5s for others

    #     # Schedule the button click after the delay
    #     if not hasattr(self, "cancel_event") or self.cancel_event is None:
    #         self.cancel_event = self.master.after(delay_time, lambda: self.on_button_click(direction))

    # def on_leave(self, event):
    #     # Cancel the scheduled event if the mouse leaves before 1s
    #     if hasattr(self, "cancel_event") and self.cancel_event is not None:
    #         self.master.after_cancel(self.cancel_event)
    #         self.cancel_event = None
        
    def display_wheelchair(self):
        image_paths = {
            "up": "/home/tom/app_v2/Images/up_arrow.png",
            "down": "/home/tom/app_v2/Images/down_arrow.png",
            "left": "/home/tom/app_v2/Images/left_arrow.png",
            "right": "/home/tom/app_v2/Images/right_arrow.png",
            "up_left": "/home/tom/app_v2/Images/up_left_arrow.png",
            "up_right": "/home/tom/app_v2/Images/up_right_arrow.png",
            "down_left": "/home/tom/app_v2/Images/down_left_arrow.png",
            "down_right": "/home/tom/app_v2/Images/down_right_arrow.png",
            "center": "/home/tom/app_v2/Images/eye.png"
        }
        
        for direction, path in image_paths.items():
            image = Image.open(path)
            image = image.resize((260, 260), resample=Image.BICUBIC)
            self.images[direction] = ImageTk.PhotoImage(image)
            
        frame = tk.Frame(self.master, background='white')
        frame.pack(expand=True)
        # frame.place(x = 0)
        
        positions = [
            ["up_left", "up", "up_right"],
            ["left", "center", "right"],
            ["down_left", "down", "down_right"]
        ]
        
        for row_index, row in enumerate(positions):
            for col_index, direction in enumerate(row):
                button = tk.Button(
                    frame, image=self.images[direction], background='#fd6100',
                    command=lambda d=direction: self.on_button_click(d)
                )
                button.image = self.images[direction]
                button.grid(row=row_index, column=col_index, padx=180, pady=20)
                
                # # Bind mouse enter and leave events
                # button.bind("<Enter>", lambda event, d=direction: self.on_enter(event, d))
                # button.bind("<Leave>", self.on_leave)

    def on_button_click(self, direction):
        mapping = {
            'up': 'i', 'down': ',', 'left': 'j', 'right': 'l',
            'up_left': 'u', 'up_right': 'o', 'down_left': 'm', 'down_right': '.',
            'center': 'k'
        }

        if direction in mapping:
            self.teleop_node.teleop_loop(mapping[direction])
            print(f"Button '{direction}' clicked!")      

        # if direction == "up":
        #     self.teleop_node.teleop_loop(mapping["up"])         #di toi
        #     time.sleep(10)
        #     self.teleop_node.teleop_loop(mapping["center"])     #dung
        #     time.sleep(1)
        #     self.teleop_node.teleop_loop(mapping["left"])       #re trai
        #     time.sleep(10)
        #     self.teleop_node.teleop_loop(mapping["center"])     #dung
        #     time.sleep(1)
