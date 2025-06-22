import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from baseColors import ORANGE
import serial
import time
import subprocess

try:
    ser = serial.Serial('/dev/arduino', 115200)
except:
    print('Can not connect with Arduino')

class WheelChair:
    def __init__(self, master):
        self.master = master
        self.images = {} 
        self.display_wheelchair()
        self.custom_frame = ttk.Frame(self.master)
        self.custom_frame.place(x=0, y=20)
        self.custom_button = tk.Button(self.custom_frame, text="Calib", height=3, command=lambda: self.custom_button_click())
        self.custom_button.grid(row=0, column=0, padx=2, pady=2)
        
    def display_wheelchair(self):
        image_paths = {
            "up": "Images/up_arrow.png",
            "down": "Images/down_arrow.png",
            "left": "Images/left_arrow.png",
            "right": "Images/right_arrow.png",
            "up_left": "Images/up_left_arrow.png",
            "up_right": "Images/up_right_arrow.png",
            "down_left": "Images/down_left_arrow.png",
            "down_right": "Images/down_right_arrow.png",
            "center": "Images/eye.png"
        }
        
        for direction, path in image_paths.items():
            image = Image.open(path)
            image = image.resize((270, 270), resample=Image.BICUBIC)
            self.images[direction] = ImageTk.PhotoImage(image)
            
        frame = tk.Frame(self.master, background='white')
        frame.pack(expand=True)
        
        positions = [
            ["up_left", "up", "up_right"],
            ["left", "center", "right"],
            ["down_left", "down", "down_right"]
        ]
        
        for row_index, row in enumerate(positions):
            for col_index, direction in enumerate(row):
                button = tk.Button(
                    frame, image=self.images[direction], background=ORANGE,
                    command=lambda d=direction: self.on_button_click(d)
                )
                button.image = self.images[direction]
                button.grid(row=row_index, column=col_index, padx=150, pady=20)

    def on_button_click(self, direction):
        # direction là 1 trong các giá trị của list: ['up', 'down', 'left', 'right', 'up_left', 'up_right', 'down_left', 'down_right', 'center']
        print(direction)
        if direction == 'up':
            try:
                ser.write(b'm0.7 0.7\n')
                time.sleep(0.1) 
            except:
                print('Not connected with Arduino')
        elif direction == 'down':
            try:
                ser.write(b'm-0.7 -0.7\n') 
                time.sleep(0.1)
            except:
                print('Not connected with Arduino')
        elif direction == "center":
            try:
                ser.write(b'm0 0\n') 
                time.sleep(0.1)
            except:
                print('Not connected with Arduino')
        elif direction == "left":
            try:
                ser.write(b'm-0.7 0.7\n') 
                time.sleep(0.1)
            except:
                print('Not connected with Arduino')
        elif direction == "right":
            try:
                ser.write(b'm0.7 -0.7\n') 
                time.sleep(0.1)
            except:
                print('Not connected with Arduino')
        elif direction == "up_left":
            try:
                ser.write(b'm0.5 0.7\n') 
                time.sleep(0.1)
            except:
                print('Not connected with Arduino')
        elif direction == "up_right":
            try:
                ser.write(b'm0.7 0.5\n') 
                time.sleep(0.1)
            except:
                print('Not connected with Arduino')
        elif direction == "down_left":
            try:
                ser.write(b'm-0.5 -0.7\n') 
                time.sleep(0.1)
            except:
                print('Not connected with Arduino')
        elif direction == "down_right":
            try:
                ser.write(b'm-0.7 -0.5\n') 
                time.sleep(0.1)
            except:
                print('Not connected with Arduino')

    def custom_button_click(self):
        print("Đã bấm vào nút")
        
        # Gaze Tracking
        try:
            subprocess.Popen(["python3", "/home/vip/gaze/calib.py"])  # Replace 'another_file.py' with your file name
        except Exception as e:
            print(f"Error opening another file: {e}")
                