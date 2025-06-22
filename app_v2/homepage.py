import tkinter as tk
from tkinter import ttk, messagebox
from baseColors import ORANGE
from utils import create_rounded_rectangle
import subprocess
import os

def find_camera_process_linux():
    try:
        result = subprocess.run(["fuser", "/dev/video2"], capture_output=True, text=True)
        pids = result.stdout.strip().split()
        return [int(pid) for pid in pids if pid.isdigit()]
    except Exception as e:
        print(f"Error finding camera process: {e}")
        return []

def kill_process_linux(pid):
    try:
        os.system(f"kill -9 {pid}")
        print(f"Process {pid} terminated successfully.")
    except Exception as e:
        print(f"Error terminating process {pid}: {e}")

class Homepage:
    def __init__(self, master):
        self.master = master
        self.display_homepage()
        self.custom_frame = ttk.Frame(self.master)
        self.custom_frame.place(x=0, y=15)

        # Calib Button
        self.custom_button = tk.Button(
            self.custom_frame, text="Calib", font=('Segoe UI', 10, 'bold'),
            width=15, height=16, bg="#fd6100", fg="white", 
            activebackground="gray", activeforeground="white",
            command=lambda: self.calib_button_click()
        )
        self.custom_button.grid(row=0, column=0, padx=2, pady=2)

        # Mouse Button
        self.mouse_button = tk.Button(
            self.custom_frame, text="Mouse", font=('Segoe UI', 10, 'bold'),
            width=15, height=16, bg="#fd6100", fg="white", 
            activebackground="gray", activeforeground="white",
            command=lambda: self.mouse_button_click()
        )
        self.mouse_button.grid(row=1, column=0, padx=2, pady=2)

        # Camera Button
        self.camera_button = tk.Button(
            self.custom_frame, text="Camera", font=('Segoe UI', 10, 'bold'),
            width=15, height=16, bg="#fd6100", fg="white", 
            activebackground="gray", activeforeground="white",
            command=lambda: self.camera_button_click()
        )
        self.camera_button.grid(row=2, column=0, padx=2, pady=2)

        self.check_camera()

    def is_camera_available(self):
        """Check if the camera device exists in /dev/"""
        return os.path.exists("/dev/eyecamera")

    def check_camera(self):
        """Check if the camera is available using system commands."""
        is_available = self.is_camera_available()

        # Clear previous status message
        self.canvas.delete("camera_status")

        # Display message based on camera availability
        text = "Eyecamera is Detected" if is_available else "Eyecamera is not Detected"
        color = "green" if is_available else "red"

        self.canvas.create_text(
            self.master.winfo_width() / 5,
            self.master.winfo_height() / 2 + 320,
            text=text,
            font=("Arial", 20, "bold"),
            fill=color,
            tags="camera_status"  # Use a tag to update the message
        )

        # Check the camera status every 2 seconds (2000 ms)
        self.master.after(2000, self.check_camera)
        
    def display_homepage(self):
        # Configure grid layout
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(0, weight=1)
        
        # Hide grid lines by setting highlightthickness to 0
        self.master.grid_rowconfigure(0, weight=1, uniform="equal")
        self.master.grid_columnconfigure(0, weight=1, uniform="equal")
        
        # Create canvas for rectangle
        self.canvas = tk.Canvas(self.master, bg="white")
        self.canvas.grid(row=0, column=0, sticky="nsew")
        
        # Bind canvas resize event
        self.canvas.bind("<Configure>", self.resize_rectangle)
        
    def resize_rectangle(self, event):
        # Clear the canvas
        self.canvas.delete("all")

        # Get canvas dimensions
        canvas_width = event.width
        canvas_height = event.height

        # Calculate rectangle dimensions
        rect_width = canvas_width * 3 / 4
        rect_height = canvas_height * 2 / 3

        # Calculate rectangle position
        rect_x1 = (canvas_width - rect_width) / 2
        rect_y1 = (canvas_height - rect_height) / 2
        rect_x2 = rect_x1 + rect_width
        rect_y2 = rect_y1 + rect_height

        # Draw rectangle
        create_rounded_rectangle(self.canvas, rect_x1, rect_y1, rect_x2, rect_y2, 20, fill="#fd6100")

        # Add text inside the rectangle
        self.canvas.create_text(
            canvas_width / 2,  # X position (center)
            canvas_height / 2,  # Y position (center)
            text="ISEF 2025 - Autonomous Wheelchair for Mobility and Communication Assitance for ALS Patients",  # Text to display
            font=("Arial", 20, "bold"),  # Font settings
            fill="white"  # Text color
        )

        self.canvas.create_text(
            canvas_width / 2,  # X position (center)
            canvas_height / 2 + 50,  # Y position (center)
            text="Trung Quan Cao - Minh Hieu Le",  # Text to display
            font=("Arial", 20, "bold"),  # Font settings
            fill="white"  # Text color
        )
    
    def calib_button_click(self):
        global gaze_process
        print("Đã bấm vào nút Calib")
        
        # Calib + Gaze Tracking
        try:
            pids = find_camera_process_linux()
            for pid in pids:
                kill_process_linux(pid)

            subprocess.Popen(["python3", "/home/tom/yolov7_2/calib_2.py"])  # Replace 'another_file.py' with your file name
        except Exception as e:
            print(f"Error opening another file: {e}")

        # Start the countdown
        self.start_countdown(3, "Calib is going to run in ", "Calib is running...")

    def mouse_button_click(self):
        print("Đã bấm vào nút Mouse")
        global gaze_process
        
        gaze_process = subprocess.Popen(["python3", "/home/tom/yolov7_2/gaze_2.py"])
        print("Gaze tracking started.")

        # Start the countdown
        self.start_countdown(3, "Mouse is going to run in ", "Mouse is running...")

    def camera_button_click(self):
        print("Đã bấm vào nút Camera")
        
        # Only Camera
        try:
            subprocess.Popen(["python3", "/home/tom/yolov7_2/run_onnx.py"])  # Replace 'another_file.py' with your file name
        except Exception as e:
            print(f"Error opening another file: {e}")

        # Start the countdown
        self.start_countdown(2, "Camera is going to run in ", "Camera is running...")


    def start_countdown(self, count, countdown_message, running_message):
        """Displays a countdown message on the canvas before running the script."""
        if count >= 0:
            # Clear previous message
            self.canvas.delete("countdown_text")

            # Display countdown text
            self.canvas.create_text(
                self.master.winfo_width() / 2,
                self.master.winfo_height() / 7,
                text=f"{countdown_message}{count}...",
                font=("Arial", 20, "bold"),
                fill="blue",
                tags="countdown_text"
            )

            # Schedule the next update
            self.master.after(1000, self.start_countdown, count - 1, countdown_message, running_message)
        else:
            # Clear countdown text
            self.canvas.delete("countdown_text")

            # Display "Running..." message
            self.canvas.create_text(
                self.master.winfo_width() / 2,
                self.master.winfo_height() / 7,
                text=running_message,
                font=("Arial", 20, "bold"),
                fill="green",
                tags="countdown_text"
            )

            # Schedule the text removal after 1 second
            self.master.after(1000, self.canvas.delete, "countdown_text")

    def update_language(self):
        return