import onnxruntime as ort
import numpy as np
import cv2
import argparse
import random

import time
import requests
import random
from PIL import Image
from pathlib import Path
from collections import OrderedDict, namedtuple
import pyautogui
import tkinter as tk
import os
import subprocess
from pynput import keyboard

# Calibration file
calibration_file = "/home/tom/yolov10/calibration.txt"
with open(calibration_file, "w") as file:
    pass  # Clear file content

print(f"The content of {calibration_file} has been cleared.")

# Set up the main application window
root = tk.Tk()
root.title("Box Animation")

# Get the screen width and height
screen_width = root.winfo_screenwidth()
screen_height = root.winfo_screenheight()

# Create a canvas to fill the screen
canvas = tk.Canvas(root, width=screen_width, height=screen_height, bg="black")
canvas.pack()

# Box properties
box_size = 120
box_color = "#fd6100"
horizontal_speed = 15  # Smooth speed for left/right movement
downward_step = 100  # Larger downward step size for moving down smoothly
pause_time = 1.0  # Pause time at corners

# Initial position: Center of the screen
x_start = (screen_width - box_size) // 2
y_start = (screen_height - box_size) // 2
x_end, y_end = x_start + box_size, y_start + box_size
box = canvas.create_rectangle(x_start, y_start, x_end, y_end, fill=box_color)

cx, cy = 0, 0

def move_smoothly_to_corner():
    global cx, cy

    """ Moves the box smoothly from center to the top-left corner. """
    current_x, current_y = x_start, y_start

    while current_x > 0 or current_y > 0:
        step_x = min(horizontal_speed, current_x)
        step_y = min(horizontal_speed, current_y)

        canvas.move(box, -step_x, -step_y)
        root.update()
        time.sleep(0.02)

        # Get current position
        x1, y1, x2, y2 = canvas.coords(box)

        # Convert canvas coordinates to screen coordinates
        canvas_offset_x = canvas.winfo_rootx()
        canvas_offset_y = canvas.winfo_rooty()
        box_center = (
            (x1 + x2) // 2 + canvas_offset_x,
            (y1 + y2) // 2 + canvas_offset_y
        )

        formatted_data = f"Box: ({box_center[0]}, {box_center[1]}), Pupil: ({cx}, {cy})\n"
        print(formatted_data)
        with open(calibration_file, "a") as file:
            file.write(formatted_data)

        current_x -= step_x
        current_y -= step_y

def move_box():
    global want_exit

    root.update()
    time.sleep(pause_time)  # Pause at center

    # Move smoothly to the top-left corner
    move_smoothly_to_corner()
    time.sleep(pause_time)  # Pause at top-left corner

    x_direction = horizontal_speed
    y_direction = 0  # Start moving horizontally

    listener = keyboard.Listener(on_press=on_press)
    listener.start()  # Start the listener in a non-blocking way

    while True:
        canvas.move(box, x_direction, y_direction)
        root.update()
        time.sleep(0.02)

        # Get current position
        x1, y1, x2, y2 = canvas.coords(box)

        # Convert canvas coordinates to screen coordinates
        canvas_offset_x = canvas.winfo_rootx()
        canvas_offset_y = canvas.winfo_rooty()
        box_center = (
            (x1 + x2) // 2 + canvas_offset_x,
            (y1 + y2) // 2 + canvas_offset_y
        )

        formatted_data = f"Box: ({box_center[0]}, {box_center[1]}), Pupil: ({cx}, {cy})\n"
        print(formatted_data)
        with open(calibration_file, "a") as file:
            file.write(formatted_data)

        # Detect when reaching a corner and pause
        if (x1 <= 0 and y1 <= 0) or (x2 >= screen_width and y1 <= 0) or (x2 >= screen_width and y2 >= screen_height - downward_step) or (x1 <= 15 and y2 >= screen_height - downward_step):
            time.sleep(pause_time)

        # Exit
        if x2 >= screen_width and y2 >= screen_height - downward_step:

            turn_off_camera()

            try:
                subprocess.Popen(["python3", "/home/tom/yolov10/gaze.py"])  # Replace 'another_file.py' with your file name
            except Exception as e:
                print(f"Error opening another file: {e}")
                
            root.destroy()
            os._exit(0)
        
        if want_exit == True:
            turn_off_camera()
            root.destroy()
            os._exit(0)

        # Smooth downward movement with a larger step
        if (x2 >= screen_width and x_direction > 0) or (x1 <= 0 and x_direction < 0):
            x_direction = 0  # Stop horizontal movement
            for _ in range(downward_step // horizontal_speed):  # Move smoothly downward
                canvas.move(box, 0, horizontal_speed)
                root.update()
                time.sleep(0.02)
            x_direction = horizontal_speed if x1 <= 0 else -horizontal_speed  # Change horizontal direction

# Function to preprocess the image (resize and normalize)
def preprocess_image(image, input_size):    
    original_shape = image.shape[:2]  # Save original image size for later scaling   
    
    # Resize the image to the size expected by the model
    image_resized = cv2.resize(image, input_size)
    
    # Normalize (e.g., if the model requires input normalized to [0, 1])
    image_normalized = image_resized.astype(np.float32) / 255.0
    
    # Change the channel order from (B, G, R) to (R, G, B)
    image_normalized = image_normalized[:, :, ::-1]
    
    # Change image dimensions to (1, 3, height, width) as required by the ONNX model
    image_input = np.transpose(image_normalized, (2, 0, 1))
    image_input = np.expand_dims(image_input, axis=0)
    
    return image_input, original_shape

# Function to perform post-processing on the detection results
def postprocess_output(output, original_shape, input_size, conf_threshold=0.5):
    boxes, scores, class_ids = [], [], []
    
    for detection in output:        
        confidence = detection[4]  # Confidence score
        if confidence > conf_threshold:  # Filter out detections below threshold
            x1, y1, x2, y2 = detection[0:4]  # Bounding box coordinates
            class_id = int(detection[5])  # Class ID

            # Scale bounding box coordinates back to the original size
            x1 = x1 / input_size[0]
            y1 = y1 / input_size[1]
            x2 = x2 / input_size[0]
            y2 = y2 / input_size[1]

            x1 = int(x1 * original_shape[1])
            y1 = int(y1 * original_shape[0])
            x2 = int(x2 * original_shape[1])
            y2 = int(y2 * original_shape[0])
            
            # Append the bounding box, confidence score, and class ID to respective lists
            boxes.append([x1, y1, x2, y2])
            scores.append(float(confidence))
            class_ids.append(class_id)                         
    
    return boxes, scores, class_ids

# Initialize webcam (0 is usually the default webcam)
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FPS, 30)
# Ensure webcam is opened successfully
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

zoom_factor = 1.5

def process_video():
    global cx, cy, cap

    # Add argument parser to get command-line arguments
    parser = argparse.ArgumentParser()
    # Changed defaults to use "best.onnx" as the model and "0" as the source (webcam)
    parser.add_argument("--names", type=str, default="class.names", help="Object Names")
    parser.add_argument("--model", type=str, default="best.onnx", help="Pretrained Model")
    parser.add_argument("--tresh", type=float, default=0.35, help="Confidence Threshold")    
    args = parser.parse_args()

    # Load the ONNX model
    model_path = args.model
    providers = ['CUDAExecutionProvider']
    session = ort.InferenceSession(model_path, providers=providers)

    # Get the input and output layers from the model
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name

    # Define the input size for the model
    input_size = (640, 640)  

    # Define confidence threshold for detection
    conf_thres = args.tresh

    # Load class names from the .names file and assign random colors to each class
    class_names = []
    with open(args.names, "r") as f:
        class_names = [cname.strip() for cname in f.readlines()]
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in class_names]


    while True:
        # Capture frame from webcam
        ret, img = cap.read()

        if not ret:
            print("Failed to capture image from webcam.")
            break
        
        # Get the center of the frame
        height, width, _ = img.shape
        
        # Shift the center towards the top-right
        shift_x = int(width * 0.1)  # 10% shift to the right
        shift_y = int(height * 0.1)  # 10% shift upwards

        # Calculate the new center for cropping
        center_x, center_y = width // 2 + shift_x, height // 2 - shift_y

        # Calculate the cropping box
        radius_x, radius_y = int(width // (2 * zoom_factor)), int(height // (2 * zoom_factor))

        # Crop the frame to zoom in on the top-right part
        cropped_frame = img[
            center_y - radius_y:center_y + radius_y,
            center_x - radius_x:center_x + radius_x
        ]

        # Resize cropped frame to original size
        img = cv2.resize(cropped_frame, (width, height))
        
        image = img.copy()  # Make a copy of the frame for processing

        # Preprocess the image for the model
        image_input, original_shape = preprocess_image(image, input_size)

        # Perform inference using the ONNX model
        output = session.run([output_name], {input_name: image_input})[0]

        # Post-process the output from the model
        boxes, scores, class_ids = postprocess_output(output[0], original_shape, input_size, conf_thres)

        # Draw the detection results (bounding boxes, class names, and centers)
        for box, score, class_id in zip(boxes, scores, class_ids):

            # Class name with confidence score
            class_name = class_names[0]
            score = round(float(score), 3)            
            class_name += f' {str(score)}'
            
            if score > 0.9:
                x1, y1, x2, y2 = box
                # Calculate the center of the bounding box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                print(f"Center of bounding box: ({center_x}, {center_y})")  # Print the center
                cx = center_x
                cy = center_y

                # Draw bounding box and class name on the image
                cv2.rectangle(image, (x1, y1), (x2, y2), colors[0], 2)
                cv2.putText(image, class_name, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[0], 2)

        cv2.imshow("Object Detection", image)  # Show the result
        if cv2.waitKey(1) == ord('q'):  # If 'q' is pressed, exit the loop
            break

    cv2.destroyAllWindows()  # Close all OpenCV windows


import threading
threading.Thread(target=process_video).start()

def turn_off_camera():
    global cap
    cap.release()
    cv2.destroyAllWindows()

want_exit = False
# Backspace for Exit
def on_press(key):
    global want_exit 
    try:
        if key == keyboard.Key.backspace:
            print('Backspace Detected')
            want_exit = True
    except AttributeError:
        pass

time.sleep(4)

# Start moving the box
move_box()

# Run the application
root.mainloop()