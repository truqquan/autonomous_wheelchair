import onnxruntime as ort
import numpy as np
import cv2
import argparse
import random

import time
import requests
from PIL import Image
from pathlib import Path
from collections import OrderedDict,namedtuple

import tkinter as tk
import os
from tkinter import filedialog
import matplotlib.pyplot as plt
import time
from sklearn.linear_model import LinearRegression
import pyautogui
from collections import deque

from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
import re

# Load calibration data from file
with open("/home/tom/yolov10/calibration.txt", "r", encoding="utf-8") as file:
    calibration_data = file.readlines()

# Extract coordinates
box_positions = []
pupil_positions = []

for line in calibration_data:
    try:
        # Ensure the line matches the expected pattern (handling both integers and floats)
        if not re.search(r"Box:\s*\(-?\d+(\.\d+)?,\s*-?\d+(\.\d+)?\),\s*Pupil:\s*\(-?\d+(\.\d+)?,\s*-?\d+(\.\d+)?\)", line):
            print(f"Skipping malformed line: {line.strip()}")
            continue

        # Extract numerical values (handling integers and floats)
        box_match = re.search(r"Box:\s*\((-?\d+(\.\d+)?),\s*(-?\d+(\.\d+)?)\)", line)
        pupil_match = re.search(r"Pupil:\s*\((-?\d+(\.\d+)?),\s*(-?\d+(\.\d+)?)\)", line)

        if not box_match or not pupil_match:
            print(f"Error parsing line: {line.strip()} -> not enough values to unpack")
            continue

        # Convert extracted values to floats (to handle cases with decimals)
        box_x, box_y = map(float, [box_match.group(1), box_match.group(3)])
        pupil_x, pupil_y = map(float, [pupil_match.group(1), pupil_match.group(3)])

        # print(f"Extracted -> Box: ({box_x}, {box_y}), Pupil: ({pupil_x}, {pupil_y})")

        # Store valid coordinates
        box_positions.append([box_x, box_y])
        pupil_positions.append([pupil_x, pupil_y])

    except (ValueError, IndexError) as e:
        print(f"Error parsing line: {line.strip()} -> {e}")

# Convert to NumPy arrays
if not box_positions or not pupil_positions:
    raise ValueError("No valid calibration data found. Check your calibration.txt file!")

box_positions = np.array(box_positions)
pupil_positions = np.array(pupil_positions)

# Fit a Polynomial Regression model (better accuracy)
poly_degree = 2  # Adjust for better fitting
model = make_pipeline(PolynomialFeatures(degree=poly_degree), LinearRegression())
model.fit(pupil_positions, box_positions)

poly = model.named_steps['polynomialfeatures']

# Lấy tên các đặc trưng với tên gốc là 'x' và 'y'
feature_names = poly.get_feature_names_out(['x', 'y'])

# In ra tên đặc trưng
print("Tên các đặc trưng trong mô hình Polynomial Regression:")
for i, name in enumerate(feature_names):
    print(f"{i}: {name}")

# Truy cập LinearRegression bên trong pipeline
reg = model.named_steps['linearregression']

# In hệ số cho từng đầu ra
print("\n▶️ Hệ số cho x_screen:")
for name, coef in zip(feature_names, reg.coef_[0]):
    print(f"{name}: {coef:.4f}")

print("\n▶️ Hệ số cho y_screen:")
for name, coef in zip(feature_names, reg.coef_[1]):
    print(f"{name}: {coef:.4f}")

# helper to format one equation
def format_equation(coefs, intercept, feature_names, target_name):
    terms = [f"{coefs[i]:+.4f}·{feature_names[i]}" for i in range(len(feature_names))]
    # intercept term (with sign)
    intercept_str = f"{intercept:+.4f}"
    equation = " ".join(terms) + " " + intercept_str
    return f"{target_name}(x, y) = {equation}"

# format for the two outputs
eq_x = format_equation(reg.coef_[0], reg.intercept_[0], feature_names, "x_screen")
eq_y = format_equation(reg.coef_[1], reg.intercept_[1], feature_names, "y_screen")

print("Polynomial Regression equations:\n")
print(eq_x)
print(eq_y)

print("\nIntercept (bias):", reg.intercept_)

# Get screen dimensions
screen_width, screen_height = pyautogui.size()

# Buffers for smoothing
buffer_size = 10  # Adjust for smoothness
pupil_buffer = deque(maxlen=buffer_size)
alpha = 0.2  # EMA smoothing factor (0.1 - 0.3 recommended)

# Variables for EMA smoothing
prev_pupil_x, prev_pupil_y = None, None

# Dead zone to reduce jitter
dead_zone = 20  # Pixels
last_x, last_y = None, None

def smooth_pupil_moving_avg(pupil_x, pupil_y):
    """Moving Average Smoothing"""
    pupil_buffer.append([pupil_x, pupil_y])
    avg_x = sum(p[0] for p in pupil_buffer) / len(pupil_buffer)
    avg_y = sum(p[1] for p in pupil_buffer) / len(pupil_buffer)
    return avg_x, avg_y

def smooth_pupil_ema(pupil_x, pupil_y):
    """Exponential Moving Average Smoothing"""
    global prev_pupil_x, prev_pupil_y
    if prev_pupil_x is None or prev_pupil_y is None:
        prev_pupil_x, prev_pupil_y = pupil_x, pupil_y
    smoothed_x = alpha * pupil_x + (1 - alpha) * prev_pupil_x
    smoothed_y = alpha * pupil_y + (1 - alpha) * prev_pupil_y
    prev_pupil_x, prev_pupil_y = smoothed_x, smoothed_y
    return smoothed_x, smoothed_y

def control_mouse_with_pupil(pupil_x, pupil_y):
    """Maps pupil coordinates to screen position and moves the mouse."""
    global last_x, last_y

    # Apply smoothing
    smoothed_x, smoothed_y = smooth_pupil_ema(pupil_x, pupil_y)

    # Predict mouse position
    predicted_position = model.predict([[smoothed_x, smoothed_y]])[0]

    # Clamp to screen boundaries
    x = max(0, min(predicted_position[0], screen_width - 1))
    y = max(0, min(predicted_position[1], screen_height - 1))

    # Apply dead zone filtering
    if last_x is not None and last_y is not None:
        if abs(x - last_x) < dead_zone and abs(y - last_y) < dead_zone:
            return  # Ignore small movements

    last_x, last_y = x, y

    # Move the mouse
    try:
        pyautogui.moveTo(x, y)
    except Exception as e:
        print(f"Mouse movement error: {e}")

        # Temporarily disable fail-safe
        pyautogui.FAILSAFE = False
        pyautogui.moveTo(screen_width / 2, screen_height /2)
        pyautogui.FAILSAFE = True  # Re-enable fail-safe

    # print(f"Pupil: ({pupil_x}, {pupil_y}) -> Mouse: ({x:.2f}, {y:.2f})")

# Define the wheelchair tab bounding box
wheelchair_x1, wheelchair_y1 = 480, 65  # Top-left corner
wheelchair_x2, wheelchair_y2 = 955, 160  # Bottom-right corner

# First region (Left side)
left_region_x1, left_region_y1 = 0, 65      # Top-left
left_region_x2, left_region_y2 = 475, 160   # Bottom-right

# Second region (Right side)
right_region_x1, right_region_y1 = 960, 65   # Top-left
right_region_x2, right_region_y2 = 1919, 160 # Bottom-right

# Track mode state
wheelchair_mode = False  # Default is normal mode


def is_in_wheelchair_tab(x, y):
    """Check if a given mouse position is within the wheelchair control area."""
    return wheelchair_x1 <= x <= wheelchair_x2 and wheelchair_y1 <= y <= wheelchair_y2

def is_in_disabled_region(x, y):
    """Check if a given mouse position is in the disabled regions where wheelchair mode should be OFF."""
    in_left_region = left_region_x1 <= x <= left_region_x2 and left_region_y1 <= y <= left_region_y2
    in_right_region = right_region_x1 <= x <= right_region_x2 and right_region_y1 <= y <= right_region_y2
    return in_left_region or in_right_region

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
cx = 0
cy = 0
prev_cx, prev_cy = None, None  # Store the previous coordinates
eye_closed_start = None  # Store the timestamp when the eye is detected as closed
dem = 0
alt_tab_check = False
click_check = False

start_time = time.time()  # Get the initial time

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
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            print(f"Center of bounding box: ({center_x}, {center_y})")  # Print the center
            cx = center_x
            cy = center_y
        
            # Draw bounding box and class name on the image
            cv2.rectangle(image, (x1, y1), (x2, y2), colors[0], 2)
            cv2.putText(image, class_name, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors[0], 2)
        
    if alt_tab_check == False:
        os.system("xdotool search --onlyvisible --name 'KHKT' windowactivate")  
        alt_tab_check = True

    if cx == prev_cx and cy == prev_cy:
        if eye_closed_start is None:  # First detection of no movement
            eye_closed_start = time.time()
        elif time.time() - eye_closed_start > 0.5 and click_check == False: 
            x_mouse, y_mouse = pyautogui.position()
            
            if is_in_wheelchair_tab(x_mouse, y_mouse):
                wheelchair_mode = True
                print("Switched to EYE-CLICK WHEELCHAIR MODE.")
            elif is_in_disabled_region(x_mouse, y_mouse):
                wheelchair_mode = False
                print("Switched to NORMAL MODE.")

            pyautogui.click()
            click_check = True

            print(cx, cy)
    else:
        if click_check == True:
            # If in wheelchair mode, move to center and click again after eye opens
            if wheelchair_mode:
                time.sleep(0.1)  # Small delay to simulate eye reopening
                pyautogui.moveTo(1920/2, 1080/2+50)
                pyautogui.click()

        eye_closed_start = None  # Reset if movement detected
        click_check = False

        
    prev_cx, prev_cy = cx, cy  # Update previous coordinates

    # Center of bounding box
    # print(cx,cy)    
    control_mouse_with_pupil(cx, cy)

    cv2.imshow("Object Detection", image)  # Show the result
    if cv2.waitKey(1) == ord('q'):  # If 'q' is pressed, exit the loop
        break

cv2.destroyAllWindows()  # Close all OpenCV windows

def turn_off_camera():
    global cap
    cap.release()
    cv2.destroyAllWindows()
