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

from sklearn.metrics import r2_score
from sklearn.metrics import mean_squared_error, mean_absolute_error


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

# Create a grid of pupil positions (like webcam input space)
x_range = np.linspace(pupil_positions[:, 0].min(), pupil_positions[:, 0].max(), 100)
y_range = np.linspace(pupil_positions[:, 1].min(), pupil_positions[:, 1].max(), 100)
X_grid, Y_grid = np.meshgrid(x_range, y_range)

# Flatten grid and predict
grid_points = np.column_stack((X_grid.ravel(), Y_grid.ravel()))
predicted_screen = model.predict(grid_points)

# Reshape predictions
X_screen = predicted_screen[:, 0].reshape(X_grid.shape)
Y_screen = predicted_screen[:, 1].reshape(Y_grid.shape)

# 3D Surface Plot for x_screen
fig = plt.figure(figsize=(12, 5))
ax1 = fig.add_subplot(121, projection='3d')
ax1.plot_surface(X_grid, Y_grid, X_screen, cmap='viridis', edgecolor='k')
ax1.set_title("Predicted X Screen from Pupil Positions")
ax1.set_xlabel("x_pupil")
ax1.set_ylabel("y_pupil")
ax1.set_zlabel("x_screen")

# 3D Surface Plot for y_screen
ax2 = fig.add_subplot(122, projection='3d')
ax2.plot_surface(X_grid, Y_grid, Y_screen, cmap='plasma', edgecolor='k')
ax2.set_title("Predicted Y Screen from Pupil Positions")
ax2.set_xlabel("x_pupil")
ax2.set_ylabel("y_pupil")
ax2.set_zlabel("y_screen")

plt.tight_layout()
plt.show()

# Predict on training data
predicted = model.predict(pupil_positions)

# Separate predictions
x_screen_pred = predicted[:, 0]
y_screen_pred = predicted[:, 1]

# Actual values
x_screen_actual = box_positions[:, 0]
y_screen_actual = box_positions[:, 1]

# R² score
r2_x = r2_score(x_screen_actual, x_screen_pred)
r2_y = r2_score(y_screen_actual, y_screen_pred)

print(f"R² score for x_screen prediction: {r2_x:.4f}")
print(f"R² score for y_screen prediction: {r2_y:.4f}")

# Calculate errors
mae_x = mean_absolute_error(x_screen_actual, x_screen_pred)
mae_y = mean_absolute_error(y_screen_actual, y_screen_pred)

rmse_x = np.sqrt(mean_squared_error(x_screen_actual, x_screen_pred))
rmse_y = np.sqrt(mean_squared_error(y_screen_actual, y_screen_pred))

print(f"MAE (X): {mae_x:.2f}, RMSE (X): {rmse_x:.2f}")
print(f"MAE (Y): {mae_y:.2f}, RMSE (Y): {rmse_y:.2f}")
