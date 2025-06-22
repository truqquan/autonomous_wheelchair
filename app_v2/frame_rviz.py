import subprocess
import cv2
import numpy as np
import tkinter as tk
from PIL import Image, ImageTk

def get_rviz_window_id():
    """Find the RViz2 window ID using wmctrl."""
    try:
        result = subprocess.run(["wmctrl", "-l"], capture_output=True, text=True)
        for line in result.stdout.split("\n"):
            if "RViz*" in line:  # Remove '*' wildcard
                return line.split()[0]  # Extract window ID
    except Exception as e:
        print("Error finding RViz2 window:", e)
    return None

def capture_rviz2():
    """Capture RViz2 window using Xcomposite."""
    window_id = get_rviz_window_id()
    if not window_id:
        print("RViz2 window not found!")
        return None

    try:
        # Use ffmpeg with Xcomposite to capture the window even if hidden
        cmd = f"ffmpeg -f x11grab -draw_mouse 0 -r 30 -i :0+window_id={window_id} -vframes 1 -f image2 -"
        result = subprocess.run(cmd, shell=True, capture_output=True)

        img_array = np.frombuffer(result.stdout, dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img
    except Exception as e:
        print("Error capturing RViz2:", e)
        return None

def update_tkinter():
    img = capture_rviz2()
    if img is not None:
        img = Image.fromarray(img)
        img = img.resize((800, 600))
        imgtk = ImageTk.PhotoImage(image=img)
        label.imgtk = imgtk
        label.config(image=imgtk)

    root.after(50, update_tkinter)

# Tkinter GUI
root = tk.Tk()
root.title("RViz2 in Tkinter")

label = tk.Label(root)
label.pack()

update_tkinter()
root.mainloop()
