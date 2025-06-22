import os
import subprocess

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

# Example usage
pids = find_camera_process_linux()
for pid in pids:
    kill_process_linux(pid)

