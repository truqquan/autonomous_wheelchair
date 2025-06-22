import sys
import os
import signal
import psutil
import re  # For extracting pose data
import subprocess  # NEW: For running Tkinter app
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QTextEdit, QHBoxLayout, QComboBox, QMessageBox
from PyQt5.QtCore import QProcess

class ROS2Launcher(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Autonomous System")
        self.setGeometry(100, 100, 800, 600)
        self.setStyleSheet("background-color: #ff914d;")

        self.processes = {}  # Store running QProcess instances
        self.selected_map = "map4.yaml"  # Default map
        self.selected_mode = "Only Run"  # Default mode
        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        self.command_buttons = {
            "Robot Core": "ros2 launch e_wheel launch_robot2.launch.py",
            "RViz": "ros2 launch e_wheel launch_.launch.py",
            "Localization": "",  # Command updated dynamically
            "Navigation": "ros2 launch e_wheel nav2_navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true",
            # "Mapping": "ros2 launch e_wheel online_async_launch.py params_file:=./src/e_wheel/config/mapper_params_online_async.yaml use_sim_time:=false"
        }

        self.output_boxes = {}

        for name in self.command_buttons.keys():
            command_layout = QVBoxLayout()

            # Special handling for Localization (Map Selector + Mode Selector + Reset Goal Button)
            if name == "Localization":
                options_layout = QHBoxLayout()

                # Map Selector
                self.map_selector = QComboBox()
                self.map_selector.addItems([f"map{i}.yaml" for i in range(0, 6)])
                self.map_selector.currentTextChanged.connect(self.update_map)
                options_layout.addWidget(self.map_selector)

                # Mode Selector
                self.mode_selector = QComboBox()
                self.mode_selector.addItems(["Only Run", "Choose Goal"])
                self.mode_selector.currentTextChanged.connect(self.update_mode)
                options_layout.addWidget(self.mode_selector)

                # Reset Goal Button
                reset_goal_btn = QPushButton("Reset Goal")
                reset_goal_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #D3D3D3;
                        color: black;
                        font-size: 10pt;
                        border-radius: 8px;
                        padding: 6px;
                    }
                    QPushButton:hover {
                        background-color: red;
                        color: white;
                    }
                """)
                reset_goal_btn.clicked.connect(self.reset_goals)
                options_layout.addWidget(reset_goal_btn)

                # Start Mapping Button
                mapping_btn = QPushButton("Start Mapping")
                mapping_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #D3D3D3;
                        color: black;
                        font-size: 10pt;
                        border-radius: 8px;
                        padding: 6px;
                    }
                    QPushButton:hover {
                        background-color: blue;
                        color: white;
                    }
                """)
                mapping_btn.clicked.connect(self.start_mapping)
                options_layout.addWidget(mapping_btn)

                # Stop Mapping Button
                stop_mapping_btn = QPushButton("Stop Mapping")
                stop_mapping_btn.setStyleSheet("""
                    QPushButton {
                        background-color: #D3D3D3;
                        color: black;
                        font-size: 10pt;
                        border-radius: 8px;
                        padding: 6px;
                    }
                    QPushButton:hover {
                        background-color: darkblue;
                        color: white;
                    }
                """)
                stop_mapping_btn.clicked.connect(self.stop_mapping)
                options_layout.addWidget(stop_mapping_btn)

                command_layout.addLayout(options_layout)

            # Start and Stop buttons
            btn_layout = QHBoxLayout()
            start_btn = QPushButton(f"Start {name}")
            stop_btn = QPushButton(f"Stop {name}")

            start_btn.clicked.connect(lambda checked, n=name: self.start_command(n))
            stop_btn.clicked.connect(lambda checked, n=name: self.stop_command(n))

            btn_layout.addWidget(start_btn)
            btn_layout.addWidget(stop_btn)

            # Output box
            output_box = QTextEdit()
            output_box.setReadOnly(True)
            output_box.setStyleSheet("background-color: white; color: black; font-size: 10pt; padding: 5px; border-radius: 5px;")

            self.output_boxes[name] = output_box

            command_layout.addLayout(btn_layout)
            command_layout.addWidget(output_box)
            layout.addLayout(command_layout)

        # Exit and Open Tkinter App Buttons
        btn_layout = QHBoxLayout()

        # Open Tkinter App Button
        open_tkinter_btn = QPushButton("Open KHKT App")
        open_tkinter_btn.setStyleSheet("""
            QPushButton {
                background-color: #D3D3D3;
                color: black;
                font-size: 14pt;
                font-weight: bold;
                border-radius: 10px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: green;
                color: white;
            }
        """)
        open_tkinter_btn.clicked.connect(self.open_tkinter_app)

        # Exit Button
        exit_btn = QPushButton("Exit")
        exit_btn.setStyleSheet("""
            QPushButton {
                background-color: red;
                color: white;
                font-size: 14pt;
                font-weight: bold;
                border-radius: 10px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: darkred;
            }
        """)
        exit_btn.clicked.connect(self.exit_app)

        btn_layout.addWidget(open_tkinter_btn)
        btn_layout.addWidget(exit_btn)

        layout.addLayout(btn_layout)
        self.setLayout(layout)

    def start_mapping(self):
        """Starts the mapping process without displaying output."""
        if "Mapping" in self.processes and self.processes["Mapping"] is not None:
            return  # Mapping is already running

        command = "ros2 launch e_wheel online_async_launch.py params_file:=./src/e_wheel/config/mapper_params_online_async.yaml use_sim_time:=false"

        process = QProcess(self)
        process.start("bash", ["-c", command])

        self.processes["Mapping"] = process

    def stop_mapping(self):
        """Stops the mapping process."""
        if "Mapping" in self.processes and self.processes["Mapping"] is not None:
            process = self.processes["Mapping"]
            pid = process.processId()

            if pid:
                try:
                    parent = psutil.Process(pid)
                    children = parent.children(recursive=True)

                    for child in children:
                        child.terminate()

                    gone, alive = psutil.wait_procs(children, timeout=3)
                    for child in alive:
                        child.kill()

                    parent.terminate()
                    parent.wait(timeout=3)

                except Exception as e:
                    print(f"Error stopping Mapping: {e}")

            self.processes["Mapping"] = None

    def update_map(self, selected_map):
        """Updates the localization command with the selected map."""
        self.selected_map = selected_map
        self.command_buttons["Localization"] = f"ros2 launch e_wheel nav2_localization_launch.py map:={self.selected_map}"
        self.log_output("Localization", f"Selected map: {self.selected_map}\n")

    def update_mode(self, selected_mode):
        """Updates the selected mode."""
        self.selected_mode = selected_mode
        self.log_output("Localization", f"Mode set to: {self.selected_mode}\n")

    def start_command(self, name):
        """Starts a ROS2 command using QProcess."""
        if name in self.processes and self.processes[name] is not None:
            self.log_output(name, f"{name} is already running.\n")
            return

        command = self.command_buttons[name]
        if not command:
            self.log_output(name, f"No command specified for {name}.\n")
            return

        self.log_output(name, f"Starting {name}...\n")

        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.read_output(name))  # FIXED: Now exists
        process.started.connect(lambda: self.log_output(name, f"{name} started.\n"))
        process.finished.connect(lambda: self.log_output(name, f"{name} stopped.\n"))

        process.start("bash", ["-c", command])

        self.processes[name] = process

    def stop_command(self, name):
        """Stops a running process and ensures full termination."""
        if name in self.processes and self.processes[name] is not None:
            process = self.processes[name]
            pid = process.processId()
            
            if pid:
                self.log_output(name, f"Stopping {name} (PID: {pid})...\n")

                # Use psutil to kill all child processes
                try:
                    parent = psutil.Process(pid)
                    children = parent.children(recursive=True)

                    for child in children:
                        self.log_output(name, f"Terminating child process {child.pid} of {name}...\n")
                        child.terminate()

                    gone, alive = psutil.wait_procs(children, timeout=3)
                    for child in alive:
                        self.log_output(name, f"Force killing {child.pid} of {name}...\n")
                        child.kill()

                    # Kill parent process
                    parent.terminate()
                    parent.wait(timeout=3)
                    self.log_output(name, f"{name} fully stopped.\n")

                except Exception as e:
                    self.log_output(name, f"Error stopping {name}: {e}\n")

            self.processes[name] = None
        else:
            self.log_output(name, f"{name} is not running.\n")
    
    def read_output(self, name):
        """Reads process output and displays it in the respective output box."""
        if name in self.processes and self.processes[name]:
            output = self.processes[name].readAllStandardOutput().data().decode()
            if output:
                self.log_output(name, output)

                # Capture and store initial pose if "Choose Goal" mode is selected
                if self.selected_mode == "Choose Goal":
                    match = re.search(r"\[amcl-2\] \[INFO\] .*?: Setting pose .*?: ([\d.-]+) ([\d.-]+) ([\d.-]+)", output)
                    if match:
                        pose = f"{match.group(1)} {match.group(2)} {match.group(3)}"
                        with open("goals.txt", "a") as file:
                            file.write(pose + "\n")
                        self.log_output(name, f"Saved goal: {pose}\n")

                # Check for Robot Core errors
                if name == "Robot Core":
                    if "[camera.camera]: No RealSense devices were found!" in output:
                        self.stop_command(name)
                        self.log_output(name, '<font color="red">Camera error</font>\n')
                    elif "[rplidar_node]: Error" in output:
                        self.stop_command(name)
                        self.log_output(name, '<font color="red">Lidar error</font>\n')
                    elif "[ERROR] [ros2_control_node-6]: process has died" in output:
                        self.stop_command(name)
                        self.log_output(name, '<font color="red">Arduino error</font>\n')

    def reset_goals(self):
        """Clears the goals.txt file after confirmation."""
        reply = QMessageBox.question(
            self, "Reset Goals", "Are you sure you want to clear all saved goals?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            try:
                open("goals.txt", "w").close()  # Clears file contents
                self.log_output("Localization", "Goals.txt has been reset.\n")
            except Exception as e:
                self.log_output("Localization", f"Error resetting goals.txt: {e}\n")

    def open_tkinter_app(self):
        """Opens the Tkinter app located at /tom/home/app_v2/app.py."""
        try:
            subprocess.Popen(["python3", "/home/tom/app_v2/app.py"])
            self.log_output("System", "Opened Tkinter App successfully.\n")
        except Exception as e:
            self.log_output("System", f"Failed to open Tkinter App: {e}\n")

    def log_output(self, name, message):
        """Logs messages to the respective output box."""
        if name in self.output_boxes:
            self.output_boxes[name].append(message)

    def exit_app(self):
        """Stops all running processes and exits."""
        for name in list(self.processes.keys()):
            self.stop_command(name)
        sys.exit()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ROS2Launcher()
    window.show()
    sys.exit(app.exec_())
