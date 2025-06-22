import tkinter as tk
import rclpy
import tkmsgcat
import os
from baseColors import LIGHT_GRAY, ORANGE

# from AbortRobot import StopPauseResumeNav  # Import StopPauseResumeNav


class Autonomous:
    def __init__(self, master, nav_client):
        self.master = master
        self.nav_client = nav_client
        self.buttons = []
        self.display_autonomous()
        

    def send_goal(self, index):
        try:
            os.system("xdotool search --onlyvisible --name 'RViz*' windowactivate")
            # Define goal positions for each room
            goals = []

            with open("/home/tom/e_wheel_ros/goals.txt", "r") as file:
                for line in file:
                    goals.append(tuple(map(float, line.strip().split())))  # Use split() instead of split(",")

            print("Final goals list:", goals)

            # goals = [
            #     ( 4.435, 0.233, -0.871),  # PHÒNG KHÁCH
            #     (6.066, -3.306, -0.297),   # PHÒNG NGỦ
            #     (0.800, -1.200, 1.570),   # PHÒNG BẾP
            #     (-1.500, 2.000, -0.785),   # PHÒNG VỆ SINH
            #     (-1.500, 2.000, -0.785)   # PHÒNG LÀM VIỆC
            # ]
            # print(goals)

            if 0 <= index < len(goals):
                x, y, theta = goals[index]
                self.nav_client.send_goal(x, y, theta)
                # rclpy.spin(self.nav_client)
            else:
                print("Invalid index")
        except:
            print('Can not send goal')

    def display_autonomous(self):
        frame = tk.Frame(self.master, background='white')
        frame.pack(expand=True, fill="both",pady=35)

        # os.system("xdotool search --onlyvisible --name 'RViz*' windowactivate")

        # Thêm 1 phòng mới ("PHÒNG LÀM VIỆC")
        self.buttons = []
        button_texts = ["PHÒNG KHÁCH", "PHÒNG NGỦ", "PHÒNG BẾP", "PHÒNG VỆ SINH", "PHÒNG LÀM VIỆC", "DỪNG LẠI"]

        for i in range(10):  # Cấu hình cột
            frame.columnconfigure(i, weight=1)

        for index, text in enumerate(button_texts):
            row, col = divmod(index, 2)  # Chia thành 3 hàng, 2 cột

            # Button đặc biệt cho "STOP"
            if text in ["STOP", "DỪNG LẠI"]:
                button = tk.Button(frame, text=text, 
                                bg='RED', fg='white', 
                                font=('Segoe UI', 13, 'bold'),  # Nhỏ hơn
                                height=8, width=30, bd=0, relief='flat',
                                command=lambda: self.nav_client.execute())
            else:
                button = tk.Button(frame, text=text, 
                                bg='#fd6100', fg='white', 
                                font=('Segoe UI', 13, 'bold'), 
                                height=8, width=30, bd=0, relief='flat',
                                command=lambda idx=index: self.send_goal(idx))
            self.buttons.append(button)

            button.grid(row=row, column=col, padx=5, pady=30, sticky="w")  # Căn trái

    def update_language(self):
        """Cập nhật text của các nút khi đổi ngôn ngữ"""
        translations = [tkmsgcat.get("PHÒNG KHÁCH"), tkmsgcat.get("PHÒNG NGỦ"), 
                        tkmsgcat.get("PHÒNG BẾP"), tkmsgcat.get("PHÒNG VỆ SINH"), 
                        tkmsgcat.get("PHÒNG LÀM VIỆC"), tkmsgcat.get("DỪNG LẠI")]

        for btn, new_text in zip(self.buttons, translations):
            btn.config(text=new_text)
