import tkinter as tk
from tkinter import ttk
from communication import Keyboard
from homepage import Homepage
from autonomous import Autonomous
from baseColors import LIGHT_GRAY, ORANGE
from pathlib import Path
import pyglet
import rclpy
from Nav2_client import NavigationClient
from wheelChair import TeleopNode
# from AbortRobot import StopPauseResumeNav
from wheelChair import WheelChair
from PIL import Image, ImageTk
import tkmsgcat
import os

class Window(tk.Tk):
    def __init__(self):
        super().__init__()
        rclpy.init()
        self.nav_client = NavigationClient()
        self.control_client = TeleopNode()
        # self.abort_client = StopPauseResumeNav()

        self.title("KHKT")
        self.configure(background='white')
        self.attributes('-zoomed', True)

        # Load translations
        msgsdir = Path(__file__).parent / "msgs"
        tkmsgcat.load(msgsdir)
        self.current_language = "vi"  # Mặc định là tiếng Việt
        tkmsgcat.locale(self.current_language)

        # Tạo Notebook (Tabs)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack()

        self.instances = []
        # Tạo Tabs
        self.create_tabs()

        # Thêm nút chuyển đổi ngôn ngữ
        self.vietnam_icon = tk.PhotoImage(file='/home/tom/app_v2/Images/vietnam_flag.png').subsample(16, 16)
        self.english_icon = tk.PhotoImage(file='/home/tom/app_v2/Images/english_flag.png').subsample(16, 16)

        self.language_button = tk.Button(self.home_tab_frame, image=self.vietnam_icon, command=self.switch_language, bd=0, bg='Cadet Blue')
        self.language_button.place(relx=1.0, rely=0, anchor="ne", y=240)

        # Style setup
        self.style = ttk.Style()
        self.style.theme_create('custom_theme', settings={
            'TNotebook': {
                'configure': {'tabposition': 'nsew'}
            },
            'TNotebook.Tab': {
                'configure': {
                    'font': ('Segoe UI', 27, 'bold'),
                    'foreground': 'white',
                    'relief': 'flat',
                    'padding': [30, 35],
                    'width': 15,  # Ensures all tabs have the same width
                    'background': LIGHT_GRAY,
                    'anchor': 'center',
                    'focuscolor': 'none',
                    'focusborderwidth': 0
                },
                'map': {
                    'background': [('selected', '#fd6100'), ('active', LIGHT_GRAY)],
                    'foreground': [('selected', 'white'), ('active', '#fd6100')]
                }
            },
            'TFrame': {
                'configure': {'background': 'white'}
            }
        })
        self.style.theme_use('custom_theme')
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_change)

    def on_tab_change(self, event):
        """Gọi hàm khi chuyển tab"""
        selected_tab = self.notebook.index(self.notebook.select())  # Lấy chỉ mục tab hiện tại
        autonomous_tab_index = 3  # Vị trí tab Autonomous (thứ 3, bắt đầu từ 0)

        if selected_tab == autonomous_tab_index:
            os.system("xdotool search --onlyvisible --name 'RViz*' windowactivate")   

    def create_tabs(self):
        """Tạo các tab với nội dung dịch"""
        # Xóa tất cả các tab trước khi tạo lại
        for tab_id in self.notebook.tabs():
            self.notebook.forget(tab_id)

        # Tab trống
        self.home_tab_frame = ttk.Frame(self.notebook)
        self.home_icon = tk.PhotoImage(file='/home/tom/app_v2/Images/homepage.png').subsample(3, 3)
        self.notebook.add(self.home_tab_frame, text=tkmsgcat.get("TRANG CHỦ"), image=self.home_icon, compound=tk.LEFT)
        self.instances.append(Homepage(self.home_tab_frame))

        # Tab xe lăn
        wheelchair_tab_frame = ttk.Frame(self.notebook)
        self.wheelchair_icon = tk.PhotoImage(file='/home/tom/app_v2/Images/wheelchair.png').subsample(3, 3)
        self.notebook.add(wheelchair_tab_frame, text=tkmsgcat.get("XE LĂN"), image=self.wheelchair_icon, compound=tk.LEFT)
        WheelChair(wheelchair_tab_frame, self.control_client)

        # Tab giao tiếp
        keyboard_tab_frame = ttk.Frame(self.notebook)
        self.keyboard_icon = tk.PhotoImage(file='/home/tom/app_v2/Images/keyboard.png').subsample(3, 3)
        self.notebook.add(keyboard_tab_frame, text=tkmsgcat.get("GIAO TIẾP"), image=self.keyboard_icon, compound=tk.LEFT)
        self.instances.append(Keyboard(keyboard_tab_frame))

        # Tab tự hành
        autonomous_tab_frame = ttk.Frame(self.notebook)
        self.autonomous_icon = tk.PhotoImage(file='/home/tom/app_v2/Images/autonomous.png').subsample(3, 3)
        self.notebook.add(autonomous_tab_frame, text=tkmsgcat.get("TỰ HÀNH"), image=self.autonomous_icon, compound=tk.LEFT)
        self.instances.append(Autonomous(autonomous_tab_frame, self.nav_client))


    def switch_language(self):
        """Hàm chuyển đổi ngôn ngữ"""
        self.current_language = "en" if self.current_language == "vi" else "vi"
        tkmsgcat.locale(self.current_language)

        # Cập nhật hình ảnh của nút chuyển đổi
        self.language_button.config(image=self.english_icon if self.current_language == "vi" else self.vietnam_icon)

        # Cập nhật text của các tab mà không tạo lại chúng
        tab_texts = [ tkmsgcat.get("TRANG CHỦ"), tkmsgcat.get("XE LĂN"), tkmsgcat.get("GIAO TIẾP"), tkmsgcat.get("TỰ HÀNH")]
        
        for i, text in enumerate(tab_texts):
            self.notebook.tab(i, text=text)

        for instance in self.instances:
            instance.update_language()

if __name__ == "__main__":
    app = Window()
    app.mainloop()
