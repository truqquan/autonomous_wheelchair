import tkinter as tk
from utils import create_rounded_rectangle
import pandas as pd
import numpy as np
from baseColors import LIGHT_GRAY
import sys
from io import StringIO
import ollama
from gtts import gTTS
from playsound import playsound
import os
import threading
from pynput import keyboard
import time
import queue
import requests
import speech_recognition as sr
import soundfile as sf
import tkmsgcat
import unidecode
import re

WORD_MODE = 0
CONTACTS_MODE = 1

# Telegram Bot Token
TOKEN = '7217782180:AAFCcTV60-xcgJG_P24UnvSvgPqyw8UWINc'
BASE_URL = f"https://api.telegram.org/bot{TOKEN}"

def custom_split(line):
    parts = line.split(',', 2)
    if len(parts) < 3:
        parts.append("")
    return parts[:2]

question = " "

class Keyboard:
    def __init__(self, master):

        self.master = master
        self.keys = [
            ['q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p'],
            ['a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l','Backspace'],
            ['Space','z', 'x', 'c', 'v', 'b', 'n', 'm', 'Submit']
        ]

        self.key_width = 178
        self.key_height = 169
        self.font_size = 13
        self.font_family = 'Segoe UI'
        self.padding = 14
        self.key_objects = {}
        self.user_text = ''
        self.mode = WORD_MODE 

        self.recognized_text = ""  
        # self.thread = None
        
        self.words = []
        self.word_dict = None
        self.language = 'vi-VN'
        self.get_word_dict(self.language)
        
        self.top_frame = tk.Frame(self.master, bg='white')
        self.top_frame.pack(pady=2, expand=True, fill='x')

        self.input_frame = tk.Frame(self.master, bg='white')
        self.input_frame.pack(pady=3, expand=True, fill='x')
        
        self.middle_frame = tk.Frame(self.master, bg='white')
        self.middle_frame.pack(pady=3, expand=True, fill='x')
        
        self.keyboard_frame = tk.Frame(self.master, bg='white')
        self.keyboard_frame.pack(pady=3, expand=True, fill='x')
        
        self.sentences_frame = tk.Frame(self.top_frame, bg='white')
        self.sentences_frame.pack(anchor='center')
        self.chosen_sentence_index = -1
        self.sentences = ['Đây là câu đầu tiên.', 'Đây là câu thứ hai.', 'Đây là câu thứ ba.']
        self.sentence_counts = len(self.sentences)
        self.sentence_buttons = []
        self.sentence_width = 30

        for i in range(self.sentence_counts):
            sentence_button = tk.Button(
                self.sentences_frame,
                text=self.sentences[i],
                font=(self.font_family, self.font_size),
                command=lambda idx=i: self.choose_sentence(idx),
                width=self.sentence_width, height=5, wraplength=250,
                activebackground='#fd6100'
            )
            self.make_sentences_hover_effect(sentence_button, i)
            sentence_button.grid(row=0, column=i, padx=5, rowspan=2)
            self.sentence_buttons.append(sentence_button)


        self.contacts = [] 
        self.contacts = [
            {"chat_id": 5629605968, "name": "Peter"},
            {"chat_id": 9876543210, "name": "Thomas"},
            {"chat_id": 1122334455, "name": "Tom"},
            {"chat_id": 5566778899, "name": "Vip"},
            {"chat_id": 1029384756, "name": "Tron"}
        ]       
        self.telegram_button = tk.Button(self.sentences_frame, 
                                         text='Telegram', 
                                         font=(self.font_family, self.font_size), 
                                         width = self.sentence_width, 
                                         height=5,
                                         activebackground='#fd6100',
                                         command=lambda: self.send_to_telegram())
        

        self.make_sentence_sound_button = tk.Button(self.sentences_frame, 
                                                    text=tkmsgcat.get("PHÁT ÂM THANH"), 
                                                    font=(self.font_family, self.font_size), 
                                                    width = self.sentence_width, 
                                                    height=5,
                                                    activebackground='#fd6100',
                                                    command=lambda: self.make_sound())
        self.make_hover_effect(self.telegram_button)
        self.make_hover_effect(self.make_sentence_sound_button)
        self.telegram_button.grid(row=0, column=3, pady=5, padx=20)
        self.make_sentence_sound_button.grid(row=0, column=4, pady=5, padx=20)

        self.question = 'Day la cau hoi'
        self.question_frame = tk.Frame(self.input_frame, bg='white')
        self.question_frame.pack(expand=True, fill='both')
        self.question_label = tk.Label(self.question_frame, text=self.question, font=(self.font_family, self.font_size), background='white', width=70)
        self.textInput = tk.Entry(self.question_frame, font=(self.font_family, self.font_size), width=100)

        # self.listener = keyboard.Listener(on_press=self.on_press)
        # self.listener.start()

        # Telegram Bot Token
        TOKEN = '7217782180:AAFCcTV60-xcgJG_P24UnvSvgPqyw8UWINc'
        BASE_URL = f"https://api.telegram.org/bot{TOKEN}"
        # Shared queue for communication between threads
        message_queue = queue.Queue()

        # Telegram Bot Functions
        def get_updates(offset=None):
            url = f"{BASE_URL}/getUpdates"
            params = {'offset': offset} if offset else {}
            response = requests.get(url, params=params)
            return response.json()

        def download_file(file_id):
            file_info_url = f"{BASE_URL}/getFile?file_id={file_id}"
            file_info = requests.get(file_info_url).json()

            if file_info.get("ok"):
                file_path = file_info["result"]["file_path"]
                file_url = f"https://api.telegram.org/file/bot{TOKEN}/{file_path}"
                response = requests.get(file_url)
                local_filename = file_path.split('/')[-1]
                with open(local_filename, 'wb') as f:
                    f.write(response.content)
                return local_filename
            return None

        def convert_voice_to_text(file_path):
            try:
                audio_data, samplerate = sf.read(file_path)
                wav_file = file_path.replace(".oga", ".wav")
                sf.write(wav_file, audio_data, samplerate)

                recognizer = sr.Recognizer()
                with sr.AudioFile(wav_file) as source:
                    audio = recognizer.record(source)
                    text = recognizer.recognize_google(audio, language=self.language)
                    return text
            except Exception as e:
                return f"Error processing audio: {e}"

        def send_message(chat_id, text):
            url = f"{BASE_URL}/sendMessage"
            params = {'chat_id': chat_id, 'text': text}
            requests.get(url, params=params)

        # Telegram Bot Loop
        def telegram_bot_loop():
            global question
            offset = None
            print("Telegram Bot is running...")
            chat_list = {
                5629605968: "Peter",
                9876543210: "Thomas",
                1122334455: "Tom",
                5566778899: "Vip",
                1029384756: "Tron"
            }

            while True:
                updates = get_updates(offset)
                if updates.get("ok"):
                    for update in updates["result"]:
                        offset = update["update_id"] + 1
                        
                        if "message" in update:
                            message = update["message"]
                            chat_id = message["chat"]["id"]

                            name = chat_list.get(chat_id, "Người lạ")
                            print(name)
                            for contact in self.contacts:
                                if contact["name"] == "Em":
                                    contact["name"] = "Người lạ"
                                    contact["chat_id"] = chat_id  # Optional: Remove chat_id as well
                            
                            # Handle text messages
                            if "text" in message:
                                self.user_text = message["text"]
                                print(f"Text message received: {self.user_text}")
                                message_queue.put({"type": "text", "chat_id": chat_id, "content": self.user_text})

                            # Handle voice messages
                            elif "voice" in message:
                                file_id = message["voice"]["file_id"]
                                print("Voice message received")
                                
                                voice_file = download_file(file_id)
                                if voice_file:
                                    self.user_text = convert_voice_to_text(voice_file)
                                    print(f"Transcribed text: {self.user_text}")
                                    message_queue.put({"type": "voice", "chat_id": chat_id, "content": self.user_text})
                                else:
                                    send_message(chat_id, "Failed to download the voice message.")
                                    
                            self.question = self.user_text  # Update self.question
                            self.question_label.config(text=name + ': ' + self.question)
                
                time.sleep(1)  # Prevent overloading Telegram API

        telegram_thread = threading.Thread(target=telegram_bot_loop, daemon=True)
        telegram_thread.start()
        
        self.question_label.grid(row=0, column=0, padx=10, pady=5, sticky='w')
        self.textInput.grid(row=0, column=1, padx=10, pady=5, sticky='e')
        
        # Middle Frame: Closest words
        self.buttons_frame = tk.Frame(self.middle_frame, bg='white')
        self.buttons_frame.pack(anchor='center', pady = 5)
        
        self.word_counts = 5
        self.closest_words = [""] * self.word_counts
        self.closest_words_buttons = []

        for i in range(self.word_counts):
            button = tk.Button(
                self.buttons_frame,
                text="",
                font=(self.font_family, (int)(self.font_size * 1.5)),
                borderwidth=0,
                bg='white',
                relief='flat',
                width=20,
                height=4,
                command=lambda idx=i: self.update_word_input(idx)
            )
            button.grid(row=0, column=i, padx=12, pady = 7)
            self.closest_words_buttons.append(button)

        self.canvasBackground = tk.Canvas(self.keyboard_frame, bg=LIGHT_GRAY, width=self.key_width * 12, height=self.key_height * 3.5)
        self.canvasBackground.pack()
        self.draw_keyboard()

    # def recognize_speech(self):
    #     """Hàm nhận diện giọng nói và chỉ chạy 1 lần trên 1 luồng."""
    #     recognizer = sr.Recognizer()
    #     with sr.Microphone() as source:
    #         print("Listening...")
    #         recognizer.adjust_for_ambient_noise(source, duration=0.5)
    #         audio = recognizer.listen(source, timeout=3, phrase_time_limit=5)

    #     try:
    #         self.recognized_text = recognizer.recognize_google(audio, language=self.language)
    #         print(f"Recognized: {self.recognized_text}")
    #     except sr.UnknownValueError:
    #         print("Could not understand audio.")
    #         self.recognized_text = "Hãy thử lại" if self.language == 'vi-VN' else "Try again"
    #     except sr.RequestError as e:
    #         print(f"Error: {e}")

    #     self.thread = None     
   
    # Enter pressed!
    # def on_press(self, key):
    #     """Khi nhấn Enter, kiểm tra và chỉ chạy 1 luồng nhận diện."""
    #     if key == keyboard.Key.enter:
    #         if self.thread is not None and self.thread.is_alive():
    #             print("Recognition is still running, please wait...")
    #             return  # Không tạo thêm luồng mới
            
    #         print("Enter key pressed! Starting recognition thread...")
    #         self.thread = threading.Thread(target=self.recognize_speech, daemon=True)
    #         self.thread.start()

    #         # Chờ cho đến khi nhận diện xong
    #         while self.thread is not None:
    #             time.sleep(0.1)  # Tránh CPU 100%

    #         print("Recognition completed!")
    #         self.question = self.recognized_text
    #         self.question_label.config(text=self.question)


    def update_language(self):
        """Cập nhật text của button khi đổi ngôn ngữ"""
        print("-------------------------------------------------------")

        self.make_sentence_sound_button.config(text=tkmsgcat.get("PHÁT ÂM THANH"))
        self.language = 'en-US' if self.language == 'vi-VN' else 'vi-VN'
        self.get_word_dict(self.language)
        for button in self.closest_words_buttons:
            button.config(text="")
        self.closest_words = [""] * self.word_counts
        self.sentences = [""] * self.sentence_counts
        for button in self.sentence_buttons:
            button.config(text="", bg=LIGHT_GRAY)
        self.textInput.delete(0, tk.END)
        

    def make_hover_effect(self, button):
        def on_enter(event, btn=button):
            btn.config(bg='#fd6100', relief="raised")  

        def on_leave(event, btn=button):
            btn.config(bg=LIGHT_GRAY, relief="flat")
        
        button.bind("<Enter>", on_enter)
        button.bind("<Leave>", on_leave)

    def make_sentences_hover_effect(self, button, sentence_index):
        def on_enter(event, btn=button):
            if sentence_index != self.chosen_sentence_index:
                btn.config(bg='#fd6100', relief="raised")  

        def on_leave(event, btn=button):
            if sentence_index != self.chosen_sentence_index:
                btn.config(bg=LIGHT_GRAY, relief="flat")
        
        button.bind("<Enter>", on_enter)
        button.bind("<Leave>", on_leave)

    def make_sound(self):
        # Passing the text and language to the engine, here we are using Google Text-to-Speech
        language = 'vi' if self.language == 'vi-VN' else 'en'
        tts = gTTS(text=self.sentences[self.chosen_sentence_index], lang=language, slow=False)

        try:
            os.remove("output.mp3")
        except:
            pass

        # Save the speech to a file
        audio_file = "output.mp3"
        tts.save(audio_file)
        playsound('output.mp3')

    def generate_text(self, text):
        global question
        question = self.question
        if question == 'Day la cau hoi':
            question = " "
        suggest_words = text

        user_prompt = f"""Dưới đây là một hướng dẫn mô tả một nhiệm vụ, kèm theo một đầu vào cung cấp ngữ cảnh bổ sung. Hãy viết một phản hồi phù hợp để hoàn thành yêu cầu.

            ### Hướng dẫn: 
            Dựa vào câu hỏi (có hoặc không) và phần gợi ý cho câu trả lời ở đầu vào để đề xuất 3 câu câu trả lời hoàn chỉnh.

            ### Đầu vào:
            câu hỏi: {question} | gợi ý: {suggest_words},

            ### Phản hồi:
            """

        if self.language == 'vi-VN':
            model_name = "e_wheel_vi:latest"
        else:  
            model_name = "e_wheel_en:latest"

        # Redirect standard output to capture printed output
        sys.stdout = StringIO()

        stream = ollama.chat(
            model=model_name,
            messages=[{'role': 'user', 'content': user_prompt}],
            stream=True
        )

        stored_output = ""
        for chunk in stream:
            st = chunk['message']['content']
            print(st, end='')
            stored_output += st  

        # Get the captured printed output
        captured_output = sys.stdout.getvalue().replace("*", "")

        # Reset standard output
        sys.stdout = sys.__stdout__

        print(captured_output) 

        return captured_output


    def send_to_telegram(self):
        # print("Write to Telegram !")
        
        self.mode = CONTACTS_MODE
        names = [contact['name'] for contact in self.contacts]
        self.show_closest_words(names)

    def send_to_telegram_with_messages_and_chatId(self, chat_id, message):
        global question
        # Send message
        url = f"{BASE_URL}/sendMessage"
        params = {'chat_id': chat_id, 'text': message}
        requests.get(url, params=params)
        # print(chat_id)
        print(message)
        question = " "
        self.question = " "  # Update self.question
        self.question_label.config(text=self.question)
        self.contacts = [
            {"chat_id": 5629605968, "name": "Peter"},
            {"chat_id": 9876543210, "name": "Thomas"},
            {"chat_id": 1122334455, "name": "Tom"},
            {"chat_id": 5566778899, "name": "Vip"},
            {"chat_id": 1029384756, "name": "Tron"}
        ]

    def get_word_dict(self, lang='vi-VN'):
        self.word_dict = None
        if lang == 'vi-VN':
            with open('/home/tom/app_v2/Từ điển tiếng Việt.txt', encoding='utf-8') as file:
                rows = [line.strip() for line in file]
            self.word_dict = rows
        else:
            with open('/home/tom/app_v2/Từ điển tiếng Anh.txt', encoding='utf-8') as file:
                rows = [line.strip() for line in file]
            self.word_dict = rows

    
    # Hàm vẽ phím
    def draw_key(self, x, y, width, height, text, key):
        rect = create_rounded_rectangle(self.canvasBackground, x, y, x + width, y + height, 12, fill="lightgray", outline="black", width=1)
        label = self.canvasBackground.create_text(x + width / 2, y + height / 2, text=text, font=(self.font_family, (int)(self.font_size * 1.5)), tags="text")

        self.key_objects[key] = {'rect': rect, 'label': label, 'hover_timer': None} 

        def on_enter(e, k=key):
            self.canvasBackground.itemconfig(self.key_objects[k]['rect'], fill='#fd6100')  # Thay đổi màu nền khi hover vào
            self.canvasBackground.itemconfig(self.key_objects[k]['label'], font=(self.font_family, (int)(self.font_size * 1.5), 'bold'))  # Tăng cỡ font khi hover vào
            
            # # Khởi động hẹn giờ 1.5 giây để tự động nhập phím
            # self.key_objects[k]['hover_timer'] = self.canvasBackground.after(1500, lambda: self.button_click(k))

        def on_leave(e, k=key):
            self.canvasBackground.itemconfig(self.key_objects[k]['rect'], fill="lightgray")  # Quay lại màu gốc khi rời chuột
            self.canvasBackground.itemconfig(self.key_objects[k]['label'], font=(self.font_family, (int)(self.font_size * 1.5)))  # Quay lại cỡ font gốc

            # # Hủy hẹn giờ nếu rời chuột
            # if self.key_objects[k]['hover_timer']:
            #     self.canvasBackground.after_cancel(self.key_objects[k]['hover_timer'])
            #     self.key_objects[k]['hover_timer'] = None

        # Gán các sự kiện hover
        self.canvasBackground.tag_bind(rect, "<Enter>", on_enter)
        self.canvasBackground.tag_bind(label, "<Enter>", on_enter)
        self.canvasBackground.tag_bind(rect, "<Leave>", on_leave)
        self.canvasBackground.tag_bind(label, "<Leave>", on_leave)

        self.canvasBackground.tag_bind(rect, "<Button-1>", lambda e, k=key: self.button_key_click(k))
        self.canvasBackground.tag_bind(label, "<Button-1>", lambda e, k=key: self.button_key_click(k))

    def update_word_input(self, index):
        if self.mode == WORD_MODE:
            word = self.closest_words[index]
            current_word, left_boundary, right_boundary = self.get_current_word()
            
            full_text = self.textInput.get()
            new_text = full_text[:left_boundary] + word + full_text[right_boundary:]
            self.textInput.delete(0, tk.END)
            self.textInput.insert(0, new_text)

            self.textInput.icursor(left_boundary + len(word))
            
            # Lại ẩn các button và set lại giá trị cho các biến word về rỗng
            self.show_closest_words([])
        else:
            self.send_to_telegram_with_messages_and_chatId(self.contacts[index]['chat_id'], self.sentences[self.chosen_sentence_index])

    def choose_sentence(self, sentence_number):
        self.sentences_frame.update_idletasks()
        for i in range(self.sentence_counts):
            if i == sentence_number:
                self.sentence_buttons[i].config(bg='#fd6100')
                self.chosen_sentence_index = i
                print(self.sentences[self.chosen_sentence_index])
            else:
                self.sentence_buttons[i].config(bg=LIGHT_GRAY)

    def remove_tones(self, word):
        """Loại bỏ dấu thanh khỏi từ tiếng Việt."""
        return unidecode.unidecode(word)

    def find_closest_words(self, current_word):
        """Tìm n từ gần nhất trong từ điển dựa trên current_word bằng cách so khớp tiền tố."""
        if not current_word.strip():  # Nếu từ rỗng, trả về danh sách trống
            return []
        
        current_word_no_tones = self.remove_tones(current_word)  # Loại bỏ dấu thanh
        suggestions = []
        
        for word in self.word_dict:
            if self.remove_tones(word).startswith(current_word_no_tones):
                suggestions.append(word)
                if len(suggestions) >= self.word_counts:
                    break
        
        return suggestions

    def handle_text_input(self, key):
        """Xử lý văn bản nhập vào khi nhấn phím."""
        self.textInput.insert(tk.INSERT, key)

        # Lấy từ hiện tại
        current_word, _, _ = self.get_current_word()
        if len(current_word) < 2:
            self.show_closest_words([])
            return

        # Tìm n từ gần nhất trong self.word_dict
        closest_words = self.find_closest_words(current_word)

        # Hiển thị n từ gần nhất trên màn hình Tkinter
        self.show_closest_words(closest_words)

    def show_closest_words(self, words):
        """Hiển thị n từ gần nhất trong giao diện Tkinter."""
        for i in range(self.word_counts):
            word = words[i] if i < len(words) else ""
            self.closest_words[i] = word
            self.closest_words_buttons[i].config(text=word)
            
            if word == "":
                self.closest_words_buttons[i].config(bg='white', borderwidth=0, relief='flat')
            else:
                self.closest_words_buttons[i].config(bg='#fd6100', borderwidth=1, relief='solid')

    def get_current_word(self):
        """Lấy từ hiện tại dựa trên vị trí con trỏ trong Entry."""
        cursor_pos = self.textInput.index(tk.INSERT)
        full_text = self.textInput.get()

        # Xác định từ trước và sau vị trí con trỏ
        left_part = full_text[:cursor_pos]
        right_part = full_text[cursor_pos:]

        # Tìm khoảng cách gần nhất bên trái
        left_space_idx = left_part.rfind(' ')
        left_boundary = left_space_idx + 1

        # Tìm khoảng cách gần nhất bên phải
        right_space_idx = right_part.find(' ')
        if right_space_idx == -1:
            right_boundary = len(full_text)
        else:
            right_boundary = cursor_pos + right_space_idx

        # Trích xuất từ hiện tại
        current_word = full_text[left_boundary:right_boundary]
        return current_word.strip(), left_boundary, right_boundary

    def button_key_click(self, key):
        self.mode = WORD_MODE
        current_pos = self.textInput.index(tk.INSERT)

        if key == 'Backspace':
            if current_pos != "1.0":
                self.textInput.delete(current_pos - 1, current_pos)

        elif key == 'Space':
            # Thêm một khoảng trắng
            self.textInput.insert(tk.INSERT, ' ')

        elif key == 'Submit':
            cursor_pos = self.textInput.index(tk.INSERT)
            full_text = self.textInput.get()

            generated_text = self.generate_text(full_text)

            # Tìm các câu theo mẫu số thứ tự
            matches = re.findall(r'\d+\.\s*([^?.!]+)', generated_text)

            # Lưu vào danh sách nếu có đủ phần tử
            if len(matches) >= 3:
                self.sentences = [matches[0].strip(), matches[1].strip(), matches[2].strip()]
            for index, sentence_button in enumerate(self.sentence_buttons):
                sentence_button.config(text=self.sentences[index], bg = LIGHT_GRAY)
        else:
            # Xử lý văn bản nhập vào
            self.handle_text_input(key)

    def draw_keyboard(self):
        self.canvasBackground.delete("all")
        y_offset = 10
        for i, key_row in enumerate(self.keys):
            if i == len(self.keys) - 1:  # Check if it's the last row
                x_offset = 35  # Apply the offset for the last row
            else:
                x_offset = 5  # No offset for other rows
            for j, key in enumerate(key_row):
                if key == 'Space':
                    display_text = key
                    width = self.key_width * 1.35
                elif key == 'Submit':
                    display_text = '✔'
                    width = self.key_width * 1.35
                elif key == 'Backspace':
                    display_text = '⌫'
                    width = self.key_width * 1.0
                else:
                    display_text = key
                    width = self.key_width

                self.draw_key(x_offset, y_offset, width, self.key_height, display_text, key)
                
                x_offset += width + self.padding
            y_offset += self.key_height + self.padding
        self.canvasBackground.update()
