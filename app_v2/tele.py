import threading
import time
import queue
import requests
import speech_recognition as sr
import soundfile as sf

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
            text = recognizer.recognize_google(audio, language='vi-VN')
            return text
    except Exception as e:
        return f"Error processing audio: {e}"

def send_message(chat_id, text):
    url = f"{BASE_URL}/sendMessage"
    params = {'chat_id': chat_id, 'text': text}
    requests.get(url, params=params)

# Telegram Bot Loop
def telegram_bot_loop():
    offset = None
    print("Telegram Bot is running...")

    while True:
        updates = get_updates(offset)
        if updates.get("ok"):
            for update in updates["result"]:
                offset = update["update_id"] + 1
                
                if "message" in update:
                    message = update["message"]
                    chat_id = message["chat"]["id"]
                    
                    # Handle text messages
                    if "text" in message:
                        user_text = message["text"]
                        print(f"Text message received: {user_text}")
                        message_queue.put({"type": "text", "chat_id": chat_id, "content": user_text})

                    # Handle voice messages
                    elif "voice" in message:
                        file_id = message["voice"]["file_id"]
                        print("Voice message received")
                        
                        voice_file = download_file(file_id)
                        if voice_file:
                            text = convert_voice_to_text(voice_file)
                            print(f"Transcribed text: {text}")
                            message_queue.put({"type": "voice", "chat_id": chat_id, "content": text})
                        else:
                            send_message(chat_id, "Failed to download the voice message.")
        
        time.sleep(1)  # Prevent overloading Telegram API

# Real-Time Loop
def real_time_loop():
    print("Real-Time Task is running...")
    while True:
        # Your real-time processing code here
        print("Real-time task working...")
        
        # Check if there are messages from Telegram
        while not message_queue.empty():
            msg = message_queue.get()
            if msg["type"] == "text":
                print(f"Processing received text: {msg['content']}")
                # Example: Send a reply
                send_message(msg["chat_id"], f"You sent: {msg['content']}")
            elif msg["type"] == "voice":
                print(f"Processing transcribed voice: {msg['content']}")
                # Example: Send a reply
                send_message(msg["chat_id"], f"You said: {msg['content']}")
        
        # Example: Send a message from the real-time loop
        # Uncomment if you want to send a message periodically
        # send_message(chat_id, "Real-time loop is running!")
        
        time.sleep(0.5)  # Simulating real-time work

# Combine Telegram Bot and Real-Time Loop
if __name__ == "__main__":
    # Run Telegram bot in a separate thread
    telegram_thread = threading.Thread(target=telegram_bot_loop, daemon=True)
    telegram_thread.start()

    # Run real-time loop in the main thread
    real_time_loop()
