import speech_recognition as sr
import time
import threading

def speech_to_text():
    # Create a recognizer instance
    r = sr.Recognizer()

    while True:
        try:
            # Use the default microphone as the audio source
            with sr.Microphone() as source:
                print("Adjusting for ambient noise...")
                r.adjust_for_ambient_noise(source)

                # Capture the audio input from the user
                print("Listening for speech...")
                audio = r.listen(source)

                # Use the Google Web Speech API to recognize the audio
                text = r.recognize_google(audio, language='vi-VN')
                print("Recognized text: ", text)

        except sr.UnknownValueError:
            # Handle the case where speech is not understood
            print("Could not understand the audio.")
        except sr.RequestError as e:
            # Handle API request errors
            print(f"Error with the Google Speech API: {e}")
        except Exception as e:
            # Handle unexpected errors
            print(f"An unexpected error occurred: {e}")

def start_speech_recognition():
    # Create a daemon thread to run speech_to_text in the background
    recognition_thread = threading.Thread(target=speech_to_text, daemon=True)
    recognition_thread.start()

# Start speech recognition in the background
start_speech_recognition()

# Main loop
while True:
    # Main program can perform other tasks here
    i = 1 + 1
    print("Main loop:", i)
    time.sleep(5)  # Sleep for 5 seconds before repeating the loop
