import sys
from io import StringIO
import ollama
from gtts import gTTS
import playsound
import os

question = ""
suggest_words = "học"

user_prompt = f"""Dưới đây là một hướng dẫn mô tả một nhiệm vụ, kèm theo một đầu vào cung cấp ngữ cảnh bổ sung. Hãy viết một phản hồi phù hợp để hoàn thành yêu cầu.

  ### Hướng dẫn: 
  Dựa vào câu hỏi (có hoặc không) và phần gợi ý cho câu trả lời ở đầu vào để hoàn thiện câu trả lời hoàn chỉnh.

  ### Đầu vào:
  câu hỏi: {question} | gợi ý: {suggest_words}

  ### Phản hồi:
  """

# Redirect standard output to capture printed output
sys.stdout = StringIO()

stream = ollama.chat(
  model='eyewheel_com:latest', 
  messages=[{'role': 'user', 'content': user_prompt}],
  stream=True)

stored_output = ""
for chunk in stream:
  st = chunk['message']['content']
  print(st, end='')
  stored_output += st  

# Get the captured printed output
captured_output = sys.stdout.getvalue()
captured_output= captured_output.replace("*", "")
# Reset standard output
sys.stdout = sys.__stdout__

print(captured_output) 

