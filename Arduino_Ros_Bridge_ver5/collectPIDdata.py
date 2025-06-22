import serial  # type: ignore
import time
import csv
import threading
import sys
import termios
import tty
import select
from datetime import datetime  # <-- Add this

# Open the serial port. Adjust the port name and baud rate as needed.
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)  # Give time for the connection to be established

# Open the CSV file for writing data.
csv_filename = 'pid_data.csv'
file = open(csv_filename, mode='w', newline='')
writer = csv.writer(file)
# Write CSV header with an added timestamp field.
writer.writerow(["Timestamp", "TargetLV", "CurrentLV", "ControlSignal1", "VerticalVoltage"])

def read_serial():
    """Continuously read from the serial port and log CSV-formatted data with a timestamp."""
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                values = line.split(',')
                if len(values) == 4:
                    # Get a timestamp with milliseconds.
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # e.g. 16:56:20.123
                    row = [timestamp] + values
                    writer.writerow(row)
                    file.flush()  # Ensure the data is written to disk immediately.
                    print(row)
    except Exception as e:
        print("Serial reading error:", e)

def get_key_press():
    """Continuously monitor for keypresses and send commands via the serial port."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == '1':
                    ser.write(b"m0.5 0.5\n")
                    print("Sent: m0.5 0.5")
                elif key == '2':
                    ser.write(b"m0 0\n")
                    print("Sent: m0 0")
                elif key == '3':
                    ser.write(b"m-0.5 -0.5\n")
                    print("Sent: m-0.5 -0.5")
            time.sleep(0.1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

# Start threads
thread_serial = threading.Thread(target=read_serial, daemon=True)
thread_key = threading.Thread(target=get_key_press, daemon=True)
thread_serial.start()
thread_key.start()

print("Logging PID data to", csv_filename, "with millisecond timestamps and listening for key commands...")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Logging stopped.")
    file.close()
    ser.close()
