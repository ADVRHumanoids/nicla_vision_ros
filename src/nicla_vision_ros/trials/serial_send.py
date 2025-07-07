import serial
import time

START_BYTE = b'\xFF'
END_BYTE = b'\xFE'

# Set your correct serial port
ser = serial.Serial('/dev/pts/9', 115200)  # Or /dev/ttyUSB0

# Example: send bytes or strings
while True:
    data = START_BYTE
    ser.write(data)
    print(f"Sent: {data}")
    time.sleep(1)
