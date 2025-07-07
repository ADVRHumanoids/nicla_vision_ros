import serial
import time

ser = serial.Serial('/dev/ttyACM0', timeout=1)
start = time.time()
total = 0

while time.time() - start < 5:  # 5 seconds test
    data = ser.read(100)
    total += len(data)

print(f"Received {total} bytes in 5 seconds")
print(f"Effective rate: {total / 5:.2f} bytes/sec")