#!/usr/bin/env python3

import queue
from threading import Thread
import time
import serial

import numpy as np
import cv2

START_BYTE = 0xFF
END_BYTE = 0xFE

IMAGE_TYPE = 0b00
AUDIO_TYPE = 0b01
RANGE_TYPE = 0b10
IMU_TYPE = 0b11

START_BYTE = 0b11111111 # 0xFF
END_BYTE = 0b11111110 # 0xFE

JPEG_FIRST_BYTE = 0xFF
JPEG_SECOND_BYTE = 0xD8
JPEG_SECOND_TO_LAST_BYTE = 0xFF
JPEG_LAST_BYTE = 0xD9


class NiclaReceiverSerial(serial.Serial):

    def __init__(
        self, 
        port='/dev/ttyACM0', 
        baudrate=115200, 
        timeout=0.1,
        enable_range=False,
        enable_image=False,
        enable_audio=False,
        enable_imu=False
    ):
        print("Initializing NiclaReceiverSerial with port:", port,  "baudrate:", baudrate, "timeout:", timeout)
        
        super().__init__(port=port, baudrate=baudrate, timeout=timeout)

        self.enable_range = enable_range
        self.enable_image = enable_image
        self.enable_audio = enable_audio
        self.enable_imu = enable_imu

        if self.enable_range:
            self.range_buffer = queue.Queue(maxsize=10)
        if self.enable_image:
            self.image_buffer = queue.Queue(maxsize=10)
        if self.enable_audio:
            self.audio_buffer = queue.Queue(maxsize=10)
        if self.enable_imu:
            self.imu_buffer = queue.Queue(maxsize=10)

        self.server_thread = None
        self.server_thread_stop = False

        self.last_timestamp_img = None
        self.last_idx_img = None
        self.last_half_img = None

    def serve(self):
        self.server_thread_stop = False
        self.server_thread = Thread(target=self.__handle)
        self.server_thread.start()

    def stop_serve(self):
        print("stopping serve")
        self.server_thread_stop = True
        self.server_thread.join()

    def get_range(self):

        if not self.range_buffer.empty():
            return self.range_buffer.get_nowait()
        else:
            return None

    def get_image(self):
        if not self.image_buffer.empty():
            return self.image_buffer.get_nowait()
        else:
            return None

    def get_audio(self):
        if not self.audio_buffer.empty():
            return self.audio_buffer.get_nowait()
        else:
            return None

    def get_imu(self):
        if not self.imu_buffer.empty():
            return self.imu_buffer.get_nowait()
        else:
            return None
       

    def __handle(self):

        while not self.server_thread_stop:
            
            first_byte_int = int.from_bytes(self.read(1), "little")

            if first_byte_int == START_BYTE:
                size_packet = int.from_bytes(self.read(4), "little")
                packet = self.read(size_packet)
                last_byte_int = int.from_bytes(self.read(1), "little")
                if last_byte_int != END_BYTE:
                    print("Received packet with wrong end byte, skipping...")
                    continue
            else:
                continue

            timestamp = time.time()  # int.from_bytes(packet[4:8], "big")

            data_type = packet[4]
            data = packet[5:]

            if len(data) == 0:
                print("Received packet with data of zero len, skipping...")
                continue

            if data_type == RANGE_TYPE:
                if self.enable_range:
                    try:
                        self.range_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.range_buffer.get()
                        self.range_buffer.put_nowait((timestamp, data))

                else:
                    pass

            elif data_type == IMAGE_TYPE:
                if self.enable_image:

                    timestamp_img = int.from_bytes(packet[0:4], "little")

                    #JPEG standard initial and end bytes check
                    if (data[0] != JPEG_FIRST_BYTE or
                        data[1] != JPEG_SECOND_BYTE or
                        data[-2] != JPEG_SECOND_TO_LAST_BYTE or
                        data[-1] != JPEG_LAST_BYTE):

                        print(f"Received image with wrong jpeg delimiters bytes, skipping it: {data[0]}, {data[1]}, {data[-2]}, {data[-1]}")
                        continue     

                    data_conv = np.asarray(
                        bytearray(data), dtype="uint8"
                    )

                    img_dec = cv2.imdecode(
                        data_conv, cv2.IMREAD_UNCHANGED
                    )

                    #bgr to rgb conversion
                    img_dec = np.dstack(
                        (
                            img_dec[:, :, 2],
                            img_dec[:, :, 1],
                            img_dec[:, :, 0],
                        )
                    )

                    try:
                        self.image_buffer.put_nowait(
                            (timestamp, img_dec)
                        )
                    except queue.Full:
                        self.image_buffer.get()
                        self.image_buffer.put_nowait(
                            (timestamp, img_dec)
                        )

                else:
                    pass

            elif data_type == AUDIO_TYPE:
                if self.enable_audio:
                    try:
                        self.audio_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.audio_buffer.get_nowait()
                        self.audio_buffer.put_nowait((timestamp, data))

                else:
                    pass

            elif data_type == IMU_TYPE:
                if self.enable_imu:
                    try:
                        self.imu_buffer.put_nowait((timestamp, data))
                    except queue.Full:
                        self.imu_buffer.get_nowait()
                        self.imu_buffer.put_nowait((timestamp, data))
                else:
                    pass


