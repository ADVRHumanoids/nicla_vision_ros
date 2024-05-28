# BSD 3-Clause License

# Copyright (c) 2023, Edoardo Del Bianco, Federico Rollo

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



# receiving distance and picture from Arduino Nicla Vision (client)
# and sending them on two ROS messages
# the picture fits into one UDP packet after compression

import socket

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from sensor_msgs.msg import Range
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Server:

    # set maximum packet size
    packet_size = 65540

    def __init__(self) -> None:

        # server address and port (the address of the machine running this code, any available port)
        ip = rospy.get_param("~server_ip")
        port = rospy.get_param("~server_port")

        # server socket init 
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((ip, port))
        rospy.loginfo("Waiting for niclabox to stream on " + ip + " : " + str(port))

        distance_topic = rospy.get_param("~distance_topic")
        self.picture_topic = rospy.get_param("~picture_topic")
        self.image_raw_topic = rospy.get_param("~image_raw_topic")
        camera_info_topic = rospy.get_param("~camera_info_topic")

        self.distance_pub = rospy.Publisher(distance_topic, Range, queue_size=5)
        if self.picture_topic != "":
            self.picture_pub  = rospy.Publisher(self.picture_topic, CompressedImage, queue_size=5)
        if self.image_raw_topic != "":
            self.image_raw_pub  = rospy.Publisher(self.image_raw_topic, Image, queue_size=5)
        self.camera_info_pub  = rospy.Publisher(camera_info_topic, CameraInfo, queue_size=5)

        self.range_msg = Range()
        self.range_msg.header.frame_id = rospy.get_param("~niclabx_distance_tf")
        self.range_msg.radiation_type = Range.INFRARED
        self.range_msg.min_range = 0
        self.range_msg.max_range = 4
        self.range_msg.field_of_view = 15

        self.image_msg = CompressedImage()
        self.image_msg.format = "jpeg"

        self.image_raw_msg = Image()
        self.camera_info_msg = CameraInfo()

    #     self.set_camera_info = rospy.Service('/niclabox/picture/set_camera_info', CameraInfo, self.handle_set_camera_info)

    # def handle_set_camera_info(self, msg):
    #     print(msg)


    def receive_and_ros(self):
        # receive distance value
        packet, _ = self.server.recvfrom(self.packet_size)
        ros_time = rospy.Time.now()

        if len(packet) < 100: # a small packet is the distance
            distance = packet
            distance = int.from_bytes(distance, "big")

            self.range_msg.range = distance
            self.range_msg.header.stamp = ros_time

            self.distance_pub.publish(self.range_msg)


        else:  # a big packet is the picture

            picture = packet

            ### PUBLISH COMPRESSED
            if self.picture_topic != "":
                self.image_msg.header.stamp = ros_time
                self.image_msg.data = picture
                self.picture_pub.publish(self.image_msg)

            
            ### PUBLISH IMG RAW
            if self.image_raw_topic != "":
                # Convert the byte array to a numpy array
                nparr = np.frombuffer(picture, np.uint8)

                # Decode the compressed image
                img_raw = cv2.imdecode(nparr, cv2.IMREAD_COLOR) #NOTE: BGR CONVENTION 

                #To visualize the received img converted in img_raw 
                # cv2.imshow("Visualize img_raw", img_raw)
                # cv2.waitKey(1) 

                # Fill in the header
                self.image_raw_msg.header.stamp = ros_time
                self.image_raw_msg.header.frame_id = "nicla"   

                # Fill in image height and width
                self.image_raw_msg.height = img_raw.shape[0]
                self.image_raw_msg.width = img_raw.shape[1]

                # Fill in encoding
                self.image_raw_msg.encoding = "bgr8"  # Assuming OpenCV returns BGR format

                # Fill in endianness and step
                self.image_raw_msg.is_bigendian = 0  # Not big endian
                self.image_raw_msg.step = img_raw.shape[1] * 3  # Width * number of channels

                # Convert the OpenCV image to ROS Image format using cv_bridge
                bridge = CvBridge()
                try:
                    self.image_raw_msg.data = bridge.cv2_to_imgmsg(img_raw, encoding="bgr8").data
                except CvBridgeError as e:
                    print(e)                

                # Publish the ROS Image message
                self.image_raw_pub.publish(self.image_raw_msg)

            ### PUBLISH CAMERA INFO
            self.camera_info_msg.header.stamp = ros_time
            self.camera_info_pub.publish(self.camera_info_msg)
 
            


if __name__ == "__main__":

    rospy.init_node("niclabox")

    server = Server()

    rate = rospy.Rate(50)

    rospy.loginfo("Starting receiving loop")


    while not rospy.is_shutdown():

        try:
            server.receive_and_ros()

        except OSError as e:
            rospy.logerr(e)

            pass # try again
        
        rate.sleep()

