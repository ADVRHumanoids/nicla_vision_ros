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
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Range

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
        picture_topic = rospy.get_param("~picture_topic")

        self.distance_pub = rospy.Publisher(distance_topic, Range, queue_size=5)
        self.picture_pub  = rospy.Publisher(picture_topic, CompressedImage, queue_size=5)

        self.range_msg = Range()
        self.range_msg.header.frame_id = rospy.get_param("~niclabx_distance_tf")
        self.range_msg.radiation_type = Range.INFRARED
        self.range_msg.min_range = 0
        self.range_msg.max_range = 4
        self.range_msg.field_of_view = 15

        self.image_msg = CompressedImage()
        self.image_msg.format = "jpeg"


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
           
            self.image_msg.header.stamp = ros_time
            self.image_msg.data = picture
            self.picture_pub.publish(self.image_msg)

            


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

