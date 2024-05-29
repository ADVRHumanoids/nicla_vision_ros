cmake_minimum_required(VERSION 3.10.2)
project(nicla_vision_ros)

find_package(catkin REQUIRED COMPONENTS
    rospy
    sensor_msgs
    audio_common_msgs
    cv_bridge
)

find_package(OpenCV)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########


#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/nicla_receiver_ros.py
  scripts/NiclaReceiverUDP.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

