cmake_minimum_required(VERSION 3.10.2)
project(nicla_vision_ros)

find_package(catkin REQUIRED COMPONENTS
    audio_common_msgs
    cv_bridge
    rospy
    sensor_msgs
)

find_package(OpenCV)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS  
    audio_common_msgs 
    cv_bridge
    sensor_msgs
)
catkin_python_setup()

###########
## Build ##
###########


#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

catkin_install_python(PROGRAMS
  scripts/nicla_receiver_ros
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/tof_on_image
  scripts/tf_on_image
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




