# Nicla Vision ROS package 
:rocket: Check out the ROS2 version: [Nicla Vision ROS2 repository](https://github.com/ADVRHumanoids/nicla_vision_ros2.git)

<img src="https://hhcm.iit.it/image/layout_set_logo?img_id=1354382&t=1715615054101" alt="Powered by IIT HHCM" width="300"/>

-------------------

## Description
This ROS package enables the Nicla Vision board to be ready-to-use in the ROS world!
It allows to stream all the sensor's data of the small form factor board in the corresponding ROS topics:
- **2MP camera** streams on
    - `/nicla/camera/camera_info` 
    - `/nicla/camera/image_raw`
    - `/nicla/camera/image_raw/compressed` 
- **Time-of-Flight (distance) sensor** streams on:
    - `/nicla/tof`
- **Microphone** streams on:
    - `/nicla/audio` 
    - `/nicla/audio_info`
    - `/nicla/audio_stamped`
- **Imu** streams on:
    - `/nicla/imu`

The implemented architecture is described in the below image:
![Alt Text](assets/Nicla_ROSpkg_Architecture.png)

The user can choose if this package should receive the sensor's data by UDP or TCP socket connection, providing just an ip address. Moreover, the user can decide which sensor to stream within the ROS environment. 
In this repository you can find the Python code optimised for receiving the data by the board, and subsequently publishing them through ROS topics.

## Table of Contents 
1. [Installation](#installation)
2. [Usage](#usage)
3. [License](#license)
4. [Contact](#contact)
   
-------------------

# Installation
Step-by-step instructions on how to get the ROS package running (tested on ROS noetic).

```bash
$ cd <your_workpace>/src
$ git clone https://github.com/ADVRHumanoids/nicla_vision_ros.git
$ cd <your_workpace>
$ catkin build
$ source <your_workpace>/devel/setup.bash
```

## Usage 
Follow the following two steps for enjoying your Arduino Nicla Vision board with ROS.
### 1. Run the ROS package
- For receiving sensors data from the board and 
    ```bash
    $ roslaunch nicla_vision_ros nicla_receiver.launch receiver_ip:="x.x.x.x" <optional arguments>
    ```
    Set the `receiver_ip` with the ip address of your ROS-running machine.
    You can get it by running the command:
    ```bash
    $ ifconfig
    ```
    and taking the inet address under the enp voice.
  
    Using the `<optional arguments>`, you can decide which sensor to simulate and which socket type, TCP or UDP,  to use (connection_type:="tcp" or "udp").
    
- For simulating the Arduino Nicla Vision in Gazebo and Rviz:
     ```bash
    $ roslaunch nicla_vision_ros nicla_sim.launch <optional arguments>
    ```
    Using the `<optional arguments>`, you can decide if to run the simulation in gazebo or in rviz, and which sensor to simulate (everything set to true as default). 
    
### 2. Arduino Nicla Vision setup
Turn on your Arduino Nicla Vision, after having completed its setup following the steps in the [Nicla Vision Drivers repository](https://github.com/ADVRHumanoids/nicla_vision_drivers.git). 
When you power on your Arduino Nicla Vision, it will automatically connect to the network and it will start streaming to your ROS-running machine .


# Package List
Here some useful links:

- [Nicla Vision Drivers repository](https://github.com/ADVRHumanoids/nicla_vision_drivers.git)
- [Nicla Vision ROS2 repository](https://github.com/ADVRHumanoids/nicla_vision_ros2.git)

# License
Distributed under the Apache-2.0 License. See LICENSE for more information.

# Citation
If you use this work or take inspiration from it, please cite:
```bash
@inproceedings{delbianco2024dagana,
  title={},
  author={},
  booktitle={},
  year={}
}
```
