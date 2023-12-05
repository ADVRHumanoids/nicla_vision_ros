 # NICLABOX
This software allows to use the Arduino Nicla Vision to stream the recorded picture and distance measurement with UDP.

The client is the Arduino Nicla Vision, and it streams to a server that can be any PC running Windows or Linux.

The code is written in Python and it is oprimised for maximum speed of transmission.

## Client setup
We are going to run a MicroPython script on the Arduino Nicla Vision:
- Start your Arduino Nicla Vision and install the latest release firmware ad described [in the official tutorial](https://docs.arduino.cc/tutorials/nicla-vision/getting-started).
- Connect your Arduino Nicla Vision to the PC and open the internal volume.
- Overwrite the default main.py with the one provided in ```client/main.py```.

To connect your Arduino Nicla Vision to the Wi-Fi and stream towards your server:
- Connect the PC that will be running the server to the Wi-Fi and get its IP.
- Open the new main.py in the internal volume of your Arduino Nicla Vision and fill in the following constants: ```ssid = "YourNetworkSSID"```, ```password = "YourNetworkPassword"```, ```ip = "YourServerIP"```
- Save and close, then disconnect the Arduino Nicla Vision from the PC.

Now when you power on your Arduino Nicla Vision it will automatically connect to the network and it will start streaming to the server.


## Teleoperation server
With the file ```scripts/niclabox_server_teleoperation.py``` we provide code that allows to visualise the stream without using ROS. This is especially useful to use the Arduino Nicla Vision in a teleoperation setup, even with a Microsoft Windows machine.

On your server machine, you need to have installed:
- Python with pip
- numpy: ```pip install numpy```
- OpenCV: ```pip install opencv-python```

Get the IP of the server machine, open ```scripts/niclabox_server_teleoperation.py``` and fill in the following constant: ```ip = "YourServerIP"```.

Then run ```scripts/niclabox_server_teleoperation.py``` from a terminal. The distance will be printed in the terminal and the picture will open in a separate window. To close, press the key ```q``` while on the picture resenting the window.

## ROS Server
With the other files, we provide a ROS node that allows to publish ```CompressedImage``` and ```Range``` messages with the received data from the Arduino Nicla Vision.

To install the node, on an Linux Ubuntu machine with ROS 1 installed:
- Clone the repository:
```bash
cd ~/catkin_ws/src
git clone https://github.com/LeoBoticsHub/niclabox.git
```
- Build your worksapce:
```bash
cd ~/catkin_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="niclabox"
```

To run the node:
```bash
roslaunch niclabox niclabox_server server_ip:=YourServerIP
```
- To run without having to pass arguments, change as needed the default settings (most likely the IP of the server and the TF of the distance sensor on the Arduino Nicla Vision) by editing the file ```niclabox/launch/niclabox_server.launch```:
```bash
<arg name="server_ip" default="YourServerIP" />
<arg name="niclabx_distance_tf" default="world" />
```
and then run:
```bash
roslaunch niclabox niclabox_server
```

## License
Distributed under the BSD-3-Clause License. See LICENSE for more information.

## Citation
if you use this work or take inspiration from it, please cite:
```bash
@inproceedings{delbianco2024dagana,
  title={},
  author={},
  booktitle={},
  year={}
}
```

## Authors
This software is provided by:
- [Edoardo Del Bianco](https://github.com/edodelbianco)
- [Federio Rollo](https://github.com/FedericoRollo)
