# std_turtlebot
This package helps to install and run a standard turtlebot using a laptop (tested on Ubuntu 16.04 and ROS Kinetic) and a Kinect v1 sensor optionally. The default setup uses an empty map to navigate without AMCL.

## Installation
Clone [tangobot repository](https://github.com/ekumenlabs/tangobot) into a workspace folder. For example:
```
mkdir -p ~/tangobot_ws/src
cd ~/tangobot_ws/src
git clone https://github.com/ekumenlabs/tangobot
```
Install all the package dependencies in case you don't have all the turtlebot dependencies installed already:
```
cd ~/tangobot_ws/src/tangobot/std_turtlebot
sudo apt-get update
rosdep install -r std_turtlebot
```
Finally, build this package and source the workspace script:
```
cd ~/tangobot_ws
catkin_make --pkg std_turtlebot
source devel/setup.bash
```
Note: you should install this package in your computer and your laptop.

## Usage
Connect the laptop to the turtlebot, and to the Kinect sensor (this one is optional; if no sensor is connected the robot will navigate blind).
Then, run the startup script:
```
roslaunch std_turtlebot turtlebot_baseline.launch
```
You are done! Your turtlebot is now running in your laptop, as well as a ROS Master.
You will want to see the turtlebot in your computer and send some target goals. To do so, follow these steps from your computer:
```
export ROS_MASTER_URI=http://TURTLEBOT_IP_ADDRESS:11311
export ROS_IP=YOUR_IP_ADDRESS
roslaunch std_turtlebot view_navigation.launch
```
You can check your computer's IP address by running `ifconfig` in a command line.
You should now see your turtlebot in the middle of an empty map, along with its costmaps and the laser scan. You can send navigation goals using RVIZ.
Optionally, use the launchfile named `view_model.launch` to view the robot model only, without the maps. You can manually add them later.
