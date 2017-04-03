# tangobot
An Android application integrating Robot platforms ([Kobuki](https://kobuki.yujinrobot.com/)) with Tango enabled device ([Yellowstone tablets](https://developers.google.com/tango/hardware/tablet)) and ROS.
The aim of this application is to enable a robot to navigate running all the required code on Android.

# Kickstart
We encourage you to read the robot's [wiki entry](http://wiki.ros.org/Robots/Tangobot) for more information about the required hardware components and detailed setup tutorials. 

# Running the app demo
Follow these steps to get your robot up and navigating in an empty map, using a computer for visualization and for sending goals. We assume you have all the required [hardware components](http://wiki.ros.org/tangobot/tutorials/kinetic/hardware%20setup).

## Pre requisites
You will need to have Ubuntu 14.04 & ROS Indigo or Ubuntu 16.04 & ROS Kinetic. You can check [Installing ROS](https://github.com/ekumenlabs/tangobot#installing-ros) section if you don't have ROS installed.
For these instructions, we will assume ROS Kinetic; replace kinetic by indigo if that's not your case when necessary.

### Software & robot setup
To get the robot up and running without needing to build the app from source, install Tangobot app from Google's Play Store in your device.
You will also need to get a helper package in your computer to launch the visualizer. 

First, clone the Tangobot respository:

```
mkdir -p ~/tangobot_ws/src
cd ~/tangobot_ws/src
git clone https://github.com/ekumenlabs/tangobot
```

Then, install the package dependencies in case you don't have them, and source the environment:
```
cd ~/tangobot_ws
source /opt/ros/kinetic/setup.bash
rosdep update
rosdep install --from-paths src/tangobot/tangobot_desktop
catkin_make --pkg tangobot_desktop
```

Connect your device to your robot base using a "USB on the go" cable. The device's main camera should have the same direction of the robot's front. You can check the robot's wiki article for more information about the [hardware setup](http://wiki.ros.org/tangobot/tutorials/kinetic/hardware%20setup) and how to place the Android device over the robot.

## Starting the demo!
First, start a ROS Master from a separate terminal by running `roscore`. Launch the application in your Android device, and specify your PC's IP address to connect to the ROS master (you may optionally run the ROS master on the device directly; for this example we will assume that the ROS master is running on your computer). 

Before running the visualizer, ensure your network is properly configured. To do so, run the following command (you can check your system's IP address running `ifconfig`):
```
export ROS_IP=YOUR_COMPUTER_IP_ADDRESS
```

Then, run the visualizer:

```
roslaunch tangobot_desktop tangobot_demo_visualizer.launch
```

You should now see the robot in the center of an empty map in RVIZ. You can now send goal poses from there to navigate.

# Building the application from source
## Pre requisites
To build the application, we assume a clean installation of Ubuntu 16.04 in combination with ROS Kinetic; it should also work in Ubuntu 14.04 & ROS Indigo.

### Installing ROS
Run the following commands to install ROS in your system (taken from [this guide](http://wiki.ros.org/kinetic/Installation/Ubuntu)). Full desktop installation is recommended:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install ros-kinetic-rosjava-build-tools
```

### Downloading source code
Clone the repository if you haven't done it already:
```
mkdir -p ~/tangobot_ws/src
cd ~/tangobot_ws/src
git clone https://github.com/ekumenlabs/tangobot
```

### Building roscpp_android
This application uses native code to run. To be able to cross-compile native code to Android devices, the `roscpp_android` environment has to be installed and built.
Run the commands described below (taken from the official [guide](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages)). It will take a long while on the first build:

```
sudo apt-get update
sudo apt-get install curl git
curl -sSL https://get.docker.com/ | sudo sh
mkdir ~/ros-android-ndk
cd ~/ros-android-ndk
git clone https://github.com/ekumenlabs/roscpp_android.git
cd roscpp_android
./do_docker.sh --portable
``` 

### Android Studio
This is the recommended tool to edit the code. It will also help you install the required Android SDKs. We will need to install Java first:
```
sudo apt-get update
sudo apt-get install openjdk-8-jdk
echo "export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64/" >> ~/.bashrc
```
Use openjdk-7 if your are in Ubuntu 14.04.

Then, follow these steps:

* Download Android Studio from [here](https://developer.android.com/studio/index.html), install it, and launch it. 
* Import the "tangobot_app" directory by clicking file -> new -> import (it should be in ~/tangobot_ws/src/tangobot/tangobot_app).
* From the tools menu, select Android -> SDK manager. Install SDK versions 25 & 22, as well as Android NDK (this should download NDK > r13).
* Check whether Android Studio created a local.properties file inside "tangobot_app", and if it contains the proper directories for your recently installed SDK and NDK.
* Add your SDK directory to your `PATH` environment variable: ```export PATH=$PATH:$HOME/Android/Sdk/ndk-bundle``` (assuming the default path where Android Studio installs it). You may optionally add that line to your ~/.bashrc file to add it permanently.
* Inside the directory `tangobot_app/app/src/main/cpp` create a file named `local-properties.mk`. Add a single line to it: `$(call import-add-path, {your path to your roscpp_android environment}/ros-android-ndk/roscpp_android/output)` (i.e. the output folder of your recently installed `roscpp_android` environment. Use the absolute path without the curly braces).

## Build and install the app
Almost there! Once you have all the pre-requisites installed, just go to the root of your catkin workspace and run catkin make:
```
cd ~/tangobot_ws
catkin_make
```
Congratulations! You just built your apk file. You can find it in `tangobot_app/app/build/outputs/apk`. You can install it running `adb install app-debug.apk` from that directory.
