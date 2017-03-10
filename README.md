# tangobot
An Android application integrating Robot platforms (Kobuki, Husky) with Tango and ROS.
The aim of this application is to enable a robot to navigate running all the required code on Android.

# Build Instructions
## Pre requisites
To build the application, we assume a clean installation of Ubuntu 16.04 in combination with ROS Kinetic; it may also work in Ubuntu 14.04 & ROS Indigo.
Check that you have Java installed in your system too; you can follow Rosjava Android [pre-requisite installation instructions](http://wiki.ros.org/android/kinetic/Android%20Studio/Download) if you haven't installed it yet.

### Installing ROS
If you don't have ROS Kinetic installed in your system, check out [this guide](http://wiki.ros.org/kinetic/Installation/Ubuntu). Full desktop installation is recommended.

### Create a catkin workspace
Catkin requires that your source code is placed inside a `src` folder. A good example could be:
```
mkdir -p ~/tangobot_ws/src
cd ~/tangobot_ws/src
git clone https://github.com/ekumenlabs/tangobot
```

### Building roscpp_android
This application uses native code to run. To be able to cross-compile native code to Android devices, the `roscpp_android` environment has to be installed and built.
Follow [this guide](http://wiki.ros.org/android_ndk/Tutorials/BuildingNativeROSPackages) to download and build the environment (it takes a long while on the first build).

## Rosjava Build Tools
Follow [this tutorial](http://wiki.ros.org/rosjava/Tutorials/kinetic/Deb%20Installation) to install Rosjava on your computer. This will install Rosjava build tools.
You may install Rosjava from source instead of using debian packages from [here](http://wiki.ros.org/rosjava/Tutorials/indigo/Source%20Installation).

### Android Studio
This is not strictly required to build the application, but it makes Android SDK installation easier.
* Download Android Studio from [here](https://developer.android.com/studio/index.html), install it, and launch it. Import the "tangobot_app" directory by clicking file -> new -> import.
* From the tools menu, select Android -> SDK manager. Install SDK versions 25 & 22, as well as Android NDK (this should download NDK r13).
* Check whether Android Studio created a local.properties file inside "tangobot_app", and if it contains the proper directories for your recently installed SDK and NDK.
* Add your SDK directory to your `PATH` environment variable: ```export PATH=$PATH:$HOME/Android/Sdk/ndk-bundle``` (assuming the default path where Android Studio installs it). You may optionally add that line to your ~/.bashrc file to add it permanently.
* Inside the directory `tangobot_app/app/src/main/cpp` create a file named `local-properties.mk`. Add a single line to it: `$(call import-add-path, {your path to your roscpp_android environment}/ros-android-ndk/roscpp_android/output)` (i.e. the output folder of your recently installed `roscpp_android` environment. Use the absolute path without the curly braces).

## Build and install the app
Almost there! Once you have all the pre-requisites installed, just go to the root of your catkin workspace and run catkin make:
```
cd ~/tangobot_ws
catkin_make
```
Congratulations! You just built your apk file. You can find it in `tangobot_app/app/build/outputs/apk`. You can install it using `adb install app-debug.apk` from that directory.

# Running the app demo
Run a `roscore` in your computer and connect your device to your robot base using a "USB on the go" cable. The device's main camera should have the same direction of the robot's front. Launch the application, and specify your PC's IP address to connect to the ROS master (you may optionally run the ROS master on the device directly; for this example we will assume that the ROS master is running on your computer). Then, run the visualizer:

```
roslaunch navstack_tools turtlebot_demo_visualizer.launch
```

You should now see the robot in the center of an empty map in RVIZ. You can now send goal poses from there to navigate.
You can turn on/off the point cloud for obstacle avoidance by running 
```
rosrun dynamic_reconfigure dynparam set /tango publish_point_cloud [true/false]
```

### Running the navigation stack in your computer
You may choose to run the navigation stack in your computer optionally. After following the steps described above, run the following command:
```
roslaunch navstack_tools turtlebot_navigation_bringup.launch
```
