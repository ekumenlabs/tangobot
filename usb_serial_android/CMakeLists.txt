##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(usb_serial_android)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosjava_build_tools)
# Set the gradle targets you want catkin's make to run by default
# e.g. usually catkin_android_setup(assembleRelease uploadArchives)
catkin_android_setup(assembleRelease uploadArchives)
catkin_package()


##############################################################################
# Installation
##############################################################################
