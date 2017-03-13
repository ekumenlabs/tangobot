#!/usr/bin/env python  
import roslib
import rospy
import numpy
import math
import tf
import geometry_msgs.msg
from tf import transformations as t
import tf2_ros

# This script listens to the /tf topic and fetches transformations between 'start_of_service' and 'device' frames.
# It assumes that the Tango device is turned on over the robot, so SOS frame has its Y axis pointing to the front of the robot (and Tango)
# in its initial state.

# Two corrections are applied, and the transformation is republished from odom to base_footprint.
# Odom is a fixed frame in space, which is located in the same coordinates of SOS frame, but rotated 90 degrees in Z axis. That way,
# the odometry starts with X aligned to the front of the robot instead of Y.
# The second correction is between the device and the robot. The script assumes that the device will be placed over a dock on the 
# top of the robot, and that the dock will be parallel to the robot's front. That way, the Tango device will have its X axis pointing
# to the right of the robot, Y pointing to the front, and Z pointing to the back (these last two rotated by the dock). Then, the
# second correction will correct the dock's angle, and the axis orientation difference (robot's front is X axis, not Y).

# The script waits for a transformation to be ready, performs the mentioned transformations, and then republishes the information to /tf.
# The loop of the script should run faster that the publisher, so that no messages are missed. Each transformation received is published only once.

# Transformations are named using the following convention: tx_child_parent.
# Child = Target
# Parent = Source
# SOS = start of service
# odom = odometry
# dev = device (Tango device)
# rob = robot

# SendTransform: (trans, rot, time, child, parent)

def getTransformFromMatrix(matrix, childFrame, parentFrame):
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = parentFrame
    static_transformStamped.child_frame_id = childFrame
  
    translation = t.translation_from_matrix(matrix)
    static_transformStamped.transform.translation.x = translation[0]
    static_transformStamped.transform.translation.y = translation[1]
    static_transformStamped.transform.translation.z = translation[2]
  
    quat = t.quaternion_from_matrix(matrix)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    
    return static_transformStamped

if __name__ == '__main__':
    rospy.init_node('static_extrinsics_broadcaster')

    listener = tf.TransformListener()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    sourceFrame = '/start_of_service'
    childFrame = '/device'
    rate = rospy.Rate(200)
            
    # Static definitions: theta is the inclination angle [rad] of the dock holding the Tango device.
    theta = 1.267150092972726
    sin_t = numpy.sin(theta)
    cos_t = numpy.cos(theta)
    robot_height = 0.5  # In meters
    v_device_base = [0, 0, -robot_height]
        
    tx_sos_odom = [[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, robot_height], [0, 0, 0, 1]]
    tx_odom_sos = t.inverse_matrix(tx_sos_odom)
    
    
    tx_dev_devStraight = [[0, cos_t, -sin_t, 0], [-1, 0, 0, 0], [0, sin_t, cos_t, 0], [0, 0, 0, 1]]
    tx_devStraight_dev = t.inverse_matrix(tx_dev_devStraight)
    tx_rob_devStraight = t.translation_matrix(v_device_base)
    
    tx_rob_dev = numpy.dot(tx_devStraight_dev, tx_rob_devStraight)
    
    tx_odom_map = [[1, 0, 0, 5], [0, 1, 0, 5], [0, 0, 1, 0], [0, 0, 0, 1]]
    tx_map_odom = t.inverse_matrix(tx_odom_map)

    lastCommonTime = 0
    sendTransform = False

    while not rospy.is_shutdown():
        
        try:
            time = listener.getLatestCommonTime(sourceFrame, childFrame)
            if time <> lastCommonTime:

                lastCommonTime = time
                sendTransform = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, Exception):
            print 'Got exception'
            continue

        finally:
            if lastCommonTime == 0:
                print 'Continuing..'
        
        if sendTransform:
            broadcaster.sendTransform(getTransformFromMatrix(tx_sos_odom, 
                'start_of_service',
                'odom'))

            broadcaster.sendTransform(getTransformFromMatrix(tx_rob_dev, 
                'base_footprint',
                'device'))
                
            broadcaster.sendTransform(getTransformFromMatrix(tx_odom_map,
                'odom',
                'map'))
            
        rate.sleep()
