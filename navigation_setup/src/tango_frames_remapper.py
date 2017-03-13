#!/usr/bin/env python  
import roslib
import rospy
import numpy
import math
import tf
import geometry_msgs.msg
from tf import transformations as t
import sys

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

def main(argv):
    rospy.init_node('tango_tf_remapper')

    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(200.0)
    lastCommonTime = 0

    sourceFrame = '/start_of_service'
    childFrame = '/device'
    
    if len(argv) < 2:
        print 'Using default target frames: odom -> base_footprint'
        targetSourceFrame = 'odom'
        targetChildFrame = 'base_footprint'
    else:
        targetSourceFrame = str(argv[0])
        targetChildFrame = str(argv[1])
            
    # Static definitions: theta is the inclination angle [rad] of the dock holding the Tango device.
    theta = 1.267150092972726
    sin_t = numpy.sin(theta)
    cos_t = numpy.cos(theta)
    tx_sos_odom = [[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
    tx_odom_sos = t.inverse_matrix(tx_sos_odom)
    tx_dev_rob = [[0, cos_t, -sin_t, 0], [-1, 0, 0, 0], [0, sin_t, cos_t, 0], [0, 0, 0, 1]]
    tx_rob_dev = t.inverse_matrix(tx_dev_rob)

    while not rospy.is_shutdown():
        try:
            time = listener.getLatestCommonTime(sourceFrame, childFrame)
            if time <> lastCommonTime:
                (trans,rot) = listener.lookupTransform(sourceFrame, childFrame, time)
                
                tx_dev_sos = t.concatenate_matrices(t.translation_matrix(trans), t.quaternion_matrix(rot))
                tx_rob_sos = numpy.dot(tx_dev_sos, tx_rob_dev)
                tx_rob_odom = numpy.dot(tx_sos_odom, tx_rob_sos)
                
                lastCommonTime = time
                sendTransform = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, Exception):
            print 'Got exception'
            continue

        finally:
            if lastCommonTime == 0:
                print 'Continuing..'

        if sendTransform:
            broadcaster.sendTransform(t.translation_from_matrix(tx_rob_odom), 
                t.quaternion_from_matrix(tx_rob_odom),
                rospy.Time.now(),
                targetChildFrame,
                targetSourceFrame)
            sendTransform = False

        rate.sleep()

if __name__ == "__main__":
    main(sys.argv[1:])
