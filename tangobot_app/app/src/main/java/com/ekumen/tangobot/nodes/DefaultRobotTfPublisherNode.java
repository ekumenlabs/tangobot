/*
 * Copyright 2017 Ekumen, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.ekumen.tangobot.nodes;

import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

/**
 *  This class defines transformations to complete the TF tree with the frames required
 *  by the navigation stack, according to the ones provided by Tango.
 *  TODO: these transformations should be configurable with a YAML file or a similar mechanism.
 */
public class DefaultRobotTfPublisherNode extends ExtrinsicsTfPublisherNode {
    public static final String NODE_NAME = "robot_extrinsics_publisher";

    // This works for the Tango Development Kit tablet, placed over the kit dock as shown
    // in the Tangobot wiki: http://wiki.ros.org/tangobot/tutorials/kinetic/hardware%20setup
    public static final Transform TRANSFORM_DEVKIT = new Transform(
            new Vector3(0, -0.4771263, -0.14950081),
            new Quaternion(-0.41862823, 0.41862823, 0.56986876, 0.56986876));

    // This works for the Lenovo Phab 2 Pro, positioned on a 3D printed mount as shown
    // in the Tangobot wiki: http://wiki.ros.org/tangobot/tutorials/kinetic/hardware%20setup
    public static final Transform TRANSFORM_PHAB2PRO = new Transform(
            new Vector3(0, -0.05, 0.1),
            new Quaternion(0, Math.sqrt(2)/2.0, 0, Math.sqrt(2)/2.0));

    // Identity transform, only for debugging and error cases.
    public static final Transform TRANSFORM_IDENTITY = new Transform(
            new Vector3(0, 0, 0),
            new Quaternion(0, 0, 0, 1));

    // Transformation between device and start of service frames to use.
    private final Transform mDeviceMountTransform;

    /*
     * Two corrections are applied to complement Tango frames, so that a complete transformation between
     * /odom and /base_footprint can be constructed.
     * Odom is a fixed frame in space, which is located in the same coordinates of SOS frame, but rotated -90 degrees in Z axis. That way,
     * the odometry starts with X aligned to the front of the robot instead of Y.
     * The second correction is between the device and the robot. It assumes that the device will be placed over a dock on the
     * top of the robot, and that the dock will be parallel to the robot's front. That way, the Tango device will have its X axis pointing
     * to the right of the robot, Y pointing to the front, and Z pointing to the back (these last two rotated by the dock). Then, the
     * second correction will correct the dock's angle, and the axis orientation difference (robot's front is X axis, not Y).
     * Apart from that, the height at which the device is placed is accounted in both transformations.
     *
     * @param deviceMountTransform  Which transform to use from device to base_footprint. This
     *          depends on the device used and the way it is mounted on the robot. Two constants
     *          TRANSFORM_DEVKIT and TRANSFORM_PHAB2PRO are provided with known transform based on
     *          the Tangobot tutorial.
     */
    public DefaultRobotTfPublisherNode(Transform deviceMountTransform) {
        super();
        mDeviceMountTransform = deviceMountTransform;

        // device --> base_footprint transformation
        // This transformation accounts dock inclination and placement over the robot.
        addTransformation(
                deviceMountTransform,
                "device",
                "base_footprint"
        );
    }
}
