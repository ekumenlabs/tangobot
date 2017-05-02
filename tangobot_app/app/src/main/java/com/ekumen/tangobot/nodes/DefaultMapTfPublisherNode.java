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
 * This class defines a transformation between Map and Odom frames for demo purposes.
 * TODO: these transformations should be configurable with a YAML file or a similar mechanism.
 */
public class DefaultMapTfPublisherNode extends ExtrinsicsTfPublisherNode {
    public static final String NODE_NAME = "map_extrinsics_publisher";

    public DefaultMapTfPublisherNode() {
        super();

        // map --> odom transformation
        // Using a (x,y) = (0,0) transformation assuming a 10x10 empty map.
        addTransformation(
                new Transform(
                        new Vector3(0, 0, 0),
                        Quaternion.identity()),
                "map",
                "odom"
        );
    }
}
