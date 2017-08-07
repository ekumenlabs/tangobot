/*
 * Copyright 2017 Ekumen, Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.ekumen.tangobot.nodes;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Vector3;

import geometry_msgs.Pose;
import nav_msgs.MapMetaData;
import std_msgs.Header;

/**
 * Class that generates message for an empty map of 10x10 meters.
 * TODO: This should be replaced by a Map Generator that could generate a map grid from an image with metadata.
 */
public class EmptyMapGenerator implements OccupancyGridGenerator {
    private static final int WIDTH = 500;
    private static final int HEIGHT = 500;
    private static final float RESOLUTION = (float) 0.05;

    @Override
    public void fillHeader(Header header) {
        header.setFrameId("map");
        header.setStamp(Time.fromMillis(System.currentTimeMillis()));
    }

    @Override
    public void fillInformation(MapMetaData information) {
        information.setMapLoadTime(Time.fromMillis(System.currentTimeMillis()));
        // Set the origin to the center of the map
        Pose origin = information.getOrigin();
        origin.getPosition().setX(-1 * WIDTH * RESOLUTION / 2);
        origin.getPosition().setY(-1 * HEIGHT * RESOLUTION / 2);
        Quaternion.identity().toQuaternionMessage(origin.getOrientation());
        information.setWidth(WIDTH);
        information.setHeight(HEIGHT);
        information.setResolution(RESOLUTION);
    }

    @Override
    public ChannelBuffer generateData() {
        ChannelBufferOutputStream output = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        try {
            output.write(new byte[WIDTH * HEIGHT]);
        } catch (Exception e) {
            throw new RuntimeException("Empty map generator generateData error: " + e.getMessage());
        }
        return output.buffer();
    }
}
