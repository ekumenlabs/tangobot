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

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import nav_msgs.OccupancyGrid;

/**
 * Publishes an {@link OccupancyGrid} created by a {@link OccupancyGridGenerator}.
 */
public class OccupancyGridPublisherNode extends AbstractNodeMain {
    public static final String NODE_NAME = "occupancy_grid_publisher";
    private Publisher<OccupancyGrid> mPublisher;
    private OccupancyGridGenerator mGridGenerator;

    public OccupancyGridPublisherNode(OccupancyGridGenerator generator) {
        mGridGenerator = generator;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);

        mPublisher = connectedNode.newPublisher("map", OccupancyGrid._TYPE);
        mPublisher.setLatchMode(true);

        OccupancyGrid message = mPublisher.newMessage();

        mGridGenerator.fillHeader(message.getHeader());
        mGridGenerator.fillInformation(message.getInfo());
        message.setData(mGridGenerator.generateData());
        mPublisher.publish(message);
    }
}