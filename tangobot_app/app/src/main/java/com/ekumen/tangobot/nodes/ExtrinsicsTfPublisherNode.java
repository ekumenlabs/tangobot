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

import com.google.common.base.Preconditions;

import org.ros.concurrent.CancellableLoop;
import org.ros.concurrent.WallTimeRate;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.util.ArrayList;
import java.util.List;

import geometry_msgs.TransformStamped;
import tf2_msgs.TFMessage;

/**
 * Node that publishes a specified set of transforms between frames at a given rate to /tf.
 */
public class ExtrinsicsTfPublisherNode extends AbstractNodeMain {

    public static final String NODE_NAME = "extrinsics_publisher";

    private NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
    protected MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
    protected Publisher<TFMessage> mTfPublisher;
    private List<TransformStamped> mTfList = new ArrayList<>();
    private int mPublishRate = 100;

    public ExtrinsicsTfPublisherNode() {}

    public ExtrinsicsTfPublisherNode(List<TransformStamped> tfsList, int publishRate) {
        Preconditions.checkNotNull(tfsList);
        mTfList = tfsList;
        setmPublishRate(publishRate);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        mTfPublisher = connectedNode.newPublisher("/tf", TFMessage._TYPE);

        startPublishing(connectedNode);
    }

    /**
     * Add a transform to publish from parentFrame to childFrame.
     * @param transform Transform to publish, containing a translation ({@link Vector3}) and a rotation ({@link Quaternion}).
     * @param parentFrame Name of the parent frame.
     * @param childFrame Name of the child frame.
     * @return Reference to the added transformation; save it to remove it afterwards.
     */
    public Object addTransformation(Transform transform, String parentFrame, String childFrame) {
        Preconditions.checkNotNull(transform);
        Preconditions.checkNotNull(parentFrame);
        Preconditions.checkNotNull(childFrame);

        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        tfs.getHeader().setFrameId(parentFrame);
        tfs.setChildFrameId(childFrame);

        transform.toTransformMessage(tfs.getTransform());
        mTfList.add(tfs);
        return tfs;
    }

    /**
     * Remove transformation from the list to publish.
     * @param tfsIndex Transformation reference to remove.
     */
    public void removeTransformation(Object tfsIndex) {
        mTfList.remove(tfsIndex);
    }

    /**
     * Clears all transformations from the list
     */
    public void removeAllTransformations() {
        mTfList.clear();
    }

    /**
     * Sets publish rate [Hz] for all the previously added transformations.
     * @param publishRate Rate in Hz to publish the transformations.
     */
    public void setmPublishRate(int publishRate) {
        Preconditions.checkArgument(publishRate > 0);
        publishRate = publishRate;
    }

    private void startPublishing(ConnectedNode connectedNode) {
        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                WallTimeRate rate = new WallTimeRate(mPublishRate);

                if (!mTfList.isEmpty()) {
                    Time time = Time.fromMillis(System.currentTimeMillis());
                    for (TransformStamped tfs : mTfList) {
                        tfs.getHeader().setStamp(time);
                    }

                    TFMessage tfm = mTfPublisher.newMessage();
                    tfm.setTransforms(mTfList);
                    mTfPublisher.publish(tfm);
                }
                rate.sleep();
            }
        });
    }
}
