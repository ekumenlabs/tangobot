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

import com.ekumen.base_driver.BaseDevice;
import com.google.common.base.Preconditions;
import com.google.common.primitives.UnsignedBytes;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;

import java.util.Arrays;

import diagnostic_msgs.DiagnosticArray;
import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;

/**
 * Publishes the battery from the robot base obtained via serial communication.
 * The published format can be read by Android dashboards that display the battery level graphically.
 */
public class RobotBatteryPublisherNode extends AbstractNodeMain {
    public static final String NODE_NAME = "robot_battery_publisher";
    private static final String TOPIC_NAME = "/diagnostics_agg";

    private static final double BASE_MAX_VOLTAGE = 16.7;
    private static final double BASE_MIN_VOLTAGE = 14.0;

    private final BaseDevice mBaseDevice;
    private NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
    private MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();

    public RobotBatteryPublisherNode(BaseDevice baseDevice) {
        Preconditions.checkNotNull(baseDevice);
        this.mBaseDevice = baseDevice;
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Publisher<DiagnosticArray> batteryPublisher = connectedNode.newPublisher(TOPIC_NAME, DiagnosticArray._TYPE);
        final DiagnosticArray diagnosticArray = batteryPublisher.newMessage();
        final DiagnosticStatus robotBattery = mMessageFactory.newFromType(DiagnosticStatus._TYPE);
        robotBattery.setName("/Power System/Battery");
        final KeyValue robotKeyValueCapacity = mMessageFactory.newFromType(KeyValue._TYPE);
        robotKeyValueCapacity.setKey("Capacity (Ah)");
        robotKeyValueCapacity.setValue("100.0");
        final KeyValue robotKeyValueCharge = mMessageFactory.newFromType(KeyValue._TYPE);
        robotKeyValueCharge.setKey("Charge (Ah)");
        robotBattery.setValues(Arrays.asList(robotKeyValueCapacity, robotKeyValueCharge));

        diagnosticArray.setStatus(Arrays.asList(robotBattery));

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                diagnosticArray.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
                double baseBatteryVoltage = UnsignedBytes.toInt(mBaseDevice.getBaseStatus().getBattery()) * 0.1;
                double baseBatteryPercentage =
                        (baseBatteryVoltage - BASE_MIN_VOLTAGE) /
                                (BASE_MAX_VOLTAGE - BASE_MIN_VOLTAGE) * 100.0;
                robotKeyValueCharge.setValue(Double.toString(baseBatteryPercentage));

                batteryPublisher.publish(diagnosticArray);
                Thread.sleep(1000);
            }
        });
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }
}