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

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.BatteryManager;

import com.google.common.base.Preconditions;
import com.google.common.util.concurrent.AtomicDouble;

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
 * Publishes the Android device's battery.
 * The published format can be read by Android dashboards that display the battery level graphically.
 */
public class DeviceBatteryPublisherNode extends AbstractNodeMain {
    public static final String NODE_NAME = "battery_publisher";
    private static final String TOPIC_NAME = "/diagnostics_agg";

    private NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
    private MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
    private AtomicDouble mBatteryLevel;

    public DeviceBatteryPublisherNode(Context context) {
        Preconditions.checkNotNull(context);
        mBatteryLevel = new AtomicDouble(-1);
        context.registerReceiver(new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                int level = intent.getIntExtra(BatteryManager.EXTRA_LEVEL, -1);
                if (level > 0) {
                    mBatteryLevel.set(level);
                }
            }
        }, new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Publisher<DiagnosticArray> batteryPublisher = connectedNode.newPublisher(TOPIC_NAME, DiagnosticArray._TYPE);
        final DiagnosticArray diagnosticArray = batteryPublisher.newMessage();

        final DiagnosticStatus deviceBattery = mMessageFactory.newFromType(DiagnosticStatus._TYPE);
        deviceBattery.setName("/Power System/Laptop Battery");
        KeyValue deviceKeyValueCapacity = mMessageFactory.newFromType(KeyValue._TYPE);
        deviceKeyValueCapacity.setKey("Capacity (Ah)");
        deviceKeyValueCapacity.setValue("100.0");
        final KeyValue deviceKeyValueCharge = mMessageFactory.newFromType(KeyValue._TYPE);
        deviceKeyValueCharge.setKey("Charge (Ah)");
        deviceBattery.setValues(Arrays.asList(deviceKeyValueCapacity, deviceKeyValueCharge));

        diagnosticArray.setStatus(Arrays.asList(deviceBattery));

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                diagnosticArray.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));

                double deviceBatteryPercentage = mBatteryLevel.get();
                if (deviceBatteryPercentage > 0) {
                    deviceKeyValueCharge.setValue(Double.toString(deviceBatteryPercentage));
                }

                batteryPublisher.publish(diagnosticArray);
                Thread.sleep(1000);
            }
        });
    }
}