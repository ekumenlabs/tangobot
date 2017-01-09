/*
 * Copyright (C) 2013 Creativa77.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.ekumenlabs.base_controller;

import com.github.ekumenlabs.base_driver.BaseDevice;
import com.github.ekumenlabs.base_driver.BaseStatus;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;

import std_msgs.Byte;

/**
 * Created by Lucas Chiesa on 10/10/13.
 */
public class BaseStatusPublisher extends AbstractNodeMain {

    private Thread basePublisherThread;

    private final BaseDevice baseDevice;

    private Publisher<std_msgs.Byte> bumperPublisher;
    private Publisher<std_msgs.Byte> wheelDropPublisher;
    private Publisher<std_msgs.Byte> cliffPublisher;
    private Publisher<std_msgs.Byte> chargerPublisher;
    private Publisher<std_msgs.Byte> batteryPublisher;

    private static final Log log = LogFactory.getLog(BaseStatusPublisher.class);

    public BaseStatusPublisher(BaseDevice baseDevice) {
        this.baseDevice = baseDevice;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("mobile_base/state_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        basePublisherThread = new Thread() {
            @Override
            public void run() {
                BaseStatus baseStatus = new BaseStatus();
                try {
                    while(true){
                        baseStatus = baseDevice.getBaseStatus();
                        BaseStatusPublisher.this.publishState(baseStatus);
                        Thread.sleep(10);
                    }
                } catch (Throwable t) {
                    log.error("Exception occurred during state publisher loop.", t);
                }
            }
        };

        bumperPublisher = connectedNode.newPublisher("mobile_base/bumper", "std_msgs/Byte");
        wheelDropPublisher = connectedNode.newPublisher("mobile_base/wheel_drop",  "std_msgs/Byte");
        cliffPublisher = connectedNode.newPublisher("mobile_base/cliff",  "std_msgs/Byte");
        chargerPublisher = connectedNode.newPublisher("mobile_base/charger",  "std_msgs/Byte");
        batteryPublisher = connectedNode.newPublisher("mobile_base/battery",  "std_msgs/Byte");

        basePublisherThread.start();
    }

    private void publishState(BaseStatus baseStatus) {
        Byte bumper = bumperPublisher.newMessage();
        bumper.setData(baseStatus.getBumper());
        bumperPublisher.publish(bumper);

        Byte wheelDrop = wheelDropPublisher.newMessage();
        wheelDrop.setData(baseStatus.getWheelDrop());
        wheelDropPublisher.publish(wheelDrop);

        Byte cliff = cliffPublisher.newMessage();
        cliff.setData(baseStatus.getCliff());
        cliffPublisher.publish(cliff);

        Byte charger = chargerPublisher.newMessage();
        charger.setData(baseStatus.getCharger());
        chargerPublisher.publish(charger);

        Byte battery = batteryPublisher.newMessage();
        battery.setData(baseStatus.getBattery());
        batteryPublisher.publish(battery);
    }

    @Override
    public void onShutdown(Node node) {
        super.onShutdown(node);
    }

    @Override
    public void onShutdownComplete(Node node) {
        super.onShutdownComplete(node);
    }

}
