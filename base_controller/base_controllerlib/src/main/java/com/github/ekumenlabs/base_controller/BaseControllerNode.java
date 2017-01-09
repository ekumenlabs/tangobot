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

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Subscriber;

import geometry_msgs.Twist;

/**
 * @author jcerruti@creativa77.com.ar (Julian Cerruti)
 */

public class BaseControllerNode extends AbstractNodeMain implements MessageListener<Twist> {
    private final BaseDevice baseDevice;
    private double linearVelX = 0.0;
    private double angVelZ = 0.0;
    private String CMD_VEL_TOPIC;

    private long lastUpdate = 0;

    private static final Log log = LogFactory.getLog(BaseControllerNode.class);
    Thread baseControllerThread;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("android/move_base");
    }

    /**
     * Creates a new base controller node that listens twists
     * in a given topic. Usually this will be cmd_vel.
     * @param vel_topic: The topic in which to listen for twists.
     * @param baseDevice: The base device that wants to be used (Kobuki, Create or Husky for now)
     */
    public BaseControllerNode(BaseDevice baseDevice, String vel_topic) {
        // TODO: Use ROS params to configure topic names
        CMD_VEL_TOPIC = vel_topic;
        this.baseDevice = baseDevice;
    }

    /**
     * Should be called to finish the node initialization. The base driver, already initialize should
     * be provided to this method. This allows to defer the device creation to the moment Android gives
     * the application the required USB permissions.
     */

    @Override
    public void onStart(ConnectedNode connectedNode) {
        log.info("Base controller starting.");

        /**
         * This thread decouples the receiving of messages via ROS Topics from the sending of messages
         * to the base. Most bases require a continuous stream of messages to be sent. This code
         * makes sure to keep sending messages to the base, repeating the previously received message
         * if necessary, to keep a continuous stream going.
         */
        baseControllerThread = new Thread() {
            @Override
            public void run() {
                // thread to constantly send commands to the base
                try {
                    while(true) {
                        if(System.currentTimeMillis() - lastUpdate > 1000){
                            baseDevice.move(0,0);
                            log.info("No cmd vel for 1 s. Stopping...");
                        }else{
                            baseDevice.move(linearVelX, angVelZ);
                        }
                        Thread.sleep(250);
                    }
                } catch (Throwable t) {
                    log.error("Exception occurred during move loop", t);
                    // Whenever we get interrupted out of the loop, for any reason
                    // we try to stop the base, just in case
                    try {
                        setTwistValues(0.0, 0.0);
                        baseDevice.move(0.0, 0.0);
                    } catch(Throwable t0) {
                    }
                }
            }
        };

        // Initialize base.
        baseDevice.initialize();
        baseControllerThread.start();

        // Start base_controller subscriber
        Subscriber<Twist> vel_listener = connectedNode.newSubscriber(CMD_VEL_TOPIC, Twist._TYPE);
        vel_listener.addMessageListener(this);

        log.info("Base controller initialized.");
    }

    @Override
    public void onShutdown(Node node) {
        // TODO: Make sure shutdown is working properly. i.e.: shutdown controller thread
        super.onShutdown(node);
    }

    @Override
    public void onShutdownComplete(Node node) {
        // TODO: Verify shutdown is working properly
        super.onShutdownComplete(node);
    }

    private synchronized void setTwistValues(double linearVelX, double angVelZ) {
        this.linearVelX = linearVelX;
        this.angVelZ = angVelZ;
        this.lastUpdate = System.currentTimeMillis();
        log.info("synchronized setting: (" + this.linearVelX + "," + this.angVelZ + ")");
    }

    /**
     * Callback from the subscriber to the CMD_VEL topic.
     * This method is called each time a command velocity message is received
     *
     * @param twist The command velocity message received
     */
    @Override
    public void onNewMessage(Twist twist) {
        log.info("Current Twist msg: " + twist);
        setTwistValues(twist.getLinear().getX(), twist.getAngular().getZ());
    }
}
