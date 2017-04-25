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

package com.ekumen.tangobot.loaders;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import com.ekumen.base_controller.BaseControllerNode;
import com.ekumen.base_controller.BaseOdomPublisher;
import com.ekumen.base_driver.BaseDevice;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;

import org.ros.namespace.GraphName;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

/**
 * Starts nodes (base controller and odometry publisher) for a robot base which
 * uses the ekumen/base_controller interface
 *
 * @author jcerruti@ekumenlabs.com (Julian Cerruti)
 */
public abstract class AbstractBaseNodeLoader extends UsbDeviceNodeLoader {
    private BaseControllerNode mBaseControllerNode;
    private BaseOdomPublisher mBaseOdomPublisher;

    public AbstractBaseNodeLoader(NodeMainExecutor nme, URI rosMasterUri, String rosHostname) {
        super(nme, rosMasterUri, rosHostname);
    }

    protected abstract BaseDevice getBaseDevice(UsbSerialPort port, UsbDeviceConnection connection) throws Exception;

    @Override
    public NodeMain[] startNodes(UsbDevice baseUsbDevice, UsbManager usbManager) throws Exception {
        if(baseUsbDevice == null) {
            throw new Exception("null USB device provided");
        }
        log.info("Starting base node");

        // Wrap the UsbDevice in the HoHo Driver
        UsbSerialDriver driver = serialDriverForDevice(baseUsbDevice, usbManager);
        UsbDeviceConnection connection = serialConnectionForDevice(usbManager, driver);

        if (connection == null) {
            throw new Exception("No USB connection available to initialize device");
        }

        UsbSerialPort port = serialPortForDevice(driver);

        // Choose the appropriate BaseDevice implementation for the particular
        // robot base, using the corresponding subclass
        BaseDevice baseDevice = getBaseDevice(port, connection);

        // Create the ROS nodes
        log.info("Create base controller node");
        mBaseControllerNode = new BaseControllerNode(baseDevice, "/cmd_vel");
        NodeConfiguration baseControllerNodeConf = NodeConfiguration.newPublic(mRosHostname);
        baseControllerNodeConf.setNodeName(GraphName.of("base_controller"));
        baseControllerNodeConf.setMasterUri(mRosMasterUri);
        mNodeMainExecutor.execute(mBaseControllerNode, baseControllerNodeConf);

        return new NodeMain[]{mBaseControllerNode, mBaseOdomPublisher};
    }
}
