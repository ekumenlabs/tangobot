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

import android.content.Context;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialPort;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.util.List;

/**
 * @author jcerruti@ekumenlabs.com (Julian Cerruti)
 */
public abstract class UsbDeviceNodeLoader {
    protected final NodeMainExecutor mNodeMainExecutor;
    protected final URI mRosMasterUri;
    protected final String mRosHostname;

    Log log = LogFactory.getLog(getClass().getName());

    public UsbDeviceNodeLoader(NodeMainExecutor nme, URI rosMasterUri, String rosHostname) {
        mNodeMainExecutor = nme;
        mRosHostname = rosHostname;
        mRosMasterUri = rosMasterUri;
    }

    public abstract NodeMain[] startNodes(UsbDevice device, UsbManager usbManager, Context context) throws Exception;

    /**
     * Internal helper method that returns a single UsbSerialDriver from the android_usb_serial
     * library given a UsbDevice for which we already have access to
     */
    protected UsbSerialDriver serialDriverForDevice(UsbDevice device, UsbManager usbManager) throws Exception {
        // Wrap the UsbDevice in the HoHo Driver
        List<UsbSerialDriver> driverList = UsbSerialProber.getDefaultProber().findAllDrivers(usbManager);
        // For now, continue only if we have a single driver in the list
        if (driverList.isEmpty()) {
            throw new Exception("No drivers found for the supplied USB device: " + device);
        }
        if (driverList.size() > 1) {
            log.warn("There are " + driverList.size() + " drivers found for the provided USB device: "
                    + device + ". Will continue using the first one in the list");
        }
        return driverList.get(0);
    }

    protected UsbDeviceConnection serialConnectionForDevice(UsbManager manager, UsbSerialDriver driver) {
        return manager.openDevice(driver.getDevice());
    }

    protected UsbSerialPort serialPortForDevice(UsbSerialDriver driver) {
        return driver.getPorts().get(0);
    }
}
