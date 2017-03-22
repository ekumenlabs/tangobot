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

import android.hardware.usb.UsbDeviceConnection;

import com.ekumen.base_driver.BaseDevice;
import com.ekumen.base_driver.kobuki.KobukiBaseDevice;
import com.hoho.android.usbserial.driver.UsbSerialPort;

import org.ros.node.NodeMainExecutor;

import java.net.URI;

/**
 * @author jcerruti@ekumenlabs.com (Julian Cerruti)
 */
public class KobukiNodeLoader extends AbstractBaseNodeLoader {
    public KobukiNodeLoader(NodeMainExecutor nme, URI rosMasterUri, String rosHostname) {
        super(nme, rosMasterUri, rosHostname);
    }

    @Override
    protected BaseDevice getBaseDevice(UsbSerialPort port, UsbDeviceConnection connection) throws Exception {
        return new KobukiBaseDevice(port, connection);
    }
}
