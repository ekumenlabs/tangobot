package com.ekumen.tangobot.loaders;



import android.hardware.usb.UsbDeviceConnection;

import com.ekumen.base_driver.BaseDevice;
import com.ekumen.base_driver.kobuki.KobukiBaseDevice;
import com.hoho.android.usbserial.driver.UsbSerialPort;

import org.ros.node.NodeMainExecutor;

import java.net.URI;

/**
 * @author jcerruti@creativa77.com.ar (Julian Cerruti)
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
