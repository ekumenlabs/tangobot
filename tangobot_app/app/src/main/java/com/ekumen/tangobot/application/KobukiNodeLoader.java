package com.ekumen.tangobot.application;



import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.ekumen.base_driver.kobuki.KobukiBaseDevice;
import com.ekumen.base_driver.BaseDevice;

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
    protected BaseDevice getBaseDevice(UsbSerialDriver driver) throws Exception {
        return new KobukiBaseDevice(driver);
    }
}
