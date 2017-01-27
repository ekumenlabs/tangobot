package com.github.ekumenlabs.tangobot_app.application;



import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.github.c77.base_driver.kobuki.KobukiBaseDevice;
import com.github.c77.base_driver.BaseDevice;

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
