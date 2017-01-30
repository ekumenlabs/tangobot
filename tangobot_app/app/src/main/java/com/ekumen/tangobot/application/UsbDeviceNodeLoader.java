package com.ekumen.tangobot.application;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.util.List;

/**
 * @author jcerruti@creativa77.com.ar (Julian Cerruti)
 */
public abstract class UsbDeviceNodeLoader {
    protected final NodeMainExecutor nodeMainExecutor;
    protected final URI rosMasterUri;
    protected final String rosHostname;

    Log log = LogFactory.getLog(getClass().getName());

    public UsbDeviceNodeLoader(NodeMainExecutor nme, URI rosMasterUri, String rosHostname) {
        this.nodeMainExecutor = nme;
        this.rosHostname = rosHostname;
        this.rosMasterUri = rosMasterUri;
    }

    public abstract NodeMain[] startNodes(UsbDevice device, UsbManager usbManager) throws Exception;

    /**
     * Internal helper method that returns a single UsbSerialDriver from the android_usb_serial
     * library given a UsbDevice for which we already have access to
     */
    protected UsbSerialDriver serialDriverForDevice(UsbDevice device, UsbManager usbManager) throws Exception {
        // Wrap the UsbDevice in the HoHo Driver
        List<UsbSerialDriver> driverList = UsbSerialProber.probeSingleDevice(usbManager, device);
        // For now, continue only if we have a single driver in the list
        if(driverList.isEmpty()) {
            throw new Exception("No drivers found for the supplied USB device: " + device);
        }
        if(driverList.size() > 1) {
            log.warn("There are " + driverList.size() + " drivers found for the provided USB device: "
                    + device + ". Will continue using the first one in the list");
        }
        return driverList.get(0);
    }
}
