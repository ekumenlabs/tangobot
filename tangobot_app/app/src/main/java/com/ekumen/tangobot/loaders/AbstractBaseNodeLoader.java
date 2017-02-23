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
 * @author jcerruti@creativa77.com.ar (Julian Cerruti)
 */
public abstract class AbstractBaseNodeLoader extends UsbDeviceNodeLoader {
    private BaseControllerNode baseControllerNode;
    private BaseOdomPublisher baseOdomPublisher;

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
        baseControllerNode = new BaseControllerNode(baseDevice, "/cmd_vel");
        NodeConfiguration baseControllerNodeConf = NodeConfiguration.newPublic(rosHostname);
        baseControllerNodeConf.setNodeName(GraphName.of("base_controller"));
        baseControllerNodeConf.setMasterUri(rosMasterUri);
        nodeMainExecutor.execute(baseControllerNode, baseControllerNodeConf);

        return new NodeMain[]{ baseControllerNode, baseOdomPublisher };
    }
}
