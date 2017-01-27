package com.github.ekumen.tangobot_app.application;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;

import com.github.c77.base_controller.BaseControllerNode;
import com.github.c77.base_controller.BaseOdomPublisher;
import com.github.c77.base_driver.BaseDevice;
import com.hoho.android.usbserial.driver.UsbSerialDriver;

import org.ros.namespace.GraphName;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.URI;

/**
 * Starts nodes (base controller and odometry publisher) for a robot base which
 * uses the creativa77/base_controller interface
 *
 * @author jcerruti@creativa77.com.ar (Julian Cerruti)
 */
public abstract class AbstractBaseNodeLoader extends UsbDeviceNodeLoader {
    private BaseControllerNode baseControllerNode;
    private BaseOdomPublisher baseOdomPublisher;

    public AbstractBaseNodeLoader(NodeMainExecutor nme, URI rosMasterUri, String rosHostname) {
        super(nme, rosMasterUri, rosHostname);
    }

    protected abstract BaseDevice getBaseDevice(UsbSerialDriver driver) throws Exception;

    @Override
    public NodeMain[] startNodes(UsbDevice baseUsbDevice, UsbManager usbManager) throws Exception {
        if(baseUsbDevice == null) {
            throw new Exception("null USB device provided");
        }
        log.info("Starting base node");

        // Wrap the UsbDevice in the HoHo Driver
        UsbSerialDriver driver = serialDriverForDevice(baseUsbDevice, usbManager);

        // Choose the appropriate BaseDevice implementation for the particular
        // robot base, using the corresponding subclass
        BaseDevice baseDevice = getBaseDevice(driver);

        // Create the ROS nodes
        log.info("Create base controller node");
        baseControllerNode = new BaseControllerNode(baseDevice, "/cmd_vel");
        NodeConfiguration baseControllerNodeConf = NodeConfiguration.newPublic(rosHostname);
        baseControllerNodeConf.setNodeName(GraphName.of("base_controller"));
        baseControllerNodeConf.setMasterUri(rosMasterUri);
        nodeMainExecutor.execute(baseControllerNode, baseControllerNodeConf);


        log.info("Creating odom publisher node");
        baseOdomPublisher = new BaseOdomPublisher(baseDevice);
        NodeConfiguration odomPublisherNodeConf = NodeConfiguration.newPublic(rosHostname);
        odomPublisherNodeConf.setMasterUri(rosMasterUri);
        nodeMainExecutor.execute(baseOdomPublisher, odomPublisherNodeConf);

        return new NodeMain[]{ baseControllerNode, baseOdomPublisher };
    }
}
