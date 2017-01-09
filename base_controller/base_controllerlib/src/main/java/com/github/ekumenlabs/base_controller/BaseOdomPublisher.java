package com.github.ekumenlabs.base_controller;

import com.github.ekumenlabs.base_driver.BaseDevice;
import com.github.ekumenlabs.base_driver.OdometryStatus;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;

import java.util.List;

import geometry_msgs.Point;
import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Twist;
import geometry_msgs.Vector3;
import nav_msgs.Odometry;
import std_msgs.Header;
import tf2_msgs.TFMessage;

/**
 * @author jcerruti@willowgarage.com (Julian Cerruti)
 */
public class BaseOdomPublisher extends AbstractNodeMain {
    Thread basePublisherThread;
    private final BaseDevice baseDevice;
    NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
    private Publisher<Odometry> odometryPublisher;
    private Publisher<TFMessage> tfPublisher;
    private TransformStamped odomToBaseLink;
    private TransformStamped baseToLaser;

    private static final Log log = LogFactory.getLog(BaseOdomPublisher.class);

    public BaseOdomPublisher(BaseDevice baseDevice) {
        this.baseDevice = baseDevice;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("mobile_base/odom_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        basePublisherThread = new Thread() {
            @Override
            public void run() {
                OdometryStatus odometryStatus;
                try {
                    while(true){
                        odometryStatus = baseDevice.getOdometryStatus();
                        BaseOdomPublisher.this.publish(odometryStatus);
                        Thread.sleep(100, 0);
                    }
                } catch (Throwable t) {
                    log.error("Exception occurred during state publisher loop.", t);
                }
            }
        };

        odometryPublisher = connectedNode.newPublisher("/odom", "nav_msgs/Odometry");
        tfPublisher = connectedNode.newPublisher("/tf", TFMessage._TYPE);
        odomToBaseLink = mMessageFactory.newFromType(TransformStamped._TYPE);
        baseToLaser = mMessageFactory.newFromType(TransformStamped._TYPE);

        // Set laser transform.  (Static, never changes)
        Header h = baseToLaser.getHeader();
        h.setFrameId("base_link");
        baseToLaser.setChildFrameId("laser_link");
        Transform t = baseToLaser.getTransform();
        Vector3 tr = t.getTranslation();
        Quaternion q = t.getRotation();
        //TODO: Move laser_link tf publisher out of the Odometry Publisher.
        tr.setX(0.235); // For the Husky
        tr.setY(0.135); // For the Husky
        //tr.setX(0);   // For the Kobuky
        //tr.setY(0);   // For the Kobuky
        tr.setZ(0.0);
        q.setW(1.0);
        q.setX(0.0);
        q.setY(0.0);
        q.setZ(0.0);

        // Set constant childFrame
        odomToBaseLink.setChildFrameId("base_link");

        basePublisherThread.start();
    }

    private void publish(OdometryStatus odometryStatus) {
        // Create odomentry message
        Odometry odometryMessage = odometryPublisher.newMessage();

        // Header
        Header header = odometryMessage.getHeader();
        header.setFrameId("odom");
        header.setStamp(Time.fromMillis(System.currentTimeMillis()));

        // Child frame id
        odometryMessage.setChildFrameId("base_link");

        // Pointers to important data fields
        Point position = odometryMessage.getPose().getPose().getPosition();
        Quaternion orientation = odometryMessage.getPose().getPose().getOrientation();
        Twist twist = odometryMessage.getTwist().getTwist();

        // Populate the fields. Synchronize to avoid having the data updated in between gets
        synchronized (odometryStatus) {
            position.setX(odometryStatus.getPoseX());
            position.setY(odometryStatus.getPoseY());
            orientation.setZ(Math.sin(odometryStatus.getPoseTheta()/2.0));
            orientation.setW(Math.cos(odometryStatus.getPoseTheta()/2.0));
            twist.getLinear().setX(odometryStatus.getSpeedLinearX());
            twist.getAngular().setZ(odometryStatus.getSpeedAngularZ());
        }

        // Publish!
        odometryPublisher.publish(odometryMessage);

        // Set odom transform
        odomToBaseLink.setHeader(header);
        Transform t = odomToBaseLink.getTransform();
        Vector3 tr = t.getTranslation();
        tr.setX(position.getX());
        tr.setY(position.getY());
        tr.setZ(position.getZ());
        t.setRotation(orientation);

        // Set laser transform time
        baseToLaser.getHeader().setStamp(header.getStamp());

        // Publish transforms
        TFMessage tfm = tfPublisher.newMessage();
        List<TransformStamped> tfl = tfm.getTransforms();
        tfl.add(odomToBaseLink);
        tfl.add(baseToLaser);
        tfPublisher.publish(tfm);
    }
}
