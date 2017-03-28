package com.ekumen.tangobot.nodes;

import org.apache.commons.logging.Log;
import org.ros.concurrent.WallTimeRate;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.Quaternion;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

import java.util.ArrayList;
import java.util.List;

import geometry_msgs.TransformStamped;
import std_msgs.Header;
import tf2_msgs.TFMessage;

/*
    First version:
        - Publish hardcoded extrinsics added to a list of transform stamped
    Second version:
        - Use a YAML file and parameter loader node to load configurations into parameter server
        - Retrieve configuration from extrinsics publisher for a given namespace in constructor and
        then iterate through that tree/ list for all transform names available.
        /extrinsics/transform_name_1/frame
        /extrinsics/transform_name_1/child_frame
        /extrinsics/transform_name_1/translation/x
        /extrinsics/transform_name_1/translation/y
        /extrinsics/transform_name_1/translation/z
        /extrinsics/transform_name_1/rotation/qw
        /extrinsics/transform_name_1/rotation/qx
        /extrinsics/transform_name_1/rotation/qy
        /extrinsics/transform_name_1/rotation/qz
 */

public class ExtrinsicsPublisherNode extends AbstractNodeMain {

    public static final String NODE_NAME = "extrinsics_publisher";

    NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();

    protected Publisher<TFMessage> tfPublisher;
    List<TransformStamped> tfsList = new ArrayList<>();
    private TransformStamped tfsOdomToSos;
    private Log mLog;
    private TransformStamped tfsDeviceToRobot;
    private TransformStamped tfsMapToOdom;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        super.onStart(connectedNode);
        mLog = connectedNode.getLog();
        tfPublisher = connectedNode.newPublisher("/tf", TFMessage._TYPE);

        // odom --> start_of_service transformation
        tfsOdomToSos = mMessageFactory.newFromType(TransformStamped._TYPE);
        tfsOdomToSos.getHeader().setFrameId("odom");
        tfsOdomToSos.setChildFrameId("start_of_service");

        Transform tfOdomToSos = new Transform(
                new Vector3(0, 0, 0.5),
                Quaternion.fromAxisAngle(Vector3.zAxis(), -Math.PI/2));
        tfOdomToSos.toTransformMessage(tfsOdomToSos.getTransform());
        tfsList.add(tfsOdomToSos);

        // device --> base_footprint transformation
        tfsDeviceToRobot = mMessageFactory.newFromType(TransformStamped._TYPE);
        tfsDeviceToRobot.getHeader().setFrameId("device");
        tfsDeviceToRobot.setChildFrameId("base_footprint");

        Transform tfDeviceToRobot = new Transform(
                new Vector3(0, -0.4771263, -0.14950081),
                new Quaternion(-0.41862823, 0.41862823, 0.56986876, 0.56986876));
        tfDeviceToRobot.toTransformMessage(tfsDeviceToRobot.getTransform());
        tfsList.add(tfsDeviceToRobot);

        // map --> odom transmformation
        tfsMapToOdom = mMessageFactory.newFromType(TransformStamped._TYPE);
        tfsMapToOdom.getHeader().setFrameId("map");
        tfsMapToOdom.setChildFrameId("odom");

        Transform tfMapToOdom = new Transform(
                new Vector3(5, 5, 0),
                Quaternion.identity());
        tfMapToOdom.toTransformMessage(tfsMapToOdom.getTransform());
        tfsList.add(tfsMapToOdom);

        startPublishing();
    }

    private void startPublishing() {
        try {
            WallTimeRate rate = new WallTimeRate(100);

            while (true) {
                Time time = Time.fromMillis(System.currentTimeMillis());
                for (TransformStamped tfs : tfsList) {
                    Header header = tfs.getHeader();
                    header.setStamp(time);
                }

                TFMessage tfm = tfPublisher.newMessage();
                tfm.setTransforms(tfsList);
                tfPublisher.publish(tfm);
                rate.sleep();
            }

        } catch (Throwable t) {
            mLog.error("Exception occurred in publisher loop", t);
        }
    }
}
