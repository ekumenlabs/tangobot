package com.ekumen.tangobot.nodes;

import android.content.Intent;
import android.os.BatteryManager;

import com.ekumen.base_driver.BaseDevice;
import com.google.common.base.Preconditions;
import com.google.common.primitives.UnsignedBytes;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import diagnostic_msgs.DiagnosticArray;
import diagnostic_msgs.DiagnosticStatus;
import diagnostic_msgs.KeyValue;


public class BatteryPublisherNode extends AbstractNodeMain {
    public static final String NODE_NAME = "battery_publisher";
    private static final String TOPIC_NAME = "/diagnostics_agg";

    private static final double BASE_MAX_VOLTAGE = 16.7;
    private static final double BASE_MIN_VOLTAGE = 14.0;

    private final BaseDevice mBaseDevice;
    private final Intent mBatteryIntent;
    private Publisher<DiagnosticArray> mBatteryPublisher;
    private NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
    private MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
    private List<KeyValue> robotKeyValueList;
    private List<KeyValue> deviceKeyValueList;

    public BatteryPublisherNode(BaseDevice baseDevice, Intent batteryIntent) {
        Preconditions.checkNotNull(baseDevice);
        Preconditions.checkNotNull(batteryIntent);
        this.mBaseDevice = baseDevice;
        this.mBatteryIntent = batteryIntent;
        robotKeyValueList = new ArrayList<KeyValue>();
        deviceKeyValueList = new ArrayList<KeyValue>();
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        final Log log = connectedNode.getLog();
        mBatteryPublisher = connectedNode.newPublisher(TOPIC_NAME, DiagnosticArray._TYPE);
        final DiagnosticArray diagnosticArray = mBatteryPublisher.newMessage();
        final DiagnosticStatus robotBattery = mMessageFactory.newFromType(DiagnosticStatus._TYPE);
        robotBattery.setName("/Power System/Battery");
        final KeyValue robotKeyValueCapacity = mMessageFactory.newFromType(KeyValue._TYPE);
        robotKeyValueCapacity.setKey("Capacity (Ah)");
        robotKeyValueCapacity.setValue("100.0");
        final KeyValue robotKeyValueCharge = mMessageFactory.newFromType(KeyValue._TYPE);
        robotKeyValueCharge.setKey("Charge (Ah)");
        robotKeyValueList.add(robotKeyValueCapacity);
        robotKeyValueList.add(robotKeyValueCharge);
        robotBattery.setValues(robotKeyValueList);

        final DiagnosticStatus deviceBattery = mMessageFactory.newFromType(DiagnosticStatus._TYPE);
        deviceBattery.setName("/Power System/Laptop Battery");
        KeyValue deviceKeyValueCapacity = mMessageFactory.newFromType(KeyValue._TYPE);
        deviceKeyValueCapacity.setKey("Capacity (Ah)");
        deviceKeyValueCapacity.setValue("100.0");
        final KeyValue deviceKeyValueCharge = mMessageFactory.newFromType(KeyValue._TYPE);
        deviceKeyValueCharge.setKey("Charge (Ah)");
        deviceKeyValueList.add(deviceKeyValueCapacity);
        deviceKeyValueList.add(deviceKeyValueCharge);
        deviceBattery.setValues(deviceKeyValueList);

        diagnosticArray.setStatus(Arrays.asList(robotBattery, deviceBattery));

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {
                diagnosticArray.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
                double baseBatteryVoltage = UnsignedBytes.toInt(mBaseDevice.getBaseStatus().getBattery()) * 0.1;
                double baseBatteryPercentage = (baseBatteryVoltage - BASE_MIN_VOLTAGE) / (BASE_MAX_VOLTAGE - BASE_MIN_VOLTAGE) * 100.0;
                robotKeyValueCharge.setValue(Double.toString(baseBatteryPercentage));

                double deviceBatteryPercentage = getDeviceBatteryLevel();
                if (deviceBatteryPercentage > 0) {
                    deviceKeyValueCharge.setValue(Double.toString(deviceBatteryPercentage));
                }

                mBatteryPublisher.publish(diagnosticArray);
                Thread.sleep(1000);
            }
        });
    }

    double getDeviceBatteryLevel() {
        int rawlevel = mBatteryIntent.getIntExtra(BatteryManager.EXTRA_LEVEL, -1);
        double scale = mBatteryIntent.getIntExtra(BatteryManager.EXTRA_SCALE, -1);
        double level = -1;
        if (rawlevel >= 0 && scale > 0) {
            level = rawlevel / scale * 100;
        }
        return level;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(NODE_NAME);
    }
}
