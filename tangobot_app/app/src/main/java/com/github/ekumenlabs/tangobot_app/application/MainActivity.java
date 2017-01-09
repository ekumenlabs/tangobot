/*
 * Copyright 2015 Ekumen
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.github.ekumenlabs.tangobot_app.application;


import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.SensorManager;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;

import com.github.ekumenlabs.base_controller.BaseControllerNode;
import com.github.ekumenlabs.base_controller.BaseStatusPublisher;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialProber;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.RosCore;
import org.ros.android.RosActivity;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;

import java.net.URI;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

public class MainActivity extends RosActivity
{
    private static final String ACTION_USB_PERMISSION = "com.github.rosjava.android.androidp1.USB_PERMISSION";
    private RosCore myRoscore;
    private Log log = LogFactory.getLog(MainActivity.class);
    private NodeMainExecutor nodeMainExecutor = null;
    private URI masterUri;
    private String hostName;
    private ImuPublisher imuNode;
    final static String appName = "RoLoc_test1";
    private BaseControllerNode baseControllerNode;
    private BaseStatusPublisher baseStatusPublisher;
    // USB
    private UsbManager usbManager;
    private BroadcastReceiver usbAttachedReceiver;
    private BroadcastReceiver usbDetachedReceiver;
    private PendingIntent usbPermissionIntent;
    private Map<UsbDevice, NodeMain[]> usbNodes = new HashMap<UsbDevice, NodeMain[]>();

    private CountDownLatch nodeMainExecutorLatch;

    public MainActivity()
    {
        super(appName, appName);

        //log.info("Creating a ROS master...");

        // create the ROS master listening at the specified port
        //myRoscore = RosCore.newPublic(11311);
        /*myRoscore = RosCore.newPrivate();

        myRoscore.start();
        try {
            myRoscore.awaitStart();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }*/

        nodeMainExecutorLatch = new CountDownLatch(1);
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // Keep the screen always on
        //getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        // UI
        setContentView(R.layout.main);
        // TODO(pablisho) Set UI components

        // USB handling code
        usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        usbPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
        usbAttachedReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                log.info("Received USB Intent");
                if (intent.getAction() == ACTION_USB_PERMISSION &&
                        intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                    onDeviceReady((UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE));
                }
            }
        };
        usbDetachedReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                log.info("Received USB disconnection Intent");
                UsbDevice device = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                onDeviceDetached(device);
            }
        };
      }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor)
    {
        log.info("MainActivity init");

        // Store a reference to the NodeMainExecutor and unblock any processes that were waiting
        // for this to start ROS Nodes
        this.nodeMainExecutor = nodeMainExecutor;
        nodeMainExecutorLatch.countDown();

        masterUri = getMasterUri();
        hostName = getRosHostname();

        log.info(masterUri);

        // Trigger asking permission to access any devices that are already connected
        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
        for (UsbDevice device : manager.getDeviceList().values()) {
            manager.requestPermission(device, usbPermissionIntent);
        }

        // Find the first available driver.
        UsbSerialDriver driver = UsbSerialProber.findFirstDevice(usbManager);


       /* startParameterLoaderNode();
        startImu();
        startLocalization();*/

        // Create ROS node for base move
       /*log.info("Starting move base jni node");
       NodeConfiguration nodeConf4 = NodeConfiguration.newPublic(hostName);
       nodeConf4.setMasterUri(masterUri);
       NativeNodeMain moveBase = new MoveBaseNativeNode();
       nodeMainExecutor.execute(moveBase, nodeConf4);*/
    }

    public void onStart(final ConnectedNode connectedNode)
    {
        // Load start-up parameters
        /*ParameterTree parameterTree = connectedNode.getParameterTree();

        configRobotLocalization(parameterTree);*/

    }

  @Override
  public void onResume() {
    super.onResume();
    registerReceiver(usbAttachedReceiver, new IntentFilter(ACTION_USB_PERMISSION));
    registerReceiver(usbDetachedReceiver, new IntentFilter(
            UsbManager.ACTION_USB_DEVICE_DETACHED));
  }

  @Override
  public void onPause() {
    super.onPause();
    /*unregisterReceiver(usbAttachedReceiver);
    unregisterReceiver(usbDetachedReceiver);
    mTango.disconnect();*/
  }

  @Override
  protected void onNewIntent(Intent intent) {
    super.onNewIntent(intent);
    onUsbDeviceAttached(intent);
  }

  private void onUsbDeviceAttached(Intent intent) {
    if (intent.getAction().equals(UsbManager.ACTION_USB_DEVICE_ATTACHED)) {
        UsbDevice usbDevice = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
        onDeviceReady(usbDevice);
    }
  }

  /**
   * Called when permission has been granted to a device
   * It routes to the appropriate node starting code
   */
  private void onDeviceReady(final UsbDevice device) {
      new Thread() {
          @Override
          public void run() {
              log.info("Connected device: vendor" + device.getVendorId() + "product: " + device.getProductId());
              // Only proceed if the application is ready to start nodes
              try {
                  nodeMainExecutorLatch.await();
              } catch (InterruptedException e) {
                  throw new RuntimeException(e);
              }

              // See what type of device it is and start the appropriate node depending on it
              try {
                  // Start base controller

                  // Dynamically load the corresponding NodeLoader class
                  if (device.getVendorId() == 1027 && device.getProductId() == 24577) {
                      log.info("KOBUKIII :D");
                      // Instantiate it
                      UsbDeviceNodeLoader loader = new KobukiNodeLoader(nodeMainExecutor, getMasterUri(), getRosHostname());
                      log.info("Loader found and instantiated. About to start node");

                      // Create the node, keeping a reference of created nodes to allow shutting
                      // down properly on application shutdown or when the device is disconnected
                      NodeMain[] newUsbNodes = loader.startNodes(device, usbManager);
                      if (newUsbNodes != null) {
                          usbNodes.put(device, newUsbNodes);
                          log.info(newUsbNodes.length + " nodes started");
                      } else {
                          log.info("startNodes returned null");
                      }
                  }
              } catch (Exception e) {
                  log.info("Couldn't start Node for connected device", e);
              }
          }
      }.start();
  }

  /**
   * Called when a USB device has been disconnected
   */
  private void onDeviceDetached(UsbDevice device) {
      NodeMain[] nodeMains = usbNodes.get(device);
      if (nodeMains != null) {
          for (NodeMain nodeMain : nodeMains) {
              // Shutdown this node, considering it has been unplugged
              log.info("Device for node unplugged, shutting down");
              nodeMainExecutor.shutdownNodeMain(nodeMain);
          }
      } else {
          log.info("USB device unplugged but no corresponding node found");
      }
  }

    // Create IMU publisher node
    private void startImu()
    {
        log.info("Starting imu...");

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(hostName);
        SensorManager mSensorManager = (SensorManager)this.getSystemService(SENSOR_SERVICE);

        nodeConfiguration.setMasterUri(masterUri);
        nodeConfiguration.setNodeName("ImuNode");

        imuNode = new ImuPublisher(mSensorManager);

        log.info("About to execute ImuNode...");
        nodeMainExecutor.execute(imuNode, nodeConfiguration);
    }

}
