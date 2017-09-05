/**
 * Copyright 2017 Ekumen, Inc.  All Rights Reserved.
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

package com.ekumen.tangobot.application;


import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.support.v7.app.AppCompatDelegate;
import android.support.v7.widget.Toolbar;
import android.util.Pair;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import com.ekumen.tangobot.loaders.KobukiNodeLoader;
import com.ekumen.tangobot.loaders.UsbDeviceNodeLoader;
import com.ekumen.tangobot.nodes.DefaultMapTfPublisherNode;
import com.ekumen.tangobot.nodes.DefaultRobotTfPublisherNode;
import com.ekumen.tangobot.nodes.DeviceBatteryPublisherNode;
import com.ekumen.tangobot.nodes.EmptyMapGenerator;
import com.ekumen.tangobot.nodes.ExtrinsicsTfPublisherNode;
import com.ekumen.tangobot.nodes.MoveBaseNode;
import com.ekumen.tangobot.nodes.OccupancyGridPublisherNode;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;
import org.ros.android.AppCompatRosActivity;
import org.ros.android.NodeMainExecutorService;
import org.ros.android.NodeMainExecutorServiceListener;
import org.ros.helpers.ParameterLoaderNode;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeListener;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.rosjava_geometry.Transform;

import java.io.ByteArrayInputStream;
import java.net.URI;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

import eu.intermodalics.nodelet_manager.TangoInitializationHelper;
import eu.intermodalics.nodelet_manager.TangoInitializationHelper.DefaultTangoServiceConnection.AfterConnectionCallback;
import eu.intermodalics.nodelet_manager.TangoNodeletManager;
import eu.intermodalics.tango_ros_common.TangoServiceClientNode;
import tango_ros_messages.TangoConnectRequest;


public class MainActivity extends AppCompatRosActivity implements TangoServiceClientNode.CallbackListener {
    private static final String ACTION_USB_PERMISSION = "com.github.rosjava.android.androidp1.USB_PERMISSION";
    public final static String NOTIFICATION_TITLE = "Tangobot";
    public final static String NOTIFICATION_TICKER = "Tangobot application running. Touch here to initiate shutdown.";

    public final static int MAX_TANGO_CONNECTION_TRIES = 50;

    private static final String REQUEST_TANGO_PERMISSION_ACTION = "android.intent.action.REQUEST_TANGO_PERMISSION";
    public static final String EXTRA_KEY_PERMISSIONTYPE = "PERMISSIONTYPE";
    public static final String EXTRA_VALUE_ADF = "ADF_LOAD_SAVE_PERMISSION";
    private static final String EXTRA_VALUE_DATASET = "DATASET_PERMISSION";
    private static final int REQUEST_CODE_ADF_PERMISSION = 111;
    private static final int REQUEST_CODE_DATASET_PERMISSION = 112;
    private static final String TANGO_NAMESPACE = "/tango";
    private boolean mAdfPermissionHasBeenAnswered;
    private boolean mDatasetPermissionHasBeenAnswered;

    private Log mLog = LogFactory.getLog(MainActivity.class);
    private NodeMainExecutor mNodeMainExecutor = null;
    private TextView mUriTextView;

    // Preferences & TangoNodeletManager
    private SharedPreferences mSharedPref;
    private URI mMasterUri;
    private String mHostName;

    // USB
    private UsbManager mUsbManager;
    private UsbDevice mUsbDevice;
    private BroadcastReceiver mUsbAttachedReceiver;
    private BroadcastReceiver mUsbDetachedReceiver;
    private PendingIntent mUsbPermissionIntent;
    private Map<UsbDevice, NodeMain[]> mUsbNodes = new HashMap<UsbDevice, NodeMain[]>();
    private CountDownLatch mUsbDeviceLatch;

    // Nodes
    private TangoNodeletManager mTangoNodeletManager;
    private TangoServiceClientNode mTangoServiceClient;
    private MoveBaseNode mMoveBaseNode;
    private ParameterLoaderNode mParameterLoaderNode;
    private ExtrinsicsTfPublisherNode mRobotExtrinsicsTfPublisherNode;
    private ExtrinsicsTfPublisherNode mMapExtrinsicsTfPublisherNode;
    private OccupancyGridPublisherNode mOccupancyGridPublisherNode;
    private DeviceBatteryPublisherNode mDeviceBatteryPublisherNode;

    // Status
    private ModuleStatusIndicator mRosMasterStatusIndicator;
    private ModuleStatusIndicator mTangoStatusIndicator;
    private ModuleStatusIndicator mRosParametersStatusIndicator;
    private ModuleStatusIndicator mRosNavigationStatusIndicator;
    private ModuleStatusIndicator mMobileBaseStatusIndicator;

    // Resources
    private static ArrayList<Pair<Integer, String>> mResourcesToLoad = new ArrayList<Pair<Integer, String>>() {{
        add(new Pair<>(R.raw.costmap_common_params, MoveBaseNode.NODE_NAME + "/local_costmap"));
        add(new Pair<>(R.raw.costmap_common_params, MoveBaseNode.NODE_NAME + "/global_costmap"));
        add(new Pair<>(R.raw.local_costmap_params, MoveBaseNode.NODE_NAME + "/local_costmap"));
        add(new Pair<>(R.raw.global_costmap_params, MoveBaseNode.NODE_NAME + "/global_costmap"));
        add(new Pair<>(R.raw.dwa_local_planner_params, MoveBaseNode.NODE_NAME + "/DWAPlannerROS"));
        add(new Pair<>(R.raw.move_base_params, MoveBaseNode.NODE_NAME));
        add(new Pair<>(R.raw.global_planner_params, MoveBaseNode.NODE_NAME + "/GlobalPlanner"));
        add(new Pair<>(R.raw.navfn_global_planner_params, MoveBaseNode.NODE_NAME + "/NavfnROS"));
        add(new Pair<>(R.raw.tango_node_params, TANGO_NAMESPACE));
    }};

    private ArrayList<ParameterLoaderNode.Resource> mOpenedResources = new ArrayList<>();

    ServiceConnection mTangoServiceConnection = new TangoInitializationHelper.DefaultTangoServiceConnection(
        new AfterConnectionCallback() {
            @Override
            public void execute() {
                if (TangoInitializationHelper.isTangoServiceBound()) {
                    mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.LOADING);
                    mLog.info("Bound to Tango Service");
                } else {
                    mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                    mLog.error(getString(R.string.tango_bind_error));
                    displayToastMessage(R.string.tango_bind_error);
                    onDestroy();
                }
            }
        });
    private CountDownLatch mRosConnectionlatch;

    public MainActivity() {
        super(NOTIFICATION_TICKER, NOTIFICATION_TITLE, SettingsActivity.class, MASTER_CHOOSER_REQUEST_CODE);
        mUsbDeviceLatch = new CountDownLatch(1);
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        AppCompatDelegate.setCompatVectorFromResourcesEnabled(true);

        // Load raw resources
        for (Pair<Integer, String> ip : mResourcesToLoad) {
            mOpenedResources.add(new ParameterLoaderNode.Resource(
                    getResources().openRawResource(ip.first.intValue()), ip.second));
        }
        addRuntimeParameters();

        mSharedPref = PreferenceManager.getDefaultSharedPreferences(getBaseContext());

        // UI
        initializeUI();

        // USB handling code
        mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        mUsbPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
        mUsbAttachedReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                mLog.info("Received USB Intent");
                if (intent.getAction() == ACTION_USB_PERMISSION &&
                        intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                    onDeviceReady((UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE));
                }
            }
        };
        mUsbDetachedReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                mLog.info("Received USB disconnection Intent");
                UsbDevice device = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                onDeviceDetached(device);
            }
        };
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu, menu);
        return true;
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        mLog.info("MainActivity init");

        // Store a reference to the NodeMainExecutor
        mNodeMainExecutor = nodeMainExecutor;
        getTangoPermission(EXTRA_VALUE_ADF, REQUEST_CODE_ADF_PERMISSION);
        getTangoPermission(EXTRA_VALUE_DATASET, REQUEST_CODE_DATASET_PERMISSION);
    }

    protected void startNodes() {
        this.nodeMainExecutorService.addListener(new NodeMainExecutorServiceListener() {
            @Override
            public void onShutdown(NodeMainExecutorService nodeMainExecutorService) {
                unbindFromTango();
                // This ensures to kill the process started by the app.
                android.os.Process.killProcess(android.os.Process.myPid());
            }
        });

        mMasterUri = getMasterUri();
        mHostName = getRosHostname();

        mLog.info(mMasterUri);
        updateMasterUriUI(mMasterUri.toString());

        checkRosMasterConnection();
        configureParameterServer();

        startBaseControllerNode();
        startDeviceBatteryPublisherNode();

        startExtrinsicsPublisherNodes();
        startMapServerNode();

        // Start Tango node and navigation stack.
        startTangoRosNode();
        startMoveBaseNode();

        requestUSBPermission();
    }

    /**
     * Requests USB permissions for connected device.
     * This method triggers a task that scans the USB port in the background
     * to support connections after this method has been called the first time.
     */
    private void requestUSBPermission() {
        new Thread() {
            @Override
            public void run() {
                // Trigger asking permission to access any devices that are already connected
                boolean permissionsTriggered = false;

                while (permissionsTriggered == false) {
                    mLog.info("Triggering USB permissions");
                    UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
                    if (manager.getDeviceList().values().isEmpty()) {
                        mLog.info("No USB device found, sleeping and retrying in a while");
                        try {
                            Thread.sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        continue;
                    }

                    for (UsbDevice device : manager.getDeviceList().values()) {
                        mLog.info("Requesting permission for device " + device.toString());
                        manager.requestPermission(device, mUsbPermissionIntent);
                        permissionsTriggered = true;
                    }
                }
            }
        }.start();
    }

    /**
     * Attempts a connection to the configured ROS Master URI, handling status lights.
     * Blocks this thread if the connection is not successful.
     */
    private void checkRosMasterConnection() {
        mRosMasterStatusIndicator.updateStatus(ModuleStatusIndicator.Status.LOADING);
        mRosConnectionlatch = new CountDownLatch(1);
        new MasterConnectionChecker(mMasterUri.toString(),
                new MasterConnectionChecker.UserHook() {
                    @Override
                    public void onSuccess(Object o) {
                        if (o != null) {
                            ((CountDownLatch) o).countDown();
                        }
                        mLog.info("ROS OK");
                        mRosMasterStatusIndicator.updateStatus(ModuleStatusIndicator.Status.OK);
                        displayToastMessage(R.string.ros_init_ok);
                    }

                    @Override
                    public void onError(Throwable t) {
                        mLog.info("ROS init error");
                        mRosMasterStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                        displayToastMessage(R.string.ros_init_error);
                    }},
                mRosConnectionlatch
        ).runTest();
        waitForLatchUnlock(mRosConnectionlatch, "ROS");
    }

    /**
     * Starts {@link ParameterLoaderNode} and waits for it to finish setting parameters.
     */
    private void configureParameterServer() {
        CountDownLatch latch = new CountDownLatch(1);
        startParameterLoaderNode(latch);
        waitForLatchUnlock(latch, "parameter");
    }

    private void startParameterLoaderNode(final CountDownLatch latch) {
        // Create node to load configuration to Parameter Server
        mLog.info("Setting parameters in Parameter Server");
        mRosParametersStatusIndicator.updateStatus(ModuleStatusIndicator.Status.LOADING);
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(mHostName);
        nodeConfiguration.setMasterUri(mMasterUri);
        nodeConfiguration.setNodeName(ParameterLoaderNode.NODE_NAME);
        mParameterLoaderNode = new ParameterLoaderNode(mOpenedResources);
        mNodeMainExecutor.execute(mParameterLoaderNode, nodeConfiguration,
                new ArrayList<NodeListener>() {{
                    add(new DefaultNodeListener() {
                        @Override
                        public void onShutdown(Node node) {
                            latch.countDown();
                            mRosParametersStatusIndicator.updateStatus(ModuleStatusIndicator.Status.OK);
                        }

                        @Override
                        public void onError(Node node, Throwable throwable) {
                            mLog.error("Error loading parameters to ROS parameter server: " + throwable.getMessage(), throwable);
                            mRosParametersStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                        }
                    });
                }});
    }

    private void startMoveBaseNode() {
        // Create ROS node for base move
        mLog.info("Starting move base native node");
        mRosNavigationStatusIndicator.updateStatus(ModuleStatusIndicator.Status.LOADING);
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(mHostName);
        nodeConfiguration.setMasterUri(mMasterUri);
        nodeConfiguration.setNodeName(MoveBaseNode.NODE_NAME);
        mMoveBaseNode = new MoveBaseNode();
        mNodeMainExecutor.execute(mMoveBaseNode, nodeConfiguration,
                new ArrayList<NodeListener>(){{
                    add(new DefaultNodeListener() {
                        @Override
                        public void onStart(ConnectedNode connectedNode) {
                            mRosNavigationStatusIndicator.updateStatus(ModuleStatusIndicator.Status.OK);
                        }

                        @Override
                        public void onError(Node node, Throwable throwable) {
                            mRosNavigationStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                        }
                    });
                }});
    }

    private void startExtrinsicsPublisherNodes() {
        // Create ROS node for extrinsics publisher node
        mLog.info("Starting extrinsics publishers");
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(mHostName);
        nodeConfiguration.setMasterUri(mMasterUri);
        nodeConfiguration.setNodeName(DefaultMapTfPublisherNode.NODE_NAME);
        mMapExtrinsicsTfPublisherNode = new DefaultMapTfPublisherNode();
        mNodeMainExecutor.execute(mMapExtrinsicsTfPublisherNode, nodeConfiguration);

        nodeConfiguration.setNodeName(DefaultRobotTfPublisherNode.NODE_NAME);
        mRobotExtrinsicsTfPublisherNode = new DefaultRobotTfPublisherNode(getDeviceTransform());
        mNodeMainExecutor.execute(mRobotExtrinsicsTfPublisherNode, nodeConfiguration);
    }

    private void startMapServerNode() {
        // Create ROS node to publish the map
        mLog.info("Starting map server");
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(mHostName);
        nodeConfiguration.setMasterUri(mMasterUri);
        nodeConfiguration.setNodeName(OccupancyGridPublisherNode.NODE_NAME);
        mOccupancyGridPublisherNode = new OccupancyGridPublisherNode(new EmptyMapGenerator());
        mNodeMainExecutor.execute(mOccupancyGridPublisherNode, nodeConfiguration);
    }

    private void startBaseControllerNode() {
        mLog.info("Starting base controller");
        mMobileBaseStatusIndicator.updateStatus(ModuleStatusIndicator.Status.LOADING);
        new Thread() {
            @Override
            public void run() {
                try {
                    // Wait for USB device to be ready, permissions granted and a Kobuki base detected.
                    mUsbDeviceLatch.await();

                    // Instantiate it
                    UsbDeviceNodeLoader loader = new KobukiNodeLoader(mNodeMainExecutor, getMasterUri(), getRosHostname());
                    mLog.info("Loader found and instantiated. About to start node.");

                    // Create the node, keeping a reference of created nodes to allow shutting
                    // down properly on application shutdown or when the device is disconnected
                    NodeMain[] newUsbNodes = loader.startNodes(mUsbDevice, mUsbManager);
                    if (newUsbNodes != null) {
                        mUsbNodes.put(mUsbDevice, newUsbNodes);
                        mLog.info(newUsbNodes.length + " nodes started");
                        mMobileBaseStatusIndicator.updateStatus(ModuleStatusIndicator.Status.OK);
                    } else {
                        mLog.warn("startNodes returned null");
                    }
                } catch (InterruptedException e) {
                    mLog.error("Interrupted while waiting for USB device.", e);
                } catch (Exception e) {
                    mLog.error("Exception launching base controller node.", e);
                }
            }
        }.start();
    }

    private void startDeviceBatteryPublisherNode() {
        mLog.info("Starting device battery publisher");
        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(mHostName);
        nodeConfiguration.setMasterUri(mMasterUri);
        nodeConfiguration.setNodeName(DeviceBatteryPublisherNode.NODE_NAME);
        mDeviceBatteryPublisherNode = new DeviceBatteryPublisherNode(getApplicationContext());
        mNodeMainExecutor.execute(mDeviceBatteryPublisherNode, nodeConfiguration);
    }

    @Override
    public void onResume() {
        super.onResume();
        registerReceiver(mUsbAttachedReceiver, new IntentFilter(ACTION_USB_PERMISSION));
        registerReceiver(mUsbDetachedReceiver, new IntentFilter(
                UsbManager.ACTION_USB_DEVICE_DETACHED));
    }

    @Override
    public void onPause() {
        super.onPause();
        unregisterReceiver(mUsbAttachedReceiver);
        unregisterReceiver(mUsbDetachedReceiver);
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
        // If we got a Kobuki base, save the device pointer and unlock the latch so that the waiting
        // ROS node can be launched
        mLog.info("Connected device: vendor" + device.getVendorId() + "product: " + device.getProductId());
        if (device.getVendorId() == 1027 && device.getProductId() == 24577) {
            mUsbDevice = device;
            mUsbDeviceLatch.countDown();
        }
    }

    /**
     * Called when a USB device has been disconnected
     */
    private void onDeviceDetached(UsbDevice device) {
        NodeMain[] nodeMains = mUsbNodes.get(device);
        if (nodeMains != null) {
            for (NodeMain nodeMain : nodeMains) {
                // Shutdown this node, considering it has been unplugged
                mLog.info("Device for node unplugged, shutting down");
                mNodeMainExecutor.shutdownNodeMain(nodeMain);
            }
        } else {
            mLog.info("USB device unplugged but no corresponding node found");
        }
    }

    @Override
    public void onStart() {
        super.onStart();
    }

    @Override
    public void onStop() {
        super.onStop();
    }

    public void startTangoRosNode() {
        mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.LOADING);

        if (TangoInitializationHelper.loadTangoSharedLibrary() != TangoInitializationHelper.ARCH_ERROR &&
                TangoInitializationHelper.loadTangoRosNodeSharedLibrary() != TangoInitializationHelper.ARCH_ERROR) {

            // Remap topic names from default Tango ROS Node to those used in the standard
            // Turtlebot demos and apps.
            String[] topicMap = {
                    "/tango/laser_scan:=/scan",
                    "/tango/camera/color_1/image_raw/compressed:=/compressed_image"
            };

            mTangoNodeletManager = new TangoNodeletManager(topicMap);
            TangoInitializationHelper.bindTangoService(this, mTangoServiceConnection);
            if (TangoInitializationHelper.isTangoVersionOk()) {
                mLog.info("Tango Core version is supposedly OK, starting Tango node.");

                // ServiceClient node which is responsible for calling ros services.
                mTangoServiceClient = new TangoServiceClientNode();
                mTangoServiceClient.setCallbackListener(this);
                NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(mHostName);
                nodeConfiguration.setMasterUri(mMasterUri);
                nodeConfiguration.setNodeName(mTangoServiceClient.getDefaultNodeName());
                mNodeMainExecutor.execute(mTangoServiceClient, nodeConfiguration);

                // Create and start Tango ROS Node
                nodeConfiguration.setNodeName(TangoNodeletManager.NODE_NAME);
                ArrayList<NodeListener> listeners = new ArrayList<>();
                listeners.add(new DefaultNodeListener() {
                    @Override
                    public void onStart(ConnectedNode connectedNode) {
                        boolean connected = false;
                        try {
                            for (int i = 0; i < MAX_TANGO_CONNECTION_TRIES; i++) {
                                if (mTangoServiceClient.callTangoConnectService(TangoConnectRequest.CONNECT)) {
                                    mLog.info("Successfully connected to Tango");
                                    connected = true;
                                    break;
                                }
                                mLog.warn("Failed to connect to Tango, try " + i);
                                Thread.sleep(200);
                            }
                        } catch (InterruptedException e) {
                            mLog.warn("Tango connection loop interrupted.", e);
                        }
                        if (!connected) {
                            mLog.error("Failed to connect to Tango.");
                            mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                        }
                    }

                    @Override
                    public void onError(Node node, Throwable throwable) {
                        mLog.error("Error running TangoRosNode", throwable);
                        mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                    }
                });
                mNodeMainExecutor.execute(mTangoNodeletManager, nodeConfiguration, listeners);
            } else {
                mLog.error(getString(R.string.tango_version_error));
                mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
                displayToastMessage(R.string.tango_version_error);
            }
        } else {
            mLog.error(getString(R.string.tango_lib_error));
            mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
            displayToastMessage(R.string.tango_lib_error);
        }
    }

    /**
     * Helper method to display a toast message with the given message.
     * @param messageId String id of the message to display.
     */
    private void displayToastMessage(final int messageId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), messageId, Toast.LENGTH_SHORT).show();
            }
        });
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (TangoInitializationHelper.isTangoServiceBound()) {
            mLog.info("Unbinding tango service");
            unbindService(mTangoServiceConnection);
        }

        super.nodeMainExecutorService.forceShutdown();
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.settings:
                Intent settingsActivityIntent = new Intent(this, SettingsActivity.class);
                settingsActivityIntent.putExtra("user_forced_launch", true);
                startActivity(settingsActivityIntent);
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    /**
     * Helper method to configure UI elements in the activity (status indicators, Master URI, toolbar).
     * This method shall be called in {@link #onCreate(Bundle)}.
     */
    private void initializeUI() {
        setContentView(R.layout.main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        mRosMasterStatusIndicator = new ModuleStatusIndicator(this, (ImageView) findViewById(R.id.is_ros_ok_image));
        mTangoStatusIndicator = new ModuleStatusIndicator(this, (ImageView) findViewById(R.id.is_tango_ok_image));
        mRosParametersStatusIndicator = new ModuleStatusIndicator(this, (ImageView) findViewById(R.id.is_config_ok_image));
        mRosNavigationStatusIndicator = new ModuleStatusIndicator(this, (ImageView) findViewById(R.id.is_navigation_ok_image));
        mMobileBaseStatusIndicator = new ModuleStatusIndicator(this, (ImageView) findViewById(R.id.is_base_ok_image));
        String masterUri = mSharedPref.getString(getString(R.string.pref_master_uri_key),
                getResources().getString(R.string.pref_master_uri_default));
        mUriTextView = (TextView) findViewById(R.id.master_uri);
        updateMasterUriUI(masterUri);
    }

    /**
     * Helper method to update the Master URI field in the UI.
     * @param masterUri URI to display.
     */
    private void updateMasterUriUI(final String masterUri) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mUriTextView.setText(masterUri);
            }
        });
    }

    /**
     * Helper method to block the calling thread until the latch is zeroed by some other task.
     * @param latch Latch to wait for.
     * @param latchName Name to be used in log messages for the given latch.
     */
    private void waitForLatchUnlock(CountDownLatch latch, String latchName) {
        try {
            mLog.info("Waiting for " + latchName + " latch release...");
            latch.await();
            mLog.info(latchName + " latch released!");
        } catch (InterruptedException ie) {
            mLog.warn("Warning: continuing before " + latchName + " latch was released");
        }
    }

    /**
     * Chooses a device extrinsics configuration publisher according to the detected device
     * type. NOTE: you want to tailor this for your particular robot configuration.
     */
    public Transform getDeviceTransform() {
        if (Build.DEVICE.equalsIgnoreCase("PB2PRO")) {
            mLog.info("Lenovo Phab2Pro detected. Using default Tangobot tutorial configuration.");
            return DefaultRobotTfPublisherNode.TRANSFORM_PHAB2PRO;
        } else if (Build.DEVICE.equalsIgnoreCase("yellowstone")) {
            mLog.info("Tango Development Kit detected. Using default Tangobot tutorial configuration.");
            return DefaultRobotTfPublisherNode.TRANSFORM_DEVKIT;
        } else if (Build.DEVICE.equalsIgnoreCase("ASUS_A002")) {
            mLog.info("Asus Zenfone AR detected. Using default Tangobot tutorial configuration.");
            return DefaultRobotTfPublisherNode.TRANSFORM_ASUS_ZENFONE;
        } else {
            mLog.warn("Couldn't identify device automatically. Will publish an identity transform; " +
                    "device is "+ Build.DEVICE);
            return DefaultRobotTfPublisherNode.TRANSFORM_IDENTITY;
        }
    }

    /**
     * Triggers intents to get Tango permission.
     * Required in Android N.
     * @param permissionType Permission type string.
     * @param requestCode Activity request code.
     */
    private void getTangoPermission(String permissionType, int requestCode) {
        Intent intent = new Intent();
        intent.setAction(REQUEST_TANGO_PERMISSION_ACTION);
        intent.putExtra(EXTRA_KEY_PERMISSIONTYPE, permissionType);
        startActivityForResult(intent, requestCode);
    }

    /**
     * Unbinds from Tango service.
     */
    private void unbindFromTango() {
        if (TangoInitializationHelper.isTangoServiceBound()) {
            mLog.info("Unbind tango service");
            TangoInitializationHelper.unbindTangoService(this, mTangoServiceConnection);
            mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.ERROR);
        }
    }

    /**
     * Adds resources to be loaded that depend on the device
     * (they cannot be set at compile time statically).
     */
    private void addRuntimeParameters() {
        String androidApiLevel = "android_api_level: " + Integer.toString(Build.VERSION.SDK_INT);
        mLog.info("Adding android API level parameter: " + androidApiLevel);
        mOpenedResources.add(new ParameterLoaderNode.Resource(
                new ByteArrayInputStream(androidApiLevel.getBytes()), TANGO_NAMESPACE));
    }

    /**
     * Handles results from intents (Tango permission requests).
     * Other types of requests are forwarded to parent class.
     * @param requestCode Intent request code.
     * @param resultCode Intent result code.
     * @param data Intent data.
     */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        mLog.info("onActivityResult - request code: " + Integer.toString(requestCode) + " result code: " + Integer.toString(resultCode));

        if (requestCode == REQUEST_CODE_ADF_PERMISSION || requestCode == REQUEST_CODE_DATASET_PERMISSION) {
            if (resultCode == RESULT_CANCELED) {
                // No Tango permissions granted by the user.
                displayToastMessage(R.string.tango_permission_denied);
            }
            if (requestCode == REQUEST_CODE_ADF_PERMISSION) {
                // The user answered the ADF permission popup (the permission has not been necessarily granted).
                mAdfPermissionHasBeenAnswered = true;
            }
            if (requestCode == REQUEST_CODE_DATASET_PERMISSION) {
                // The user answered the dataset permission popup (the permission has not been necessarily granted).
                mDatasetPermissionHasBeenAnswered = true;
            }
            if (mAdfPermissionHasBeenAnswered && mDatasetPermissionHasBeenAnswered) {
                // Both ADF and dataset permissions popup have been answered by the user, the node
                // can start.
                mLog.info("Starting Nodes");
                startNodes();
            }
        } else {
            super.onActivityResult(requestCode, resultCode, data);
        }
    }

    @Override
    public void onSaveMapServiceCallFinish(boolean b, String s, String t, String u) {

    }

    @Override
    public void onTangoConnectServiceFinish(int i, String s) {

    }

    @Override
    public void onTangoDisconnectServiceFinish(int i, String s) {

    }

    @Override
    public void onTangoReconnectServiceFinish(int i, String s) {

    }

    @Override
    public void onGetMapUuidsFinish(List<String> list, List<String> list1) {

    }

    @Override
    public void onTangoStatus(int i) {
        // NOTE: This status value corresponds to SERVICE_CONNECTED, according to tano_ros_node.h
        // https://github.com/Intermodalics/tango_ros/blob/v1.3.0/tango_ros_common/tango_ros_native/include/tango_ros_native/tango_ros_node.h#L116
        mLog.info("onTangoStatus called: code " + Integer.toString(i));
        if (i == 3) {
            mTangoStatusIndicator.updateStatus(ModuleStatusIndicator.Status.OK);
        }
    }

    @Override
    public void onLoadOccupancyGridServiceCallFinish(boolean b, String s, boolean b1, String s1) {

    }
}
