/*
 * Copyright (C) 2013 Creativa77.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.ekumenlabs.base_driver.husky;

/**
 * Created by Sebastian Garcia Marra on 05/08/13.
 */

import com.github.ekumenlabs.base_driver.BaseDevice;
import com.github.ekumenlabs.base_driver.BaseStatus;
import com.github.ekumenlabs.base_driver.OdometryStatus;
import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class HuskyBaseDevice implements BaseDevice {
    private final long initialTime;

    HuskyPacketReader packetReader = new HuskyPacketReader();
    HuskyOdometryStatus odometryStatus = new HuskyOdometryStatus();

    // Husky low level commands
    private static final byte SOH = (byte) 0xAA;
    private static final byte PROTOCOL_VERSION = (byte) 0x1;
    private static final byte STX = (byte) 0x55;

    // Hardcoded speed (linear and angular) scale and limits
    private static final double SPEED_LIMIT = 100.0;
    private static final double SPEED_SCALE = 100.0;

    private static final Log log = LogFactory.getLog(HuskyBaseDevice.class);

    // Underlying USB-serial driver
    private final UsbSerialDriver serialDriver;

    public BaseStatus getBaseStatus() {
        BaseStatus baseStatus;
        baseStatus = new BaseStatus();
        return baseStatus;
    }

    @Override
    public OdometryStatus getOdometryStatus() {
        return odometryStatus;
    }


    public HuskyBaseDevice(UsbSerialDriver driver) {
        // Initialize timestamp for messages to be written to the Husky base
        initialTime = System.currentTimeMillis();

        // Open and initialize the underlying USB-serial driver
        serialDriver = driver;
        try {
            serialDriver.open();
            serialDriver.setParameters(115200, UsbSerialDriver.DATABITS_8,
                    UsbSerialDriver.STOPBITS_1, UsbSerialDriver.PARITY_NONE);
            log.info("Serial device opened correctly");
        } catch (IOException e) {
            log.error("Error setting up device: " + e.getMessage(), e);
            try {
                serialDriver.close();
            } catch (Throwable t) {
            }
            throw new RuntimeException("Couldn't open USB device driver", e);
        }

        // Listen for USB-serial input events and call updateReceivedData whenever new data
        // is received
        final ExecutorService executorService = Executors.newSingleThreadExecutor();
        SerialInputOutputManager serialInputOutputManager;
        final SerialInputOutputManager.Listener listener =
            new SerialInputOutputManager.Listener() {
                @Override
                public void onRunError(Exception e) {
                }

                @Override
                public void onNewData(final byte[] data) {
                    HuskyBaseDevice.this.updateReceivedData(data);
                }
            };
        serialInputOutputManager = new SerialInputOutputManager(serialDriver, listener);
        executorService.submit(serialInputOutputManager);
    }

    /**
     * Called every time there is new data received through the USB-serial interface
     */
    private void updateReceivedData(final byte[] bytes) {
        HuskyPacket packet = null;
        try {
            // Parse the packet
            packet = packetReader.parse(ByteBuffer.wrap(bytes));

        } catch (HuskyParserException e) {
            log.error("Error parsing incoming packet", e);
        }

        // If we did get a new packet
        if(packet != null) {
            switch(packet.getMessageType()) {

                // It's encoder data: update odometry
                case HuskyPacket.TYPE_ENCODER_DATA:
                    odometryStatus.update(packet.getPayload());
                    break;

                // Ignore the rest of the packets
                default:
                    break;
            }
        }
    }

    /**
     * Initializes the Husky base device
     */
    @Override
    public void initialize() {
        log.info("Initializing");
        // Request the base to publish the encoders value
        sendEncodersRequest();
    }

    /**
     * Move the Husky base device with the given speeds
     * @param linearVelX: linear speed
     * @param angVelZ: rotational speed
     */
    @Override
    public void move(double linearVelX, double angVelZ) {
        // The Husky base takes linear and angular velocities.
        // All we need to do is to scale and limit each value and fit it in the right format
        sendMovementPackage(
                scaleAndLimitSpeed(linearVelX),
                scaleAndLimitSpeed(angVelZ));
    }

    private static int scaleAndLimitSpeed(double speed) {
        return (int)Math.round(Math.max(Math.min(speed * SPEED_SCALE, SPEED_LIMIT), -1.0*SPEED_LIMIT));
    }

    /**
     * Request publishing of encoder information from the Husky base
     */
    private void sendEncodersRequest() {
        byte [] encoderRequestMessage = new byte[] {
            0x00, 0x48, 0x55, 0x0A, 0x00
        };
        write(buildPackage(encoderRequestMessage));
    }

    /**
     * Sends a movement command to the Husky base
     *
     */
    private void sendMovementPackage(int linearSpeed, int angSpeed) {
        int linearAccel = 0x00C8;         // Fixed acceleration of 5[m/s²]
        int MSGType = 0x0204;             // Set velocities using kinematic model

        //Little-endian encoding
        byte[] baseControlMsg = new byte[]{
            (byte) MSGType,
            (byte) (MSGType >> 8),
            STX,
            (byte) linearSpeed,
            (byte) (linearSpeed >> 8),
            (byte) angSpeed,
            (byte) (angSpeed >> 8),
            (byte) linearAccel,
            (byte) (linearAccel >> 8)
        };

        write(buildPackage(baseControlMsg));
    }


    /**
     * Builds a correctly-formatted Husky package
     */
    byte[] buildPackage(byte[] payload) {
        char checksum = 0;
        int payloadLength = payload.length;
        byte[] pkg = new byte[payloadLength + 11];
        long msgTime = System.currentTimeMillis();

        byte Flags = (byte) 0x01;                        // ACK suppressed
        byte Length0 = (byte) (payloadLength + 8);
        byte Length1 = (byte) ~Length0;                  // It always is Length0's complement
        int TimeStamp = (int)msgTime - (int)initialTime; // Set TimeStamp in milliseconds using four bytes

        pkg[0] = SOH;
        pkg[1] = Length0;
        pkg[2] = Length1;
        pkg[3] = PROTOCOL_VERSION;
        pkg[4] = (byte) TimeStamp;
        pkg[5] = (byte) (TimeStamp >> 8);
        pkg[6] = (byte) (TimeStamp >> 16);
        pkg[7] = (byte) (TimeStamp >> 24);
        pkg[8] = Flags;

        for (int i = 9; i < payloadLength + 9; i++) {
            pkg[i] = payload[i - 9];
        }

        checksum = HuskyBaseUtils.checkSum(pkg);
        pkg[pkg.length - 2] = (byte)checksum;
        pkg[pkg.length - 1] = (byte)(checksum >> 8);

        return pkg;
    }

    /**
     * Writes bytes to the underlying device
     * @param command  Byte buffer
     */
    private void write(byte[] command) {
        try {
            serialDriver.write(command, 1000);
        } catch(Throwable t) {
            log.error("Exception writing command: " + HuskyBaseUtils.byteArrayToString(command), t);
        }
    }
}
