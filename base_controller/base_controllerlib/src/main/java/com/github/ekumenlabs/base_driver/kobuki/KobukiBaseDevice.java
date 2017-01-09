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

package com.github.ekumenlabs.base_driver.kobuki;

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

import org.ros.exception.RosRuntimeException;

public class KobukiBaseDevice implements BaseDevice {

    private final byte SetBaudrate115200 = (byte) 6;
    // Kobuki low level commands
    private final byte Header0 = (byte) 0xAA;
    private final byte Header1 = (byte) 0x55;
    private final byte BaseControl = (byte) 0x01;
    private final byte SoundSequence = (byte) 0x04;

    private final KobukiPacketReader packetReader = new KobukiPacketReader();
    private final KobukiPacketParser packetParser = new KobukiPacketParser();
    private BaseStatus baseStatus = new BaseStatus();
    private KobukiOdometryStatus odometryStatus = new KobukiOdometryStatus();

    private static final Log log = LogFactory.getLog(KobukiBaseDevice.class);

    private final UsbSerialDriver serialDriver;

    private class BaseSpeedValues {
        private final int linearSpeed;
        private final int rotationRadius;

        private BaseSpeedValues(int linearSpeed, int rotationRadius) {
            this.linearSpeed = linearSpeed;
            this.rotationRadius = rotationRadius;
        }

        public int getLinearSpeed() {
            return linearSpeed;
        }

        public int getRotationRadius() {
            return rotationRadius;
        }
    }

    public KobukiBaseDevice(UsbSerialDriver driver) throws Exception {
        if(driver == null) {
            throw new Exception("null USB driver provided");
        }
        serialDriver = driver;
        try {
            serialDriver.open();
            serialDriver.setParameters(115200, UsbSerialDriver.DATABITS_8,
                    UsbSerialDriver.STOPBITS_1, UsbSerialDriver.PARITY_NONE);
        } catch (IOException e) {
            log.info("Error setting up device: " + e.getMessage(), e);
            try {
                serialDriver.close();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
        }

        final ExecutorService executorService = Executors.newSingleThreadExecutor();

        SerialInputOutputManager serialInputOutputManager;

        final SerialInputOutputManager.Listener listener =
            new SerialInputOutputManager.Listener() {
                @Override
                public void onRunError(Exception e) {
                }

                @Override
                public void onNewData(final byte[] data) {
                    KobukiBaseDevice.this.updateReceivedData(data);
                }
            };

        serialInputOutputManager = new SerialInputOutputManager(serialDriver, listener);
        executorService.submit(serialInputOutputManager);
    }

    public BaseStatus getBaseStatus() {
        return baseStatus;
    }

    @Override
    public OdometryStatus getOdometryStatus() {
        return odometryStatus;
    }

    private void updateReceivedData(final byte[] bytes) {
        int readBytes = bytes.length;
        packetReader.newPacket(ByteBuffer.allocateDirect(readBytes).put(bytes, 0, readBytes));
        baseStatus = packetParser.parseBaseStatus(packetReader.getSensorPacket());
        odometryStatus.update(baseStatus);
    }

    public void initialize() {
        byte[] baud = new byte[]{SetBaudrate115200};
        write(baud); // configure base baudrate
    }

    public void move(double linearVelX, double angVelZ) {
        BaseSpeedValues speeds = twistToBase(linearVelX, angVelZ);
        sendMovementPackage(speeds);
    }

    private BaseSpeedValues twistToBase(double linearVelX, double angVelZ) {
        // vx: in m/s
        // wz: in rad/s
        final double epsilon = 0.0001;
        final double bias = 0.23;
        double radius;
        double speed;

        // Special Case #1 : Straight Run
        if (Math.abs(angVelZ) < epsilon) {
            radius = 0.0f;
            speed = 1000.0f * linearVelX;
            return new BaseSpeedValues((int) speed, (int) radius);
        }

        radius = linearVelX * 1000.0f / angVelZ;
        // Special Case #2 : Pure Rotation or Radius is less than or equal to 1.0 mm
        if (Math.abs(linearVelX) < epsilon || Math.abs(radius) <= 1.0f) {
            speed = 1000.0f * bias * angVelZ / 2.0f;
            radius = 1.0f;
            return new BaseSpeedValues((int) speed, (int) radius);
        }

        // General Case :
        if (radius > 0.0f) {
            speed = (radius + 1000.0f * bias / 2.0f) * angVelZ;
        } else {
            speed = (radius - 1000.0f * bias / 2.0f) * angVelZ;
        }
        return new BaseSpeedValues((int) speed, (int) radius);
    }

    private void sendMovementPackage(BaseSpeedValues speeds) {
        int linearSpeed = speeds.getLinearSpeed();
        int rotationRadius = speeds.getRotationRadius();

        byte[] baseControlMsg = new byte[]{
                BaseControl,
                (byte) 0x04,
                (byte) linearSpeed,
                (byte) (linearSpeed >> 8),
                (byte) rotationRadius,
                (byte) (rotationRadius >> 8),
        };

        write(buildPackage(baseControlMsg));
    }

    private void sendSoundPackage(int sound) {
        byte[] soundSequenceMsg = new byte[]{
                SoundSequence,
                (byte) 0x01,
                (byte) sound
        };

        write(buildPackage(soundSequenceMsg));
    }

    byte checkSum(byte[] cmdPackage) {
        byte checksum = 0;

        for (int i = 2; i < (cmdPackage.length - 1); i++) {
            checksum ^= cmdPackage[i];
        }
        return checksum;
    }

    byte[] buildPackage(byte[] payload) {
        int payloadLength = payload.length;
        byte[] pkg = new byte[payloadLength + 4];
        pkg[0] = Header0;
        pkg[1] = Header1;
        pkg[2] = (byte) (payloadLength);
        for (int i = 3; i < payloadLength + 3; i++) {
            pkg[i] = payload[i - 3];
        }
        pkg[pkg.length - 1] = checkSum(pkg);

        return pkg;
    }

    private void write(byte[] command) {
        try {
            log.info("Writing a command to USB Device.");
            serialDriver.write(command, 1000);
        } catch (IOException e) {
            throw new RosRuntimeException(e);
        }
    }
}