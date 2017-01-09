package com.github.ekumenlabs.base_driver.husky;

import com.github.ekumenlabs.base_driver.AbstractOdometryStatus;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * @author jcerruti@creativa77.com (Julian Cerruti)
 */
public class HuskyOdometryStatus extends AbstractOdometryStatus {

    // TODO: Allow setting (and load from ROS param in node)
    private static final double WIDTH = 0.55;

    public HuskyOdometryStatus() {
        super(WIDTH);
    }

    public void update(byte[] encoderData) {
        if(encoderData.length != 13) {
            throw new RuntimeException("wrong size encoder data = " + encoderData.length);
        }

        // --------------------------------
        // Parse buffer into encoder travels and speeds
        // --------------------------------
        ByteBuffer buffer = ByteBuffer.wrap(encoderData);
        buffer.order(ByteOrder.LITTLE_ENDIAN);

        // Number of encoders
        // TODO: Verify it's two encoders
        byte nEncoders = buffer.get();
        // Left encoder travel
        int leftTravel = buffer.getInt(1);
        // Right encoder travel
        int rightTravel = buffer.getInt(5);
        // Left encoder speed
        short leftSpeed = buffer.getShort(9);
        // Right encoder speed
        short rightSpeed = buffer.getShort(11);

        // Update the current estimated pose
        calculateAndUpdate(leftTravel, rightTravel, rightSpeed, leftSpeed);
    }
}
