package com.github.ekumenlabs.base_driver.kobuki;

import com.github.ekumenlabs.base_driver.AbstractOdometryStatus;
import com.github.ekumenlabs.base_driver.BaseStatus;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

/**
 * @author jcerruti@willowgarage.com (Julian Cerruti)
 */
public class KobukiOdometryStatus extends AbstractOdometryStatus {
    private short lastTimestamp;
    private static final double WIDTH = 0.280; // in m

    Log log = LogFactory.getLog(KobukiOdometryStatus.class);

    public KobukiOdometryStatus() {
        super(WIDTH);
    }

    void update(BaseStatus baseStatus) {
        //log.info("Updating odometry. Left Ticks = " + baseStatus.getLeftDistance() +
        //        ", Right Ticks = " + baseStatus.getRightDistance());

        if (baseStatus.getTimestamp() == lastTimestamp) {
            return;
        }

        // Approximate speed using last known distance and time lapsed
        int timeLapsed = baseStatus.getTimestamp() - lastTimestamp;
        double leftSpeed = haveLastTravel ? (baseStatus.getLeftDistance() - lastLeftTravel) / timeLapsed : 0;
        double rightSpeed = haveLastTravel ? (baseStatus.getRightDistance() - lastRightTravel) / timeLapsed : 0;

        // Calculate new robot pose
        calculateAndUpdate(baseStatus.getLeftDistance(), baseStatus.getRightDistance(),
                leftSpeed, rightSpeed);

        lastTimestamp = baseStatus.getTimestamp();
    }
}
