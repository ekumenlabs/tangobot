package com.github.ekumenlabs.base_driver;

/**
 * @author jcerruti@willowgarage.com (Julian Cerruti)
 */
public class AbstractOdometryStatus implements OdometryStatus {
    // Actual data properties
    protected double poseX;
    protected double poseY;
    protected double poseTheta;
    protected double speedLinearX;
    protected double speedAngularZ;

    // Work variables
    protected int lastLeftTravel;
    protected int lastRightTravel;
    protected boolean haveLastTravel = false;

    private final double WIDTH;

    public AbstractOdometryStatus(double width) {
        WIDTH = width;
    }

    @Override
    public double getPoseX() {
        return poseX;
    }

    @Override
    public double getPoseY() { return poseY; }

    @Override
    public double getPoseTheta() {
        return poseTheta;
    }

    @Override
    public double getSpeedLinearX() {
        return speedLinearX;
    }

    @Override
    public double getSpeedAngularZ() {
        return speedAngularZ;
    }

    /**
     * Updates the currently estimated pose based on the travel and speed of the encoders
     */
    protected void calculateAndUpdate(int leftTravel, int rightTravel, double leftSpeed,
                                      double rightSpeed) {

        // Special case: first time ever we can't calculate differences
        if(!haveLastTravel) {
            lastLeftTravel = leftTravel;
            lastRightTravel = rightTravel;
            haveLastTravel = true;
            return;
        }

        // Calculate deltas
        double dr = ((leftTravel - lastLeftTravel) + (rightTravel - lastRightTravel))/2000.0;
        double da = ((rightTravel - lastRightTravel) - (leftTravel - lastLeftTravel))/(1000.0*WIDTH);
        lastLeftTravel = leftTravel;
        lastRightTravel = rightTravel;

        // Update data
        synchronized (this) {
            this.speedLinearX = (leftSpeed + rightSpeed) / 2000.0;
            speedAngularZ = (rightSpeed - leftSpeed) / (1000.0*WIDTH);
            poseX += dr * Math.cos(poseTheta);
            poseY += dr * Math.sin(poseTheta);
            poseTheta += da;
        }
    }
}
