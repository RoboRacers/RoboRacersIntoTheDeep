package org.firstinspires.ftc.teamcode.test;

public class KalmanFilterHeading extends kalmanfilter {
    protected double angularVelocity;
    protected double dt; // Time step

    public KalmanFilterHeading(double processNoise, double measurementNoise, double initialEstimate, double dt) {
        super(processNoise, measurementNoise, initialEstimate);
        this.dt = dt;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }

    @Override
    public void predict() {
        // Prediction step: Update the estimate based on the process model
        // Update estimated value based on angular velocity and time step
        estimatedValue += angularVelocity * dt;

        // Update error covariance
        errorCovariance += processNoise * dt;
    }

}