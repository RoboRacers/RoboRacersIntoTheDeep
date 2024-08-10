package org.firstinspires.ftc.teamcode.test.kalmanfilterrssss;

public class kalmanfilter {

    private double processNoise;
    private double measurementNoise;
    private double estimatedValue;
    private double errorCovariance;

    public kalmanfilter(double processNoise, double measurementNoise, double initialEstimate) {
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
        this.estimatedValue = initialEstimate;
        this.errorCovariance = 1.0; // Initialize with a large uncertainty
    }

    public void predict() {
        // Prediction step: Update the estimate based on the process model
        // (In simple cases, the estimate might not change in the prediction step)// For example, if modeling a constant value, the prediction would be:
        // estimatedValue = estimatedValue;
        // errorCovariance += processNoise;
        estimatedValue = estimatedValue;
        errorCovariance += processNoise;
        
    }

    public void update(float measurement) {
        // Measurement update step: Incorporate the new measurement
        double kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
        estimatedValue += kalmanGain * (measurement - estimatedValue);
        errorCovariance = (1 - kalmanGain) * errorCovariance;
    }

    public double getEstimate() {
        return estimatedValue;
    }
}