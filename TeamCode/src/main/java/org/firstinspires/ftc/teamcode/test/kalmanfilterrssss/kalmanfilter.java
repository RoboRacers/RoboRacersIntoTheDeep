package org.firstinspires.ftc.teamcode.test.kalmanfilterrssss;

public class kalmanfilter {

    private float processNoise;
    private float measurementNoise;
    private float estimatedValue;
    private float errorCovariance;

    public kalmanfilter(float processNoise, float measurementNoise, float initialEstimate) {
        this.processNoise = processNoise;
        this.measurementNoise = measurementNoise;
        this.estimatedValue = initialEstimate;
        this.errorCovariance = 1.0f; // Initialize with a large uncertainty
    }

    public void predict() {
        // Prediction step: Update the estimate based on the process model
        // (In simple cases, the estimate might not change in the prediction step)// For example, if modeling a constant value, the prediction would be:
        // estimatedValue = estimatedValue;
        // errorCovariance += processNoise;
    }

    public void update(float measurement) {
        // Measurement update step: Incorporate the new measurement
        float kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
        estimatedValue += kalmanGain * (measurement - estimatedValue);
        errorCovariance = (1 - kalmanGain) * errorCovariance;
    }

    public float getEstimate() {
        return estimatedValue;
    }
}