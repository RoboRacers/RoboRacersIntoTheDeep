package org.firstinspires.ftc.teamcode.test;

public class DualWeightedMovingAverage {
    private double imuWeight;
    private double odoWeight;
    private double averageYaw;

    public DualWeightedMovingAverage(double imuWeight, double odoWeight) {
        this.imuWeight = imuWeight;
        this.odoWeight = odoWeight;
        this.averageYaw = 0.0;
    }

    public double updateAverage(double imuYaw, double odoYaw) {
        averageYaw = imuYaw * imuWeight + odoYaw * odoWeight;
        return averageYaw;
    }
}