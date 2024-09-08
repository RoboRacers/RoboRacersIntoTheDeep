package org.firstinspires.ftc.teamcode.test;

import java.util.LinkedList;
import java.util.Queue;

public class WeightedMovingAverage {
    private final int maxSize;
    private final Queue<Double> window;
    private final double[] weights;
    private double weightedSum;
    private double totalWeights;

    public WeightedMovingAverage(int size, double[] weights) {
        if (size <= 0 || weights == null || weights.length != size) {
            throw new IllegalArgumentException("Size must be greater than 0 and weights array length must match size");
        }
        this.maxSize = size;
        this.window = new LinkedList<>();
        this.weights = weights;
        this.weightedSum = 0.0;
        this.totalWeights = 0.0;
        for (double weight : weights) {
            totalWeights += weight;
        }
    }

    public void add(double value) {
        if (window.size() == maxSize) {
            weightedSum -= window.remove() * weights[maxSize - window.size() - 1];
        }
        window.add(value);
        weightedSum += value * weights[maxSize - window.size()];
    }

    public double getAverage() {
        if (window.isEmpty()) {
            return 0.0; // Or throw an exception based on your requirements
        }
        return weightedSum / totalWeights;
    }

    public int getSize() {
        return window.size();
    }

    public boolean isFull() {
        return window.size() == maxSize;
    }
}
