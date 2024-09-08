package org.firstinspires.ftc.teamcode.test;

import java.util.LinkedList;
import java.util.Queue;

public class MovingAverage {
    private final int maxSize;
    private final Queue<Double> window;
    private double sum;

    public MovingAverage(int size) {
        if (size <= 0) {
            throw new IllegalArgumentException("Size must be greater than 0");
        }
        this.maxSize = size;
        this.window = new LinkedList<>();
        this.sum = 0.0;
    }

    public void add(double value) {
        sum += value;
        window.add(value);
        if (window.size() > maxSize) {
            sum -= window.remove();
        }
    }

    public double getAverage() {
        if (window.isEmpty()) {
            return 0.0; // Or throw an exception based on your requirements
        }
        return sum / window.size();
    }

    public int getSize() {
        return window.size();
    }

    public boolean isFull() {
        return window.size() == maxSize;
    }
}
