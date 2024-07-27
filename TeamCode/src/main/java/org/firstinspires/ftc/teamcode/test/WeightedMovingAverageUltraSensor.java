package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.LinkedList;
import java.util.Queue;

public class WeightedMovingAverageUltraSensor {

    private static final int MOVING_AVERAGE_WINDOW_SIZE = 250;

    private AnalogInput ultrasonicSensor;
    private Queue<Double> distanceReadings = new LinkedList<>();
    private double weightedDistanceSum = 0;
    private int totalWeight = 0;

    public WeightedMovingAverageUltraSensor(AnalogInput ultrasonicSensor) {
        this.ultrasonicSensor = ultrasonicSensor;
        initializeWeights();
    }

    /**
     * Reads the current distance from the sensor and updates the weighted moving average.
     * @return The current distance in centimeters.
     */
    public double readDistance() {
        double voltage = ultrasonicSensor.getVoltage();
        double distance = voltageToDistance(voltage);

        // Update weighted moving average
        addReading(distance);

        return distance;
    }

    /**
     * Gets the current weighted moving average of the distance readings.
     * @return The weighted moving average distance in centimeters.
     */
    public double getWeightedMovingAverage() {
        if (distanceReadings.isEmpty()) {
            return 0;
        }
        return weightedDistanceSum / totalWeight;
    }

    /**
     * Converts the voltage from the MaxBotix sensor to a distance in centimeters.
     * This conversion assumes a specific MaxBotix sensor model. Adjust the conversion
     * factor based on your sensor's datasheet.
     */
    private double voltageToDistance(double voltage) {
        // Example conversion for MaxBotix MB1013 sensor: Vcc/512 per cm
        double distanceCm = voltage * (512 / 3.3); // Assuming a 3.3V system
        return distanceCm;
    }

    /**
     * Adds a new reading to the weighted moving average.
     */
    private void addReading(double distance) {
        if (distanceReadings.size() >= MOVING_AVERAGE_WINDOW_SIZE) {
            // Remove the oldest reading
            double oldestReading = distanceReadings.poll();
            int oldestWeight = MOVING_AVERAGE_WINDOW_SIZE;
            weightedDistanceSum -= oldestReading * oldestWeight;
        }

        distanceReadings.add(distance);
        weightedDistanceSum += distance * distanceReadings.size();
    }

    /**
     * Initializes the total weight for the weighted moving average.
     */
    private void initializeWeights() {
        for (int i = 1; i <= MOVING_AVERAGE_WINDOW_SIZE; i++) {
            totalWeight += i;
        }
    }
}