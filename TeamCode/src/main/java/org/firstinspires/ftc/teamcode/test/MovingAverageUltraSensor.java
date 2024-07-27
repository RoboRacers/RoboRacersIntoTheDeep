package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.LinkedList;
import java.util.Queue;

public class MovingAverageUltraSensor {

    private static final int MOVING_AVERAGE_WINDOW_SIZE = 250;

    private AnalogInput ultrasonicSensor;
    private Queue<Double> distanceReadings = new LinkedList<>();
    private double distanceSum = 0;

    public MovingAverageUltraSensor(AnalogInput ultrasonicSensor) {
        this.ultrasonicSensor = ultrasonicSensor;
    }

    /**
     * Reads the current distance from the sensor and updates the moving average.
     * @return The current distance in centimeters.
     */
    public double readDistance() {
        double voltage = ultrasonicSensor.getVoltage();
        double distance = voltageToDistance(voltage);

        // Update moving average
        addReading(distance);

        return distance;
    }

    /**
     * Gets the current moving average of the distance readings.
     * @return The moving average distance in centimeters.
     */
    public double getMovingAverage() {
        if (distanceReadings.isEmpty()) {
            return 0;
        }
        return distanceSum / distanceReadings.size();
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
     * Adds a new reading to the moving average.
     */
    private void addReading(double distance) {
        if (distanceReadings.size() >= MOVING_AVERAGE_WINDOW_SIZE) {
            distanceSum -= distanceReadings.poll();
        }
        distanceReadings.add(distance);
        distanceSum += distance;
    }
}
