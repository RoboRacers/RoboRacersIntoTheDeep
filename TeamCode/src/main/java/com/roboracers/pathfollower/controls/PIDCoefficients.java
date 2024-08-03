package com.roboracers.pathfollower.controls;

/**
 * The PIDCoefficients class stores the proportional, integral, and derivative gains
 * for a PID controller.
 */
public class PIDCoefficients {
    public double kp; // Proportional gain
    public double ki; // Integral gain
    public double kd; // Derivative gain

    /**
     * Constructs a PIDCoefficients object with the specified gains.
     *
     * @param kp the proportional gain
     * @param ki the integral gain
     * @param kd the derivative gain
     */
    public PIDCoefficients(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
}

