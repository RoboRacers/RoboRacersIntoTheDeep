package com.roboracers.pathfollower.controls;

/**
 * The PIDController class implements a simple Proportional-Integral-Derivative (PID) controller.
 * This controller can be used to regulate the behavior of various systems, such as maintaining a
 * desired speed, position, or other measurable quantity.
 *
 * @version 1.0
 * @since 2024-07-28
 */
public class PIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Derivative gain
    private double setpoint; // Desired value
    private double previousError; // Previous error value
    private double integral; // Integral term

    /**
     * Constructs a PIDController with the specified gains.
     *
     * @param kp the proportional gain
     * @param ki the integral gain
     * @param kd the derivative gain
     */
    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.setpoint = 0.0;
        this.previousError = 0.0;
        this.integral = 0.0;
    }

    public PIDController(PIDCoefficients coefficients) {
        this.kp = coefficients.kp;
        this.ki = coefficients.ki;
        this.kd = coefficients.kd;
        this.setpoint = 0.0;
        this.previousError = 0.0;
        this.integral = 0.0;
    }

    /**
     * Sets the desired setpoint value for the PID controller.
     *
     * @param setpoint the desired setpoint value
     */
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    /**
     * Updates the PID controller with the current measured value and returns the control output.
     * @param currentValue the current measured value
     * @return the control output
     */
    public double update(double currentValue) {
        double error = setpoint - currentValue;
        integral += error;
        double derivative = error - previousError;
        double output = (kp * error) + (ki * integral) + (kd * derivative);
        previousError = error;
        return output;
    }
}
