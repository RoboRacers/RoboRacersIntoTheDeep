package org.firstinspires.ftc.teamcode.robot.subsystems;

public class PIDController {
    private double kP, kI, kD;
    private double setpoint;
    private double lastError;
    private double integral;
    private double outputMin = Double.NEGATIVE_INFINITY;
    private double outputMax = Double.POSITIVE_INFINITY;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.lastError = 0;
        this.integral = 0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setOutputLimits(double min, double max) {
        outputMin = min;
        outputMax = max;
    }

    public double calculate(double current) {
        // Calculate error
        double error = setpoint - current;

        // Proportional term
        double P = kP * error;

        // Integral term
        integral += error;
        double I = kI * integral;

        // Derivative term
        double D = kD * (error - lastError);

        // Compute the output
        double output = P + I + D;

        // Save the current error as the last error for the next cycle
        lastError = error;

        // Clamp output to min and max bounds
        output = Math.max(outputMin, Math.min(outputMax, output));

        return output;
    }

    public void reset() {
        lastError = 0;
        integral = 0;
    }
}