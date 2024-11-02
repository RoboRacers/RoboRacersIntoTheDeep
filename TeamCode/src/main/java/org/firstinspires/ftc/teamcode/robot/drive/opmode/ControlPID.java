package org.firstinspires.ftc.teamcode.controlsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID Control Wheel Speed", group = "ControlSystems")
public class ControlPID extends LinearOpMode {

    private DcMotorEx wheelMotor;
    private static final double kP = 0.01;  // Proportional gain, tune these
    private static final double kI = 0.0005;  // Integral gain
    private static final double kD = 0.005;  // Derivative gain
    private static final double targetVelocity = 1000.0;  // Target speed in ticks/sec

    private double integralSum = 0.0;
    private double lastError = 0.0;

    @Override
    public void runOpMode() {
        wheelMotor = hardwareMap.get(DcMotorEx.class, "wheelMotor");
        wheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime timer = new ElapsedTime();
        double lastTime = timer.seconds();

        waitForStart();

        while (opModeIsActive()) {
            // Calculate error
            double currentVelocity = wheelMotor.getVelocity();
            double error = targetVelocity - currentVelocity;

            // Calculate time difference
            double currentTime = timer.seconds();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            // Integral sum
            integralSum += error * deltaTime;

            // Derivative (change in error)
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            // PID calculation
            double power = (kP * error) + (kI * integralSum) + (kD * derivative);

            // Limit power to motor range
            power = Math.max(-1.0, Math.min(1.0, power));

            // Set motor power
            wheelMotor.setPower(power);

            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Power Output", power);
            telemetry.update();
        }
    }
}
