package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Feedforward Motor Control", group = "Control")
public class FeedforwardMotorControl extends LinearOpMode {

    private DcMotorEx motor;

    // Constants for feedforward control - tune these for your specific motor
    private static final double kS = 0.1; // Static gain
    private static final double kV = 0.02; // Velocity gain
    private static final double kA = 0.0; // Acceleration gain (optional)

    // Desired velocity in ticks per second (adjust as needed)
    private static final double targetVelocity = 1000.0;

    @Override
    public void runOpMode() {
        // Initialize motor
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Timer to calculate acceleration if needed
        ElapsedTime timer = new ElapsedTime();

        // Wait for start command from driver station
        waitForStart();

        // Initial conditions
        double lastVelocity = 0.0;
        double lastTime = timer.seconds();

        // Feedforward control loop
        while (opModeIsActive()) {
            // Calculate the time difference
            double currentTime = timer.seconds();
            double deltaTime = currentTime - lastTime;
            lastTime = currentTime;

            // Measure the motor's current velocity
            double currentVelocity = motor.getVelocity();  // Encoder velocity in ticks/sec

            // Calculate desired acceleration if `kA` is used
            double acceleration = kA != 0 ? (targetVelocity - lastVelocity) / deltaTime : 0.0;
            lastVelocity = currentVelocity;

            // Apply feedforward control
            double power = kS + kV * targetVelocity + kA * acceleration;

            // Set motor power
            motor.setPower(power);

            // Telemetry for debugging
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Power Output", power);
            telemetry.update();
        }

        // Stop motor after OpMode stops
        motor.setPower(0);
    }
}
