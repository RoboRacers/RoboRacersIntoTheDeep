package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Config
@TeleOp(name = "Motor with Encoder Reset", group = "Examples")
public class PitchTouchSensor extends LinearOpMode {

    private DcMotor motor;
    private TouchSensor touchSensor;
    public static int target;

    @Override
    public void runOpMode() {
        // Initialize hardware
        motor = hardwareMap.get(DcMotor.class, "pitchMotor");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        // Set motor to use encoders
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            // Move the motor forward when the gamepad's left stick is pushed up
//            double power = -gamepad1.left_stick_y; // Negative to make up correspond to forward
//            motor.setPower(power);

            motor.setTargetPosition(target);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.8);

            // Check if the touch sensor is pressed
            if (touchSensor.isPressed()) {
                telemetry.addData("Touch Sensor", "Pressed");

                // Reset encoders
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                telemetry.addData("Touch Sensor", "Not Pressed");
            }

            // Display the current encoder position
            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
