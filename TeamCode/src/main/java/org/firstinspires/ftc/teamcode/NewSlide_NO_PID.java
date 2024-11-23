package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="NO PID Motor Control", group="Linear Opmode")
public class NewSlide_NO_PID extends LinearOpMode {

    private DcMotor motor;

    private int targetPosition = 0;


    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "Slides_Left");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Set your target position, e.g., from gamepad input or pre-set value
            targetPosition = 400; // Example target

            motor.setTargetPosition(targetPosition);

            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.8);

            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.update();
        }
    }

}