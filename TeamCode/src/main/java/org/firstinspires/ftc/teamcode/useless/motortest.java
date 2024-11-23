package org.firstinspires.ftc.teamcode.useless;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="motor test for sldiess", group="Linear OpMode")

public class motortest extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("motor");

        waitForStart();

        while (opModeIsActive()){
            motor.setPower(gamepad1.left_stick_y);
        }
    }
}
