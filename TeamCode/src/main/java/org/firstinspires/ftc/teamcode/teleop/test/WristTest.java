package org.firstinspires.ftc.teamcode.teleop.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Wrist Test", group = "16481-IntoTheDeep")
public class WristTest extends LinearOpMode {

    Servo rotateClaw;
    @Override
    public void runOpMode() throws InterruptedException {
        rotateClaw = hardwareMap.get(Servo.class, "rotateClaw");

        while (!isStopRequested()){
            rotateClaw.setPosition(gamepad1.left_stick_y);
            telemetry.addData("Pos Rotate Claw", rotateClaw.getPosition());
            telemetry.update();
        }
    }
}
