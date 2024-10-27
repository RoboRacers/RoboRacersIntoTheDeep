package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp(name = "SLIDDESES TESTSET", group = "16481-IntoTheDeep")
public class SlidesTest extends LinearOpMode {
    DcMotorImplEx motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorImplEx.class, "Horizontal_Slides");

        while(isStopRequested() == false){
            motor.setPower(gamepad1.left_stick_y);
        }

    }
}
