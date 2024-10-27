package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp(name = "SLIDDESES TESTSET ", group = "16481-IntoTheDeep")
public class SlidesTest extends LinearOpMode {
    DcMotorImplEx motor;
    DcMotorImplEx slidesRight;
    DcMotorImplEx slidesLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");

        while(isStopRequested() == false){
            slidesRight.setPower(0.5 * gamepad1.right_stick_y);
            slidesLeft.setPower(0.5 * gamepad1.right_stick_y);

            telemetry.addData("RIGHT SLIDES", slidesRight.getCurrentPosition());
            telemetry.addData("LEFT SLIDES", slidesLeft.getCurrentPosition());
            telemetry.update();
        }

    }
}
