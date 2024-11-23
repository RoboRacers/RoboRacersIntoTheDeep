package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "SLIDDESES TESTSET ", group = "16481-IntoTheDeep")
public class SlidesTest extends LinearOpMode {
    DcMotorImplEx motor;
    DcMotorImplEx slidesRight;
    DcMotorImplEx slidesLeft;


    @Override
    public void runOpMode() throws InterruptedException {
        slidesRight = hardwareMap.get(DcMotorImplEx.class, "Slides_Right");
        slidesLeft = hardwareMap.get(DcMotorImplEx.class, "Slides_Left");
        slidesLeft.setDirection(DcMotorImplEx.Direction.REVERSE);
        slidesLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slidesLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!isStopRequested()){
            slidesRight.setPower(1 * gamepad1.right_stick_y);
            slidesLeft.setPower(1 * gamepad1.right_stick_y);

            if(gamepad1.right_trigger > 0.1 && gamepad1.right_stick_y < 0.1 && gamepad1.right_stick_y > -0.1){
                slidesRight.setPower(-.2);
                slidesLeft.setPower(-.2);
            }else if(gamepad1.left_trigger > 0.1 && gamepad1.right_stick_y < 0.1 && gamepad1.right_stick_y > -0.1){
                slidesRight.setPower(-.8);
                slidesLeft.setPower(-.8);
            }


            telemetry.addData("RIGHT SLIDES", slidesRight.getCurrentPosition());
            telemetry.addData("LEFT SLIDES", slidesLeft.getCurrentPosition());
            telemetry.addData("RIGHT SLIDES CURRET", slidesRight.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Left SLIDES CURRET", slidesLeft.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }

    }
}
