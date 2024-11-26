package org.firstinspires.ftc.teamcode.teleop;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "Motor Test", group = "Test")
public class MotorTest extends LinearOpMode {
    public DcMotorImplEx slidesMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        slidesMotor = hardwareMap.get(DcMotorImplEx.class, "slidesMotor");

        while (opModeInInit()) {
            // 175
            // 420
        }

//        pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!isStopRequested()) {

//            if (gamepad1.triangle) {
//                pitchMotor.setTargetPosition(420);
//                pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pitchMotor.setPower(0.75);
//            } else if (gamepad1.cross) {
//                pitchMotor.setTargetPosition(175);
//                pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pitchMotor.setPower(0.75);
//            } else if (gamepad1.circle) {
//                pitchMotor.setTargetPosition(30);
//                pitchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pitchMotor.setPower(0.75);
//            } else {
//                pitchMotor.setPower(gamepad1.left_stick_y);
//            }

//            telemetry.addData("Pitch Motor Position", pitchMotor.getCurrentPosition());
//            telemetry.addData("Pitch Motor Power", pitchMotor.getPower());
//            telemetry.addData("Pitch Current", pitchMotor.getCurrent(CurrentUnit.MILLIAMPS));
            if(gamepad1.a){
                slidesMotor.setPower(gamepad1.right_stick_x);
            }else {
                slidesMotor.setPower(gamepad1.right_stick_x*0.4);
            }
            telemetry.addData("Slides Power", slidesMotor.getPower());
            telemetry.addData("slides Pos", slidesMotor.getCurrentPosition());

            telemetry.update();

        }
    }
}
